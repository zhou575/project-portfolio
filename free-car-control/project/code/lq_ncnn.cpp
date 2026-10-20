#include "lq_ncnn.hpp"
#include "debug_switch.hpp"
#include "red_roi.hpp"
#include "car_init.hpp"
#include "image.hpp"
#include "encoder.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>
#include <sys/stat.h>
#include <sys/types.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

//模型推理是否成功
bool is_model_infer_complete=false;
//绕行状态
char model_control_access='0';
// 红色矩形是否存在（每帧由 red_roi 预检测刷新）
bool model_red_found=false;

namespace
{
//日志输出
#define LQ_NCNN_LOG_INFO(fmt, ...) std::printf("[LQ_NCNN][INFO][%s] " fmt "\r\n", __func__, ##__VA_ARGS__)
#define LQ_NCNN_LOG_ERR(fmt, ...)  std::printf("[LQ_NCNN][ERR][%s] " fmt "\r\n", __func__, ##__VA_ARGS__)

//三分类模型的类别名字
const char* kClassLabels[] = {"supplies", "vehicle", "weapon"};
constexpr int kClassLabelCount = static_cast<int>(sizeof(kClassLabels) / sizeof(kClassLabels[0]));

// 连续 N 帧稳定后才锁定类别，避免单帧抖动导致误判。
constexpr int kGuardedStableFrameCount = 1;
constexpr char kModelControlNone = '0';
constexpr char kModelControlLeft = 'l';
constexpr char kModelControlRight = 'r';
constexpr char kModelControlStraight = 'g';

// true: class_scores 输出原始 logit（仅调试）；false: 输出 softmax 概率（推荐）
constexpr bool kDebugOutputRawLogits = false;
// 只有 top1 概率超过该阈值，才允许作为最终输出候选
constexpr float kMinOutputClassProb = 0.4f;
//网络结构定义
constexpr const char* kDefaultModelParam = "tiny_classifier_fp32.ncnn.param";
//权重参数
constexpr const char* kDefaultModelBin = "tiny_classifier_fp32.ncnn.bin";
// 默认模型输入尺寸（推理统一 48x48）
constexpr int kDefaultInputWidth = 48;
constexpr int kDefaultInputHeight = 48;

bool g_lq_ncnn_ready = false;           //模型是否已经完成初始化
int g_guarded_last_predict_index = -1;  //上一帧预测类别
int g_guarded_same_predict_count = 0;   //连续相同预测计数
int g_guarded_last_output_predict_index = -1; // 上次已输出的类
bool g_guarded_output_latched = false;  // 成功输出一次后锁存，直到近处红块消失才解锁

LQ_NCNN g_lq_ncnn_model;                //全局 NCNN 分类器实例

// ===== 调试代码开始：lq_ncnn ROI 调试缓存 =====
cv::Mat g_debug_raw_roi_patch;          // red_roi真实宽高拉正后的原始ROI图
cv::Mat g_debug_roi_patch;              // extract_rotated_roi_patch 后的ROI图
// 这两张图都不参与模型推理结果计算，只服务于 TCP 看图和 ROI 链路排查。
// g_debug_raw_roi_patch 用来看“ROI 预测框本身是否合理”；
// g_debug_roi_patch 用来看“真正送入模型的图像是什么”。
// ===== 调试代码结束：lq_ncnn ROI 调试缓存 =====

// 判断是不是支持的图片文件
bool lq_ncnn_is_supported_image_file(const std::string& file_name)
{
    // 找到最后一个点
    const std::size_t dot_pos = file_name.find_last_of('.');
    // 如果没有点 → 不是图片
    if (dot_pos == std::string::npos)
    {
        return false;
    }
    // 截取后缀（包含点 .）
    std::string ext = file_name.substr(dot_pos);
    // 全部转小写
    std::transform(ext.begin(), ext.end(), ext.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    // 判断是不是支持的图片格式
    return ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".webp";
}

// 遍历目录，收集所有图片路径
bool lq_ncnn_collect_picture_files(const char* picture_dir, std::vector<std::string>* file_paths)
{
    // 1. 检查输入是否合法
    if (picture_dir == nullptr || picture_dir[0] == '\0' || file_paths == nullptr)
    {
        return false;
    }
    // 2. 打开文件夹
    DIR* dir = opendir(picture_dir);
    if (dir == nullptr)
    {
        LQ_NCNN_LOG_ERR("open dir failed: %s errno=%d", picture_dir, errno);
        return false;
    }
    // 3. 清空输出列表
    file_paths->clear();
    // 4. 遍历文件夹里所有文件
    struct dirent* entry = nullptr;
    while ((entry = readdir(dir)) != nullptr)
    {
        // 拿到文件名
        const char* name_ptr = entry->d_name;
        if (name_ptr == nullptr)
        {
            continue;
        }
        // 跳过 . 和 ..
        const std::string name(name_ptr);
        if (name == "." || name == "..")
        {
            continue;
        }
        // 不是支持的图片 → 跳过
        if (!lq_ncnn_is_supported_image_file(name))
        {
            continue;
        }
        // 拼接完整路径：目录 + / + 文件名
        const std::string full_path = std::string(picture_dir) + "/" + name;
        // 获取文件信息，判断是不是普通文件
        struct stat st = {};
        if (stat(full_path.c_str(), &st) != 0)
        {
            continue;
        }
        if (!S_ISREG(st.st_mode))
        {
            continue;
        }
        // 是图片文件 → 加入列表
        file_paths->push_back(full_path);
    }
    // 5. 关闭文件夹
    closedir(dir);

    // 6. 按文件名排序
    std::sort(file_paths->begin(), file_paths->end());
    return true;
}

/**
 * @brief 配置默认运行时模型
 */
void configure_default_runtime_model()
{
    LQ_NCNN_LOG_INFO("apply runtime model config: param=%s bin=%s input=%dx%d",
                     kDefaultModelParam,
                     kDefaultModelBin,
                     kDefaultInputWidth,
                     kDefaultInputHeight);
    g_lq_ncnn_model.SetModelPath(kDefaultModelParam, kDefaultModelBin);
    g_lq_ncnn_model.SetInputSize(kDefaultInputWidth, kDefaultInputHeight);
    g_lq_ncnn_model.SetLabels(std::vector<std::string>(std::begin(kClassLabels), std::end(kClassLabels)));

    //归一化参数
    const float mean_vals[3] = {123.675f, 116.28f, 103.53f};
    const float norm_vals[3] = {0.01712475f, 0.017507f, 0.01742919f};
    g_lq_ncnn_model.SetNormalize(mean_vals, norm_vals);
}

/**
 * @brief 填充推理结果结构体
 * @param class_id 预测类别ID
 * @param score 预测得分
 * @param class_scores 各类别得分
 * @param invoke_time_us 推理耗时
 * @param result 推理结果结构体
 */
void fill_infer_result(const int class_id,
                       const float score,
                       const std::vector<float>& class_scores,
                       const int64_t invoke_time_us,
                       LqNcnnInferResult* result)
{
    if (result == nullptr)
    {
        return;
    }

    result->predict_index = class_id;
    result->predict_score = score;
    result->invoke_time_us = invoke_time_us;

    //计算要拷贝多少个类别分数
    const int count = std::min(static_cast<int>(class_scores.size()), LQ_NCNN_OUTPUT_CLASS_MAX_NUM);
    result->class_count = count;
    //清零结构体所有类别
    std::fill(std::begin(result->class_scores), std::end(result->class_scores), 0.0f);
    //把有效类别分数拷进去
    for (int i = 0; i < count; ++i)
    {
        result->class_scores[i] = class_scores[i];
    }
}

/**
 * @brief 把“连续帧稳定判断”的内部状态清零，恢复到初始状态
 */
void lq_ncnn_reset_guarded_state(void)
{
    g_guarded_last_predict_index = -1;
    g_guarded_same_predict_count = 0;
}

/**
 * @brief debug_roi,清除debug_roi区域
 */
void lq_ncnn_reset_debug_roi_patch(void)
{
#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：清空 lq_ncnn 的 ROI 调试缓存 =====
    g_debug_raw_roi_patch.release();
    g_debug_roi_patch.release();
    // ===== 调试代码结束：清空 lq_ncnn 的 ROI 调试缓存 =====
#endif
}

#if ENABLE_VISION_DEBUG
/**
 * @brief debug_roi,保存red_roi真实宽高拉正后的原始ROI图
 */
void lq_ncnn_save_debug_raw_roi_patch(const cv::Mat& roi_img)
{
    // ===== 调试代码开始：保存按真实 ROI 宽高拉正后的原始 ROI 调试图 =====
    if (roi_img.empty())
    {
        g_debug_raw_roi_patch.release();
        return;
    }
    g_debug_raw_roi_patch = roi_img.clone();
    // ===== 调试代码结束：保存按真实 ROI 宽高拉正后的原始 ROI 调试图 =====
}

/**
 * @brief debug_roi,保存传入模型的图片，g_debug_roi_patch
 */
void lq_ncnn_save_debug_roi_patch(const cv::Mat& roi_img)
{
    // ===== 调试代码开始：保存真正送入模型的 ROI 调试图 =====
    if (roi_img.empty())
    {
        g_debug_roi_patch.release();
        return;
    }
    g_debug_roi_patch = roi_img.clone();
    // ===== 调试代码结束：保存真正送入模型的 ROI 调试图 =====
}
#endif

/**
 * @brief 重置推理结果结构体为默认值,把输出结果结构体清零
 * @note 一旦后面检测失败或推理失败，调用方就不会误读到旧数据
 */
void reset_infer_result(LqNcnnInferResult* result)
{
    if (result == nullptr)
    {
        return;
    }

    result->predict_index = -1;
    result->predict_score = 0.0f;
    result->class_count = 0;
    result->invoke_time_us = 0;
    std::fill(std::begin(result->class_scores), std::end(result->class_scores), 0.0f);
}

/**
 * @brief 把分类结果映射为绕行控制模式
 * @note index0=supplies(物资)->右绕；index1=vehicle(交通)->直行；index2=weapon(武器)->左绕
 */
char map_predict_to_bypass_mode(const int predict_index)
{
    if (predict_index == 0)
    {
        return kModelControlRight;
    }
    if (predict_index == 1)
    {
        return kModelControlStraight;
    }
    if (predict_index == 2)
    {
        return kModelControlLeft;
    }
    return kModelControlNone;
}


}

/**
 * @brief LQ_NCNN 分类器实现
 */
LQ_NCNN::LQ_NCNN()
    : m_initialized(false)
    , m_input_width(kDefaultInputWidth)
    , m_input_height(kDefaultInputHeight)
    , m_input_name("in0")
    , m_output_name("out0")
{
    m_mean_vals[0] = 123.675f;
    m_mean_vals[1] = 116.28f;
    m_mean_vals[2] = 103.53f;
    m_norm_vals[0] = 0.01712475f;
    m_norm_vals[1] = 0.017507f;
    m_norm_vals[2] = 0.01742919f;
}

/**
 * @brief 初始化 NCNN 分类器
 * @return 是否初始化成功
 */
bool LQ_NCNN::Init()
{
    // 设置NCNN选项,只初始化一次
    if (m_initialized)
    {
        LQ_NCNN_LOG_INFO("already initialized, skip.");
        return true;
    }

    //设置 NCNN 运行选项
    m_net.opt.use_vulkan_compute = false;//不用 Vulkan GPU 推理
    m_net.opt.num_threads = 1;//单线程推理
    LQ_NCNN_LOG_INFO("init begin: param=%s bin=%s input=%dx%d in_blob=%s out_blob=%s",
                     m_param_path.c_str(),
                     m_bin_path.c_str(),
                     m_input_width,
                     m_input_height,
                     m_input_name.c_str(),
                     m_output_name.c_str());

    if (m_param_path.empty())
    {
        LQ_NCNN_LOG_ERR("model param path is empty.");
        return false;
    }
    // 加载模型参数
    const int param_ret = m_net.load_param(m_param_path.c_str());
    if (param_ret != 0)
    {
        LQ_NCNN_LOG_ERR("load_param failed: path=%s ret=%d", m_param_path.c_str(), param_ret);
        return false;
    }

    if (m_bin_path.empty())
    {
        LQ_NCNN_LOG_ERR("model bin path is empty.");
        return false;
    }

    // 加载模型权重
    const int model_ret = m_net.load_model(m_bin_path.c_str());
    if (model_ret != 0)
    {
        LQ_NCNN_LOG_ERR("load_model failed: path=%s ret=%d", m_bin_path.c_str(), model_ret);
        return false;
    }

    m_initialized = true;
    LQ_NCNN_LOG_INFO("init success.");
    return true;
}

/**
 * @brief 找到 logits 的最大概率类别并输出概率分布
 * @param logits 输入的 logits
 * @param best_score 输出的最大分数
 * @param class_scores 输出的类别分数
 * @return 把 NCNN 网络输出 logits 展平成一串 float，必要时先解包，先做 softmax，再找到最大概率类别；可选输出每类分数
 */
int LQ_NCNN::Argmax(const ncnn::Mat& logits, float* best_score, std::vector<float>* class_scores)
{
    //做一个浅拷贝/引用式赋值，默认先把 logits 当作待处理数据,如果后面需要“解包”，就把结果放到 unpacked
    ncnn::Mat unpacked = logits;
    //NCNN 为了优化计算和内存，可能会把数据按 pack 方式存
    //elempack = 1：普通标量排列,不等于1就是打包
    if (logits.elempack != 1)
    {
        ncnn::Option opt;
        opt.num_threads = 1;
        //把它转换成最普通的 pack1 形式
        ncnn::convert_packing(logits, unpacked, 1, opt);
        //如果转换后 unpacked 是空的，说明解包失败，函数直接返回错误
        if (unpacked.empty())
        {
            LQ_NCNN_LOG_ERR("convert_packing to pack1 failed.");
            return -1;
        }
    }
    //计算总元素个数 宽*高*深度*通道，三分类通道就是3
    const int total = unpacked.w * unpacked.h * unpacked.d * unpacked.c;
    //检查输出是否有效
    if (total <= 0 || unpacked.empty())
    {
        LQ_NCNN_LOG_ERR("argmax invalid unpacked mat: empty=%d total=%d dims=%d w=%d h=%d d=%d c=%d elempack=%d",
                        unpacked.empty() ? 1 : 0,
                        total,
                        unpacked.dims,
                        unpacked.w,
                        unpacked.h,
                        unpacked.d,
                        unpacked.c,
                        unpacked.elempack);
        return -1;
    }

    //取底层 float 指针，转换类型
    const float* ptr = static_cast<const float*>(unpacked);
    if (ptr == nullptr)
    {
        LQ_NCNN_LOG_ERR("argmax unpacked data pointer is null.");
        return -1;
    }

    // 先做数值稳定 softmax，把 logits 转为 0~1 概率
    std::vector<float> probabilities(total, 0.0f);
    float max_logit = ptr[0];
    for (int i = 1; i < total; ++i)
    {
        if (ptr[i] > max_logit)
        {
            max_logit = ptr[i];
        }
    }

    double exp_sum = 0.0;
    //计算 softmax 分子和总和,暂时存到 probabilities[i]
    for (int i = 0; i < total; ++i)
    {
        const double exp_v = std::exp(static_cast<double>(ptr[i] - max_logit));
        probabilities[i] = static_cast<float>(exp_v);
        exp_sum += exp_v;
    }
    //检查 softmax 分母是否合法
    if (exp_sum <= std::numeric_limits<double>::epsilon())
    {
        LQ_NCNN_LOG_ERR("softmax exp_sum invalid: %.12f", exp_sum);
        return -1;
    }
    //计算分母倒数
    const float inv_sum = static_cast<float>(1.0 / exp_sum);
    //完成 softmax 归一化
    for (int i = 0; i < total; ++i)
    {
        probabilities[i] *= inv_sum;
    }

    //如果需要，把每类分数输出给调用方
    if (class_scores != nullptr)
    {   
        //先分配大小
        class_scores->assign(total, 0.0f);
        for (int i = 0; i < total; ++i)
        {
            //写每个类别的分数
            //如果 kDebugOutputRawLogits == true,写原始 logits：ptr[i],否则写 softmax 概率：probabilities[i]
            (*class_scores)[i] = kDebugOutputRawLogits ? ptr[i] : probabilities[i];
        }
    }

    //初始化当前最优类别
    int best_index = 0;
    float best_value = probabilities[0];
    //遍历找最大概率
    for (int i = 1; i < total; ++i)
    {
        if (probabilities[i] > best_value)
        {
            best_value = probabilities[i];
            best_index = i;
        }
    }

    //如果需要，把最大概率写出去
    if (best_score != nullptr)
    {
        *best_score = best_value;
    }
    //返回最大值下标
    return best_index;
}

/**
 * @brief 推理输入图像的类别
 * @param bgr_image 输入图像，BGR 格式
 * @param predict_score 输出的预测分数
 * @param class_scores 输出的类别分数
 * @param invoke_time_us 输出的推理时间（微秒）
 * @return 推理得到的类别 ID
 * @throws std::runtime_error 如果推理失败或模型未初始化
 * @throws std::invalid_argument 如果输入图像为空
 */
int LQ_NCNN::InferClass(const cv::Mat& bgr_image,
                        float* predict_score,
                        std::vector<float>* class_scores,
                        int64_t* invoke_time_us)
{
    //输入检查，模型没有初始化不进行推理
    if (!m_initialized)
    {
        LQ_NCNN_LOG_ERR("infer called before init.");
        //没初始化 NCNN，直接报错、停止运行
        throw std::runtime_error("NCNN not initialized. Call Init() first.");
    }
    if (bgr_image.empty())
    {
        LQ_NCNN_LOG_ERR("input image is empty.");
        throw std::invalid_argument("Input image is empty.");
    }

    //resize 到固定输入尺寸
    cv::Mat resized;
    cv::resize(bgr_image, resized, cv::Size(m_input_width, m_input_height));

    //BGR 转 RGB，NCNN 默认使用 RGB 输入
    cv::Mat rgb;
    cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

    //转成 NCNN 的ncnn::Mat
    ncnn::Mat input = ncnn::Mat::from_pixels(
        rgb.data,
        ncnn::Mat::PIXEL_RGB,
        m_input_width,
        m_input_height);
    //做归一化
    input.substract_mean_normalize(m_mean_vals, m_norm_vals);

    //创建提取器
    ncnn::Extractor ex = m_net.create_extractor();
    //启用轻量模式，尽量减少中间 blob 的内存占用
    //blob 指的是：网络中某一层的输出数据块，也就是中间特征张量
    ex.set_light_mode(true);
    
    //计时并送入输入 blob
    const auto start_time = std::chrono::high_resolution_clock::now();
    //把 input 这个张量喂到网络输入层 m_input_name 上。
    const int input_ret = ex.input(m_input_name.c_str(), input);
    if (input_ret != 0)
    {
        LQ_NCNN_LOG_ERR("extractor input failed: blob=%s ret=%d", m_input_name.c_str(), input_ret);
        throw std::runtime_error("NCNN extractor input failed.");
    }

    //从网络输出层 m_output_name 中提取结果，保存到 logits
    ncnn::Mat logits;
    const int extract_ret = ex.extract(m_output_name.c_str(), logits);
    const auto end_time = std::chrono::high_resolution_clock::now();

    if (extract_ret != 0)
    {
        LQ_NCNN_LOG_ERR("extractor output failed: blob=%s ret=%d", m_output_name.c_str(), extract_ret);
        throw std::runtime_error("NCNN extractor output failed.");
    }

    //计算推理时间
    int64_t invoke_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
    if (invoke_time_us != nullptr)
    {
        *invoke_time_us = invoke_us;
    }

    //找最大类别
    float best_score = 0.0f;
    const int class_id = Argmax(logits, &best_score, class_scores);
    if (predict_score != nullptr)
    {
        *predict_score = best_score;
    }
    //没有输出类别
    if (class_id < 0)
    {
        LQ_NCNN_LOG_ERR("argmax failed.");
    }
    return class_id;
}

/**
 * @brief 对输入图像进行推理，返回预测类别名称
 * @param bgr_image 输入图像，BGR 格式
 * @return 预测类别名称，如果类别 ID 超出标签范围，则返回类别 ID 的字符串形式
 * @throws std::runtime_error 如果推理失败或模型未初始化
 * @throws std::invalid_argument 如果输入图像为空
 */
std::string LQ_NCNN::Infer(const cv::Mat& bgr_image)
{
    std::vector<float> class_scores;
    float predict_score = 0.0f;
    const int class_id = InferClass(bgr_image, &predict_score, &class_scores, nullptr);
    if (class_id < 0)
    {
        throw std::runtime_error("Failed to infer class id from ncnn output.");
    }

    if (class_id < static_cast<int>(m_labels.size()))
    {
        return m_labels[class_id];
    }
    return std::to_string(class_id);
}

/**
 * @brief 设置模型路径
 * @param param_path 参数文件路径
 * @param bin_path 权重文件路径
 */
void LQ_NCNN::SetModelPath(const std::string& param_path, const std::string& bin_path)
{
    m_param_path = param_path;
    m_bin_path = bin_path;
}

/**
 * @brief 设置输入图像尺寸
 * @param width 输入图像宽度
 * @param height 输入图像高度
 */
void LQ_NCNN::SetInputSize(int width, int height)
{
    m_input_width = width;
    m_input_height = height;
}

/**
 * @brief 设置标签
 * @param labels 标签向量
 */
void LQ_NCNN::SetLabels(const std::vector<std::string>& labels)
{
    m_labels = labels;
}

/**
 * @brief 设置归一化参数
 * @param mean_vals 均值数组
 * @param norm_vals 标准差数组
 */
void LQ_NCNN::SetNormalize(const float mean_vals[3], const float norm_vals[3])
{
    m_mean_vals[0] = mean_vals[0];
    m_mean_vals[1] = mean_vals[1];
    m_mean_vals[2] = mean_vals[2];
    m_norm_vals[0] = norm_vals[0];
    m_norm_vals[1] = norm_vals[1];
    m_norm_vals[2] = norm_vals[2];
}

//释放类对象内部的 NCNN 网络资源
void LQ_NCNN::Deinit()
{
    if (!m_initialized)
    {
        LQ_NCNN_LOG_INFO("deinit skip, model not initialized.");
        return;
    }

    m_net.clear();
    m_initialized = false;
    LQ_NCNN_LOG_INFO("deinit success, net resources released.");
}

LQ_NCNN::~LQ_NCNN() = default;

//释放全局单例模型，并顺带把 guarded 状态一起清掉
int lq_ncnn_deinit(void)
{
    if (!g_lq_ncnn_ready)
    {
        LQ_NCNN_LOG_INFO("global model deinit skip, not ready.");
        return 0;
    }

    g_lq_ncnn_model.Deinit();
    g_lq_ncnn_ready = false;
    lq_ncnn_reset_guarded_state();
    g_guarded_last_output_predict_index = -1;
    g_guarded_output_latched = false;
    //清零绕行状态
    model_red_found = false;
    is_model_infer_complete = false;
    model_control_access = kModelControlNone;

    // ===== 调试代码开始：模型释放时顺手清空 ROI 调试缓存 =====
    lq_ncnn_reset_debug_roi_patch();
    // ===== 调试代码结束：模型释放时顺手清空 ROI 调试缓存 =====
    LQ_NCNN_LOG_INFO("global model deinit finished.");
    return 0;
}

// 轻量复位：仅清模型控制状态
void lq_ncnn_reset_control_state(void)
{
    lq_ncnn_reset_guarded_state();
    g_guarded_last_output_predict_index = -1;
    g_guarded_output_latched = false;
    model_red_found = false;
    is_model_infer_complete = false;
    model_control_access = kModelControlNone;
}

/**
 * @brief 初始化全局 NCNN 分类器模型
 * @return 是否初始化成功,只初始化一次
 */
int lq_ncnn_init(void)
{
    if (g_lq_ncnn_ready)
    {
        return 0;
    }

    LQ_NCNN_LOG_INFO("global model init start.");
    configure_default_runtime_model();
    if (!g_lq_ncnn_model.Init())
    {
        g_lq_ncnn_ready = false;
        LQ_NCNN_LOG_ERR("global model init failed.");
        return -1;
    }

    g_lq_ncnn_ready = true;
    LQ_NCNN_LOG_INFO("global model init success.");
    return 0;
}

/**
 * @brief 进行 NCNN 推理
 * @param src_bgr 输入的 BGR 图像
 * @param result 输出的推理结果
 * @return 是否推理成功
 */
int lq_ncnn_infer(const cv::Mat& src_bgr, LqNcnnInferResult* result)
{
    if (result == nullptr)
    {
        LQ_NCNN_LOG_ERR("infer result pointer is null.");
        return -1;
    }
    reset_infer_result(result);

    if (src_bgr.empty())
    {
        LQ_NCNN_LOG_ERR("infer source image empty.");
        return -1;
    }
    if (lq_ncnn_init() != 0)
    {
        LQ_NCNN_LOG_ERR("infer aborted because lq_ncnn_init failed.");
        return -1;
    }

    float predict_score = 0.0f;
    int64_t invoke_time_us = 0;
    std::vector<float> class_scores;

    try
    {
        //得到最大的分数的类别索引
        const int class_id = g_lq_ncnn_model.InferClass(src_bgr, &predict_score, &class_scores, &invoke_time_us);
        if (class_id < 0)
        {
            LQ_NCNN_LOG_ERR("infer got invalid class_id=%d", class_id);
            return -1;
        }

        fill_infer_result(class_id, predict_score, class_scores, invoke_time_us, result);
        return 0;
    }
    catch (const std::exception& e)
    {
        std::printf("NCNN infer error: %s\n", e.what());
        return -1;
    }
}

/**
 * @brief 批量读取目录下图片并执行模型推理
 * @param picture_dir 图片目录（空时默认使用 "picture"）
 * @return 0=全部成功，-1=存在失败
 */
int lq_ncnn_batch_infer_picture_dir(const char* picture_dir)
{
    //为空就用默认目录 "picture"
    const char* target_dir = (picture_dir != nullptr && picture_dir[0] != '\0') ? picture_dir : "picture";
    std::vector<std::string> image_files;
    //收集所有图片路径
    if (!lq_ncnn_collect_picture_files(target_dir, &image_files))
    {
        LQ_NCNN_LOG_ERR("collect picture files failed: %s", target_dir);
        return -1;
    }
    // 如果一张图片都没找到 → 退出
    if (image_files.empty())
    {
        LQ_NCNN_LOG_ERR("no supported image files found in: %s", target_dir);
        return -1;
    }
    // 初始化统计变量
    const int class_count = kClassLabelCount;
    int success_count = 0;
    int fail_count = 0;
    // 打印批处理开始日志
    std::printf("[LQ_NCNN][TEST] batch infer start, dir=%s files=%zu\r\n",
                target_dir,
                static_cast<std::size_t>(image_files.size()));
    // 遍历每一张图片，开始 AI 推理
    for (std::size_t i = 0; i < image_files.size(); ++i)
    {
        const std::string& path = image_files[i];
        cv::Mat roi_img = cv::imread(path, cv::IMREAD_COLOR);
        // 读取图片
        if (roi_img.empty())
        {
            ++fail_count;
            std::printf("[LQ_NCNN][TEST][%zu/%zu] %s => read_failed\r\n",
                        i + 1,
                        image_files.size(),
                        path.c_str());
            continue;
        }
        // 执行核心：NCNN 模型 AI 推理
        LqNcnnInferResult infer_result = {};
        if (lq_ncnn_infer(roi_img, &infer_result) != 0)
        {
            ++fail_count;
            std::printf("[LQ_NCNN][TEST][%zu/%zu] %s => infer_failed\r\n",
                        i + 1,
                        image_files.size(),
                        path.c_str());
            continue;
        }
        // 推理成功：计数 + 打印结果
        ++success_count;
        const int cls = infer_result.predict_index;
        const char* cls_name = (cls >= 0 && cls < class_count) ? kClassLabels[cls] : "unknown";
        std::printf("[LQ_NCNN][TEST][%zu/%zu] %s => \r\n cls=%s(%d) score=%.4f cost=%lldus\r\n",
                    i + 1,
                    image_files.size(),
                    path.c_str(),
                    cls_name,
                    cls,
                    infer_result.predict_score,
                    static_cast<long long>(infer_result.invoke_time_us));
    }
    // 所有图片处理完成：打印最终统计
    std::printf("[LQ_NCNN][TEST] batch infer done, total=%zu success=%d fail=%d\r\n",
                image_files.size(),
                success_count,
                fail_count);
    return (fail_count == 0) ? 0 : -1;
}

/**
 * @brief 从摄像头获取图像并进行检测和推理
 * @note  至少3帧推理一致才算成功；且成功输出一次后会锁存，直到近处红块消失才允许再次输出
 * @param roi_size ROI 尺寸
 * @param infer_result 推理结果
 * @return 0 成功
 */
int lq_ncnn_detect_and_infer_guarded(const cv::Mat& src_img,int roi_size,LqNcnnInferResult* infer_result)
{
    if(imgpro_var.protect_flag==true)
    {
        model_red_found = false;
        return -1;
    }
    if (infer_result == nullptr)
    {
        model_red_found = false;
        LQ_NCNN_LOG_ERR("null_pointer_input");
        return -1;
    }
    //先取当前相机帧
    if (src_img.empty())
    {
        model_red_found = false;
        // ===== 调试代码开始：当前帧无图像时清空 ROI 调试缓存，避免上位机看到旧图 =====
        lq_ncnn_reset_debug_roi_patch();
        // ===== 调试代码结束：当前帧无图像时清空 ROI 调试缓存，避免上位机看到旧图 =====
        reset_infer_result(infer_result);
        lq_ncnn_reset_guarded_state();
        LQ_NCNN_LOG_ERR("camera_frame_empty");
        return -1;
    }
    //先做一次 ROI 预检测
    RedRoiResult precheck_roi = {};
    // red_roi_find 返回3说明找到了近处 ROI，只有该状态才开始推理。
    const int precheck_ret = red_roi_find(src_img, &precheck_roi);
    model_red_found = precheck_roi.red_found;
    if (precheck_ret != 3)
    {
        // ===== 调试代码开始：当前帧未进入有效 ROI 推理链时清空 ROI 调试缓存 =====
        lq_ncnn_reset_debug_roi_patch();
        // ===== 调试代码结束：当前帧未进入有效 ROI 推理链时清空 ROI 调试缓存 =====
        reset_infer_result(infer_result);
        lq_ncnn_reset_guarded_state();
        // red_roi_find:
        // 0=无红，1=有红但不近，2=近处红但ROI失败，3=近处红且ROI成功
        // 只有近处红块消失（ret<=1）才允许解锁再次输出。
        if (precheck_ret <= 1)
        {
            g_guarded_last_output_predict_index = -1;
            g_guarded_output_latched = false;
        }
        return -1;
    }

    // 已输出过一次且红块仍在近处：直接锁住，不再进入 ROI 预测与模型推理。
    if (g_guarded_output_latched)
    {
        lq_ncnn_reset_debug_roi_patch();
        reset_infer_result(infer_result);
        return -1;
    }
    cv::Mat roi_img;
    cv::Mat raw_roi_img;
    const int target_roi_width = kDefaultInputWidth;
    const int target_roi_height = kDefaultInputHeight;
    (void)roi_size;
    if (target_roi_width <= 0 || target_roi_height <= 0)
    {
        // ===== 调试代码开始：ROI 目标尺寸异常时清空 ROI 调试缓存 =====
        lq_ncnn_reset_debug_roi_patch();
        // ===== 调试代码结束：ROI 目标尺寸异常时清空 ROI 调试缓存 =====
        reset_infer_result(infer_result);
        lq_ncnn_reset_guarded_state();
        LQ_NCNN_LOG_ERR("invalid_target_roi_size=%dx%d", target_roi_width, target_roi_height);
        return -1;
    }

#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：额外生成“真实 ROI 宽高拉正图”用于调试左图显示 =====
    // 先按 red_roi 本帧预测出来的真实宽高做一次拉正，专门给 TCP 调试看 ROI 预测质量。
    if (precheck_roi.roi_width_px > 0 && precheck_roi.roi_height_px > 0 &&
        extract_rotated_roi_patch(src_img,
                                  precheck_roi.roi_points,
                                  precheck_roi.roi_width_px,
                                  precheck_roi.roi_height_px,
                                  &raw_roi_img))
    {
        lq_ncnn_save_debug_raw_roi_patch(raw_roi_img);
    }
    else
    {
        lq_ncnn_save_debug_raw_roi_patch(cv::Mat());
    }
    // ===== 调试代码结束：额外生成“真实 ROI 宽高拉正图”用于调试左图显示 =====
#endif

    // 这里通过 red_roi 对外接口调用到 extract_rotated_roi_patch_impl
    if (!extract_rotated_roi_patch(src_img,
                                   precheck_roi.roi_points,
                                   target_roi_width,
                                   target_roi_height,
                                   &roi_img))
    {
        // ===== 调试代码开始：模型输入 ROI 提取失败时清空 ROI 调试缓存 =====
        lq_ncnn_reset_debug_roi_patch();
        // ===== 调试代码结束：模型输入 ROI 提取失败时清空 ROI 调试缓存 =====
        reset_infer_result(infer_result);
        lq_ncnn_reset_guarded_state();
        LQ_NCNN_LOG_ERR("extract_rotated_roi_patch_failed");
        return -1;
    }
#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：缓存真正送入模型的 ROI 图，供上位机或调试代码读取 =====
    lq_ncnn_save_debug_roi_patch(roi_img);
    // ===== 调试代码结束：缓存真正送入模型的 ROI 图，供上位机或调试代码读取 =====
#endif

    //模型推理
    if (lq_ncnn_infer(roi_img, infer_result) != 0)
    {
        reset_infer_result(infer_result);
        lq_ncnn_reset_guarded_state();
        LQ_NCNN_LOG_ERR("lq_ncnn_infer_failed");
        return -1;
    }

    //infer_result->predict_index 为当前预测的索引
    const int predict_idx = infer_result->predict_index;
    //predict_idx<0和大于类别数量说明没预测到或者有问题
    if (predict_idx < 0 || predict_idx >= infer_result->class_count)
    {
        lq_ncnn_reset_guarded_state();
        LQ_NCNN_LOG_ERR("invalid_predict_idx=%d class_count=%d", predict_idx, infer_result->class_count);
        return -1;
    }
    //top1_prob得到预测分数，分数至少大于0.4才能输出
    const float top1_prob = infer_result->class_scores[predict_idx];
    //分数低预测失败
    if (top1_prob <= kMinOutputClassProb)
    {
        lq_ncnn_reset_guarded_state();
        LQ_NCNN_LOG_ERR("score_too_low: idx=%d prob=%.4f", predict_idx, top1_prob);
        return -1;
    }

    //做“连续同类计数”，如果和上一帧预测类别一样就计数+1，否则重置为1
    if (infer_result->predict_index == g_guarded_last_predict_index)
    {
        g_guarded_same_predict_count++;
    }
    else
    {
        g_guarded_last_predict_index = infer_result->predict_index;
        g_guarded_same_predict_count = 1;//给1是本帧为第一帧
    }

    //至少连续 2 帧相同才返回成功
    if (g_guarded_same_predict_count < kGuardedStableFrameCount)
    {
        // LQ_NCNN_LOG_ERR("guarded_log_unstable");
        return -1;
    }

    // 记录本次成功输出并锁存：后续只有红块消失才允许再次输出。
    g_guarded_last_output_predict_index = infer_result->predict_index;
    g_guarded_output_latched = true;

    return 0;//返回0说明推理成功
}

/**
 * @brief 放主函数里面调用
 * @note 只做模型的识别，不做绕行，用于测试模型的识别能力
 */
void model_use(const cv::Mat& src_img)
{
    LqNcnnInferResult infer_result = {};
    int ret = lq_ncnn_detect_and_infer_guarded(src_img,-1, &infer_result);
    if(ret == 0)
    {
        const int cls = infer_result.predict_index;
        const char* cls_name = (cls >= 0 && cls < kClassLabelCount) ? kClassLabels[cls] : "unknown";
        std::printf("infer ok: cls=%s(%d) score=%.4f cost=%lldus\r\n",
                    cls_name,
                    cls,
                    infer_result.predict_score,
                    static_cast<long long>(infer_result.invoke_time_us));
    }
}

/**
 * @brief 放在主函数里面，添加绕行逻辑
 */
void model_use_bypass(const cv::Mat& src_img)
{
    //直行状态的时候，正常搜线和保护，然后编码器积分一段距离之后在打开模型识别
    if (is_model_infer_complete &&
        model_control_access == kModelControlStraight &&
        model_encoder_integral > model_straight_reopen_integral)
    {
        //打开模型识别，清零模型状态
        const float reopen_integral = model_encoder_integral;
        model_red_found = false;
        is_model_infer_complete = false;
        model_control_access = kModelControlNone;
        model_encoder_clear_increment();
        lq_ncnn_reset_guarded_state();
        g_guarded_last_output_predict_index = -1;
        g_guarded_output_latched = false;

        //如果编码器积分到直行结束状态时，输出此时编码器走的距离
        std::printf("直行状态结束: encoder_integral=%.1f threshold=%d\r\n",
                    reopen_integral,
                    model_straight_reopen_integral);
    }

    LqNcnnInferResult infer_result = {};
    if(!is_model_infer_complete)
    {
        model_control_access = kModelControlNone;//每次识别前清零状态
        int ret = lq_ncnn_detect_and_infer_guarded(src_img,-1, &infer_result);
        if(ret == 0)
        {
            // 控制开关：稳定识别后锁定一次，不再频繁改方向。
            model_control_access = map_predict_to_bypass_mode(infer_result.predict_index);
            //模型推理完成
            is_model_infer_complete = (model_control_access != kModelControlNone);
            //直行绕行状态前先把编码器清零
            if (is_model_infer_complete)
            {
                model_encoder_clear_increment();
            }
            const int cls = infer_result.predict_index;
            const char* cls_name = (cls >= 0 && cls < kClassLabelCount) ? kClassLabels[cls] : "unknown";
            std::printf("infer ok: cls=%s(%d) score=%.4f cost=%lldus\r\n",
                        cls_name,
                        cls,
                        infer_result.predict_score,
                        static_cast<long long>(infer_result.invoke_time_us));
        }
    }
}


/**
 * @brief debug_roi 得到调试路径
 */
bool lq_ncnn_get_debug_roi_patch(cv::Mat* roi_img)
{
#if !ENABLE_VISION_DEBUG
    if (roi_img != nullptr)
    {
        roi_img->release();
    }
    return false;
#else
    // ===== 调试代码开始：导出“真正送入模型的 ROI 调试图” =====
    if (roi_img == nullptr)
    {
        return false;
    }
    if (g_debug_roi_patch.empty())
    {
        roi_img->release();
        return false;
    }
    *roi_img = g_debug_roi_patch.clone();
    // ===== 调试代码结束：导出“真正送入模型的 ROI 调试图” =====
    return true;
#endif
}

bool lq_ncnn_get_debug_raw_roi_patch(cv::Mat* roi_img)
{
#if !ENABLE_VISION_DEBUG
    if (roi_img != nullptr)
    {
        roi_img->release();
    }
    return false;
#else
    // ===== 调试代码开始：导出“按真实 ROI 宽高拉正后的原始 ROI 调试图” =====
    if (roi_img == nullptr)
    {
        return false;
    }
    if (g_debug_raw_roi_patch.empty())
    {
        roi_img->release();
        return false;
    }
    *roi_img = g_debug_raw_roi_patch.clone();
    // ===== 调试代码结束：导出“按真实 ROI 宽高拉正后的原始 ROI 调试图” =====
    return true;
#endif
}
