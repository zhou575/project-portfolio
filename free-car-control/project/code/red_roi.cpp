#include "red_roi.hpp"
#include "debug_switch.hpp"
#include "car_init.hpp"
#include "image.hpp"
#include "menu.hpp"

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <sys/stat.h>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>

//限制只在本
namespace
{
// red_roi 模块统一日志宏。
// RED_ROI_ERR：
//   打“明确错误/防御失败”日志，比如输入非法、几何退化、文件打开失败。
#define RED_ROI_ERR(fmt, ...) \
    printf("ERROR_INFO:%s:%s: " fmt "\r\n", __FILE__, __func__, ##__VA_ARGS__)
// 红色检测与ROI截取参数：
// 采集图 320x240，搜线图 160x120（2x2下采样）
constexpr int kDefaultRoiWidth = 96;//ROI宽度（仅调试兜底）
constexpr int kModelRoiWidth = 48;//当前正式模型输入ROI宽
constexpr int kModelRoiHeight = 48;//当前正式模型输入ROI高
constexpr float kRoiHeightScaleByLongSide = 0.875f;// ROI高 = 长边 * 0.875
constexpr int kMinRedAreaPx = 5;//最小的红色区域面积
constexpr float kMinContourPerimeterPx = 5.0f;//周长
constexpr float kMinRectFillRatio = 0.3f;// 非线状目标填充率阈值          
//BGR初筛到HSV精修的时候扩大一下面积
constexpr int kCoarseRoiExpandPx = 2;
// 仅在红色最小包围框内做开运算，降低全图形态学开销
// constexpr int kOpenKernelSize = 3;
// constexpr int kOpenRoiExpandPx = 1;
//最优矩形打分规则
constexpr float kScoreWeightCenterY = 0.1f;
constexpr float kScoreWeightBottomY = 0.4f;
constexpr float kScoreWeightArea = 0.4f;
constexpr float kScoreWeightFill = 0.1f;
constexpr float kStep10RoiSafetyOffsetPx = 3.0f;//第十步ROI底边沿图片方向安全上移距离
constexpr float kStep10RoiHorizontalExpandPx = 2.0f;//第十步ROI底边沿左右方向各外扩距离
constexpr int kRoiPredictStartCenterRow = 130;      // 红块中心行达到该阈值后，才开始执行 ROI 预测,实在不行给115
// 色块识别窗口上边界目标行：
// 最长白列覆盖到 row=40 时，识别上边界固定到 40。
constexpr int kDetectTopRowLimit = 80;//改这个可以改红块识别的距离，红块不用特别远识别，有个减速距离的就行了，到时候调一下
constexpr int kDetectBottomRowLimit = UVC_HEIGHT - 1;
constexpr int kDetectTopReachEdgeNums = UVC_HEIGHT - kDetectTopRowLimit;

//（BGR 粗筛）参数：
// 满足以下条件才进入 HSV 精筛。
constexpr int kBgrRMin = 90;//70;
constexpr int kBgrRGDiffMin = 40;//25;
constexpr int kBgrRBDiffMin = 35;//25;
// HSV 红色阈值（双区间）
constexpr int kRedH1Min = 0;
constexpr int kRedH1Max = 8;//12;
constexpr int kRedH2Min = 170;//165;
constexpr int kRedH2Max = 179;
constexpr int kRedSMin = 120;//80;
constexpr int kRedSMax = 255;
constexpr int kRedVMin = 60;
constexpr int kRedVMax = 255;

//这个重新测
constexpr float kMaxRedLongSidePx = 100.0f;         // 红块长边上限（像素）
constexpr float kMaxRedShortSidePx = 50.0f;         // 红块短边上限（像素）
constexpr float kMaxRedLongSideFarPx = 70.0f;       // 远处（上方）红块长边上限（像素）
constexpr float kMaxRedShortSideFarPx = 25.0f;      // 远处（上方）红块短边上限（像素）
cv::Mat g_red_mask;
cv::Mat g_red_bgr_coarse_mask;
cv::Mat g_red_hsv_img;
cv::Mat g_red_full_mask;//正式链路使用的整图坐标系红色掩码

#if ENABLE_VISION_DEBUG
// ===== 调试代码开始：red_roi 在线调试缓存 =====
// 这一组变量不参与 ROI 正常预测，只服务于 TCP 看图、离线采图和定位问题。
// 后续如果切到“正式运行模式”，这部分就是最优先关闭的额外开销。
cv::Mat g_red_debug_mask_full;//整图坐标系下的红色掩码，供后续裁调试 mask patch 使用
cv::Mat g_red_debug_min_rect_patch;//第十步原图局部调试叠加图，给 TCP 左图显示
cv::Mat g_red_debug_min_rect_mask_patch;//第十步最终透视后的 48x48 ROI 图，给 TCP 右图显示
bool g_red_debug_rect_valid = false;//当前帧是否得到了可用的最优红块旋转矩形
cv::Point2f g_red_debug_rect_points[4];//最优红块旋转矩形四个角点，便于后续恢复画框/打日志
bool g_red_debug_roi_valid = false;//当前帧是否得到了最终模型 ROI 四边形
cv::Point2f g_red_debug_roi_points[4];//最终模型 ROI 四角点（tl,tr,bl,br）
bool g_red_debug_selected_edge_valid = false;//当前帧是否得到了 build_rotated_roi_above_red 选中的最靠下长边
cv::Point2f g_red_debug_selected_edge_points[2];//被选中的最靠下长边两个端点
bool g_red_debug_bottom_axis_valid = false;//当前帧是否得到了最终 roi_bottom_axis 参考线
cv::Point2f g_red_debug_bottom_axis_points[2];//最终 roi_bottom_axis 参考线两个端点
// ===== 调试代码结束：red_roi 在线调试缓存 =====
#endif

bool normalize_vec2(const cv::Point2f& v, cv::Point2f* out);
bool extract_rotated_roi_patch_impl(const cv::Mat& src_bgr,
                                    const cv::Point2f* roi_points,
                                    int roi_width,
                                    int roi_height,
                                    cv::Mat* roi_img);
bool extract_perspective_roi_patch_impl(const cv::Mat& src_bgr,
                                        const cv::Point2f* roi_points,
                                        int roi_width,
                                        int roi_height,
                                        cv::Mat* roi_img);
bool are_points_inside_image(const cv::Point2f* points, int image_w, int image_h);

/**
 * @brief 把整数限制在给定区间内
 * @param value 待限制的整数
 * @param low   下界
 * @param high  上界
 * @return int  限幅后的结果
 * @note  如果调用方给出了非法区间（high < low），则退化返回 low。
 */
inline int clamp_int(int value, int low, int high)
{
    if (high < low)
    {
        return low;
    }
    return (value < low) ? low : ((value > high) ? high : value);
}

/**
 * @brief 把浮点数限制在给定区间内
 * @param value 待限制的浮点数
 * @param low   下界
 * @param high  上界
 * @return float 限幅后的结果
 * @note  主要用于比例、长度、得分等连续量的防御式保护。
 */
inline float clamp_float(float value, float low, float high)
{
    if (high < low)
    {
        return low;
    }
    return (value < low) ? low : ((value > high) ? high : value);
}

/**
 * @brief 将 160x120 搜线图中的左边线 x 坐标映射回 320x240 原图坐标
 * @param x_line 搜线图中的列坐标
 * @return int   原图中的左侧像素列
 * @note  由于采用 2x2 下采样，映射后取的是该小图像素对应块的左边界。
 */
inline int line_x_to_raw_left(int x_line)
{
    return clamp_int(x_line * IMG_PROC_DOWNSAMPLE, 0, UVC_WIDTH - 1);
}

/**
 * @brief 将 160x120 搜线图中的右边线 x 坐标映射回 320x240 原图坐标
 * @param x_line 搜线图中的列坐标
 * @return int   原图中的右侧像素列
 * @note  由于采用 2x2 下采样，映射后取的是该小图像素对应块的右边界。
 */
inline int line_x_to_raw_right(int x_line)
{
    return clamp_int(x_line * IMG_PROC_DOWNSAMPLE + (IMG_PROC_DOWNSAMPLE - 1), 0, UVC_WIDTH - 1);
}

/**
 * @brief 将原图行号映射成搜线结果使用的 edge 索引
 * @param raw_row 原图中的行号（0 在顶部）
 * @return int    对应的 edge 索引
 * @note  当前搜线缓存按“从底往上”的顺序存储，所以需要做一次倒序映射。
 */
inline int raw_row_to_line_edge_idx(int raw_row)
{
    return (UVC_HEIGHT - 1 - raw_row) / IMG_PROC_DOWNSAMPLE;
}

/**
 * @brief 计算二维向量长度
 * @param v 输入向量
 * @return float 向量长度
 * @note  用于计算红块倾斜方向长度，辅助归一化。
 */
float vec2_length(const cv::Point2f& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

struct LocalRowRunInfo
{
    bool valid = false;
    int x0 = 0;
    int x1 = -1;
    int longest_width = 0;
    int foreground_count = 0;
    float center_x = 0.0f;
    float fill_ratio = 0.0f;
};

struct LocalBandInfo
{
    bool valid = false;
    int top_row = -1;
    int bottom_row = -1;
    float mean_width = 0.0f;
    float mean_center_x = 0.0f;
};

/**
 * @brief 红块主体行组分析结果
 * @note  第五步使用：
 *        先在局部图下部估计 Wref / Cref，再从底部往上找最靠下的稳定主体行组。
 */
struct MarkerBodyRowsResult
{
    bool valid = false;
    float w_ref = 0.0f;
    float c_ref = 0.0f;
    int body_top_row = -1;
    int body_bottom_row = -1;
    int stable_group_row_count = 0;
    std::vector<uint8_t> row_flags;
};

/**
 * @brief 红块主体底边拟合结果
 * @note  第六步使用：
 *        只在主体行组里按列找最底红点，再拟合出主体下包络线方向。
 */
struct MarkerBottomEdgeFitResult
{
    bool valid = false;
    std::vector<cv::Point2f> sample_points;
    cv::Point2f line_point = cv::Point2f(0.0f, 0.0f);
    cv::Point2f line_dir = cv::Point2f(0.0f, 0.0f);
};

/**
 * @brief 红块主体左边界拟合结果
 * @note  第七步使用：
 *        只在主体中下部按行取最长连续红段左端点，再拟合真实左边方向。
 */
struct MarkerLeftEdgeFitResult
{
    bool valid = false;
    std::vector<cv::Point2f> sample_points;
    cv::Point2f line_point = cv::Point2f(0.0f, 0.0f);
    cv::Point2f line_dir = cv::Point2f(0.0f, 0.0f);
};

/**
 * @brief 红块主体右边界拟合结果
 * @note  第七步新增：
 *        只在主体中下部按行取最长连续红段右端点，再拟合真实右边方向。
 */
struct MarkerRightEdgeFitResult
{
    bool valid = false;
    std::vector<cv::Point2f> sample_points;
    cv::Point2f line_point = cv::Point2f(0.0f, 0.0f);
    cv::Point2f line_dir = cv::Point2f(0.0f, 0.0f);
};

/**
 * @brief 基于主体几何重建的红块平行四边形
 * @note  第九步使用：
 *        不再依赖 minAreaRect 四点，而是用 red_bl、底边方向、左边方向、主体宽高重建。
 */
struct MarkerParallelogramResult
{
    bool valid = false;
    cv::Point2f points[4] = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f)};
    float width_px = 0.0f;
    float height_px = 0.0f;
};

/**
 * @brief 局部 patch 与原图之间的仿射几何描述
 * @note  第四步把原图红块区域拉正成局部 patch 之后，
 *        第十步需要依赖它把第九步局部主体几何再映射回原图坐标。
 */
struct LocalPatchTransform
{
    bool valid = false;
    cv::Point2f quad[4] = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(0.0f, 0.0f)}; // raw tl,tr,bl,br
    int patch_width = 0;
    int patch_height = 0;
};

/**
 * @brief 第十步白色阈值估计结果
 * @note  阈值按“原图最下面5行、且位于赛道左右边界之间”的像素逐帧统计得到，
 *        最终亮度下限取平均亮度的 0.7 倍。
 */
struct Step10WhiteThreshold
{
    bool valid = false;
    float mean_b = 0.0f;
    float mean_g = 0.0f;
    float mean_r = 0.0f;
    float mean_gray = 0.0f;
    float white_floor = 0.0f;
    float color_delta_max = 45.0f;
    int sample_count = 0;
};

/**
 * @brief 线性插值
 * @param a 起点值
 * @param b 终点值
 * @param t 插值系数，通常在 [0,1] 范围内
 * @return float 插值结果
 * @note  当前主要用于根据红块 y 位置平滑调整长短边阈值。
 */
float lerp_float(float a, float b, float t)
{
    return a + (b - a) * t;
}

/**
 * @brief 计算一组浮点数的中位数
 * @note  用于估计 Wref / Cref，避免均值被少量异常宽度或中心点拖偏。
 */
float compute_median_float(std::vector<float> values)
{
    if (values.empty())
    {
        return 0.0f;
    }
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    const float upper = values[mid];
    if ((values.size() & 1U) != 0U)
    {
        return upper;
    }
    std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
    return 0.5f * (values[mid - 1] + upper);
}

bool analyze_longest_foreground_run_in_row(const cv::Mat& binary_mask,
                                           int row,
                                           LocalRowRunInfo* row_info)
{
    if (row_info == nullptr || binary_mask.empty() || binary_mask.type() != CV_8UC1 ||
        row < 0 || row >= binary_mask.rows)
    {
        return false;
    }

    *row_info = LocalRowRunInfo();
    const uint8_t* row_ptr = binary_mask.ptr<uint8_t>(row);
    int best_start = -1;
    int best_end = -1;
    int cur_start = -1;
    int foreground_count = 0;
    for (int x = 0; x < binary_mask.cols; ++x)
    {
        if (row_ptr[x] > 0)
        {
            ++foreground_count;
            if (cur_start < 0)
            {
                cur_start = x;
            }
        }
        else if (cur_start >= 0)
        {
            const int cur_end = x - 1;
            if ((cur_end - cur_start) > (best_end - best_start))
            {
                best_start = cur_start;
                best_end = cur_end;
            }
            cur_start = -1;
        }
    }
    if (cur_start >= 0)
    {
        const int cur_end = binary_mask.cols - 1;
        if ((cur_end - cur_start) > (best_end - best_start))
        {
            best_start = cur_start;
            best_end = cur_end;
        }
    }

    row_info->foreground_count = foreground_count;
    if (best_start < 0 || best_end < best_start)
    {
        return false;
    }

    row_info->valid = true;
    row_info->x0 = best_start;
    row_info->x1 = best_end;
    row_info->longest_width = best_end - best_start + 1;
    row_info->center_x = 0.5f * static_cast<float>(best_start + best_end);
    row_info->fill_ratio = static_cast<float>(row_info->longest_width) /
                           static_cast<float>(std::max(1, foreground_count));
    return true;
}

/**
 * @brief 在局部 patch 中寻找“最下方稳定红色宽带”
 * @param row_infos   每一行最长连续红段统计结果
 * @param patch_width 局部 patch 宽度
 * @param band_info   输出稳定宽带信息
 * @return bool       true=找到稳定宽带 false=未找到
 * @note  这是前四步中的第 4 步：
 *        从局部图底部往上，寻找一段“宽度足够大、中心稳定、填充率高”的连续红带。
 */
bool find_bottom_stable_red_band_in_local_patch(const std::vector<LocalRowRunInfo>& row_infos,
                                                int patch_width,
                                                LocalBandInfo* band_info)
{
    if (band_info == nullptr || row_infos.empty() || patch_width <= 0)
    {
        return false;
    }

    *band_info = LocalBandInfo();
    constexpr float kStableMinWidthRatio = 0.45f;
    constexpr float kStableMinFillRatio = 0.75f;
    constexpr int kStableMinRows = 4;
    constexpr int kWidthTolPx = 5;
    constexpr int kCenterTolPx = 6;
    constexpr float kWidthTolRatio = 0.18f;
    constexpr float kCenterTolRatio = 0.12f;

    const int min_width_px = std::max(4, static_cast<int>(std::lround(static_cast<float>(patch_width) * kStableMinWidthRatio)));

    for (int start_row = static_cast<int>(row_infos.size()) - 1; start_row >= 0; --start_row)
    {
        const LocalRowRunInfo& seed = row_infos[start_row];
        if (!seed.valid || seed.longest_width < min_width_px || seed.fill_ratio < kStableMinFillRatio)
        {
            continue;
        }

        int top_row = start_row;
        int row_count = 1;
        float mean_width = static_cast<float>(seed.longest_width);
        float mean_center_x = seed.center_x;

        for (int row = start_row - 1; row >= 0; --row)
        {
            const LocalRowRunInfo& cur = row_infos[row];
            if (!cur.valid || cur.longest_width < min_width_px || cur.fill_ratio < kStableMinFillRatio)
            {
                break;
            }

            const float width_tol = std::max(static_cast<float>(kWidthTolPx), mean_width * kWidthTolRatio);
            const float center_tol = std::max(static_cast<float>(kCenterTolPx), mean_width * kCenterTolRatio);
            if (std::abs(static_cast<float>(cur.longest_width) - mean_width) > width_tol ||
                std::abs(cur.center_x - mean_center_x) > center_tol)
            {
                break;
            }

            top_row = row;
            mean_width = (mean_width * static_cast<float>(row_count) + static_cast<float>(cur.longest_width)) /
                         static_cast<float>(row_count + 1);
            mean_center_x = (mean_center_x * static_cast<float>(row_count) + cur.center_x) /
                            static_cast<float>(row_count + 1);
            ++row_count;
        }

        if (row_count >= kStableMinRows)
        {
            band_info->valid = true;
            band_info->top_row = top_row;
            band_info->bottom_row = start_row;
            band_info->mean_width = mean_width;
            band_info->mean_center_x = mean_center_x;
            return true;
        }
    }
    return false;
}

/**
 * @brief 在局部 patch 中判断哪些行属于“红块主体”
 * @param row_infos         每一行最长连续红段统计结果
 * @param patch_width       局部 patch 宽
 * @param patch_height      局部 patch 高
 * @param stable_band       前四步得到的最下方稳定红色宽带
 * @param result            输出主体行组分析结果
 * @return bool             true=找到可信主体行组 false=未找到
 * @note  第五步逻辑：
 *        1) 用局部下部 30% 行估计 Wref
 *        2) 用宽行中心估计 Cref
 *        3) 判断每一行是否满足主体候选条件
 *        4) 从最下面往上找最靠下的稳定行组，允许中间断 1 行
 */
bool estimate_marker_body_rows_from_local_patch(const std::vector<LocalRowRunInfo>& row_infos,
                                                int patch_width,
                                                int patch_height,
                                                const LocalBandInfo& stable_band,
                                                MarkerBodyRowsResult* result)
{
    if (result == nullptr || row_infos.empty() || patch_width <= 0 || patch_height <= 0)
    {
        return false;
    }

    *result = MarkerBodyRowsResult();
    result->row_flags.assign(row_infos.size(), 0);

    // 只拿局部图下部 30% 行去估计主体参考宽度：
    // - 下部最接近小车，最像“红块主体下带”
    // - 上部更容易被粘连、文字、阴影干扰
    const int lower_start_row = clamp_int(static_cast<int>(std::floor(static_cast<float>(patch_height) * 0.7f)),
                                          0,
                                          patch_height - 1);
    std::vector<float> lower_widths;
    lower_widths.reserve(static_cast<size_t>(patch_height - lower_start_row));
    for (int row = lower_start_row; row < patch_height; ++row)
    {
        const LocalRowRunInfo& info = row_infos[row];
        if (!info.valid || info.fill_ratio < 0.65f)
        {
            continue;
        }
        lower_widths.push_back(static_cast<float>(info.longest_width));
    }
    // 如果下部 30% 行一个可信宽度都拿不到，就退回到前四步找到的稳定宽带，
    // 至少保证 Wref 不会完全失去参考。
    if (lower_widths.empty() && stable_band.valid)
    {
        for (int row = stable_band.top_row; row <= stable_band.bottom_row; ++row)
        {
            const LocalRowRunInfo& info = row_infos[row];
            if (info.valid)
            {
                lower_widths.push_back(static_cast<float>(info.longest_width));
            }
        }
    }
    if (lower_widths.empty())
    {
        return false;
    }

    // 用中位数而不是均值，避免少量异常超宽/超窄行把主体参考宽度拖偏。
    result->w_ref = compute_median_float(lower_widths);
    if (result->w_ref <= 1e-3f)
    {
        return false;
    }

    // 只用“足够宽”的那些行来估计主体中心，避免细碎行把中心位置拉偏。
    std::vector<float> center_candidates;
    center_candidates.reserve(lower_widths.size());
    for (int row = lower_start_row; row < patch_height; ++row)
    {
        const LocalRowRunInfo& info = row_infos[row];
        if (!info.valid || info.fill_ratio < 0.65f)
        {
            continue;
        }
        if (static_cast<float>(info.longest_width) >= result->w_ref * 0.8f)
        {
            center_candidates.push_back(info.center_x);
        }
    }
    if (center_candidates.empty() && stable_band.valid)
    {
        for (int row = stable_band.top_row; row <= stable_band.bottom_row; ++row)
        {
            const LocalRowRunInfo& info = row_infos[row];
            if (info.valid)
            {
                center_candidates.push_back(info.center_x);
            }
        }
    }
    if (center_candidates.empty())
    {
        return false;
    }

    // Cref 表示主体宽带在局部图中的横向中心参考位置。
    result->c_ref = compute_median_float(center_candidates);
    const float min_width = std::max(4.0f, result->w_ref * 0.72f);
    const float max_center_delta = std::max(4.0f, result->w_ref * 0.16f);
    const float min_fill_ratio = 0.70f;

    std::vector<uint8_t> marker_row_flags(row_infos.size(), 0);
    for (int row = 0; row < static_cast<int>(row_infos.size()); ++row)
    {
        const LocalRowRunInfo& info = row_infos[row];
        if (!info.valid)
        {
            continue;
        }
        if (static_cast<float>(info.longest_width) < min_width)
        {
            continue;
        }
        if (std::abs(info.center_x - result->c_ref) > max_center_delta)
        {
            continue;
        }
        if (info.fill_ratio < min_fill_ratio)
        {
            continue;
        }
        // 这一行通过了“宽度、中心、填充率”三重约束，先记为主体候选行。
        marker_row_flags[row] = 1;
    }

    constexpr int kMinStableRows = 5;
    constexpr int kMaxGapRows = 1;
    int best_top = -1;
    int best_bottom = -1;
    int best_count = 0;
    for (int start_row = patch_height - 1; start_row >= 0; --start_row)
    {
        if (marker_row_flags[start_row] == 0)
        {
            continue;
        }
        int top_row = start_row;
        int good_count = 1;
        int gap_count = 0;
        // 从最靠近小车的候选行开始，向上寻找“最靠下的稳定主体行组”。
        // 这里允许中间断 1 行，避免掩码小孔洞把主体硬切断。
        for (int row = start_row - 1; row >= 0; --row)
        {
            if (marker_row_flags[row] != 0U)
            {
                // 注意：这里 top_row 记录的是“稳定组的上边界包围范围”，
                // 允许它包含一次断行。真正的主体有效行仍以后面的 row_flags 为准。
                top_row = row;
                ++good_count;
                gap_count = 0;
                continue;
            }
            if (gap_count < kMaxGapRows)
            {
                ++gap_count;
                // 允许单行空洞时，也把包围范围继续往上延一行，
                // 这样后续可以保留“主体组整体上下边界”的概念。
                top_row = row;
                continue;
            }
            break;
        }
        if (good_count >= kMinStableRows)
        {
            best_top = top_row;
            best_bottom = start_row;
            best_count = good_count;
            break;
        }
    }
    if (best_top < 0 || best_bottom < best_top)
    {
        return false;
    }

    result->valid = true;
    result->body_top_row = best_top;
    result->body_bottom_row = best_bottom;
    result->stable_group_row_count = best_count;
    // row_flags 只给真正通过主体判定的行打标记；
    // best_top/best_bottom 则表示整段主体组的上下包围范围，两者含义不同。
    for (int row = best_top; row <= best_bottom; ++row)
    {
        if (marker_row_flags[row] != 0U)
        {
            result->row_flags[row] = 1;
        }
    }
    return true;
}

/**
 * @brief 用一组二维点拟合直线
 * @param points      输入点集
 * @param line_point  输出直线上的一点
 * @param line_dir    输出单位方向向量
 * @return bool       true=拟合成功 false=点数不足或方向异常
 * @note  第六、七步共用，内部调用 OpenCV 的 cv::fitLine。
 */
bool fit_line_from_points(const std::vector<cv::Point2f>& points,
                          cv::Point2f* line_point,
                          cv::Point2f* line_dir)
{
    if (line_point == nullptr || line_dir == nullptr || points.size() < 2)
    {
        return false;
    }

    cv::Vec4f fit_line;
    cv::fitLine(points, fit_line, cv::DIST_HUBER, 0.0, 0.01, 0.01);
    const cv::Point2f dir(fit_line[0], fit_line[1]);
    if (!normalize_vec2(dir, line_dir))
    {
        return false;
    }
    *line_point = cv::Point2f(fit_line[2], fit_line[3]);
    return true;
}

/**
 * @brief 从主体行组中恢复红块主体的底边
 * @param local_mask_patch  局部二值掩码图
 * @param body_rows         第五步得到的主体行组结果
 * @param result            输出底边拟合结果
 * @return bool             true=拟合成功 false=点不够或拟合失败
 * @note  第六步逻辑：
 *        只在主体行组内，按列找最靠近小车的红点，形成底部包络点后再拟合直线。
 */
bool fit_marker_bottom_edge_from_body_rows(const cv::Mat& local_mask_patch,
                                           const MarkerBodyRowsResult& body_rows,
                                           MarkerBottomEdgeFitResult* result)
{
    if (result == nullptr || local_mask_patch.empty() || local_mask_patch.type() != CV_8UC1 ||
        !body_rows.valid)
    {
        return false;
    }

    *result = MarkerBottomEdgeFitResult();
    const int cols = local_mask_patch.cols;
    const int rows = local_mask_patch.rows;
    // 这里仍使用主体组的上下包围范围来限定扫描区间，
    // 但真正“可信”的主体行应该进一步结合 row_flags 使用。
    const int body_top = clamp_int(body_rows.body_top_row, 0, rows - 1);
    const int body_bottom = clamp_int(body_rows.body_bottom_row, 0, rows - 1);
    const int trim_cols = std::max(1, static_cast<int>(std::floor(static_cast<float>(cols) * 0.08f)));
    for (int col = trim_cols; col < cols - trim_cols; ++col)
    {
        int bottom_row = -1;
        int foreground_count = 0;
        for (int row = body_bottom; row >= body_top; --row)
        {
            // 只要进入主体包围范围，就统计该列总共有多少前景点。
            // 后面的 foreground_count 主要是过滤“只有孤立单点”的列。
            if (local_mask_patch.at<uint8_t>(row, col) > 0)
            {
                ++foreground_count;
                if (bottom_row < 0)
                {
                    // 从主体底部往上第一次命中的白点，才是这一列真正的“最底红点”。
                    bottom_row = row;
                }
            }
        }
        // 没找到底点，或者这一列只有非常稀疏的零星前景，都不拿来参与底边拟合。
        if (bottom_row < 0 || foreground_count < 2)
        {
            continue;
        }
        result->sample_points.emplace_back(static_cast<float>(col), static_cast<float>(bottom_row));
    }
    if (result->sample_points.size() < 6)
    {
        result->sample_points.clear();
        return false;
    }

    std::vector<cv::Point2f> filtered_points;
    filtered_points.reserve(result->sample_points.size());
    // 先用相邻列 y 跳变做一轮轻量过滤。
    // 这里是最小实现，目的只是先去掉明显突刺列，后面仍可继续加强为更稳的鲁棒过滤。
    filtered_points.push_back(result->sample_points.front());
    for (size_t i = 1; i < result->sample_points.size(); ++i)
    {
        const float dy = std::abs(result->sample_points[i].y - filtered_points.back().y);
        if (dy <= 6.0f)
        {
            filtered_points.push_back(result->sample_points[i]);
        }
    }
    if (filtered_points.size() < 4)
    {
        return false;
    }
    result->sample_points = filtered_points;
    if (!fit_line_from_points(result->sample_points, &result->line_point, &result->line_dir))
    {
        return false;
    }
    // 统一底边方向：优先让它朝局部图像右侧，便于第 8、9 步语义稳定。
    if (result->line_dir.x < 0.0f)
    {
        result->line_dir = -result->line_dir;
    }
    result->valid = true;
    return true;
}

/**
 * @brief 从主体中下部恢复红块主体的左边界
 * @param row_infos   每一行最长连续红段统计结果
 * @param body_rows   第五步得到的主体行组结果
 * @param result      输出左边界拟合结果
 * @return bool       true=拟合成功 false=点不够或拟合失败
 * @note  第七步逻辑：
 *        只在主体中下部，按行取最长连续红段左端点，再拟合真实左边方向。
 */
bool fit_marker_left_edge_from_body_rows(const std::vector<LocalRowRunInfo>& row_infos,
                                         const MarkerBodyRowsResult& body_rows,
                                         MarkerLeftEdgeFitResult* result)
{
    if (result == nullptr || row_infos.empty() || !body_rows.valid)
    {
        return false;
    }

    *result = MarkerLeftEdgeFitResult();
    const int body_height = body_rows.body_bottom_row - body_rows.body_top_row + 1;
    if (body_height <= 0)
    {
        return false;
    }
    // 左边界只用主体中下部来拟合：
    // - 顶部最容易被粘连、白字、阴影影响
    // - 中下部更接近真实左边界方向
    const int fit_top = clamp_int(body_rows.body_top_row + body_height / 3,
                                  body_rows.body_top_row,
                                  body_rows.body_bottom_row);
    for (int row = fit_top; row <= body_rows.body_bottom_row; ++row)
    {
        if (row < 0 || row >= static_cast<int>(row_infos.size()) || body_rows.row_flags[row] == 0U)
        {
            continue;
        }
        const LocalRowRunInfo& info = row_infos[row];
        if (!info.valid)
        {
            continue;
        }
        // 每一行只取“最长连续红段”的左端点，而不是所有红点最左值，
        // 这样更抗局部噪声和细连接。
        result->sample_points.emplace_back(static_cast<float>(info.x0), static_cast<float>(row));
    }
    if (result->sample_points.size() < 4)
    {
        result->sample_points.clear();
        return false;
    }

    std::vector<cv::Point2f> filtered_points;
    filtered_points.reserve(result->sample_points.size());
    // 用相邻行左端点的横向跳变做一次轻量过滤，先去掉明显离群行。
    filtered_points.push_back(result->sample_points.front());
    for (size_t i = 1; i < result->sample_points.size(); ++i)
    {
        const float dx = std::abs(result->sample_points[i].x - filtered_points.back().x);
        if (dx <= std::max(5.0f, body_rows.w_ref * 0.18f))
        {
            filtered_points.push_back(result->sample_points[i]);
        }
    }
    if (filtered_points.size() < 3)
    {
        return false;
    }
    result->sample_points = filtered_points;
    if (!fit_line_from_points(result->sample_points, &result->line_point, &result->line_dir))
    {
        return false;
    }
    // 统一左边界方向：这里先让方向尽量朝局部图像下方，
    // 后面第九步会再根据“要朝红块上方生长”的语义做一次翻转。
    if (result->line_dir.y < 0.0f)
    {
        result->line_dir = -result->line_dir;
    }
    result->valid = true;
    return true;
}

/**
 * @brief 从主体中下部恢复红块主体的右边界
 * @param row_infos   每一行最长连续红段统计结果
 * @param body_rows   第五步得到的主体行组结果
 * @param result      输出右边界拟合结果
 * @return bool       true=拟合成功 false=点不够或拟合失败
 * @note  与左边界拟合严格对称：
 *        使用相同的主体中下部范围、相同的离群点过滤，只是每行取最长连续红段的右端点。
 */
bool fit_marker_right_edge_from_body_rows(const std::vector<LocalRowRunInfo>& row_infos,
                                          const MarkerBodyRowsResult& body_rows,
                                          MarkerRightEdgeFitResult* result)
{
    if (result == nullptr || row_infos.empty() || !body_rows.valid)
    {
        return false;
    }

    *result = MarkerRightEdgeFitResult();
    const int body_height = body_rows.body_bottom_row - body_rows.body_top_row + 1;
    if (body_height <= 0)
    {
        return false;
    }
    const int fit_top = clamp_int(body_rows.body_top_row + body_height / 3,
                                  body_rows.body_top_row,
                                  body_rows.body_bottom_row);
    for (int row = fit_top; row <= body_rows.body_bottom_row; ++row)
    {
        if (row < 0 || row >= static_cast<int>(row_infos.size()) || body_rows.row_flags[row] == 0U)
        {
            continue;
        }
        const LocalRowRunInfo& info = row_infos[row];
        if (!info.valid)
        {
            continue;
        }
        result->sample_points.emplace_back(static_cast<float>(info.x1), static_cast<float>(row));
    }
    if (result->sample_points.size() < 4)
    {
        result->sample_points.clear();
        return false;
    }

    std::vector<cv::Point2f> filtered_points;
    filtered_points.reserve(result->sample_points.size());
    filtered_points.push_back(result->sample_points.front());
    for (size_t i = 1; i < result->sample_points.size(); ++i)
    {
        const float dx = std::abs(result->sample_points[i].x - filtered_points.back().x);
        if (dx <= std::max(5.0f, body_rows.w_ref * 0.18f))
        {
            filtered_points.push_back(result->sample_points[i]);
        }
    }
    if (filtered_points.size() < 3)
    {
        return false;
    }
    result->sample_points = filtered_points;
    if (!fit_line_from_points(result->sample_points, &result->line_point, &result->line_dir))
    {
        return false;
    }
    // 统一右边界方向：同样让它朝“图片方向/局部下方”，
    // 这样和左边界在求交、调试显示时语义一致。
    if (result->line_dir.y < 0.0f)
    {
        result->line_dir = -result->line_dir;
    }
    result->valid = true;
    return true;
}

/**
 * @brief 计算两条二维直线的交点
 * @param p0    第一直线上的一点
 * @param d0    第一直线单位方向
 * @param p1    第二直线上的一点
 * @param d1    第二直线单位方向
 * @param cross 输出交点
 * @return bool true=求交成功 false=近似平行或输入非法
 */
bool compute_line_intersection(const cv::Point2f& p0,
                               const cv::Point2f& d0,
                               const cv::Point2f& p1,
                               const cv::Point2f& d1,
                               cv::Point2f* cross)
{
    if (cross == nullptr)
    {
        return false;
    }
    const float denom = d0.x * d1.y - d0.y * d1.x;
    if (std::abs(denom) <= 1e-4f)
    {
        return false;
    }
    const cv::Point2f delta = p1 - p0;
    const float t = (delta.x * d1.y - delta.y * d1.x) / denom;
    *cross = p0 + d0 * t;
    return true;
}

/**
 * @brief 用底边、左边、右边和主体顶部参考行重建红块主体四边形
 * @param body_rows_result    第五步主体行组结果
 * @param bottom_edge_result  第六步底边拟合结果
 * @param left_edge_result    第七步左边拟合结果
 * @param right_edge_result   第七步右边拟合结果
 * @param result              输出主体四边形结果
 * @return bool               true=构建成功 false=输入无效或几何退化
 * @note  第九步新版逻辑：
 *        直接用底边/左边/右边求出 bl、br，再用主体顶部参考行求出 tl、tr，
 *        不再依赖“左下角 + 底边方向 + 单侧边方向”构造平行四边形。
 */
bool build_marker_quad_from_body_edges(const MarkerBodyRowsResult& body_rows_result,
                                       const MarkerBottomEdgeFitResult& bottom_edge_result,
                                       const MarkerLeftEdgeFitResult& left_edge_result,
                                       const MarkerRightEdgeFitResult& right_edge_result,
                                       MarkerParallelogramResult* result)
{
    if (result == nullptr || !body_rows_result.valid || !bottom_edge_result.valid ||
        !left_edge_result.valid || !right_edge_result.valid)
    {
        return false;
    }

    *result = MarkerParallelogramResult();
    const float top_ref_row = static_cast<float>(body_rows_result.body_top_row);
    const cv::Point2f top_row_point(0.0f, top_ref_row);
    const cv::Point2f top_row_dir(1.0f, 0.0f);

    cv::Point2f bl;
    cv::Point2f br;
    cv::Point2f tl;
    cv::Point2f tr;
    if (!compute_line_intersection(bottom_edge_result.line_point,
                                   bottom_edge_result.line_dir,
                                   left_edge_result.line_point,
                                   left_edge_result.line_dir,
                                   &bl))
    {
        return false;
    }
    if (!compute_line_intersection(bottom_edge_result.line_point,
                                   bottom_edge_result.line_dir,
                                   right_edge_result.line_point,
                                   right_edge_result.line_dir,
                                   &br))
    {
        return false;
    }
    if (!compute_line_intersection(top_row_point,
                                   top_row_dir,
                                   left_edge_result.line_point,
                                   left_edge_result.line_dir,
                                   &tl))
    {
        return false;
    }
    if (!compute_line_intersection(top_row_point,
                                   top_row_dir,
                                   right_edge_result.line_point,
                                   right_edge_result.line_dir,
                                   &tr))
    {
        return false;
    }

    const float width_px = vec2_length(br - bl);
    const float left_height_px = vec2_length(bl - tl);
    const float right_height_px = vec2_length(br - tr);
    const float height_px = 0.5f * (left_height_px + right_height_px);
    if (width_px <= 1.0f || height_px <= 1.0f)
    {
        return false;
    }

    // 保持当前 marker quad 既有点序：tl, tr, br, bl。
    result->points[0] = tl;
    result->points[1] = tr;
    result->points[2] = br;
    result->points[3] = bl;
    result->width_px = width_px;
    result->height_px = height_px;
    result->valid = true;
    return true;
}

/**
 * @brief 计算一条射线在图像内部还能沿给定方向前进的最大距离
 * @note  第十步用它来把理论 H_max 截断到“原图里真实还能继续走的高度”。
 */
bool compute_ray_max_distance_inside_image(const cv::Point2f& start,
                                           const cv::Point2f& dir,
                                           int image_w,
                                           int image_h,
                                           float* max_t)
{
    if (max_t == nullptr || image_w <= 0 || image_h <= 0)
    {
        return false;
    }

    constexpr float kEps = 1e-6f;
    float limit = std::numeric_limits<float>::max();
    bool has_bound = false;

    if (std::abs(dir.x) > kEps)
    {
        const float x_limit = (dir.x > 0.0f)
                                  ? (static_cast<float>(image_w - 1) - start.x) / dir.x
                                  : (0.0f - start.x) / dir.x;
        limit = std::min(limit, x_limit);
        has_bound = true;
    }
    if (std::abs(dir.y) > kEps)
    {
        const float y_limit = (dir.y > 0.0f)
                                  ? (static_cast<float>(image_h - 1) - start.y) / dir.y
                                  : (0.0f - start.y) / dir.y;
        limit = std::min(limit, y_limit);
        has_bound = true;
    }
    if (!has_bound)
    {
        return false;
    }

    *max_t = std::max(0.0f, limit);
    return true;
}

/**
 * @brief 基于第九步主体顶部构造第十步 ROI 的基础边
 * @note  先取红块主体 top edge，再沿“红块指向图片”的方向上移一个安全距离，
 *        作为第十步高度搜索的起始边。
 */
bool build_roi_base_from_marker_top(const MarkerParallelogramResult& marker_quad_result,
                                    float safety_offset_px,
                                    float horizontal_expand_px,
                                    cv::Point2f* roi_bl,
                                    cv::Point2f* roi_br,
                                    cv::Point2f* left_up_dir,
                                    cv::Point2f* right_up_dir,
                                    cv::Point2f* roi_bottom_dir,
                                    float* roi_width,
                                    float* side_scale_ratio)
{
    if (roi_bl == nullptr || roi_br == nullptr || left_up_dir == nullptr || right_up_dir == nullptr ||
        roi_bottom_dir == nullptr || roi_width == nullptr || side_scale_ratio == nullptr ||
        !marker_quad_result.valid)
    {
        return false;
    }

    const cv::Point2f tl = marker_quad_result.points[0];
    const cv::Point2f tr = marker_quad_result.points[1];
    const cv::Point2f br = marker_quad_result.points[2];
    const cv::Point2f bl = marker_quad_result.points[3];
    cv::Point2f left_dir = tl - bl;
    cv::Point2f right_dir = tr - br;
    if (!normalize_vec2(left_dir, &left_dir) || !normalize_vec2(right_dir, &right_dir))
    {
        return false;
    }
    if (left_dir.y > 0.0f)
    {
        left_dir = -left_dir;
    }
    if (right_dir.y > 0.0f)
    {
        right_dir = -right_dir;
    }

    const cv::Point2f base_left = tl + left_dir * safety_offset_px;
    const cv::Point2f base_right = tr + right_dir * safety_offset_px;
    cv::Point2f bottom_dir = base_right - base_left;
    if (!normalize_vec2(bottom_dir, &bottom_dir))
    {
        return false;
    }
    *roi_bl = base_left - bottom_dir * horizontal_expand_px;
    *roi_br = base_right + bottom_dir * horizontal_expand_px;
    *left_up_dir = left_dir;
    *right_up_dir = right_dir;
    *roi_bottom_dir = bottom_dir;
    *roi_width = vec2_length(*roi_br - *roi_bl);
    const float left_height = vec2_length(tl - bl);
    const float right_height = vec2_length(tr - br);
    if (left_height <= 1e-3f || right_height <= 1e-3f || *roi_width <= 1.0f)
    {
        return false;
    }
    *side_scale_ratio = right_height / left_height;
    return *side_scale_ratio > 1e-3f;
}

void build_roi_scanline(const cv::Point2f& roi_bl,
                        const cv::Point2f& roi_br,
                        const cv::Point2f& left_up_dir,
                        const cv::Point2f& right_up_dir,
                        float side_scale_ratio,
                        float h,
                        cv::Point2f* scan_left,
                        cv::Point2f* scan_right)
{
    if (scan_left != nullptr)
    {
        *scan_left = roi_bl + left_up_dir * h;
    }
    if (scan_right != nullptr)
    {
        *scan_right = roi_br + right_up_dir * (h * side_scale_ratio);
    }
}

/**
 * @brief 参考 get_reference_point 的思路，为第十步估计当前帧白色阈值
 * @note  只统计原图最下面5行、且位于赛道左右边界之间的像素。
 */
bool estimate_step10_white_threshold_from_track_bottom(const cv::Mat& src_bgr,
                                                       Step10WhiteThreshold* white_th)
{
    if (white_th == nullptr || src_bgr.empty() || src_bgr.type() != CV_8UC3)
    {
        return false;
    }

    *white_th = Step10WhiteThreshold();
    const int valid_len = clamp_int(static_cast<int>(max_white.nums), 0, IMG_PROC_HEIGHT);
    if (valid_len <= 0)
    {
        return false;
    }

    uint64_t sum_b = 0;
    uint64_t sum_g = 0;
    uint64_t sum_r = 0;
    uint64_t sum_gray = 0;
    int sample_count = 0;
    const int start_row = std::max(0, UVC_HEIGHT - 5);
    for (int raw_row = UVC_HEIGHT - 1; raw_row >= start_row; --raw_row)
    {
        const int idx = raw_row_to_line_edge_idx(raw_row);
        if (idx < 0 || idx >= valid_len)
        {
            continue;
        }
        const BoundaryInfo& left = g_image_info.filter_left[idx];
        const BoundaryInfo& right = g_image_info.filter_right[idx];
        // 第十步白阈值参考区不再强依赖 sign 有效。
        // 如果某一侧边界无效，就把这一行退化成整行可用区域；
        // 这样“底部整行发白导致搜线边界失效”的情况仍然可以给第十步提供白参考。
        const int x_left = (left.sign != 0U)
                               ? clamp_int(line_x_to_raw_left(left.pos[0]), 0, src_bgr.cols - 1)
                               : 0;
        const int x_right = (right.sign != 0U)
                                ? clamp_int(line_x_to_raw_right(right.pos[0]), 0, src_bgr.cols - 1)
                                : (src_bgr.cols - 1);
        if (x_right < x_left)
        {
            continue;
        }

        const cv::Vec3b* row_ptr = src_bgr.ptr<cv::Vec3b>(raw_row);
        for (int x = x_left; x <= x_right; ++x)
        {
            const cv::Vec3b& bgr = row_ptr[x];
            sum_b += bgr[0];
            sum_g += bgr[1];
            sum_r += bgr[2];
            sum_gray += static_cast<uint64_t>(bgr[0] + bgr[1] + bgr[2]) / 3ULL;
            ++sample_count;
        }
    }

    if (sample_count <= 0)
    {
        return false;
    }

    white_th->mean_b = static_cast<float>(sum_b) / static_cast<float>(sample_count);
    white_th->mean_g = static_cast<float>(sum_g) / static_cast<float>(sample_count);
    white_th->mean_r = static_cast<float>(sum_r) / static_cast<float>(sample_count);
    white_th->mean_gray = static_cast<float>(sum_gray) / static_cast<float>(sample_count);
    white_th->white_floor = clamp_float(white_th->mean_gray * 0.7f, 0.0f, 255.0f);
    white_th->color_delta_max = 45.0f;
    white_th->sample_count = sample_count;
    white_th->valid = true;
    return true;
}

inline bool is_white_bgr_pixel(const cv::Vec3b& bgr, const Step10WhiteThreshold& white_th)
{
    const int b = bgr[0];
    const int g = bgr[1];
    const int r = bgr[2];
    const int min_v = std::min(b, std::min(g, r));
    const int max_v = std::max(b, std::max(g, r));
    return (static_cast<float>(min_v) >= white_th.white_floor) &&
           (static_cast<float>(max_v - min_v) <= white_th.color_delta_max);
}

/**
 * @brief 沿与红块底边平行的斜扫描线统计白色比例
 * @note  第十步不能直接用原图水平行，而要始终在主体几何定义的局部方向上采样。
 */
bool sample_white_ratio_on_scanline_bgr(const cv::Mat& src_bgr,
                                        const cv::Point2f& scan_left,
                                        const cv::Point2f& scan_right,
                                        const Step10WhiteThreshold& white_th,
                                        float* white_ratio)
{
    if (white_ratio == nullptr || src_bgr.empty() || src_bgr.type() != CV_8UC3)
    {
        return false;
    }

    const float line_len = vec2_length(scan_right - scan_left);
    const int sample_count = std::max(2, static_cast<int>(std::lround(line_len)));
    int white_count = 0;
    int valid_count = 0;
    for (int i = 0; i < sample_count; ++i)
    {
        const float t = (sample_count <= 1) ? 0.0f : static_cast<float>(i) / static_cast<float>(sample_count - 1);
        const cv::Point2f p = scan_left + (scan_right - scan_left) * t;
        const int x = static_cast<int>(std::lround(p.x));
        const int y = static_cast<int>(std::lround(p.y));
        if (x < 0 || x >= src_bgr.cols || y < 0 || y >= src_bgr.rows)
        {
            continue;
        }
        ++valid_count;
        if (is_white_bgr_pixel(src_bgr.at<cv::Vec3b>(y, x), white_th))
        {
            ++white_count;
        }
    }
    if (valid_count <= 0)
    {
        *white_ratio = 0.0f;
        return false;
    }
    *white_ratio = static_cast<float>(white_count) / static_cast<float>(valid_count);
    return true;
}

/**
 * @brief 在 0.5W~0.8W 范围内搜索最终 ROI 高度
 * @note  只有当 h/h+1/h+2 三条平行扫描线都足够白时，才认为找到了图片顶部。
 */
bool estimate_final_roi_height_from_white_scan(const cv::Mat& src_bgr,
                                               const cv::Point2f& roi_bl,
                                               const cv::Point2f& roi_br,
                                               const cv::Point2f& left_up_dir,
                                               const cv::Point2f& right_up_dir,
                                               float side_scale_ratio,
                                               float roi_width,
                                               const Step10WhiteThreshold& white_th,
                                               float* roi_h_min,
                                               float* roi_h_max,
                                               float* roi_h_final,
                                               std::vector<float>* white_ratios)
{
    if (roi_h_min == nullptr || roi_h_max == nullptr || roi_h_final == nullptr || white_ratios == nullptr ||
        src_bgr.empty() || roi_width <= 1.0f || !white_th.valid)
    {
        return false;
    }

    const float h_min_theory = std::max(1.0f, roi_width * 0.5f);
    const float h_max_theory = std::max(h_min_theory, roi_width * 0.8f);

    float bl_limit = 0.0f;
    float br_limit = 0.0f;
    if (side_scale_ratio <= 1e-3f ||
        !compute_ray_max_distance_inside_image(roi_bl, left_up_dir, src_bgr.cols, src_bgr.rows, &bl_limit) ||
        !compute_ray_max_distance_inside_image(roi_br, right_up_dir, src_bgr.cols, src_bgr.rows, &br_limit))
    {
        return false;
    }

    const float right_h_limit = br_limit / side_scale_ratio;
    const float h_limit = std::max(0.0f, std::min(bl_limit, right_h_limit));
    if (h_limit < 1.0f)
    {
        return false;
    }

    *roi_h_max = std::min(h_max_theory, h_limit);
    *roi_h_min = std::min(h_min_theory, *roi_h_max);
    if (*roi_h_max < 1.0f)
    {
        return false;
    }

    const int max_h_int = std::max(0, static_cast<int>(std::floor(*roi_h_max))) + 2;
    white_ratios->assign(static_cast<size_t>(max_h_int + 1), 0.0f);
    for (int h = 0; h <= max_h_int; ++h)
    {
        cv::Point2f scan_left;
        cv::Point2f scan_right;
        build_roi_scanline(roi_bl,
                           roi_br,
                           left_up_dir,
                           right_up_dir,
                           side_scale_ratio,
                           static_cast<float>(h),
                           &scan_left,
                           &scan_right);
        float ratio = 0.0f;
        sample_white_ratio_on_scanline_bgr(src_bgr, scan_left, scan_right, white_th, &ratio);
        (*white_ratios)[static_cast<size_t>(h)] = ratio;
    }

    constexpr float kWhiteRatioThreshold = 0.95f;
    const int start_h = std::max(0, static_cast<int>(std::ceil(*roi_h_min)));
    const int stop_h = std::max(start_h, static_cast<int>(std::floor(*roi_h_max)));
    *roi_h_final = *roi_h_max;
    for (int h = start_h; h <= stop_h; ++h)
    {
        if ((*white_ratios)[static_cast<size_t>(h)] >= kWhiteRatioThreshold &&
            (*white_ratios)[static_cast<size_t>(h + 1)] >= kWhiteRatioThreshold &&
            (*white_ratios)[static_cast<size_t>(h + 2)] >= kWhiteRatioThreshold)
        {
            *roi_h_final = static_cast<float>(h);
            break;
        }
    }
    // if (*roi_h_final >= (*roi_h_max - 1e-3f))
    // {
    //     // 第 10 步高度搜索回退日志：
    //     // 在 [H_min, H_max] 范围内没有找到连续 3 行 white_ratio >= 0.95，
    //     // 所以按设计直接回退到 H_max，而不是继续向上搜索。
    //     // start_h / stop_h 用来对应上面 ratio 日志里的实际扫描区间。
    //     RED_ROI_INFO("step10 white_scan fallback to h_max: roi_h_final=%.2f roi_h_max=%.2f start_h=%d stop_h=%d",
    //                  *roi_h_final,
    //                  *roi_h_max,
    //                  start_h,
    //                  stop_h);
    // }
    return true;
}

bool build_final_roi_quad(const cv::Point2f& roi_bl,
                          const cv::Point2f& roi_br,
                          const cv::Point2f& left_up_dir,
                          const cv::Point2f& right_up_dir,
                          float side_scale_ratio,
                          float roi_height,
                          cv::Point2f* roi_points)
{
    // 第 10 步最终 ROI 四边形构造函数。
    // 输入的 roi_bl / roi_br 是 ROI 底边左右端点，它们已经在原图坐标系里，
    // 并且底边建立在“红块主体顶部再沿图片方向安全上移后的那条线”上。
    //
    // left_up_dir / right_up_dir：
    //   分别是 ROI 左边、右边朝“目标图片方向”的单位向量。
    //   这里故意保留左右两侧各自的方向，而不是只用一个统一法向，
    //   因为当前红块顶部和图片底边在透视下允许形成梯形，而不强制是平行四边形。
    //
    // side_scale_ratio：
    //   右侧高度相对左侧高度的缩放比例。
    //   例如左侧往上走 roi_height，右侧就走 roi_height * side_scale_ratio，
    //   这样可以让最终 ROI 上边与下边不必严格平行，更贴合当前视角下的真实图片区域。
    //
    // roi_height：
    //   当前要构造的 ROI 左侧高度，单位是原图像素。
    //   这个高度可能是：
    //   1) roi_h_max：最大候选高度，用于调试显示；
    //   2) roi_h_final：最终白色扫描确定的正式高度，用于真正输出 roi_points。
    //
    // 输出点序固定为：
    //   roi_points[0] = tl 左上
    //   roi_points[1] = tr 右上
    //   roi_points[2] = bl 左下
    //   roi_points[3] = br 右下
    //
    // 注意：
    //   这个点序是当前工程 red_roi 对外约定，
    //   后续做透视变换时会再重排成 getPerspectiveTransform 需要的 tl,tr,br,bl。
    if (roi_points == nullptr || roi_height <= 0.0f || side_scale_ratio <= 1e-3f)
    {
        return false;
    }
    roi_points[0] = roi_bl + left_up_dir * roi_height;
    roi_points[1] = roi_br + right_up_dir * (roi_height * side_scale_ratio);
    roi_points[2] = roi_bl;
    roi_points[3] = roi_br;
    return true;
}

bool validate_final_roi_quad(const cv::Point2f* roi_points,
                             int image_w,
                             int image_h)
{
    // 第 10 步最终 ROI 合法性校验函数。
    // 这个函数不负责“重新修正 ROI”，只负责在 ROI 四点已经构造出来之后，
    // 做一层统一的几何防御检查，避免把明显错误或退化的 ROI 继续送到后面的裁剪与推理。
    //
    // 当前检查分成 4 类：
    //
    // 1) 图像边界检查
    //    四个点都必须落在原始 BGR 图范围内。
    //    这里只检查原图边界，不再检查“必须位于赛道左右边界之间”。
    //
    // 2) 尺寸检查
    //    ROI 的上边宽度 width_px 不能过小；
    //    ROI 的左边高度 height_px 不能过小；
    //    同时 width_px * height_px 也不能太小。
    //    这样可以挡住几何退化、噪声误检、或者四点几乎重合的情况。
    //
    // 3) 方向退化检查
    //    top_dir = tr - tl
    //    left_dir = tl - bl
    //    这两条边都必须有足够长度，不能退化成接近 0 的向量。
    //
    // 4) 朝向检查
    //    top_dir.x 不能为负：
    //      表示 ROI 上边整体必须仍然是“从左往右”。
    //    left_dir.y 不能为正：
    //      表示 ROI 左边必须整体是“从下往上”。
    //    这一步主要用于防止点序翻转、上下颠倒或左右颠倒。
    //
    // 只要有一项不满足，就直接返回 false，让上层放弃当前帧 ROI。
    if (roi_points == nullptr || !are_points_inside_image(roi_points, image_w, image_h))
    {
        return false;
    }

    const float width_px = vec2_length(roi_points[1] - roi_points[0]);
    const float height_px = vec2_length(roi_points[2] - roi_points[0]);
    if (width_px < 8.0f || height_px < 8.0f)
    {
        return false;
    }
    if ((width_px * height_px) < 64.0f)
    {
        return false;
    }

    const cv::Point2f top_dir = roi_points[1] - roi_points[0];
    const cv::Point2f left_dir = roi_points[0] - roi_points[2];
    if (top_dir.dot(top_dir) <= 1e-6f || left_dir.dot(left_dir) <= 1e-6f)
    {
        return false;
    }
    if (top_dir.x < -1e-4f)
    {
        return false;
    }
    if (left_dir.y > 1e-4f)
    {
        return false;
    }
    return true;
}

#if ENABLE_VISION_DEBUG
cv::Rect build_debug_crop_from_points(const std::vector<cv::Point2f>& points,
                                      int image_w,
                                      int image_h,
                                      int margin)
{
    if (points.empty() || image_w <= 0 || image_h <= 0)
    {
        return cv::Rect();
    }
    float min_x = static_cast<float>(image_w - 1);
    float min_y = static_cast<float>(image_h - 1);
    float max_x = 0.0f;
    float max_y = 0.0f;
    for (const cv::Point2f& p : points)
    {
        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
    }
    const int x0 = clamp_int(static_cast<int>(std::floor(min_x)) - margin, 0, image_w - 1);
    const int y0 = clamp_int(static_cast<int>(std::floor(min_y)) - margin, 0, image_h - 1);
    const int x1 = clamp_int(static_cast<int>(std::ceil(max_x)) + margin, 0, image_w - 1);
    const int y1 = clamp_int(static_cast<int>(std::ceil(max_y)) + margin, 0, image_h - 1);
    return cv::Rect(x0, y0, std::max(1, x1 - x0 + 1), std::max(1, y1 - y0 + 1));
}

cv::Point2f map_raw_point_to_crop(const cv::Point2f& raw_point, const cv::Rect& crop_rect)
{
    return cv::Point2f(raw_point.x - static_cast<float>(crop_rect.x),
                       raw_point.y - static_cast<float>(crop_rect.y));
}

void draw_quad_on_crop(const cv::Point2f* quad_raw,
                       const cv::Rect& crop_rect,
                       cv::Mat* canvas,
                       const cv::Scalar& color,
                       int thickness)
{
    if (canvas == nullptr || canvas->empty() || quad_raw == nullptr)
    {
        return;
    }
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f p0 = map_raw_point_to_crop(quad_raw[i], crop_rect);
        const cv::Point2f p1 = map_raw_point_to_crop(quad_raw[(i + 1) % 4], crop_rect);
        cv::line(*canvas, p0, p1, color, thickness, cv::LINE_AA);
    }
}

void draw_roi_quad_on_crop_tl_tr_bl_br(const cv::Point2f* quad_raw,
                                       const cv::Rect& crop_rect,
                                       cv::Mat* canvas,
                                       const cv::Scalar& color,
                                       int thickness)
{
    if (canvas == nullptr || canvas->empty() || quad_raw == nullptr)
    {
        return;
    }
    const int draw_order[4] = {0, 1, 3, 2};
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f p0 = map_raw_point_to_crop(quad_raw[draw_order[i]], crop_rect);
        const cv::Point2f p1 = map_raw_point_to_crop(quad_raw[draw_order[(i + 1) % 4]], crop_rect);
        cv::line(*canvas, p0, p1, color, thickness, cv::LINE_AA);
    }
}

void build_step10_overlay_visualization(const cv::Mat& src_bgr,
                                        const MarkerParallelogramResult& marker_quad_result_raw,
                                        const cv::Point2f& roi_bl,
                                        const cv::Point2f& roi_br,
                                        const cv::Point2f& left_up_dir,
                                        const cv::Point2f& right_up_dir,
                                        float side_scale_ratio,
                                        float roi_h_min,
                                        float roi_h_max,
                                        float roi_h_final,
                                        cv::Mat* vis_patch)
{
    if (vis_patch == nullptr || src_bgr.empty() || !marker_quad_result_raw.valid || side_scale_ratio <= 1e-3f)
    {
        return;
    }

    cv::Point2f hmin_left;
    cv::Point2f hmin_right;
    cv::Point2f hmax_left;
    cv::Point2f hmax_right;
    cv::Point2f final_quad[4];
    build_roi_scanline(roi_bl, roi_br, left_up_dir, right_up_dir, side_scale_ratio, roi_h_min, &hmin_left, &hmin_right);
    build_roi_scanline(roi_bl, roi_br, left_up_dir, right_up_dir, side_scale_ratio, roi_h_max, &hmax_left, &hmax_right);
    if (!build_final_roi_quad(roi_bl, roi_br, left_up_dir, right_up_dir, side_scale_ratio, roi_h_final, final_quad))
    {
        return;
    }

    std::vector<cv::Point2f> debug_points;
    debug_points.reserve(16);
    for (int i = 0; i < 4; ++i)
    {
        debug_points.push_back(marker_quad_result_raw.points[i]);
        debug_points.push_back(final_quad[i]);
    }
    debug_points.push_back(roi_bl);
    debug_points.push_back(roi_br);
    debug_points.push_back(hmin_left);
    debug_points.push_back(hmin_right);
    debug_points.push_back(hmax_left);
    debug_points.push_back(hmax_right);
    debug_points.push_back(roi_bl + left_up_dir * 18.0f);
    debug_points.push_back(roi_br + right_up_dir * (18.0f * side_scale_ratio));
    const cv::Rect crop_rect = build_debug_crop_from_points(debug_points, src_bgr.cols, src_bgr.rows, 10);
    if (crop_rect.width <= 0 || crop_rect.height <= 0)
    {
        return;
    }

    *vis_patch = src_bgr(crop_rect).clone();
    draw_quad_on_crop(marker_quad_result_raw.points, crop_rect, vis_patch, cv::Scalar(0, 0, 255), 1);
    cv::line(*vis_patch,
             map_raw_point_to_crop(roi_bl, crop_rect),
             map_raw_point_to_crop(roi_br, crop_rect),
             cv::Scalar(255, 255, 0),
             1,
             cv::LINE_AA);
    cv::arrowedLine(*vis_patch,
                    map_raw_point_to_crop(roi_bl, crop_rect),
                    map_raw_point_to_crop(roi_bl + left_up_dir * 18.0f, crop_rect),
                    cv::Scalar(0, 255, 255),
                    1,
                    cv::LINE_AA,
                    0,
                    0.2);
    cv::arrowedLine(*vis_patch,
                    map_raw_point_to_crop(roi_br, crop_rect),
                    map_raw_point_to_crop(roi_br + right_up_dir * (18.0f * side_scale_ratio), crop_rect),
                    cv::Scalar(255, 0, 255),
                    1,
                    cv::LINE_AA,
                    0,
                    0.2);
    cv::line(*vis_patch,
             map_raw_point_to_crop(hmin_left, crop_rect),
             map_raw_point_to_crop(hmin_right, crop_rect),
             cv::Scalar(0, 255, 255),
             1,
             cv::LINE_AA);
    cv::line(*vis_patch,
             map_raw_point_to_crop(hmax_left, crop_rect),
             map_raw_point_to_crop(hmax_right, crop_rect),
             cv::Scalar(255, 0, 0),
             1,
             cv::LINE_AA);
    draw_roi_quad_on_crop_tl_tr_bl_br(final_quad, crop_rect, vis_patch, cv::Scalar(0, 255, 0), 1);
}
#endif

/**
 * @brief 把局部 patch 上的点映射回原始 BGR 图坐标
 * @note  patch 坐标采用 extract_rotated_roi_patch_impl 的目标矩形定义：
 *        (0,0)->raw tl, (w-1,0)->raw tr, (0,h-1)->raw bl。
 */
bool map_local_patch_point_to_raw(const LocalPatchTransform& tf,
                                  const cv::Point2f& local_pt,
                                  cv::Point2f* raw_pt)
{
    if (raw_pt == nullptr || !tf.valid || tf.patch_width <= 1 || tf.patch_height <= 1)
    {
        return false;
    }

    const float u = local_pt.x / static_cast<float>(tf.patch_width - 1);
    const float v = local_pt.y / static_cast<float>(tf.patch_height - 1);
    *raw_pt = tf.quad[0] +
              (tf.quad[1] - tf.quad[0]) * u +
              (tf.quad[2] - tf.quad[0]) * v;
    return true;
}

/**
 * @brief 把第九步得到的局部主体四边形恢复到原图坐标
 */
bool map_marker_quad_to_raw(const MarkerParallelogramResult& local_quad_result,
                            const LocalPatchTransform& tf,
                            MarkerParallelogramResult* raw_quad_result)
{
    if (raw_quad_result == nullptr || !local_quad_result.valid)
    {
        return false;
    }

    *raw_quad_result = MarkerParallelogramResult();
    for (int i = 0; i < 4; ++i)
    {
        if (!map_local_patch_point_to_raw(tf, local_quad_result.points[i], &raw_quad_result->points[i]))
        {
            return false;
        }
    }
    raw_quad_result->width_px = vec2_length(raw_quad_result->points[1] - raw_quad_result->points[0]);
    raw_quad_result->height_px = vec2_length(raw_quad_result->points[3] - raw_quad_result->points[0]);
    raw_quad_result->valid = (raw_quad_result->width_px > 1.0f && raw_quad_result->height_px > 1.0f);
    return raw_quad_result->valid;
}

/**
 * @brief 把拟合出来的直线画到局部 patch 上
 * @param line_point  直线上的一点
 * @param line_dir    单位方向向量
 * @param width       画布宽
 * @param height      画布高
 * @param canvas      输出画布
 * @param color       线颜色
 * @note  仅调试显示使用：把无限长直线裁到 patch 边界内再绘制。
 */
void draw_fitted_line_on_patch(const cv::Point2f& line_point,
                               const cv::Point2f& line_dir,
                               int width,
                               int height,
                               cv::Mat* canvas,
                               const cv::Scalar& color)
{
    if (canvas == nullptr || canvas->empty() || width <= 0 || height <= 0)
    {
        return;
    }
    // 直线本身是无限长的，这里先和 patch 四条边求交，
    // 再把落在画布内的两个交点连起来，得到可视化线段。
    std::vector<cv::Point2f> intersections;
    if (std::abs(line_dir.x) > 1e-4f)
    {
        for (int x_edge = 0; x_edge <= width - 1; x_edge += std::max(1, width - 1))
        {
            const float t = (static_cast<float>(x_edge) - line_point.x) / line_dir.x;
            const float y = line_point.y + line_dir.y * t;
            if (y >= 0.0f && y <= static_cast<float>(height - 1))
            {
                intersections.emplace_back(static_cast<float>(x_edge), y);
            }
        }
    }
    if (std::abs(line_dir.y) > 1e-4f)
    {
        for (int y_edge = 0; y_edge <= height - 1; y_edge += std::max(1, height - 1))
        {
            const float t = (static_cast<float>(y_edge) - line_point.y) / line_dir.y;
            const float x = line_point.x + line_dir.x * t;
            if (x >= 0.0f && x <= static_cast<float>(width - 1))
            {
                intersections.emplace_back(x, static_cast<float>(y_edge));
            }
        }
    }
    if (intersections.size() < 2)
    {
        return;
    }

    // 交点可能在拐角处重复命中两次，这里先去重再绘制。
    auto unique_end = std::unique(intersections.begin(),
                                  intersections.end(),
                                  [](const cv::Point2f& a, const cv::Point2f& b)
                                  {
                                      return std::abs(a.x - b.x) < 1e-3f && std::abs(a.y - b.y) < 1e-3f;
                                  });
    intersections.erase(unique_end, intersections.end());
    if (intersections.size() < 2)
    {
        return;
    }

    cv::line(*canvas,
             cv::Point(static_cast<int>(std::lround(intersections[0].x)),
                       static_cast<int>(std::lround(intersections[0].y))),
             cv::Point(static_cast<int>(std::lround(intersections[1].x)),
                       static_cast<int>(std::lround(intersections[1].y))),
             color,
             1,
             cv::LINE_AA);
}

/**
 * @brief 生成 TCP 右图使用的局部分析可视化图
 * @param local_mask_patch     局部二值掩码
 * @param row_infos            每行最长连续红段统计
 * @param stable_band_info     最下方稳定红带
 * @param body_rows_result     主体行组结果
 * @param bottom_edge_result   底边拟合结果
 * @param left_edge_result     左边拟合结果
 * @param right_edge_result    右边拟合结果
 * @param marker_quad_result   第九步重建出的红块平行四边形
 * @param analysis_vis_patch   输出可视化图
 * @note  只负责显示，不参与正式 ROI 几何求解。
 *        当前右图只保留：
 *        1) 白色局部二值掩码
 *        2) 黄色主体行
 *        3) 蓝色底边拟合线
 *        4) 红色左边界拟合线
 *        5) 品红色右边界拟合线
 *        6) 绿色重建红块四边形和四个角点
 */
void build_marker_analysis_visualization(const cv::Mat& local_mask_patch,
                                         const std::vector<LocalRowRunInfo>& row_infos,
                                         const LocalBandInfo& stable_band_info,
                                         const MarkerBodyRowsResult& body_rows_result,
                                         const MarkerBottomEdgeFitResult& bottom_edge_result,
                                         const MarkerLeftEdgeFitResult& left_edge_result,
                                         const MarkerRightEdgeFitResult& right_edge_result,
                                         const MarkerParallelogramResult& marker_quad_result,
                                         cv::Mat* analysis_vis_patch)
{
    if (analysis_vis_patch == nullptr || local_mask_patch.empty())
    {
        return;
    }

    // 白底直接显示局部二值掩码，后面只叠加当前真正关心的几何结果。
    cv::cvtColor(local_mask_patch, *analysis_vis_patch, cv::COLOR_GRAY2BGR);
    for (int row = 0; row < static_cast<int>(row_infos.size()); ++row)
    {
        if (!body_rows_result.valid ||
            row >= static_cast<int>(body_rows_result.row_flags.size()) ||
            body_rows_result.row_flags[row] == 0U)
        {
            continue;
        }
        // 主体行组只做“整行高亮”，便于先看第五步哪些行被认为是可信主体。
        cv::line(*analysis_vis_patch,
                 cv::Point(0, row),
                 cv::Point(analysis_vis_patch->cols - 1, row),
                 cv::Scalar(0, 255, 255),
                 1,
                 cv::LINE_8);
    }

    if (bottom_edge_result.valid)
    {
        draw_fitted_line_on_patch(bottom_edge_result.line_point,
                                  bottom_edge_result.line_dir,
                                  analysis_vis_patch->cols,
                                  analysis_vis_patch->rows,
                                  analysis_vis_patch,
                                  cv::Scalar(255, 0, 0));
    }

    if (left_edge_result.valid)
    {
        draw_fitted_line_on_patch(left_edge_result.line_point,
                                  left_edge_result.line_dir,
                                  analysis_vis_patch->cols,
                                  analysis_vis_patch->rows,
                                  analysis_vis_patch,
                                  cv::Scalar(0, 0, 255));
    }

    if (right_edge_result.valid)
    {
        draw_fitted_line_on_patch(right_edge_result.line_point,
                                  right_edge_result.line_dir,
                                  analysis_vis_patch->cols,
                                  analysis_vis_patch->rows,
                                  analysis_vis_patch,
                                  cv::Scalar(255, 0, 255));
    }

    if (marker_quad_result.valid)
    {
        // 绿色轮廓表示新版第九步重建后的红块主体四边形。
        for (int i = 0; i < 4; ++i)
        {
            const cv::Point2f p0 = marker_quad_result.points[i];
            const cv::Point2f p1 = marker_quad_result.points[(i + 1) % 4];
            cv::line(*analysis_vis_patch, p0, p1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
            cv::circle(*analysis_vis_patch, p0, 2, cv::Scalar(0, 255, 0), cv::FILLED, cv::LINE_AA);
        }
    }
}

bool build_local_mask_analysis_patch(const cv::Mat& full_mask,
                                     const cv::RotatedRect& red_rect,
                                     const cv::Point2f& bottom_center,
                                     const cv::Point2f& roi_bottom_axis,
                                     const cv::Point2f& roi_up_axis,
                                     cv::Mat* local_mask_patch,
                                     std::vector<LocalRowRunInfo>* row_infos_out,
                                     LocalBandInfo* stable_band_info,
                                     LocalPatchTransform* patch_tf,
                                     cv::Mat* analysis_vis_patch)
{
    if (local_mask_patch == nullptr || full_mask.empty() || full_mask.type() != CV_8UC1)
    {
        return false;
    }
    if (stable_band_info != nullptr)
    {
        *stable_band_info = LocalBandInfo();
    }

    // 把最优旋转矩形四点投影到局部坐标系中，
    // 后面就在这个“转正”的局部图里做逐行分析。
    cv::Point2f red_points[4];
    red_rect.points(red_points);

    float min_u = std::numeric_limits<float>::max();
    float max_u = -std::numeric_limits<float>::max();
    float min_v = std::numeric_limits<float>::max();
    float max_v = -std::numeric_limits<float>::max();
    for (int i = 0; i < 4; ++i)
    {
        const cv::Point2f rel = red_points[i] - bottom_center;
        const float u = rel.dot(roi_bottom_axis);
        const float v = rel.dot(roi_up_axis);
        min_u = std::min(min_u, u);
        max_u = std::max(max_u, u);
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
    }

    if (max_u <= min_u + 1e-3f || max_v <= min_v + 1e-3f)
    {
        return false;
    }

    // 局部 patch 的宽高直接来自红块在局部坐标系中的 u/v 投影范围，
    // 这样后续逐行分析面对的是一张“与红块姿态对齐”的图。
    const int patch_width = std::max(1, static_cast<int>(std::lround(max_u - min_u)));
    const int patch_height = std::max(1, static_cast<int>(std::lround(max_v - min_v)));
    cv::Point2f local_quad[4];
    local_quad[0] = bottom_center + roi_bottom_axis * min_u + roi_up_axis * max_v;
    local_quad[1] = bottom_center + roi_bottom_axis * max_u + roi_up_axis * max_v;
    local_quad[2] = bottom_center + roi_bottom_axis * min_u + roi_up_axis * min_v;
    local_quad[3] = bottom_center + roi_bottom_axis * max_u + roi_up_axis * min_v;

    if (patch_tf != nullptr)
    {
        patch_tf->valid = true;
        patch_tf->quad[0] = local_quad[0];
        patch_tf->quad[1] = local_quad[1];
        patch_tf->quad[2] = local_quad[2];
        patch_tf->quad[3] = local_quad[3];
        patch_tf->patch_width = patch_width;
        patch_tf->patch_height = patch_height;
    }

    // 从整图红色掩码中把当前红块区域拉正出来，得到局部二值掩码。
    cv::Mat warped_mask;
    if (!extract_rotated_roi_patch_impl(full_mask, local_quad, patch_width, patch_height, &warped_mask))
    {
        return false;
    }

    cv::threshold(warped_mask, *local_mask_patch, 127, 255, cv::THRESH_BINARY);
    if (local_mask_patch->empty())
    {
        return false;
    }

    // 对局部图每一行都统计“最长连续红段”，这是后续主体判断的基础数据。
    std::vector<LocalRowRunInfo> row_infos(local_mask_patch->rows);
    for (int row = 0; row < local_mask_patch->rows; ++row)
    {
        analyze_longest_foreground_run_in_row(*local_mask_patch, row, &row_infos[row]);
    }
    if (row_infos_out != nullptr)
    {
        *row_infos_out = row_infos;
    }

    LocalBandInfo band_info;
    find_bottom_stable_red_band_in_local_patch(row_infos,
                                               local_mask_patch->cols,
                                               &band_info);
    if (stable_band_info != nullptr)
    {
        *stable_band_info = band_info;
    }

    if (analysis_vis_patch != nullptr)
    {
        // 这里只生成“前四步默认可视化”版本；
        // 第五步之后的主体行、底边、左边、绿色四边形会在 build_rotated_roi_above_red 中统一叠加。
        build_marker_analysis_visualization(*local_mask_patch,
                                            row_infos,
                                            band_info,
                                            MarkerBodyRowsResult(),
                                            MarkerBottomEdgeFitResult(),
                                            MarkerLeftEdgeFitResult(),
                                            MarkerRightEdgeFitResult(),
                                            MarkerParallelogramResult(),
                                            analysis_vis_patch);
    }
    return true;
}

/**
 * @brief 根据红块在图像中的纵向位置，自适应调整允许的最大长短边
 * @param center_y_in_image 红块中心在原图中的 y 坐标
 * @param image_h           原图高度
 * @param max_long_side     输出：允许的最大长边
 * @param max_short_side    输出：允许的最大短边
 * @note  越靠近图像下方的红块，允许尺寸越大；
 *        主要用于区分“正常近处红块”与“粘连/拖大的异常矩形”。
 */
void get_adaptive_max_side_limits(float center_y_in_image,
                                  int image_h,
                                  float* max_long_side,
                                  float* max_short_side)
{
    //指针保护
    if (max_long_side == nullptr || max_short_side == nullptr)
    {
        RED_ROI_ERR("function input error!");
        return;
    }

    //图像高度不能太短
    if (image_h <= 1)
    {
        *max_long_side = kMaxRedLongSidePx;
        *max_short_side = kMaxRedShortSidePx;
        RED_ROI_ERR("image_h too short!");
        return;
    }

    // 归一化范围改为 ROI 预测区域：kRoiPredictStartCenterRow~(image_h-1)
    const float y_min = static_cast<float>(clamp_int(kRoiPredictStartCenterRow, 0, image_h - 20));//中心最多220
    const float y_max = static_cast<float>(image_h - 1);
    if (y_max <= y_min + 1e-3f)
    {
        *max_long_side = kMaxRedLongSidePx;
        *max_short_side = kMaxRedShortSidePx;
        RED_ROI_ERR("Defensive protection!");
        return;
    }
    //把图像中心限制到kRoiPredictStartRow~(image_h-1)
    const float y_clamped = clamp_float(center_y_in_image, y_min, y_max);
    const float y_ratio = (y_clamped - y_min) / (y_max - y_min);
    
    //动态计算“允许的最大长边”和“允许的最大短边”
    //kMaxRedLongSideFarPx远处的最大长边,kMaxRedLongSidePx远处的最大短边
    //lerp_float(a, b, t) 一般表示线性插值,a + (b - a) * t,得到每个图像轴的
    *max_long_side = lerp_float(kMaxRedLongSideFarPx, kMaxRedLongSidePx, y_ratio);
    *max_short_side = lerp_float(kMaxRedShortSideFarPx, kMaxRedShortSidePx, y_ratio);
}

/**
 * @brief 二维向量归一化
 * @param v 输入向量
 * @param out 输出归一化向量指针
 * @return bool true=成功，false=输入无效（如零向量或空指针）
 */
bool normalize_vec2(const cv::Point2f& v, cv::Point2f* out)
{
    if (out == nullptr)
    {
        return false;
    }
    const float len = vec2_length(v);
    if (len <= 1e-6f)
    {
        return false;
    }
    out->x = v.x / len;
    out->y = v.y / len;
    return true;
}

/**
 * @brief 从 filter_midline 中提取 red 中心附近 5 个原图行的局部走向（映射到 320x240 后比较）
 * @param raw_center_y 原图 y（0~image_h-1）
 * @param image_h      原图高
 * @param out_dir      输出单位方向向量
 * @return bool        true=提取成功 false=点不足或方向无效
 */
bool estimate_midline_local_direction_raw(int raw_center_y, int image_h, cv::Point2f* out_dir)
{
    if (out_dir == nullptr || image_h <= 0)
    {
        return false;
    }

    // raw_center_y 来自 320x240，直接 /2 近似映射到 160x120 的 y 坐标。
    const int line_center_y = clamp_int(raw_center_y / IMG_PROC_DOWNSAMPLE, 0, IMG_PROC_HEIGHT - 1);
    const int valid_len = clamp_int(static_cast<int>(max_white.nums), 0, IMG_PROC_HEIGHT);
    // 把处理图像素中心大致映射到原图里的中心位置
    const float raw_center_offset = 0.5f * static_cast<float>(IMG_PROC_DOWNSAMPLE - 1);

    // 取 line_center_y 上下两行，共 5 个点（-2,-1,0,+1,+2）。
    std::vector<cv::Point2f> local_points;
    local_points.reserve(5);
    for (int dy = -2; dy <= 2; ++dy)
    {
        const int row_y = clamp_int(line_center_y + dy, 0, IMG_PROC_HEIGHT - 1);
        // filter_midline 的索引是 seekline_num（从底往上）：
        // row = IMG_PROC_HEIGHT - 1 - seekline_num。
        const int idx = (IMG_PROC_HEIGHT - 1) - row_y;
        if (idx < 0 || idx >= valid_len)
        {
            continue;
        }

        local_points.emplace_back(
            static_cast<float>(g_image_info.filter_midline[idx][0] * IMG_PROC_DOWNSAMPLE) + raw_center_offset,
            static_cast<float>(g_image_info.filter_midline[idx][1] * IMG_PROC_DOWNSAMPLE) + raw_center_offset);
    }

    if (local_points.size() < 2)
    {
        return false;
    }

    // 用 5 点整体趋势（首尾点连线）作为局部方向。
    return normalize_vec2(local_points.back() - local_points.front(), out_dir);
}

/**
 * @brief 检查四个点是否全部位于图像边界内
 * @param points  四个待检查点
 * @param image_w 图像宽
 * @param image_h 图像高
 * @return bool   true=全部在图像内 false=至少一个点越界
 * @note  主要用于最终 ROI 四点的合法性检查。
 */
bool are_points_inside_image(const cv::Point2f* points, int image_w, int image_h)
{
    if (points == nullptr || image_w <= 0 || image_h <= 0)
    {
        return false;
    }
    const float max_x = static_cast<float>(image_w - 1);
    const float max_y = static_cast<float>(image_h - 1);
    for (int i = 0; i < 4; ++i)
    {
        if (points[i].x < 0.0f || points[i].x > max_x ||
            points[i].y < 0.0f || points[i].y > max_y)
        {
            return false;
        }
    }
    return true;
}

/**
 * @brief 由四点构建轴对齐外接矩形，并裁剪到图像边界
 * @param points  四个输入点
 * @param image_w 图像宽
 * @param image_h 图像高
 * @return cv::Rect 裁剪后的外接矩形
 * @note  当前仅作为兼容字段 roi_rect 的输出形式，真正的几何仍以四点为准。
 */
cv::Rect build_bbox_from_points(const cv::Point2f* points, int image_w, int image_h)
{
    if (points == nullptr || image_w <= 0 || image_h <= 0)
    {
        return cv::Rect();
    }

    float min_x = points[0].x;
    float max_x = points[0].x;
    float min_y = points[0].y;
    float max_y = points[0].y;
    for (int i = 1; i < 4; ++i)
    {
        min_x = std::min(min_x, points[i].x);
        max_x = std::max(max_x, points[i].x);
        min_y = std::min(min_y, points[i].y);
        max_y = std::max(max_y, points[i].y);
    }

    const int x0 = clamp_int(static_cast<int>(std::floor(min_x)), 0, image_w - 1);
    const int y0 = clamp_int(static_cast<int>(std::floor(min_y)), 0, image_h - 1);
    const int x1 = clamp_int(static_cast<int>(std::ceil(max_x)), 0, image_w - 1);
    const int y1 = clamp_int(static_cast<int>(std::ceil(max_y)), 0, image_h - 1);
    if (x1 < x0 || y1 < y0)
    {
        return cv::Rect();
    }
    return cv::Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
}

/**
 * @brief 重置 ROI 结果结构体
 * @param roi_result 要重置的 ROI 结果指针
 * @note  每次进入新一帧的 red_roi_find 前调用，防止上一帧结果残留。
 */
void reset_roi_result(RedRoiResult* roi_result)
{
    if (roi_result == nullptr)
    {
        return;
    }
    roi_result->red_found = false;
    roi_result->found = false;
    roi_result->roi_rect = cv::Rect();
    roi_result->roi_angle_deg = 0.0f;
    roi_result->roi_size_px = 0;
    roi_result->roi_width_px = 0;
    roi_result->roi_height_px = 0;
    for (int i = 0; i < 4; ++i)
    {
        roi_result->roi_points[i] = cv::Point2f(0.0f, 0.0f);
    }
}

/**
 * @brief 清理本帧红色检测缓存
 * @param image_w 当前原图宽
 * @param image_h 当前原图高
 * @note  这个函数同时做两件事：
 *        1) 重建正式链路使用的整图红色掩码缓存 g_red_full_mask
 *        2) 在调试模式下，清空所有 TCP/离线调试图缓存
 */
void reset_red_debug_cache(int image_w, int image_h)
{
    if (image_w > 0 && image_h > 0)
    {
        // 正式链路使用的整图掩码每帧都要重建，不能依赖调试开关。
        g_red_full_mask = cv::Mat::zeros(image_h, image_w, CV_8UC1);
#if ENABLE_VISION_DEBUG
        g_red_debug_mask_full = cv::Mat::zeros(image_h, image_w, CV_8UC1);
#endif
    }
    else
    {
        g_red_full_mask.release();
#if ENABLE_VISION_DEBUG
        g_red_debug_mask_full.release();
#endif
    }

#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：清空本帧 red_roi 调试缓存，避免上一帧结果残留 =====
    g_red_debug_min_rect_patch.release();
    g_red_debug_min_rect_mask_patch.release();
    g_red_debug_rect_valid = false;
    g_red_debug_roi_valid = false;
    g_red_debug_selected_edge_valid = false;
    g_red_debug_bottom_axis_valid = false;
    for (int i = 0; i < 4; ++i)
    {
        g_red_debug_rect_points[i] = cv::Point2f(0.0f, 0.0f);
        g_red_debug_roi_points[i] = cv::Point2f(0.0f, 0.0f);
    }
    for (int i = 0; i < 2; ++i)
    {
        g_red_debug_selected_edge_points[i] = cv::Point2f(0.0f, 0.0f);
        g_red_debug_bottom_axis_points[i] = cv::Point2f(0.0f, 0.0f);
    }
    // ===== 调试代码结束：清空本帧 red_roi 调试缓存，避免上一帧结果残留 =====
#else
    (void)image_w;
    (void)image_h;
#endif
}

/**
 * @brief 构建红色检测的ROI窗口
 * @param src_bgr 输入BGR图像
 * @return cv::Rect 检测窗口矩形
 * @note  依赖 image_pro() 先更新左右边线与最长白列：
 *        1）行范围：最长白列覆盖到 row=20 时，取 20~底部；否则只取最长白列可覆盖行。
 *        2）列范围：取上述行范围内，左右边线包围的最小矩形。
 */
cv::Rect build_detect_window(const cv::Mat& src_bgr)
{
    if (src_bgr.empty())
    {
        return cv::Rect();
    }

    // 输入必须是原始采集分辨率（320x240）。
    if (src_bgr.cols != UVC_WIDTH || src_bgr.rows != UVC_HEIGHT)
    {
        return cv::Rect();
    }

    // 最长白列是基于 160x120 搜线图的行数，需要换算到 320x240 原图坐标系。
    const int max_len_line = static_cast<int>(max_white.nums);
    if (max_len_line <= 0)
    {
        return cv::Rect();
    }
    const int max_len_raw = max_len_line * IMG_PROC_DOWNSAMPLE;

    //max_white_top_row为距离图像顶端的距离
    const int max_white_top_row = UVC_HEIGHT - max_len_raw;
    int detect_top = max_white_top_row;
    //kDetectTopReachEdgeNums是整幅图像去除最远端的大小，我设置0-20的区域不进行识别，只有120-20的区域可以识别红色区域，进行一个顶端的限幅，最远的检测区域为20
    if (max_len_raw >= kDetectTopReachEdgeNums)
    {
        //最长白列的截止行
        detect_top = kDetectTopRowLimit;
    }
    //对顶端进行一个限幅，防止越界
    detect_top = clamp_int(detect_top, 0, src_bgr.rows - 1);

    //检测窗底部的限幅,最下面是119行
    const int detect_bottom = clamp_int(kDetectBottomRowLimit, 0, src_bgr.rows - 1);
    //最下端的值一定大于最上端
    if (detect_bottom <= detect_top)
    {
        return cv::Rect();
    }
    
    //左右赋一个最大值，因为下面有min函数
    int detect_left = src_bgr.cols - 1;
    int detect_right = 0;
    bool has_valid_edge = false;
    for (int row = detect_top; row <= detect_bottom; ++row)
    {
        // 行坐标转换（预处理存储顺序是倒的），从图像最顶端开始处理，图像最顶端就是边线有效索引的最大值
        const int edge_idx = raw_row_to_line_edge_idx(row);

        // 越界跳过
        if (edge_idx < 0 || edge_idx >= max_len_line)
        {
            continue;
        }

        // 小图边线坐标映射回原图坐标。
        const int left_x = line_x_to_raw_left(static_cast<int>(g_image_info.left[edge_idx].pos[0]));
        const int right_x = line_x_to_raw_right(static_cast<int>(g_image_info.right[edge_idx].pos[0]));
        if (right_x <= left_x)
        {
            continue;
        }

        // 不断更新：得到边界的最大范围
        detect_left = std::min(detect_left, left_x);
        detect_right = std::max(detect_right, right_x);
        has_valid_edge = true;
    }

    //没有有效边界，返回空矩形
    if (!has_valid_edge || detect_right <= detect_left)
    {
        return cv::Rect();
    }
    //得到包含所有边界的最小矩形
    return cv::Rect(detect_left,
                    detect_top,
                    detect_right - detect_left + 1,
                    detect_bottom - detect_top + 1);
}

/**
 * @brief BGR快速粗筛 + HSV精筛
 * @param work_bgr   输入BGR图像
 * @param red_mask   输出二值红色掩码
 * @note  先用整数比较做粗筛，粗筛命中的最小外接区域再做HSV精筛。
 */
void build_red_mask_bgr_coarse_hsv_refine(const cv::Mat& work_bgr, cv::Mat* red_mask)
{
    if (red_mask == nullptr || work_bgr.empty())
    {
        return;
    }

    //红色掩码创建，类似于灰度图像，大小和裁剪后的图片一样
    red_mask->create(work_bgr.rows, work_bgr.cols, CV_8UC1);
    //初始全部设置为0
    red_mask->setTo(cv::Scalar(0));

    //BGR 粗筛阶段的掩码
    g_red_bgr_coarse_mask.create(work_bgr.rows, work_bgr.cols, CV_8UC1);
    //初始全部设置为0，掩码图全黑
    g_red_bgr_coarse_mask.setTo(cv::Scalar(0));

    int min_x = work_bgr.cols;
    int min_y = work_bgr.rows;
    int max_x = -1;
    int max_y = -1;

    //BGR初筛，行扫描整个截取到的图像
    for (int r = 0; r < work_bgr.rows; ++r)
    {
        //彩色图的像素指针
        const cv::Vec3b* src_row = work_bgr.ptr<cv::Vec3b>(r);
        //掩码图的像素指针
        uint8* coarse_row = g_red_bgr_coarse_mask.ptr<uint8>(r);
        //列扫描
        for (int c = 0; c < work_bgr.cols; ++c)
        {
            //三个通道的阈值
            const int b = src_row[c][0];
            const int g = src_row[c][1];
            const int red = src_row[c][2];

            //判断满足红色条件
            const bool coarse_ok = (red >= kBgrRMin) &&
                                ((red - g) >= kBgrRGDiffMin) &&
                                ((red - b) >= kBgrRBDiffMin) &&
                                (red > g * 1.25f) &&
                                (red > b * 1.25f);
            //不是红色跳过
            if (!coarse_ok)
            {
                continue;
            }
            //是红色点，掩码图显示白色，记录所有红色点
            coarse_row[c] = 255;
            //得到红色的矩形范围，包含红色点的范围
            if (c < min_x) min_x = c;
            if (c > max_x) max_x = c;
            if (r < min_y) min_y = r;
            if (r > max_y) max_y = r;
        }
    }

    //一个红色点都没有，退出
    if (max_x < 0 || max_y < 0)
    {
        return;
    }

    //限幅把BGR得到的区域稍微扩大一下，防止有漏的，然后扩大之后进行限幅防止出界
    const int x0 = clamp_int(min_x - kCoarseRoiExpandPx, 0, work_bgr.cols - 1);
    const int y0 = clamp_int(min_y - kCoarseRoiExpandPx, 0, work_bgr.rows - 1);
    const int x1 = clamp_int(max_x + kCoarseRoiExpandPx, 0, work_bgr.cols - 1);
    const int y1 = clamp_int(max_y + kCoarseRoiExpandPx, 0, work_bgr.rows - 1);

    //BGR初筛之后找到红点的最小外接矩形再进入HSV筛选
    //BGR筛选出来的区域不准确，用HSV更加的靠谱
    const cv::Rect coarse_roi(x0, y0, x1 - x0 + 1, y1 - y0 + 1);
    cv::cvtColor(work_bgr(coarse_roi), g_red_hsv_img, cv::COLOR_BGR2HSV);

    //对筛选后的矩形区域进行HSV判断，但是依旧只判断BGR筛选出来的红点
    for (int r = 0; r < coarse_roi.height; ++r)
    {
        //获得hsv图像的行指针
        const cv::Vec3b* hsv_row = g_red_hsv_img.ptr<cv::Vec3b>(r);
        //得到掩码的像素值
        const uint8* coarse_row = g_red_bgr_coarse_mask.ptr<uint8>(y0 + r) + x0;
        //red_mask最后要保存的掩码
        uint8* dst_row = red_mask->ptr<uint8>(y0 + r) + x0;

        for (int c = 0; c < coarse_roi.width; ++c)
        {
            //BGR初筛不是红色
            if (coarse_row[c] == 0)
            {
                continue;
            }

            //进行hsv判断
            const uint8 h = hsv_row[c][0];
            const uint8 s = hsv_row[c][1];
            const uint8 v = hsv_row[c][2];
            const bool h_ok = ((h >= kRedH1Min && h <= kRedH1Max) ||
                               (h >= kRedH2Min && h <= kRedH2Max));
            const bool s_ok = (s >= kRedSMin && s <= kRedSMax);
            const bool v_ok = (v >= kRedVMin && v <= kRedVMax);
            if (h_ok && s_ok && v_ok)
            {
                //保存到red_mask中
                dst_row[c] = 255;
            }
        }
    }
}


/**
 * @brief 按 image_pro() 得到的左右边线，逐行裁剪红色掩码
 * @param red_mask_local 局部掩码（检测窗口坐标系）
 * @param detect_window  检测窗口（全图坐标系）
 * @note  仅保留边线内像素，边线外全部清零，确保色块识别区域严格在赛道内。
 */
void apply_edge_limits_to_mask(cv::Mat* red_mask_local, const cv::Rect& detect_window)
{
    //空指针保护
    if (red_mask_local == nullptr || red_mask_local->empty())
    {
        return;
    }
    //窗口保护
    if (detect_window.width <= 0 || detect_window.height <= 0)
    {
        red_mask_local->setTo(cv::Scalar(0));
        return;
    }
    //最长白列类型转换
    const int max_len_line = static_cast<int>(max_white.nums);
    if (max_len_line <= 0)
    {
        red_mask_local->setTo(cv::Scalar(0));
        return;
    }
    //行扫描红色掩码区域
    for (int r = 0; r < red_mask_local->rows; ++r)
    {
        //得到行指针，指向红色掩码图的一行
        uint8* mask_row = red_mask_local->ptr<uint8>(r);
        //坐标转换，detect_window.y是检测矩形左上角所在的行，+r，就是从上向下进行处理
        const int global_row = detect_window.y + r;
        //得到边线索引，边线索引值越大，对应越远的行
        const int edge_idx = raw_row_to_line_edge_idx(global_row);
        //索引正常限幅
        if (edge_idx < 0 || edge_idx >= max_len_line)
        {
            std::memset(mask_row, 0, static_cast<size_t>(red_mask_local->cols));
            continue;
        }
        //这一行的左边界
        int left_x = line_x_to_raw_left(static_cast<int>(g_image_info.left[edge_idx].pos[0]));
        //这一行的右边界
        int right_x = line_x_to_raw_right(static_cast<int>(g_image_info.right[edge_idx].pos[0]));
        //正常边线保护
        if (right_x <= left_x)
        {
            std::memset(mask_row, 0, static_cast<size_t>(red_mask_local->cols));
            continue;
        }
        //坐标转换，我现在要在和detect_window一样大小的red_mask_local小图中进行处理，把边界left_x大图坐标转成小图坐标
        int left_local = clamp_int(left_x - detect_window.x, 0, red_mask_local->cols - 1);
        int right_local = clamp_int(right_x - detect_window.x, 0, red_mask_local->cols - 1);
        //正常保护
        if (right_local <= left_local)
        {
            //把这行全部给清零了
            std::memset(mask_row, 0, static_cast<size_t>(red_mask_local->cols));
            continue;
        }

        //memset把一段连续的内存，全部设置成同一个值
        if (left_local > 0)
        {
            //把这个mask_row行指针，0-左边线的区域全变黑
            std::memset(mask_row, 0, static_cast<size_t>(left_local));
        }
        if (right_local + 1 < red_mask_local->cols)
        {
            //把这个mask_row行指针，右边线到图像最右边区域全部变黑
            std::memset(mask_row + right_local + 1, 0,
                        static_cast<size_t>(red_mask_local->cols - right_local - 1));
        }
    }
}

/**
 * @brief 运行红块检测流程
 * @param detect_bgr        (BGR)已经裁到检测窗口的小图
 * @param detect_window     小图在整幅图里的位置偏移
 * @param image_w           图像宽度
 * @param image_h           图像高度
 * @return bool             检测是否成功
 */
bool run_red_detection_pass(const cv::Mat& detect_bgr,
                            const cv::Rect& detect_window,
                            int image_w,
                            int image_h,
                            cv::RotatedRect* best_rotated_rect)
{
    if (best_rotated_rect == nullptr || detect_bgr.empty())
    {
        return false;
    }
    if (detect_window.width <= 0 || detect_window.height <= 0 ||
        image_w <= 0 || image_h <= 0)
    {
        return false;
    }
    //一般opencv处理都是矩形连续区域进行处理这样会更快，限制在赛道内进行红色判断也要先找到赛道的最小矩形，然后再把赛道外的给红色给去掉
    
    //根据图像得到红色掩码g_red_mask，BGR+HSV筛选过的
    build_red_mask_bgr_coarse_hsv_refine(detect_bgr, &g_red_mask);
    //对红色掩码进行限制，只识别赛道内的区域
    apply_edge_limits_to_mask(&g_red_mask, detect_window);

    if (g_red_mask.empty())
    {
        return false;
    }
    if (!g_red_full_mask.empty() &&
        detect_window.x >= 0 && detect_window.y >= 0 &&
        detect_window.x + g_red_mask.cols <= g_red_full_mask.cols &&
        detect_window.y + g_red_mask.rows <= g_red_full_mask.rows)
    {
        g_red_mask.copyTo(g_red_full_mask(detect_window));
    }
#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：把局部红色掩码回填成整图调试掩码 =====
    // 把 detect_window 内的局部红色掩码回填到整图坐标系，后面才能按同一个
    // minAreaRect 从掩码图里裁出与原图 patch 一一对应的调试图。
    //确保 detect_window 这个 ROI 在 g_red_debug_mask_full 里是合法的
    if (!g_red_debug_mask_full.empty() &&
        detect_window.x >= 0 && detect_window.y >= 0 &&
        detect_window.x + g_red_mask.cols <= g_red_debug_mask_full.cols &&
        detect_window.y + g_red_mask.rows <= g_red_debug_mask_full.rows)
    {
        //把当前小掩码 g_red_mask 的内容，复制到整图调试掩码的 detect_window 区域里
        //g_red_debug_mask_full是原图像大小
        g_red_mask.copyTo(g_red_debug_mask_full(detect_window));
    }
    // ===== 调试代码结束：把局部红色掩码回填成整图调试掩码 =====
#endif
    //创建一个容器，包含所有红色区域的轮廓
    std::vector<std::vector<cv::Point>> contours;
    //根据得到的黑白掩码图，找到所有白色区域的轮廓，保存到contours中，cv::RETR_EXTERNAL只找最外层轮廓，cv::CHAIN_APPROX_SIMPLE压缩轮廓点，只保留拐角，节省内存、速度更快
    cv::findContours(g_red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //最好的分数
    float best_score = -1.0f;
    //保存当前最优的旋转矩形框
    cv::RotatedRect local_best_rotated;
    const float bottom_denom = static_cast<float>(std::max(1, image_h - 1));
    const float detect_area_denom = static_cast<float>(std::max(1, detect_window.width * detect_window.height));
    const cv::Rect image_rect(0, 0, image_w, image_h);

    //遍历所有区域，找到得分最高的矩形框
    for (const auto& contour : contours)
    {
        //计算轮廓面积
        const float area = static_cast<float>(cv::contourArea(contour));
        //面积小的直接过滤了
        if (area < static_cast<float>(kMinRedAreaPx))
        {
            continue;
        }

        //给轮廓画一个最小的正矩形框，不管轮廓是什么形状（圆、三角、不规则）都给它包一个正的框
        const cv::Rect local_rect = cv::boundingRect(contour);
        //去除没有宽度、没有高度的矩形，避免异常轮廓
        if (local_rect.width <= 0 || local_rect.height <= 0)
        {
            continue;
        }

        //能包裹住轮廓的旋转矩形,这个是真正的判断旋转的图片，因为跑车的以后图片在弯道一定是斜着看的
        //它会尽量包住整个轮廓，但有时候会有这些问题：
        //轮廓粘连了，框被拉得很大
        //红色噪声把矩形某一边拖长了
        //轮廓不干净，导致旋转框尺寸异常
        const cv::RotatedRect local_rot_rect = cv::minAreaRect(contour);
        const float side_w = local_rot_rect.size.width;
        const float side_h = local_rot_rect.size.height;
        if (side_w <= 1e-3f || side_h <= 1e-3f)
        {
            continue;
        }

        //计算轮廓周长，周长过小直接过滤
        //轮廓是由一系列的点组成的,C++中就是Point,传入true说明是一个闭合的轮廓,轮廓周长的计算方法是,把每两个相邻点之间的距离加起来
        //每一段的长度用的是欧几里得距离sqrt((x2 - x1)^2 + (y2 - y1)^2)
        //周长不能过小,这个可以省略,因为面积最小是10周长不可能小于1
        //这里保留该判断作为防御式检查，避免影响后续计算
        const double perimeter = cv::arcLength(contour, true);
        if (perimeter < static_cast<double>(kMinContourPerimeterPx))
        {
            continue;
        }

        const float rotated_area = side_w * side_h;
        if (rotated_area <= 1e-3f)
        {
            continue;
        }
        const float fill_ratio = clamp_float(area / rotated_area, 0.0f, 1.0f);
        if (fill_ratio < kMinRectFillRatio)
        {
            continue;
        }

        // 转回原图坐标系（detect_window 只是局部窗口）。
        cv::RotatedRect rotated_in_image = local_rot_rect;
        rotated_in_image.center.x += static_cast<float>(detect_window.x);
        rotated_in_image.center.y += static_cast<float>(detect_window.y);
        cv::Rect rect_in_image = rotated_in_image.boundingRect();
        //把 rect_in_image 裁剪到 image_rect 里面,防止越界
        rect_in_image &= image_rect;
        if (rect_in_image.width <= 0 || rect_in_image.height <= 0)
        {
            continue;
        }
        // 打分规则：中心点y越下 + 最下点y越下 + 面积 + 填充率，分数越高越优。
        const float center_y_score = clamp_float(rotated_in_image.center.y / bottom_denom, 0.0f, 1.0f);
        cv::Point2f rot_points[4];
        rotated_in_image.points(rot_points);
        float bottom_y = rot_points[0].y;
        for (int i = 1; i < 4; ++i)
        {
            bottom_y = std::max(bottom_y, rot_points[i].y);
        }
        const float bottom_y_score = clamp_float(bottom_y / bottom_denom, 0.0f, 1.0f);
        const float area_score = clamp_float(rotated_area / detect_area_denom, 0.0f, 1.0f);
        const float score = kScoreWeightCenterY * center_y_score +
                            kScoreWeightBottomY * bottom_y_score +
                            kScoreWeightArea * area_score +
                            kScoreWeightFill * fill_ratio;
        if (score > best_score)
        {
            best_score = score;
            local_best_rotated = rotated_in_image;
        }
    }

    if (best_score <= 0.0f)
    {
        return false;
    }

    *best_rotated_rect = local_best_rotated;
#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：缓存当前最优红块旋转矩形的几何信息 =====
    // 保存当前最终选中的最优红块旋转矩形几何信息，便于 TCP 之外的后续扩展。
    g_red_debug_rect_valid = true;
    local_best_rotated.points(g_red_debug_rect_points);
    // ===== 调试代码结束：缓存当前最优红块旋转矩形的几何信息 =====
#endif
    return true;
}

/**
 * @brief 在输入图像中寻找最优红色矩形候选
 * @param src_bgr           输入BGR图像
 * @param best_rotated_rect 输出最优旋转矩形
 * @return bool             true=找到，false=未找到
 * @note  筛选条件包含：面积、宽高比、填充率。可减少赛道背景误检。
 */
bool find_best_red_rect(const cv::Mat& src_bgr,
                        cv::RotatedRect* best_rotated_rect)
{
    //空指针和图像保护
    if (best_rotated_rect == nullptr || src_bgr.empty())
    {
        RED_ROI_ERR("function input error!");
        return false;
    }

    // ===== 调试代码开始：每帧检测前先清空 red_roi 调试缓存 =====
    // 每帧进入红块查找先清空调试缓存，避免检测窗无效等早退时残留上一帧调试图。
    reset_red_debug_cache(src_bgr.cols, src_bgr.rows);
    // ===== 调试代码结束：每帧检测前先清空 red_roi 调试缓存 =====

    //建立检测窗口
    //detect_window是包含这一帧搜索到的左右边界的最小矩形，最远端如果最长白列没搜索到20行，就是最长白列所在行，如果搜索到了第20行，那么最远端就是20行
    const cv::Rect detect_window = build_detect_window(src_bgr);
    if (detect_window.width <= 0 || detect_window.height <= 0)
    {
        RED_ROI_ERR("detect_window invalid!");
        return false;
    }

    // 根据窗口detect_window初步处理图片，得到裁剪图
    const cv::Mat detect_bgr = src_bgr(detect_window);

    cv::RotatedRect best_rotated_rect_local;
    
    const bool found = run_red_detection_pass(detect_bgr,
                                              detect_window,
                                              src_bgr.cols,
                                              src_bgr.rows,
                                              &best_rotated_rect_local);
    if (!found)
    {
        return false;
    }

    *best_rotated_rect = best_rotated_rect_local;
    return true;
}

/**
 * @brief 根据红色倾斜矩形计算其上方旋转ROI
 * @param red_rect   红色最小外接旋转矩形
 * @param image_w    图像宽
 * @param image_h    图像高
 * @param roi_rect   输出ROI外接矩形
 * @param roi_points 输出ROI四角点（tl,tr,bl,br）
 * @param roi_angle_deg 输出ROI旋转角
 * @return bool      true=构造成功，false=构造失败
 * @note  规则：ROI沿红块法线方向上移，方向与红块长边保持一致
 */
bool build_rotated_roi_above_red(const cv::Mat& src_bgr,
                                 const cv::RotatedRect& red_rect,
                                 int image_w,
                                 int image_h,
                                 cv::Rect* roi_rect,
                                 cv::Point2f* roi_points,
                                 float* roi_angle_deg)
{
    if (roi_rect == nullptr || roi_points == nullptr || src_bgr.empty() || image_w <= 0 || image_h <= 0)
    {
        return false;
    }

    //得到旋转矩形的四个顶点
    cv::Point2f red_points[4];
    red_rect.points(red_points);

    struct EdgeInfo
    {
        cv::Point2f p0;
        cv::Point2f p1;
        cv::Point2f vec;
        cv::Point2f mid;
        float len;
    };
    EdgeInfo edges[4];
    float long_side = 0.0f;
    float short_side = std::numeric_limits<float>::max();
    for (int i = 0; i < 4; ++i)
    {
        //得到旋转矩形四条边的信息，并且得到最大最小边的长度
        edges[i].p0 = red_points[i];
        edges[i].p1 = red_points[(i + 1) % 4];//四条边的起点和终点
        edges[i].vec = edges[i].p1 - edges[i].p0; //四个点的向量
        edges[i].len = vec2_length(edges[i].vec);//四条边的长度
        edges[i].mid = (edges[i].p0 + edges[i].p1) * 0.5f;//四条边的中点
        long_side = std::max(long_side, edges[i].len);
        short_side = std::min(short_side, edges[i].len);
    }
    if (long_side <= 1e-3f || short_side <= 1e-3f)
    {
        return false;
    }

    //最大最小边限制
    float max_long_limit = kMaxRedLongSidePx;
    float max_short_limit = kMaxRedShortSidePx;
    //得到根据y轴自适应的最大边
    get_adaptive_max_side_limits(red_rect.center.y, image_h, &max_long_limit, &max_short_limit);
    //判断矩形是否被干扰
    const bool merged_like = (long_side > max_long_limit) ||
                             (short_side > max_short_limit);

    // 先固定：从长边中选择最靠下的一条，作为 axis_x 参考边。
    int selected_edge_idx = -1;
    float best_y = -1.0f;
    //float计算出来的两条长边不一定完全相等，给一个容差。
    const float long_edge_min = long_side * 0.85f;
    //选最靠下的一条长边
    for (int i = 0; i < 4; ++i)
    {
        if (edges[i].len < long_edge_min)
        {
            continue;
        }
        if (edges[i].mid.y > best_y)
        {
            best_y = edges[i].mid.y;
            selected_edge_idx = i;
        }
    }

    //如果没选出来，就失败，不可能触发
    if (selected_edge_idx < 0)
    {
        RED_ROI_ERR("No roi reference edge!");
        return false;
    }

    cv::Point2f axis_x;
    //用选中的边建立局部横向轴 axis_x把选中的参考边方向归一化，得到 axis_x。
    // 这里可以把 axis_x 理解成 ROI 的“横向轴”。
    if (!normalize_vec2(edges[selected_edge_idx].vec, &axis_x))
    {
        RED_ROI_ERR("axis_x normalize fail!");
        return false;
    }
    //优先让 axis_x 朝图像右边,如果几乎竖直，就让它朝下
    if (axis_x.x < 0.0f || (std::abs(axis_x.x) < 1e-4f && axis_x.y < 0.0f))
    {
        axis_x = -axis_x;
    }

    //根据 axis_x 构造朝下方向 axis_down,尽量朝图像下方
    //图像下方240->0
    cv::Point2f axis_down(-axis_x.y, axis_x.x);
    if (axis_down.y < 0.0f)
    {
        axis_down = -axis_down;
    }

    // 非粘连块：直接使用长边最靠下方向（axis_x）作为 ROI 底边。
    // 粘连块：再用中线局部方向在 axis_x / axis_down 之间二选一。
    cv::Point2f roi_bottom_axis = axis_x;
    if (merged_like)
    {
        // 红块可能粘连了，这时只看红块自身方向不一定靠谱，所以再借助赛道中线方向 midline_dir_raw 做一次判断
        const int center_y_raw = clamp_int(static_cast<int>(std::lround(red_rect.center.y)), 0, image_h - 1);
        cv::Point2f midline_dir_raw;
        if (estimate_midline_local_direction_raw(center_y_raw, image_h, &midline_dir_raw))
        {
            const float axis_down_similarity = std::abs(midline_dir_raw.dot(axis_down));
            const float axis_x_similarity = std::abs(midline_dir_raw.dot(axis_x));
            // 中线更接近 axis_x 时，底边改用 axis_down；否则（含接近或更接近 axis_down）用 axis_x。
            if (axis_x_similarity > axis_down_similarity)
            {
                roi_bottom_axis = axis_down;
            }
        }
    }

#if ENABLE_VISION_DEBUG
    // ===== 调试代码开始：缓存 build_rotated_roi_above_red 选中的最靠下长边 =====
    g_red_debug_selected_edge_valid = true;
    g_red_debug_selected_edge_points[0] = edges[selected_edge_idx].p0;
    g_red_debug_selected_edge_points[1] = edges[selected_edge_idx].p1;
    // ===== 调试代码结束：缓存 build_rotated_roi_above_red 选中的最靠下长边 =====
#endif

    // 从“底边中点指向红块中心”的方向确定 ROI 向上法向，保证构造方向稳定。
    cv::Point2f roi_up_axis(-roi_bottom_axis.y, roi_bottom_axis.x);
    const cv::Point2f to_center = red_rect.center - edges[selected_edge_idx].mid;
    if (roi_up_axis.dot(to_center) < 0.0f)
    {
        roi_up_axis = -roi_up_axis;
    }

    const cv::Point2f bottom_center = edges[selected_edge_idx].mid;

    // 前四步属于正式 ROI 链路的一部分，这里每帧都执行：
    // 1) 使用 roi_bottom_axis / roi_up_axis 作为局部坐标系
    // 2) 从整图红色掩码中提取最优旋转矩形对应的局部掩码
    // 3) 在局部图中逐行找最长连续红段
    // 4) 从下往上寻找最下方稳定红色宽带
    cv::Mat local_mask_patch;
    std::vector<LocalRowRunInfo> row_infos;
    LocalBandInfo stable_band_info;
    LocalPatchTransform local_patch_tf;
    MarkerBodyRowsResult body_rows_result;
    MarkerBottomEdgeFitResult bottom_edge_result;
    MarkerLeftEdgeFitResult left_edge_result;
    MarkerRightEdgeFitResult right_edge_result;
    MarkerParallelogramResult marker_quad_result;
    const bool local_analysis_ok = build_local_mask_analysis_patch(g_red_full_mask,
                                                                   red_rect,
                                                                   bottom_center,
                                                                   roi_bottom_axis,
                                                                   roi_up_axis,
                                                                   &local_mask_patch,
                                                                   &row_infos,
                                                                   &stable_band_info,
                                                                   &local_patch_tf,
                                                                   nullptr);
    // 第五步：先从局部图里判断“哪些行是真正可信的红块主体”。
    const bool body_rows_ok = local_analysis_ok &&
                              estimate_marker_body_rows_from_local_patch(row_infos,
                                                                        local_mask_patch.cols,
                                                                        local_mask_patch.rows,
                                                                        stable_band_info,
                                                                        &body_rows_result);
    // 第六步：只基于主体行组恢复主体下包络线。
    const bool bottom_edge_ok = body_rows_ok &&
                                fit_marker_bottom_edge_from_body_rows(local_mask_patch,
                                                                      body_rows_result,
                                                                      &bottom_edge_result);
    // 第七步：只基于主体中下部恢复主体左边界方向。
    const bool left_edge_ok = body_rows_ok &&
                              fit_marker_left_edge_from_body_rows(row_infos,
                                                                  body_rows_result,
                                                                  &left_edge_result);
    // 第七步补充：只基于主体中下部恢复主体右边界方向。
    const bool right_edge_ok = body_rows_ok &&
                               fit_marker_right_edge_from_body_rows(row_infos,
                                                                    body_rows_result,
                                                                    &right_edge_result);
    // 第九步：用底边、左边、右边和主体顶部参考行直接重建红块主体四边形。
    const bool marker_quad_ok = bottom_edge_ok && left_edge_ok && right_edge_ok &&
                                build_marker_quad_from_body_edges(body_rows_result,
                                                                  bottom_edge_result,
                                                                  left_edge_result,
                                                                  right_edge_result,
                                                                  &marker_quad_result);
    MarkerParallelogramResult marker_quad_result_raw;
    //把第九步得到的局部主体四边形恢复到原图坐标
    const bool marker_quad_raw_ok = marker_quad_ok &&
                                    map_marker_quad_to_raw(marker_quad_result,
                                                           local_patch_tf,
                                                           &marker_quad_result_raw);
    if (!(bottom_edge_ok && left_edge_ok && marker_quad_ok && marker_quad_raw_ok))
    {
        // 第 10 步前置失败日志：
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }

    cv::Point2f roi_bl;
    cv::Point2f roi_br;
    cv::Point2f left_up_dir;
    cv::Point2f right_up_dir;
    cv::Point2f roi_bottom_dir;
    float roi_width = 0.0f;
    float side_scale_ratio = 1.0f;
    //基于第九步主体顶部构造第十步 ROI 的基础边
    if (!build_roi_base_from_marker_top(marker_quad_result_raw,
                                        kStep10RoiSafetyOffsetPx,
                                        kStep10RoiHorizontalExpandPx,
                                        &roi_bl,
                                        &roi_br,
                                        &left_up_dir,
                                        &right_up_dir,
                                        &roi_bottom_dir,
                                        &roi_width,
                                        &side_scale_ratio))
    {
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_patch.release();
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }

    float roi_h_min = 0.0f;
    float roi_h_max = 0.0f;
    float roi_h_final = 0.0f;
    std::vector<float> white_ratios;
    Step10WhiteThreshold step10_white_th;
    //参考 get_reference_point 的思路，为第十步估计当前帧白色阈值
    if (!estimate_step10_white_threshold_from_track_bottom(src_bgr, &step10_white_th))
    {
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_patch.release();
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }
    //在 0.5W~0.8W 范围内搜索最终 ROI 高度
    if (!estimate_final_roi_height_from_white_scan(src_bgr,
                                                   roi_bl,
                                                   roi_br,
                                                   left_up_dir,
                                                   right_up_dir,
                                                   side_scale_ratio,
                                                   roi_width,
                                                   step10_white_th,
                                                   &roi_h_min,
                                                   &roi_h_max,
                                                   &roi_h_final,
                                                   &white_ratios))
    {
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_patch.release();
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }
    //最大候选高度 H_max”先构造一份四边形
    cv::Point2f max_roi_quad[4];
    if (!build_final_roi_quad(roi_bl,
                              roi_br,
                              left_up_dir,
                              right_up_dir,
                              side_scale_ratio,
                              roi_h_max,
                              max_roi_quad))
    {
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_patch.release();
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }
    //roi_h_final 是白色扫描后最终确定的高度,得到最终ROI四边形结果
    if (!build_final_roi_quad(roi_bl,
                              roi_br,
                              left_up_dir,
                              right_up_dir,
                              side_scale_ratio,
                              roi_h_final,
                              roi_points))
    {
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_patch.release();
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }

#if ENABLE_VISION_DEBUG
    g_red_debug_bottom_axis_valid = true;
    g_red_debug_bottom_axis_points[0] = roi_bl;
    g_red_debug_bottom_axis_points[1] = roi_br;
#endif

#if ENABLE_VISION_DEBUG
    cv::Mat left_step10_vis_patch;
    build_step10_overlay_visualization(src_bgr,
                                       marker_quad_result_raw,
                                       roi_bl,
                                       roi_br,
                                       left_up_dir,
                                       right_up_dir,
                                       side_scale_ratio,
                                       roi_h_min,
                                       roi_h_max,
                                       roi_h_final,
                                       &left_step10_vis_patch);
    if (!left_step10_vis_patch.empty())
    {
        g_red_debug_min_rect_patch = left_step10_vis_patch.clone();
    }
    else
    {
        g_red_debug_min_rect_patch.release();
    }

    cv::Mat final_roi_48x48_bgr;
    if (extract_perspective_roi_patch_impl(src_bgr, roi_points, 48, 48, &final_roi_48x48_bgr) &&
        !final_roi_48x48_bgr.empty())
    {
        g_red_debug_min_rect_mask_patch = final_roi_48x48_bgr.clone();
    }
    else
    {
        g_red_debug_min_rect_mask_patch.release();
    }
#endif

    //检查ROI是否正常
    if (!validate_final_roi_quad(roi_points, image_w, image_h))
    {
#if ENABLE_VISION_DEBUG
        g_red_debug_min_rect_patch.release();
        g_red_debug_min_rect_mask_patch.release();
#endif
        return false;
    }

    *roi_rect = build_bbox_from_points(roi_points, image_w, image_h);
    if (roi_rect->width <= 0 || roi_rect->height <= 0)
    {
        return false;
    }

    if (roi_angle_deg != nullptr)
    {
        *roi_angle_deg = std::atan2(roi_bottom_dir.y, roi_bottom_dir.x) * 180.0f / static_cast<float>(CV_PI);
    }

    return true;
}

/**
 * @brief 将斜的矩阵转正
 * @param src_bgr 图像
 * @param roi_points 旋转ROI的四角点（tl, tr, bl, br）
 * @param roi_width 输出ROI宽度
 * @param roi_height 输出ROI高度
 * @return bool true=提取成功 false=失败
 */
bool extract_rotated_roi_patch_impl(const cv::Mat& src_bgr,
                                    const cv::Point2f* roi_points,
                                    int roi_width,
                                    int roi_height,
                                    cv::Mat* roi_img)
{
    //保护
    if (roi_img == nullptr || roi_points == nullptr ||
        roi_width <= 0 || roi_height <= 0 || src_bgr.empty())
    {
        return false;
    }

    //仿射变换
    //roi_points[0] 要去目标图左上角
    //roi_points[1] 要去目标图右上角
    //roi_points[2] 要去目标图左下角
    //一一对应得到的图像就是正的
    const cv::Point2f src_tri[3] = {roi_points[0], roi_points[1], roi_points[2]};
    //定义目标三角形 dst_tri
    const cv::Point2f dst_tri[3] = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(roi_width - 1), 0.0f),
        cv::Point2f(0.0f, static_cast<float>(roi_height - 1))};

    //求仿射变换矩阵
    const cv::Mat affine_mat = cv::getAffineTransform(src_tri, dst_tri);
    //执行图像变换src_bgr->roi_img
    cv::warpAffine(src_bgr,
                   *roi_img,
                   affine_mat,
                   cv::Size(roi_width, roi_height),
                   cv::INTER_LINEAR,
                   cv::BORDER_CONSTANT,
                   cv::Scalar(0, 0, 0));
    return !roi_img->empty();
}

/**
 * @brief 使用四点透视变换直接提取目标 ROI
 * @param src_bgr    原始 BGR 图
 * @param roi_points 四点顺序固定为 tl,tr,bl,br
 * @param roi_width  输出宽度
 * @param roi_height 输出高度
 * @param roi_img    输出图像
 * @return bool      true=提取成功 false=失败
 * @note  调用 getPerspectiveTransform 时，需要重排成 tl,tr,br,bl。
 */
bool extract_perspective_roi_patch_impl(const cv::Mat& src_bgr,
                                        const cv::Point2f* roi_points,
                                        int roi_width,
                                        int roi_height,
                                        cv::Mat* roi_img)
{
    if (roi_img == nullptr || roi_points == nullptr ||
        roi_width <= 0 || roi_height <= 0 || src_bgr.empty())
    {
        return false;
    }

    const cv::Point2f src_quad[4] = {
        roi_points[0],
        roi_points[1],
        roi_points[3],
        roi_points[2]};
    const cv::Point2f dst_quad[4] = {
        cv::Point2f(0.0f, 0.0f),
        cv::Point2f(static_cast<float>(roi_width - 1), 0.0f),
        cv::Point2f(static_cast<float>(roi_width - 1), static_cast<float>(roi_height - 1)),
        cv::Point2f(0.0f, static_cast<float>(roi_height - 1))};

    const cv::Mat perspective_mat = cv::getPerspectiveTransform(src_quad, dst_quad);
    cv::warpPerspective(src_bgr,
                        *roi_img,
                        perspective_mat,
                        cv::Size(roi_width, roi_height),
                        cv::INTER_LINEAR,
                        cv::BORDER_CONSTANT,
                        cv::Scalar(0, 0, 0));
    return !roi_img->empty();
}

}

bool extract_rotated_roi_patch(const cv::Mat& src_bgr,
                               const cv::Point2f* roi_points,
                               int roi_width,
                               int roi_height,
                               cv::Mat* roi_img)
{
    return extract_perspective_roi_patch_impl(src_bgr, roi_points, roi_width, roi_height, roi_img);
}

bool extract_rotated_roi_patch(const cv::Mat& src_bgr,
                               const cv::Point2f* roi_points,
                               int roi_size,
                               cv::Mat* roi_img)
{
    const int roi_width = roi_size;
    const int roi_height = std::max(1, static_cast<int>(std::lround(roi_size * kRoiHeightScaleByLongSide)));
    return extract_perspective_roi_patch_impl(src_bgr, roi_points, roi_width, roi_height, roi_img);
}

// ===== 调试代码开始：按键控制的 ROI 采图/录像工具状态 =====
namespace
{
bool g_roi_capture_enabled = false;   // ROI 采集开关
int g_roi_capture_frame_count = 0;    // 有效ROI帧计数
uint32_t g_roi_capture_save_index = 0;// 保存序号
bool g_key2_last_pressed = false;     // KEY2 上一拍按下状态
std::chrono::steady_clock::time_point g_key2_last_toggle_tp = std::chrono::steady_clock::now();//上一次切换 ROI 采集开关的时间点

//增加ROI调试录制视频功能
constexpr int kDebugVideoFps = 10; // 调试视频的帧率是 10 FPS
constexpr int kDebugVideoDurationSec = 8; //录 10 秒
bool g_roi_video_recording = false; //整个视频录制状态机的核心开关
bool g_key2_video_last_pressed = false; //按键边沿检测变量
uint32_t g_roi_video_save_index = 0; //视频文件编号
int g_roi_video_saved_frame_count = 0; //帧计数变量,已经写进视频文件里的帧数。
int g_roi_video_valid_frame_count = 0; //录制过程中，有多少帧是“有效帧”
int g_roi_video_invalid_frame_count = 0; //有多少帧是“无效帧”
bool g_roi_video_prev_frame_valid = false; //上一帧是否为有效帧
std::chrono::steady_clock::time_point g_key2_video_last_toggle_tp = std::chrono::steady_clock::now(); //按键防抖/触发时间
std::chrono::steady_clock::time_point g_roi_video_record_start_tp = std::chrono::steady_clock::now(); //视频录制开始时间
std::chrono::steady_clock::time_point g_roi_video_next_frame_tp = std::chrono::steady_clock::now(); //下一帧应写入的时间点
cv::VideoWriter g_roi_video_writer; //OpenCV 视频写入器
char g_roi_video_file_path[128] = {0}; //当前视频文件路径缓冲区

/**
 * @brief 确保当前工作目录下存在 picture 目录
 * @return bool true=目录可用 false=创建失败
 * @note  供 ROI 调试图片采集功能保存 BMP/JPG 时使用。
 */
bool ensure_picture_dir_exists()
{
    const char* dir = "picture";//定义目录名
    //定义一个 stat 结构体变量，用来接收文件/目录的信息
    struct stat st = {};
    //检查 picture 这个路径是否已经存在
    if (stat(dir, &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }
    //如果前面的 stat() 失败，说明路径不存在，那么就尝试创建目录
    if (mkdir(dir, 0755) == 0 || errno == EEXIST)
    {
        return true;
    }
    //创建失败了，打印错误日志
    RED_ROI_ERR("mkdir picture failed, errno=%d", errno);
    return false;
}
/**
 * @brief 确保当前工作目录下存在 video 目录
 * @return bool true=目录可用 false=创建失败
 * @note  供 ROI 调试录屏功能输出视频文件时使用。
 */
bool ensure_video_dir_exists()
{
    const char* dir = "video";
    struct stat st = {};
    if (stat(dir, &st) == 0)
    {
        return S_ISDIR(st.st_mode);
    }
    // 如果不存在，就尝试创建目录
    if (mkdir(dir, 0755) == 0 || errno == EEXIST)
    {
        return true;
    }
    RED_ROI_ERR("mkdir video failed, errno=%d", errno);
    return false;
}

/**
 * @brief 用 KEY2 按键切换 ROI 图片采集开关
 * @note  采用上升沿检测和 150ms 去抖，避免一次按下触发多次切换。
 */
void update_roi_capture_toggle_by_key2()
{
    // KEY2：低电平按下（按一下开，再按一下关）
    const bool key2_pressed = (key_2.get_level() != KEY_RELEASE_LEVEL);
    //拿当前时刻，用于计算离上次触发过去多久
    const auto now_tp = std::chrono::steady_clock::now();
    //检测到按键从“松开”变成“按下”的那一刻
    if (key2_pressed && !g_key2_last_pressed)
    {
        // 150ms 去抖，避免一次按下触发多次切换
        const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - g_key2_last_toggle_tp).count();
        if (dt_ms >= 150)
        {
            //切换 ROI 采集开关
            g_roi_capture_enabled = !g_roi_capture_enabled;
            //每次重新切换开关时，把累计帧数归零
            g_roi_capture_frame_count = 0;
            //更新时间戳
            g_key2_last_toggle_tp = now_tp;
            std::printf("ROI capture %s by KEY2\r\n", g_roi_capture_enabled ? "ON" : "OFF");
        }
    }
    g_key2_last_pressed = key2_pressed;
}
// ===== 调试代码结束：按键控制的 ROI 采图/录像工具状态 =====
}

/**
 * @brief 获取 minAreaRect 拉正后的原图 patch
 * @param rect_img 输出图像
 * @return bool    true=当前帧有可用 patch false=缓存为空
 * @note  该接口给 car_debug/TCP 左图使用。
 *        图像内容是“最优红块旋转矩形在原图里实际框住并拉正后的内容”。
 */
bool red_roi_get_debug_min_rect_patch(cv::Mat* rect_img)
{
#if !ENABLE_VISION_DEBUG
    if (rect_img != nullptr)
    {
        rect_img->release();
    }
    return false;
#else
    // ===== 调试代码开始：导出 minAreaRect 拉正后的原图 patch 给上层看图 =====
    if (rect_img == nullptr)
    {
        return false;
    }
    if (g_red_debug_min_rect_patch.empty())
    {
        rect_img->release();
        return false;
    }
    *rect_img = g_red_debug_min_rect_patch.clone();
    // ===== 调试代码结束：导出 minAreaRect 拉正后的原图 patch 给上层看图 =====
    return true;
#endif
}

/**
 * @brief 获取右侧调试 patch
 * @param mask_img 输出图像
 * @return bool    true=当前帧有可用 patch false=缓存为空
 * @note  该接口给 car_debug/TCP 右图使用。
 *        默认会优先输出“局部坐标系下的红色掩码分析图”；
 *        如果当前帧还没形成分析图，则退化为原来的掩码 patch。
 */
bool red_roi_get_debug_min_rect_mask_patch(cv::Mat* mask_img)
{
#if !ENABLE_VISION_DEBUG
    if (mask_img != nullptr)
    {
        mask_img->release();
    }
    return false;
#else
    // ===== 调试代码开始：导出 minAreaRect 拉正后的红色掩码 patch 给上层看图 =====
    if (mask_img == nullptr)
    {
        return false;
    }
    if (g_red_debug_min_rect_mask_patch.empty())
    {
        mask_img->release();
        return false;
    }
    *mask_img = g_red_debug_min_rect_mask_patch.clone();
    // ===== 调试代码结束：导出 minAreaRect 拉正后的红色掩码 patch 给上层看图 =====
    return true;
#endif
}

bool red_roi_get_debug_info(RedRoiDebugInfo* debug_info)
{
    if (debug_info == nullptr)
    {
        return false;
    }

    *debug_info = RedRoiDebugInfo{};

#if !ENABLE_VISION_DEBUG
    return false;
#else
    debug_info->mask_valid = !g_red_debug_mask_full.empty();
    if (debug_info->mask_valid)
    {
        debug_info->red_mask_full = g_red_debug_mask_full.clone();
    }

    debug_info->rect_valid = g_red_debug_rect_valid;
    if (debug_info->rect_valid)
    {
        cv::Point2f center(0.0f, 0.0f);
        float long_side = 0.0f;
        float short_side = std::numeric_limits<float>::max();

        for (int i = 0; i < 4; ++i)
        {
            debug_info->rect_points[i] = g_red_debug_rect_points[i];
            center += g_red_debug_rect_points[i];

            const float edge_len =
                vec2_length(g_red_debug_rect_points[(i + 1) % 4] - g_red_debug_rect_points[i]);
            long_side = std::max(long_side, edge_len);
            short_side = std::min(short_side, edge_len);
        }

        debug_info->rect_center = center * 0.25f;
        debug_info->rect_long_side_px = long_side;
        debug_info->rect_short_side_px =
            (short_side == std::numeric_limits<float>::max()) ? 0.0f : short_side;
    }

    debug_info->roi_valid = g_red_debug_roi_valid;
    if (debug_info->roi_valid)
    {
        for (int i = 0; i < 4; ++i)
        {
            debug_info->roi_points[i] = g_red_debug_roi_points[i];
        }
    }

    return debug_info->mask_valid || debug_info->rect_valid || debug_info->roi_valid;
#endif
}

/**
 * @brief 获取当前帧最优旋转矩形四个角点
 * @param rect_points 输出 4 个角点
 * @return bool true=当前帧有效 false=当前帧无可用矩形
 */
bool red_roi_get_debug_rect_points(cv::Point2f* rect_points)
{
#if !ENABLE_VISION_DEBUG
    (void)rect_points;
    return false;
#else
    // ===== 调试代码开始：导出最优旋转矩形四个角点给原图叠加绘制 =====
    if (rect_points == nullptr || !g_red_debug_rect_valid)
    {
        return false;
    }
    for (int i = 0; i < 4; ++i)
    {
        rect_points[i] = g_red_debug_rect_points[i];
    }
    // ===== 调试代码结束：导出最优旋转矩形四个角点给原图叠加绘制 =====
    return true;
#endif
}

/**
 * @brief 获取 build_rotated_roi_above_red 选中的最靠下长边
 * @param edge_points 输出 2 个端点
 * @return bool true=当前帧有效 false=当前帧无可用长边
 */
bool red_roi_get_debug_selected_edge_points(cv::Point2f* edge_points)
{
#if !ENABLE_VISION_DEBUG
    (void)edge_points;
    return false;
#else
    // ===== 调试代码开始：导出被选中的最靠下长边两个端点给原图叠加绘制 =====
    if (edge_points == nullptr || !g_red_debug_selected_edge_valid)
    {
        return false;
    }
    edge_points[0] = g_red_debug_selected_edge_points[0];
    edge_points[1] = g_red_debug_selected_edge_points[1];
    // ===== 调试代码结束：导出被选中的最靠下长边两个端点给原图叠加绘制 =====
    return true;
#endif
}

/**
 * @brief 获取最终 roi_bottom_axis 参考线两个端点
 * @param axis_points 输出 2 个端点
 * @return bool true=当前帧有效 false=当前帧无可用参考线
 */
bool red_roi_get_debug_bottom_axis_points(cv::Point2f* axis_points)
{
#if !ENABLE_VISION_DEBUG
    (void)axis_points;
    return false;
#else
    // ===== 调试代码开始：导出最终 roi_bottom_axis 参考线两个端点给原图叠加绘制 =====
    if (axis_points == nullptr || !g_red_debug_bottom_axis_valid)
    {
        return false;
    }
    axis_points[0] = g_red_debug_bottom_axis_points[0];
    axis_points[1] = g_red_debug_bottom_axis_points[1];
    // ===== 调试代码结束：导出最终 roi_bottom_axis 参考线两个端点给原图叠加绘制 =====
    return true;
#endif
}

/**
 * @brief 远距离找到红色色块，并做好标志，近距离的时候进行ROI区域的预测
 * @param src_bgr 输入图像
 * @param roi_result 输出结果
 * @return 0没有找到红色区域，1找到红色区域没有找到roi，2 红块在近处，ROI构造失败,3 ROI构造成功
 */
/*
输入保护
-> 清空上一帧结果
-> 在赛道内找最优红块
-> 如果没找到红块，返回0
-> 红块找到了，先记录 red_found=true
-> 如果红块还不够近，返回1
-> 红块够近了，根据红块姿态预测其上方ROI
-> 红块在近处，ROI构造失败，返回2
-> ROI构造成功，写入结果，返回3
*/
int red_roi_find(const cv::Mat& src_bgr, RedRoiResult* roi_result)
{
    //正常保护
    if (roi_result == nullptr)
    {
        RED_ROI_ERR("roi_result pointer is null.");
        return -1;
    }
    if (src_bgr.empty())
    {
        RED_ROI_ERR("input image is empty.");
        return -1;
    }

    //刷新roi_result，清除上一帧残留
    reset_roi_result(roi_result);

    //搜寻到红色区域
    cv::RotatedRect red_rotated_rect;
    //得到最优旋转矩形
    bool find_red = find_best_red_rect(src_bgr, &red_rotated_rect);
    if (!find_red)
    {
        return 0;
    }

    roi_result->red_found = true;//找到了红色矩形

    // 只根据红块中心高度判断是否进入“开始预测 ROI”的距离范围。
    // 这里不再使用旋转矩形最下方角点作为触发条件，
    // 避免矩形倾斜、粘连或单个下角被拖低时，过早进入 ROI 预测。
    const float center_y = red_rotated_rect.center.y;
    // 只有当红块中心足够靠近图像下部时，才开始执行后续 ROI 预测。
    if (center_y <= static_cast<float>(kRoiPredictStartCenterRow))
    {
        return 1;
    }

    cv::Rect roi_rect;
    cv::Point2f roi_points[4];
    float roi_angle = 0.0f;
    const bool roi_build_ok = build_rotated_roi_above_red(src_bgr,
                                                          red_rotated_rect,
                                                          src_bgr.cols,
                                                          src_bgr.rows,
                                                          &roi_rect,
                                                          roi_points,
                                                          &roi_angle);
    if (!roi_build_ok)
    {
        return 2;//红块在近处，但是没有找到ROi
    }

    //为什么不保留旋转矩阵，保留外接矩形干嘛
    roi_result->found = true;//找到了ROI区域
    roi_result->roi_rect = roi_rect;
    roi_result->roi_angle_deg = roi_angle;
    // 记录ROI尺寸（兼容字段 roi_size_px 保留为宽度）。
    roi_result->roi_width_px = static_cast<int>(std::lround(vec2_length(roi_points[1] - roi_points[0])));
    roi_result->roi_height_px = static_cast<int>(std::lround(vec2_length(roi_points[2] - roi_points[0])));
    roi_result->roi_size_px = roi_result->roi_width_px;
    for (int i = 0; i < 4; ++i)
    {
        roi_result->roi_points[i] = roi_points[i];
    }
#if ENABLE_VISION_DEBUG
    g_red_debug_roi_valid = true;
    for (int i = 0; i < 4; ++i)
    {
        g_red_debug_roi_points[i] = roi_points[i];
    }
#endif
    return 3;//找到ROI
}

/**
 * @brief ROI 预测测试（静态采图模式）
 * @note 这是离线采图工具，不属于小车正常运行主流程。
 *       打开后会周期性把预测成功的 ROI patch 保存到 picture 目录。
 */
/*
输入判空
-> 检查按键是否切换采集开关
-> 调用 red_roi_find 跑完整检测
-> 如果没有红块/ROI，直接退出
-> 如果采集开关没开，退出
-> 只有 roi_ret == 3 才继续
-> 每3个有效帧保存一次
-> 确保 picture 目录存在
-> 根据 roi_points 直接提取与正式模型输入一致的 48x48 透视 ROI
-> 保存成 png
*/
void red_roi_debug_test_pic(const cv::Mat& src_bgr)
{
    // ===== 调试代码开始：KEY2 控制的 ROI 调试采图工具 =====
    //输入图像判空
    if (src_bgr.empty())
    {
        RED_ROI_ERR("camera_frame_empty");
        return;
    }
    // 先更新按键开关状态。这个开关只服务于调试采图，不影响 red_roi 主链路。
    update_roi_capture_toggle_by_key2();

    RedRoiResult debug_roi_result = {};
    // 走完整 red_roi_find 流程，同时也会顺手刷新 red_roi 内部的调试缓存。
    const int roi_ret = red_roi_find(src_bgr, &debug_roi_result);
    //如果没有找到 ROI
    if (roi_ret <= 0)
    {
        return;
    }

    //采集开关没开
    if (!g_roi_capture_enabled)
    {
        g_roi_capture_frame_count = 0;
        return;
    }
    // 只对 roi_ret == 3 的情况进行采集，也就是 ROI 真正预测成功时才保存 patch。
    if (roi_ret != 3)
    {
        return;
    }

    //累加有效帧计数，每 5 帧保存一次
    ++g_roi_capture_frame_count;
    if (g_roi_capture_frame_count < 3)
    {
        return;
    }
    g_roi_capture_frame_count = 0;

    //确保输出目录存在
    if (!ensure_picture_dir_exists())
    {
        return;
    }

    // 调试采图真正保存的是“与正式模型输入完全一致的 48x48 ROI patch”，不是整图。
    cv::Mat roi_img;
    if (!extract_rotated_roi_patch(src_bgr,
                                   debug_roi_result.roi_points,
                                   kModelRoiWidth,
                                   kModelRoiHeight,
                                   &roi_img) ||
        roi_img.empty())
    {
        RED_ROI_ERR("extract roi image failed");
        return;
    }

    //生成保存路径
    char file_path[128];
    std::snprintf(file_path, sizeof(file_path), "picture/roi_%06u.png", g_roi_capture_save_index++);
    //保存图片
    if (!cv::imwrite(file_path, roi_img))
    {
        RED_ROI_ERR("save roi image failed: %s", file_path);
        return;
    }
    std::printf("save roi image: %s\r\n", file_path);
    // ===== 调试代码结束：KEY2 控制的 ROI 调试采图工具 =====
}

/**
 * @brief KEY2 单次触发固定时长 ROI 视频录制（保存到 video 目录）
 * @note 这是离线录屏工具，不属于小车正常运行主流程。
 *       录制期间按固定 FPS 采样：有 ROI 写入与正式模型输入一致的 48x48 ROI 图；无 ROI 写入黑帧。
 */
/*
KEY2 上升沿触发
-> 如果当前不在录制，则打开 VideoWriter，启动固定时长录制
-> 录制期间按固定 FPS 采样，不跟主循环帧率强绑定
-> 每个采样点都调用 red_roi_find 刷新当前 ROI 结果
-> 有 ROI 时写入与正式模型输入一致的 48x48 透视 ROI 图；无 ROI 时写入黑帧
-> 达到预设总帧数后自动停止并关闭视频文件
*/
void red_roi_debug_test_video(const cv::Mat& src_bgr)
{
    // ===== 调试代码开始：KEY2 触发的 ROI 调试录像工具 =====
    // 获取当前单调时钟时间点，保存到 now_tp
    const auto now_tp = std::chrono::steady_clock::now();
    // 读取 KEY2 当前状态
    const bool key2_pressed = (key_2.get_level() != KEY_RELEASE_LEVEL);

    // KEY2 按下一次触发一次固定时长录制。
    if (key2_pressed && !g_key2_video_last_pressed)
    {
        // 计算距离上次切换已经过了多少毫秒
        const auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - g_key2_video_last_toggle_tp).count();
        // 做 150ms 防抖 / 最小触发间隔判断
        // 只有当距离上次触发已经超过 150ms，才允许本次按键动作生效
        if (dt_ms >= 150)
        {
            // 更新“上次有效切换时间”
            // 后面下一次按键触发时，dt_ms 就会从这次开始算
            g_key2_video_last_toggle_tp = now_tp;
            // 如果当前没有在录，就尝试开始新录制
            if (!g_roi_video_recording)
            {
                // 检查/创建 video 目录
                if (!ensure_video_dir_exists())
                {
                    g_key2_video_last_pressed = key2_pressed;
                    return;
                }
                // 生成当前输出视频文件名
                std::snprintf(g_roi_video_file_path,
                              sizeof(g_roi_video_file_path),
                              "video/roi_video_%06u.avi",
                              g_roi_video_save_index++);
                // 定义视频编码格式
                const int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
                // 尝试打开视频写入器
                if (!g_roi_video_writer.open(g_roi_video_file_path,
                                             fourcc,
                                             static_cast<double>(kDebugVideoFps),
                                             cv::Size(kModelRoiWidth, kModelRoiHeight),
                                             true))
                {
                    // 如果打开失败，打印错误并清空路径
                    RED_ROI_ERR("open roi video failed: %s", g_roi_video_file_path);
                    g_roi_video_file_path[0] = '\0';
                }
                else
                {
                    // 打开成功后，初始化录制状态
                    g_roi_video_recording = true;
                    // 清零已写帧数
                    g_roi_video_saved_frame_count = 0;
                    // 清零有效帧数
                    g_roi_video_valid_frame_count = 0;
                    // 清零无效帧数
                    g_roi_video_invalid_frame_count = 0;
                    // 重置上一帧有效状态
                    g_roi_video_prev_frame_valid = false;
                    // 记录录制开始时间，后面拿它判断是否录满 5 秒
                    g_roi_video_record_start_tp = now_tp;
                    // 设置“下一帧应写时间”为当前时刻，录制开始后，本次调用后续就有机会立刻写第一帧，而不用再等一个完整帧间隔
                    g_roi_video_next_frame_tp = now_tp;
                    // 打印录制开始日志
                    std::printf("ROI video record START: %s, duration=%ds, fps=%d, size=%dx%d\r\n",
                                g_roi_video_file_path,
                                kDebugVideoDurationSec,
                                kDebugVideoFps,
                                kModelRoiWidth,
                                kModelRoiHeight);
                }
            }
            else
            {
                // 如果当前已经在录，按键再按一次只提示剩余时间
                const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - g_roi_video_record_start_tp).count();
                const int remaining_ms = std::max(0, kDebugVideoDurationSec * 1000 - static_cast<int>(elapsed_ms));
                std::printf("ROI video recording in progress, remaining=%d ms\r\n", remaining_ms);
            }
        }
    }
    // 记录这次调用的按键状态，供下次做边沿检测
    g_key2_video_last_pressed = key2_pressed;
    // 如果当前没在录，后面逻辑全部不用做
    if (!g_roi_video_recording)
    {
        return;
    }
    // 准备一个 ROI 结果结构体，并清零初始化
    RedRoiResult debug_roi_result = {};
    // 初始化 ROI 检测返回值
    int roi_ret = -1;
    // 如果输入图像非空，就执行 ROI 检测
    if (!src_bgr.empty())
    {   
        roi_ret = red_roi_find(src_bgr, &debug_roi_result);
    }
    // 计算视频帧间隔
    const auto frame_interval = std::chrono::milliseconds(1000 / kDebugVideoFps);
    // 只有到了“该写下一帧的时间”，才进入写帧逻辑，到点才采样一帧，不到点就先不写
    if (now_tp >= g_roi_video_next_frame_tp)
    {
        // 先定义要写入的视频帧，以及这帧是否为有效 ROI 帧
        cv::Mat frame_to_write;
        bool valid_roi_frame = false;
        
        // 只有在 ROI 检测成功且原图非空时，才尝试裁 ROI 图
        if (roi_ret == 3 && !src_bgr.empty())
        {
            cv::Mat roi_img;
            // 直接提取与正式模型输入完全一致的 48x48 透视 ROI 图。
            if (extract_rotated_roi_patch(src_bgr,
                                          debug_roi_result.roi_points,
                                          kModelRoiWidth,
                                          kModelRoiHeight,
                                          &roi_img) &&
                !roi_img.empty())
            {
                frame_to_write = roi_img.clone();
                // 检查提取后的结果是不是三通道彩色图
                if (frame_to_write.channels() == 3)
                {
                    valid_roi_frame = true;
                }
                else
                {
                    frame_to_write.release();
                }
            }
        }
        // 如果没得到有效 ROI 帧，就写黑帧
        if (frame_to_write.empty())
        {
            frame_to_write = cv::Mat::zeros(kModelRoiHeight, kModelRoiWidth, CV_8UC3);
        }
        // 把这一帧写进视频文件
        g_roi_video_writer.write(frame_to_write);
        // 已写帧数加 1
        ++g_roi_video_saved_frame_count;
        // 根据是否有效 ROI 帧，分别统计 valid / invalid
        if (valid_roi_frame)
        {
            // 仅当“上一帧无效，本帧有效”时打印获得有效帧
            if (!g_roi_video_prev_frame_valid)
            {
                std::printf("ROI video 获得有效帧: %s, frame=%d\r\n",
                            g_roi_video_file_path,
                            g_roi_video_saved_frame_count);
            }
            ++g_roi_video_valid_frame_count;
        }
        else
        {
            ++g_roi_video_invalid_frame_count;
        }
        g_roi_video_prev_frame_valid = valid_roi_frame;
        // 下一帧应写时间
        do
        {
            g_roi_video_next_frame_tp += frame_interval;
        } while (g_roi_video_next_frame_tp <= now_tp);
    }
    // 计算当前录制已经过去多久
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_tp - g_roi_video_record_start_tp).count();
    // 如果已经录满指定时长，就停止录制
    if (elapsed_ms >= kDebugVideoDurationSec * 1000)
    {
        // 如果 writer 还开着，就释放它
        if (g_roi_video_writer.isOpened())
        {
            g_roi_video_writer.release();
        }
        g_roi_video_recording = false;
        g_roi_video_prev_frame_valid = false;
        // 打印录制结束日志
        std::printf("ROI video record END: %s, frames=%d, valid=%d, invalid=%d\r\n",
                    g_roi_video_file_path,
                    g_roi_video_saved_frame_count,
                    g_roi_video_valid_frame_count,
                    g_roi_video_invalid_frame_count);
    }
    // ===== 调试代码结束：KEY2 触发的 ROI 调试录像工具 =====
}
