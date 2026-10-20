#include "car_init.hpp"
#include "image.hpp"
#include "element.hpp"
#include "obstacle_detect.hpp"
#include "beep.hpp"
#include "lq_ncnn.hpp"
#include "encoder.hpp"
#include "imu.hpp"
#include "ramp_detect.hpp"

#include <cstring>

#define IMAGE_LOG_INFO(fmt, ...) std::printf("[IMAGE][INFO][%s] " fmt "\r\n", __func__, ##__VA_ARGS__)

ImageInfo g_image_info = {0};  // 图像信息，所有成员初始化为0
Max_white_cols max_white={0};//最长白列
ImagePro_var imgpro_var={0};
uint8* gray_image = nullptr;  // 灰度图像信息，初始化为空指针，避免野指针
uint16* color_image = nullptr; // 彩色图像指针，默认置空避免野指针

static uint8 g_gray_image_proc[IMG_PROC_WIDTH * IMG_PROC_HEIGHT] = {0}; // 2x2下采样后给搜线算法使用
static uint8* gray_image_proc = g_gray_image_proc;
static uint8 s_right_enter_rescan_gray[IMG_PROC_WIDTH * IMG_PROC_HEIGHT] = {0};

// 上位机调试显示用：保存“本帧真正送入搜线/黑线重扫的 160x120 图”。
// 普通帧保存 g_gray_image_proc；十字/环岛黑线重扫帧保存画完补线后的重扫输入图。
// 这样即使重扫失败恢复了边界，也能在上位机看到当时用于重扫的补线图。
static uint8 s_debug_scan_gray[IMG_PROC_WIDTH * IMG_PROC_HEIGHT] = {0};
static bool s_debug_scan_gray_valid = false;

static ImageInfo s_rescan_backup_image_info = {0};
static Max_white_cols s_rescan_backup_max_white = {0};
static bool s_rescan_backup_valid = false;

static_assert((UVC_WIDTH % IMG_PROC_DOWNSAMPLE) == 0, "UVC_WIDTH must be divisible by IMG_PROC_DOWNSAMPLE");
static_assert((UVC_HEIGHT % IMG_PROC_DOWNSAMPLE) == 0, "UVC_HEIGHT must be divisible by IMG_PROC_DOWNSAMPLE");

//保护状态
static float out_protect_mid_average_thre = 0.0f;
static uint8 out_protect_over_frame_count = 0;
static bool out_protect_ready = false;

//绕行状态变量
//识别到图片那一刻锁存的参考白点参数
static uint8 s_bypass_ref_point = 0;
static uint8 s_bypass_white_min_point = 0;
static uint8 s_bypass_white_max_point = 0;

// 本帧搜线状态：
// s_stage_edge_valid: 只要本帧能正常 search_edge，就认为边线有效。
// s_stage_line_invalid: 最长白列不足，说明目前无法稳定搜线。
// s_stage_bottom_black: 底部区域变黑，常用于辅助判断已绕出赛道区域。
// s_stage_out_protect_risk: 若此帧切回普通 out_protect，是否会立即触发保护。
static bool s_stage_edge_valid = false;
static bool s_stage_line_invalid = false;
static bool s_stage_center_col_invalid = false;
static bool s_stage_bottom_black = false;
static bool s_stage_out_protect_risk = false;
static bypass_state_enum s_bypass_state = BYPASS_IDLE; //当前绕行状态
static char s_bypass_dir = '0';  // 锁存模型识别方向,'l' or 'r','0' 无绕行
static int s_bypass_sign = 0;    // 绕行方向符号,right=+1, left=-1
static bool s_stage_reach_target_yaw = false; // 当前阶段是否达到目标角（到角后通常会清误差或切阶段）
static uint8 s_stage_safe_count = 0; // 连续“安全帧”计数器（用于恢复阶段防抖）
static float s_bypass_output_error = 0.0f; // 绕行状态机最终输出误差（实际给控制环）
static float s_bypass_target_error = 0.0f; // 绕行目标误差（低通前）
static float s_bypass_filtered_error = 0.0f; // 绕行目标误差经过一阶低通后的结果
static char s_prev_model_ctrl = '0'; // 模型控制信号上一帧值，用于上升沿触发绕行
constexpr uint8 kBypassRecoverNeedFrames = 3; // 退出绕行所需连续安全帧数
constexpr uint8 kBypassStage1CenterCol = USE_MIDDLE_VALUE; // Stage1 切阶段使用图像对称轴列
static bool s_recover_switched_to_normal = false; // 是否已切回正常中线（只触发一次）
static bool s_recover_wait_edge_valid = false; // 切回正常中线后，先等 edge_valid 再开始安全计数
static bool s_stage2_passed_zero_yaw = false; // 阶段2是否已经先回到0度（用于“先到0再判线”）
static uint8 s_stage1_center_invalid_count = 0; // Stage1 连续“中心列失效”计数，达到3帧才允许进Stage2
static uint8 s_stage2_line_valid_count = 0; // Stage2 连续“线有效”计数，达到3帧才允许进Stage3
static uint8 s_mode_switch_smooth_count = 0; // 中线来源切换到 NORMAL 后的平滑窗口计数

static bypass_center_mode_enum s_bypass_center_mode = BYPASS_CENTER_NORMAL;//绕行时误差采用的“中线来源”
static bypass_profile_enum s_bypass_profile = BYPASS_PROFILE_UNIFIED;      //统一绕行策略
static bypass_finish_reason_enum s_bypass_finish_reason = BYPASS_FINISH_NONE;//本次绕行结束原因

// 可调绕行参数（上位机可调）：
// yaw_xxx: 各阶段目标角，单位“度”。
// error_xxx: 固定打方向时的目标误差幅值。
// lpf_alpha: 绕行误差低通系数，越大响应越快，越小越平滑。
// recover_normal_center_gain: 切回正常中线后的误差放大倍率。
float model_bypass_stage1_yaw_deg=40.0f;
float model_bypass_stage2_precheck_yaw_deg=-30.0f;
float model_bypass_stage2_yaw_deg=60.0f;
float model_bypass_stage1_error=1.0f;
float model_bypass_stage2_error=1.0f;
float model_bypass_error_lpf_alpha=0.30f;
float model_recover_normal_center_gain=2.0f; // 切回正常中线后的误差放大倍率
float model_mode_switch_lpf_alpha=0.12f; // 边线误差切回正常中线时的平滑低通系数
uint8 model_mode_switch_lpf_frames=3; // 边线误差切回正常中线后的平滑帧数

static const char* profile_name(bypass_profile_enum profile)
{
    (void)profile;
    return "UNIFIED_3_STAGE";
}

static const char* bypass_dir_name(int sign)
{
    return (sign < 0) ? "LEFT_BYPASS" : "RIGHT_BYPASS";
}

static const char* bypass_center_name(bypass_center_mode_enum mode)
{
    if (mode == BYPASS_CENTER_LEFT_EDGE) return "LEFT_EDGE_CENTER";
    if (mode == BYPASS_CENTER_RIGHT_EDGE) return "RIGHT_EDGE_CENTER";
    return "NORMAL_CENTER";
}

static const char* bypass_finish_name(bypass_finish_reason_enum reason)
{
    if (reason == BYPASS_FINISH_NORMAL) return "NORMAL";
    if (reason == BYPASS_FINISH_PROTECT) return "PROTECT";
    if (reason == BYPASS_FINISH_MANUAL) return "MANUAL_RESET"; //手动置位
    return "NONE";
}

//判断当前模型是否识别到了需要绕行的方向
static inline bool model_lr_active(void)
{
    return is_model_infer_complete && (model_control_access == 'l' || model_control_access == 'r');
}

//判断当前是否正在模型绕行控制阶段（阶段1/2/3都算 active）
static inline bool bypass_state_active(void)
{
    return s_bypass_state == BYPASS_STAGE1_TURN_OUT ||
           s_bypass_state == BYPASS_STAGE2_TURN_BACK ||
           s_bypass_state == BYPASS_STAGE3_TURN_BACK;
}

//绕行误差一阶低通，抑制阶段切换时误差突变
static inline float bypass_error_lpf_step(float target_error)
{
    // float alpha = CLAMP(model_bypass_error_lpf_alpha, 0.0f, 1.0f);
    // if (s_mode_switch_smooth_count > 0U)
    // {
    //     alpha = CLAMP(model_mode_switch_lpf_alpha, 0.0f, 1.0f);
    //     s_mode_switch_smooth_count--;
    // }
    // s_bypass_target_error = target_error;
    // s_bypass_filtered_error = alpha * s_bypass_target_error + (1.0f - alpha) * s_bypass_filtered_error;
    // return s_bypass_filtered_error;
    return target_error;
}

static inline bool yaw_reached(float target_yaw)
{
    // 统一“过目标角”判定，target_yaw 支持正负。
    if (target_yaw >= 0.0f)
    {
        return model_yaw >= target_yaw;
    }
    return model_yaw <= target_yaw;
}

// 以某一侧边线当作“中线”来计算误差（用于绕行回赛道阶段）。
static float calc_edge_as_midline_error(const BoundaryInfo edge[], uint8 nums, int lookaheadline)
{
    // 与普通中线误差同权重模型（高斯加权），仅把“边线坐标”当中线坐标来算误差。
    // 目的：在回赛道阶段，只依赖单侧可靠边线进行引导，减少中心线不稳定影响。
    float weight_sum = 0.0f;
    float line_sum = 0.0f;
    float sigma = 10.0f;
    if (nums > 5U)
    {
        for (uint8 i = error_maxw_start; i < nums; ++i)
        {
            int dis = abs((int)edge[i].pos[1] - lookaheadline);
            float x = (float)dis / sigma;
            float weight = expf(-0.5f * x * x);
            weight_sum += weight;
            line_sum += weight * ((float)edge[i].pos[0] - USE_MIDDLE_VALUE);
        }
        if (weight_sum > 1e-4f)
        {
            return (line_sum / weight_sum) / USE_MIDDLE_VALUE;
        }
    }
    return 0.0f;
}

// 按当前绕行中线来源得到误差。
static float calc_bypass_track_error(void)
{
    // 统一误差出口：
    // - NORMAL：使用普通中线误差；
    // - LEFT/RIGHT_EDGE：使用对应边线作为中线误差。
    uint8 error_line_num = (uint8)CLAMP(max_white.nums, 0, error_maxw_end);
    if (s_bypass_center_mode == BYPASS_CENTER_LEFT_EDGE)
    {
        return calc_edge_as_midline_error(g_image_info.filter_left, error_line_num, imgpro_var.lookahead);
    }
    if (s_bypass_center_mode == BYPASS_CENTER_RIGHT_EDGE)
    {
        return calc_edge_as_midline_error(g_image_info.filter_right, error_line_num, imgpro_var.lookahead);
    }
    return calc_midline_error(g_image_info.filter_midline, error_line_num, imgpro_var.lookahead);
}

static void bypass_switch_center_mode(bypass_center_mode_enum mode)
{
    // 切换误差来源时打点，方便复盘“什么时候从固定打角切到边线跟踪/正常跟踪”。
    // 上升沿触发
    if (s_bypass_center_mode != mode)
    {
        const bypass_center_mode_enum prev_mode = s_bypass_center_mode;
        s_bypass_center_mode = mode;
        if (mode == BYPASS_CENTER_NORMAL &&
            (prev_mode == BYPASS_CENTER_LEFT_EDGE || prev_mode == BYPASS_CENTER_RIGHT_EDGE))
        {
            // 从“边线当中线”切回“正常中线”时，启用短窗口低通平滑，抑制电机卡顿声。
            s_mode_switch_smooth_count = model_mode_switch_lpf_frames;
        }
        IMAGE_LOG_INFO("bypass_center_mode -> %s", bypass_center_name(mode));
    }
}

static void bypass_switch_to_normal_and_arm_recover(void)
{
    // 只执行一次：
    // 1) 切回正常中线；
    // 2) 安全计数清零；
    // 3) 强制先等待 edge_valid，避免刚切回时拿到无效中线导致误判结束。
    if (!s_recover_switched_to_normal)
    {
        bypass_switch_center_mode(BYPASS_CENTER_NORMAL);
        s_recover_switched_to_normal = true;
        s_recover_wait_edge_valid = true;
        s_stage_safe_count = 0;
        IMAGE_LOG_INFO("bypass_recover_arm wait_edge_valid=true");
    }
}

static void bypass_try_finish_when_safe(const char* tag)
{
    // 退出绕行门控分两层：
    // A. 先等 edge_valid=true（只要没线，就不允许计安全帧）。
    // B. edge_valid 后，连续 kBypassRecoverNeedFrames 帧满足“无 out_protect 风险”才退出。
    // 这样可减少“刚切正常就退出”的抖动风险。
    if (s_recover_wait_edge_valid)
    {
        if (!s_stage_edge_valid)
        {
            s_stage_safe_count = 0;
            return;
        }
        s_recover_wait_edge_valid = false;
        s_stage_safe_count = 0;
        IMAGE_LOG_INFO("bypass_recover_arm edge_valid=true start_safe_count");
    }

    if (s_stage_edge_valid && !s_stage_out_protect_risk)
    {
        if (s_stage_safe_count < 255U)
        {
            s_stage_safe_count++;
        }
    }
    else
    {
        s_stage_safe_count = 0;
    }

    if (s_stage_safe_count >= kBypassRecoverNeedFrames)
    {
        s_bypass_finish_reason = BYPASS_FINISH_NORMAL;
        IMAGE_LOG_INFO("bypass_recover_ready profile=%s tag=%s safe_frames=%u",
                       profile_name(s_bypass_profile),
                       tag,
                       s_stage_safe_count);
        s_bypass_state = BYPASS_RECOVER;
    }
}

//给逐飞助手传灰度图像
const uint8* get_gray_image_proc_ptr(void)
{
    return gray_image_proc;
}

const uint8* get_gray_image_debug_show_ptr(void)
{
    if(s_debug_scan_gray_valid)
    {
        return s_debug_scan_gray;
    }
    return gray_image_proc;
}

static void backup_lane_scan_result(void)
{
    s_rescan_backup_image_info = g_image_info;
    s_rescan_backup_max_white = max_white;
    s_rescan_backup_valid = true;
}

static void restore_lane_scan_result(void)
{
    if(!s_rescan_backup_valid)
    {
        return;
    }

    g_image_info = s_rescan_backup_image_info;
    max_white = s_rescan_backup_max_white;
}

static bool run_lane_scan_once(void)
{
    search_max_white();
    if(max_white.nums < Min_white_nums)
    {
        return false;
    }

    search_edge();
    find_middline(g_image_info.midline,g_image_info.left,g_image_info.right);
    filter_one_edge_median3(g_image_info.filter_left, g_image_info.left, max_white.nums);
    filter_one_edge_median3(g_image_info.filter_right, g_image_info.right, max_white.nums);
    find_middline(g_image_info.midline,g_image_info.filter_left,g_image_info.filter_right);
    smooth_midline_121(g_image_info.filter_midline,g_image_info.midline,max_white.nums);
    return true;
}

static inline void set_work_gray_black_safe(uint8 *img, int x, int y)
{
    if(img == nullptr)
    {
        return;
    }
    if(x < 0 || x >= IMG_PROC_WIDTH || y < 0 || y >= IMG_PROC_HEIGHT)
    {
        return;
    }
    img[y * IMG_PROC_WIDTH + x] = 0;
}

static void draw_black_thick_point(uint8 *img, int x, int y)
{
    set_work_gray_black_safe(img, x, y);
    set_work_gray_black_safe(img, x - 1, y);
    set_work_gray_black_safe(img, x + 1, y);
    set_work_gray_black_safe(img, x, y - 1);
    set_work_gray_black_safe(img, x, y + 1);
}

static void draw_black_line_on_work_gray(uint8 *img, int x1, int y1, int x2, int y2)
{
    if(img == nullptr)
    {
        return;
    }

    int dx = func_abs(x2 - x1);
    int dy = func_abs(y2 - y1);
    int sx = (x1 < x2) ? 1 : -1;
    int sy = (y1 < y2) ? 1 : -1;
    int err = dx - dy;

    while(true)
    {
        draw_black_thick_point(img, x1, y1);
        if(x1 == x2 && y1 == y2)
        {
            break;
        }

        int e2 = err << 1;
        if(e2 > -dy)
        {
            err -= dy;
            x1 += sx;
        }
        if(e2 < dx)
        {
            err += dx;
            y1 += sy;
        }
    }
}
// 计算一条线在指定 y 行对应的 x。
// 用于 EXIT_PREP 阶段，把斜线右侧区域整体涂黑。
static int black_edge_x_at_y(int x1, int y1, int x2, int y2, int y)
{
    if(y1 == y2)
    {
        return x1;
    }

    float t = (float)(y - y1) / (float)(y2 - y1);
    int x = (int)((float)x1 + t * (float)(x2 - x1));

    if(x < 0) x = 0;
    if(x >= IMG_PROC_WIDTH) x = IMG_PROC_WIDTH - 1;

    return x;
}

// EXIT_PREP 专用：把“斜线右侧区域”涂黑。
// 这样右侧黑区的左边缘不是竖直线，而是有斜率的边缘，重扫线时可以用它拟合右边界。
static void fill_right_side_black_for_exit_prep(uint8 *img, int x1, int y1, int x2, int y2)
{
    if(img == nullptr)
    {
        return;
    }

    if(y1 > y2)
    {
        int tx = x1; x1 = x2; x2 = tx;
        int ty = y1; y1 = y2; y2 = ty;
    }

    if(y1 < 0) y1 = 0;
    if(y2 >= IMG_PROC_HEIGHT) y2 = IMG_PROC_HEIGHT - 1;

    for(int y = y1; y <= y2; ++y)
    {
        int x_cut = black_edge_x_at_y(x1, y1, x2, y2, y);

        for(int x = x_cut; x < IMG_PROC_WIDTH; ++x)
        {
            set_work_gray_black_safe(img, x, y);
        }
    }
}

// EXIT_PREP 专用：把“斜线左侧区域”涂黑。
// 左环镜像右环：黑区右边缘是有斜率的边缘，重扫线时可以用它拟合左边界。
static void fill_left_side_black_for_exit_prep(uint8 *img, int x1, int y1, int x2, int y2)
{
    if(img == nullptr)
    {
        return;
    }

    if(y1 > y2)
    {
        int tx = x1; x1 = x2; x2 = tx;
        int ty = y1; y1 = y2; y2 = ty;
    }

    if(y1 < 0) y1 = 0;
    if(y2 >= IMG_PROC_HEIGHT) y2 = IMG_PROC_HEIGHT - 1;

    for(int y = y1; y <= y2; ++y)
    {
        int x_cut = black_edge_x_at_y(x1, y1, x2, y2, y);

        for(int x = 0; x <= x_cut && x < IMG_PROC_WIDTH; ++x)
        {
            set_work_gray_black_safe(img, x, y);
        }
    }
}

static bool run_right_enter_blackline_rescan(const element_circle_draw_debug_struct *dbg)
{
if(dbg == nullptr || !dbg->valid)
    {
        return false;
    }


    if(dbg->state != (uint8)CIRCLE_RIGHT_PREP &&
       dbg->state != (uint8)CIRCLE_RIGHT_ENTER &&
       dbg->state != (uint8)CIRCLE_RIGHT_INSIDE &&
       dbg->state != (uint8)CIRCLE_LEFT_PREP &&
       dbg->state != (uint8)CIRCLE_LEFT_ENTER &&
       dbg->state != (uint8)CIRCLE_LEFT_INSIDE &&
       dbg->state != (uint8)CIRCLE_EXIT_PREP)
    {
        return false;
    }
    
    memcpy(s_right_enter_rescan_gray,
           g_gray_image_proc,
           (size_t)(IMG_PROC_WIDTH * IMG_PROC_HEIGHT) * sizeof(uint8));

draw_black_line_on_work_gray(s_right_enter_rescan_gray,
                             (int)dbg->template_x1,
                             (int)dbg->template_y1,
                             (int)dbg->template_x2,
                             (int)dbg->template_y2);

// EXIT_PREP 阶段不能只画一条线。
// 右环：涂黑模板线右侧，让右边界贴住斜边；
// 左环：涂黑模板线左侧，让左边界贴住斜边。
if(dbg->state == (uint8)CIRCLE_EXIT_PREP)
{
    if(dbg->template_side == 1U)
    {
        fill_left_side_black_for_exit_prep(s_right_enter_rescan_gray,
                                           (int)dbg->template_x1,
                                           (int)dbg->template_y1,
                                           (int)dbg->template_x2,
                                           (int)dbg->template_y2);
    }
    else
    {
        fill_right_side_black_for_exit_prep(s_right_enter_rescan_gray,
                                            (int)dbg->template_x1,
                                            (int)dbg->template_y1,
                                            (int)dbg->template_x2,
                                            (int)dbg->template_y2);
    }
}

// 保存本次黑线重扫真正使用的输入图，供上位机直接观察补线是否正确。
// 注意：后面即使 run_lane_scan_once() 失败并恢复边界，这张图仍保留到下一帧更新。
memcpy(s_debug_scan_gray,
       s_right_enter_rescan_gray,
       (size_t)(IMG_PROC_WIDTH * IMG_PROC_HEIGHT) * sizeof(uint8));
s_debug_scan_gray_valid = true;

    gray_image_proc = s_right_enter_rescan_gray;
    clear_line_info();

    if(!run_lane_scan_once())
    {
        gray_image_proc = g_gray_image_proc;
        restore_lane_scan_result();
        return false;
    }

    return true;
}

//这个函数计算图像最下面5行的平均灰度,判断车底部看到的是不是黑色区域
static float calc_bottom5_avg_full_width(void)
{
    float sum_thre = 0.0f;
    for(uint8 row = IMG_PROC_HEIGHT - 1; row > IMG_PROC_HEIGHT - 6; row--)
    {
        int thre = 0;
        for(uint8 col = 0; col < IMG_PROC_WIDTH; col++)
        {
            thre += AT(gray_image_proc,row,col);
        }
        sum_thre += (float)thre / IMG_PROC_WIDTH;
    }
    return sum_thre / 5.0f;
}

//这个函数判断当前帧如果按普通 out_protect 逻辑，是否会触发保护
//返回true就是会触发保护
static bool would_trigger_out_protect_this_frame(void)
{
    //这个一般基线就是会建立成功的
    if (!out_protect_ready)
    {
        return false;
    }
    //模型判断到那一时刻的out_protect_mid_average_thre，相当于锁存了
    int dark_thre = (int)(out_protect_mid_average_thre - 20.0f);
    if(dark_thre < 0)
    {
        dark_thre = 0;
    }
    int over_count = 0;
    //得到黑色点
    for(uint8 row = IMG_PROC_HEIGHT - 1; row > IMG_PROC_HEIGHT - 6; row--)
    {
        for(uint8 col = 0; col < IMG_PROC_WIDTH; col++)
        {
            if(AT(gray_image_proc,row,col) <= dark_thre)
            {
                over_count++;
            }
        }
    }
    const int dark_pixel_limit = (IMG_PROC_WIDTH - 60) * 5;
    return over_count >= dark_pixel_limit;  //true是会触发保护
}

// 启动绕行：
// 1) 锁存本次绕行方向/统一策略；
// 2) 锁存参考白点阈值（绕行中固定阈值）；
// 3) 清编码器与 yaw，避免带入上一段状态。
static void bypass_start_from_model(char ctrl_dir)
{
    s_bypass_state = BYPASS_STAGE1_TURN_OUT;
    s_bypass_dir = ctrl_dir;
    s_bypass_sign = (ctrl_dir == 'r') ? 1 : -1;//右绕行为1
    s_stage_reach_target_yaw = false;
    s_stage_safe_count = 0;
    s_recover_switched_to_normal = false;
    s_recover_wait_edge_valid = false;
    s_stage2_passed_zero_yaw = false;
    s_stage1_center_invalid_count = 0;
    s_stage2_line_valid_count = 0;
    s_mode_switch_smooth_count = 0;
    s_bypass_center_mode = BYPASS_CENTER_NORMAL;
    s_bypass_profile = BYPASS_PROFILE_UNIFIED;
    s_bypass_finish_reason = BYPASS_FINISH_NONE;
    s_bypass_output_error = 0.0f;
    s_bypass_target_error = imgpro_var.midline_error;
    s_bypass_filtered_error = imgpro_var.midline_error;

    // 绕行开始前清积分，避免带入历史状态。
    model_encoder_clear_increment();
    model_yaw = 0.0f;

    s_bypass_ref_point = imgpro_var.ref_point;
    s_bypass_white_min_point = imgpro_var.white_min_point;
    s_bypass_white_max_point = imgpro_var.white_max_point;
    printf("\r\n绕行状态开始\r\n");
    IMAGE_LOG_INFO("bypass_start dir=%s profile=%s cur_err=%.3f",
                   bypass_dir_name(s_bypass_sign),
                   profile_name(s_bypass_profile),
                   imgpro_var.midline_error);
    IMAGE_LOG_INFO("case=UNIFIED + %s", bypass_dir_name(s_bypass_sign));
}

// 绕行结束统一出口：
// 1) 打印结束日志（正常/保护/手动）；
// 2) 清空绕行状态机内部变量；
// 3) 调模型侧复位接口，解锁下一次识别；
// 4) 清编码器和 yaw，保证下一次从干净状态进入。
static void bypass_release_to_idle(void)
{
    IMAGE_LOG_INFO("bypass_finish mode=%s dir=%s profile=%s state=%d yaw=%.2f enc=%.1f",
                   bypass_finish_name(s_bypass_finish_reason),
                   bypass_dir_name(s_bypass_sign),
                   profile_name(s_bypass_profile),
                   (int)s_bypass_state,
                   model_yaw,
                   model_encoder_integral);
    printf("绕行状态结束\r\n\r\n");
    s_bypass_state = BYPASS_IDLE;
    s_bypass_dir = '0';
    s_bypass_sign = 0;
    s_prev_model_ctrl = '0';
    s_stage_reach_target_yaw = false;
    s_stage_safe_count = 0;
    s_recover_switched_to_normal = false;
    s_recover_wait_edge_valid = false;
    s_stage2_passed_zero_yaw = false;
    s_stage1_center_invalid_count = 0;
    s_stage2_line_valid_count = 0;
    s_mode_switch_smooth_count = 0;
    s_bypass_center_mode = BYPASS_CENTER_NORMAL;
    s_bypass_profile = BYPASS_PROFILE_UNIFIED;
    s_bypass_finish_reason = BYPASS_FINISH_NONE;
    s_bypass_output_error = 0.0f;
    s_bypass_target_error = 0.0f;
    s_bypass_filtered_error = 0.0f;
    s_bypass_ref_point = 0;
    s_bypass_white_min_point = 0;
    s_bypass_white_max_point = 0;
    s_stage_bottom_black = false;
    s_stage_line_invalid = false;
    s_stage_center_col_invalid = false;
    s_stage_edge_valid = false;
    s_stage_out_protect_risk = false;

    // 释放绕行后，真正解锁下一次模型识别（含 guarded/latch）。
    // 同时清编码器/陀螺积分，保证下一次绕行从干净状态开始。
    lq_ncnn_reset_control_state();
    model_encoder_clear_increment();
    model_yaw = 0.0f;
}

void image_bypass_reset_state(void)
{
    s_bypass_finish_reason = BYPASS_FINISH_MANUAL;
    bypass_release_to_idle();
}

// 阶段切换：Stage1 -> Stage2
// 进入时机：当前策略判定“应开始回摆/找线”。
// 动作：切状态、清安全计数、清目标角达成标志、清编码器积分。
static void bypass_enter_stage2(void)
{
    s_bypass_state = BYPASS_STAGE2_TURN_BACK;
    s_stage_safe_count = 0;
    s_stage_reach_target_yaw = false;
    s_recover_switched_to_normal = false;
    s_recover_wait_edge_valid = false;
    s_stage2_passed_zero_yaw = false;
    s_stage1_center_invalid_count = 0;
    s_stage2_line_valid_count = 0;
    model_encoder_clear_increment(); //阶段切换清编码器积分
    IMAGE_LOG_INFO("bypass_stage enter S2 profile=%s", profile_name(s_bypass_profile));
}

// 阶段切换：Stage2 -> Stage3
// 进入时机：已重新找到可用边线，开始边线跟踪回赛道。
// 动作：切状态并清理阶段计数/编码器积分。
static void bypass_enter_stage3(void)
{
    s_bypass_state = BYPASS_STAGE3_TURN_BACK;
    s_stage_safe_count = 0;
    s_stage_reach_target_yaw = false;
    s_recover_switched_to_normal = false;
    s_recover_wait_edge_valid = false;
    s_stage2_passed_zero_yaw = false;
    s_stage1_center_invalid_count = 0;
    s_stage2_line_valid_count = 0;
    model_encoder_clear_increment(); //阶段切换清编码器积分
    IMAGE_LOG_INFO("bypass_stage S2->S3 profile=%s", profile_name(s_bypass_profile));
}

// 绕行状态机主函数（每帧执行一次）：
// 输入：当前搜线状态、yaw、编码器积分、风险标志。
// 输出：s_bypass_output_error（供控制环使用）。
static void bypass_state_machine_step(void)
{
    // 如果当前没有绕行，就输出误差清零
    // 正常巡线误差应该由原来的 image_update_control_error() 计算
    if (s_bypass_state == BYPASS_IDLE)
    {
        s_bypass_output_error = 0.0f;
        return;
    }

    // 绕行阶段统一编码器保护：
    // 任意策略、任意阶段只要编码器积分超阈值，立即保护退出，防止车模失控。
    if (model_encoder_integral >= model_protect_encoder_integral)
    {
        s_bypass_finish_reason = BYPASS_FINISH_PROTECT;
        IMAGE_LOG_INFO("bypass_protect reason=encoder_limit dir=%s profile=%s state=%d yaw=%.2f enc=%.1f",
                       bypass_dir_name(s_bypass_sign),
                       profile_name(s_bypass_profile),
                       (int)s_bypass_state,
                       model_yaw,
                       model_encoder_integral);
        imgpro_var.protect_flag = true;
        return;
    }

    if (s_bypass_state == BYPASS_STAGE1_TURN_OUT)
    {
        const float yaw_out = (float)(-s_bypass_sign) * model_bypass_stage1_yaw_deg;
        if (!s_stage_reach_target_yaw && yaw_reached(yaw_out))
        {
            s_stage_reach_target_yaw = true;
        }
        float target = s_stage_reach_target_yaw ? 0.0f : (float)s_bypass_sign * model_bypass_stage1_error;
        s_bypass_output_error = bypass_error_lpf_step(target);
        if (s_stage_center_col_invalid)
        {
            if (s_stage1_center_invalid_count < 255U)
            {
                s_stage1_center_invalid_count++;
            }
            if (s_stage1_center_invalid_count >= 3U)
            {
                bypass_enter_stage2();
            }
        }
        else
        {
            s_stage1_center_invalid_count = 0;
        }
        return;
    }

    if (s_bypass_state == BYPASS_STAGE2_TURN_BACK)
    {
        const bypass_center_mode_enum edge_center =
            (s_bypass_sign < 0) ? BYPASS_CENTER_LEFT_EDGE : BYPASS_CENTER_RIGHT_EDGE;
        if (!s_stage2_passed_zero_yaw)
        {
            const float target_to_zero = (float)(-s_bypass_sign) * model_bypass_stage2_error;
            const bool reach_zero = (s_bypass_sign > 0) ? (model_yaw >= model_bypass_stage2_precheck_yaw_deg)
                                                        : (model_yaw <= -model_bypass_stage2_precheck_yaw_deg);
            if (reach_zero)
            {
                s_stage2_passed_zero_yaw = true;
                s_stage_reach_target_yaw = false;
                IMAGE_LOG_INFO("bypass_stage2 reach_precheck profile=UNIFIED yaw=%.2f", model_yaw);
            }
            s_bypass_output_error = bypass_error_lpf_step(reach_zero ? 0.0f : target_to_zero);
            return;
        }

        if (!s_stage_line_invalid)
        {
            if (s_stage2_line_valid_count < 255U)
            {
                s_stage2_line_valid_count++;
            }

            if (s_stage2_line_valid_count >= 3U)
            {
                bypass_switch_center_mode(edge_center);
                s_bypass_output_error = bypass_error_lpf_step(calc_bypass_track_error());
                bypass_enter_stage3();
                return;
            }
        }
        else
        {
            s_stage2_line_valid_count = 0;
        }

        bypass_switch_center_mode(BYPASS_CENTER_NORMAL);
        const float yaw_back = (float)s_bypass_sign * model_bypass_stage2_yaw_deg;
        if (!s_stage_reach_target_yaw && yaw_reached(yaw_back))
        {
            s_stage_reach_target_yaw = true;
        }
        const float target = s_stage_reach_target_yaw ? 0.0f : (float)(-s_bypass_sign) * model_bypass_stage2_error;
        s_bypass_output_error = bypass_error_lpf_step(target);
        return;
    }

    // 策略B/C阶段3：
    // 1) 首选同侧边线中线跟踪；
    // 2) 到各自积分阈值后切回正常中线；
    // 3) 通过“edge_valid + 连续安全帧”门控后退出绕行。
    if (s_bypass_state == BYPASS_STAGE3_TURN_BACK)
    {
        const bypass_center_mode_enum edge_center =
            (s_bypass_sign < 0) ? BYPASS_CENTER_LEFT_EDGE : BYPASS_CENTER_RIGHT_EDGE;
        float target = 0.0f;
        if (!s_stage_line_invalid)
        {
            // 一旦 Stage3 已经开始“切回正常中线准备退出”，后续帧就锁住 NORMAL_CENTER，
            // 不再回到单边线模式，避免退出门控期间来回切换中线来源。
            if (!s_recover_switched_to_normal)
            {
                bypass_switch_center_mode(edge_center);
            }
            target = calc_bypass_track_error();
        }
        else
        {
            imgpro_var.protect_flag = true;
            return;
        }

        if (model_encoder_integral >= model_bypass_stage3_integral)
        {
            bypass_switch_to_normal_and_arm_recover();
            target = calc_bypass_track_error() * model_recover_normal_center_gain;
            bypass_try_finish_when_safe("UNIFIED");
        }

        s_bypass_output_error = bypass_error_lpf_step(target);
        return;
    }

    // RECOVER 状态：
    // 统一走释放函数，完成日志、清状态、解锁模型下一次识别。
    if (s_bypass_state == BYPASS_RECOVER)
    {
        bypass_release_to_idle();
    }
}

/**
 * @brief 图像处理变量初始化
 * @note  使用变量以及保护清零，变量是方便上位机调试
 */
void image_pro_init()
{
    imgpro_var.diff_sum_mov=3;
    imgpro_var.mw_diff_thershold=25;
    imgpro_var.diff_thershold=30;
    imgpro_var.lookahead=0;//前瞻距离70是比较近，60就比较远了，根据路径调整前瞻   68
    imgpro_var.protect_flag=false;
    image_protect_reset_state();
    element_init();
    obstacle_detect_init();
}

void image_max_white_test()
{
    int ret;
    //清除边线信息
    clear_line_info();
    //得到灰度图像
    ret=get_gray_image();
    //保护，防止处理空指针
    if(ret==-1) 
    {
        max_white.nums = 0;
        max_white.col = 0;
        imgpro_var.midline_error = 0.0f;
        imgpro_var.protect_flag=true;
        printf("\r\n");
        printf("%s:%s:%d,Error: 获取灰度图像指针失败！\r\n", __FILE__, __FUNCTION__, __LINE__);
        printf("\r\n");
        return;
    }
    get_color_image();//得到彩色图像,逐飞助手看彩图的时候打开就行了
    //全图搜线，得到边线和中线信息
    get_reference_point();
    search_max_white();  
}

/**
 * @brief 图像处理接口，包括搜线，元素处理
 * @note 正常搜线无绕行逻辑
 */
void image_pro()
{
    int ret;
    const bool obstacle_active_before = obstacle_detect_is_active();
    const bool ramp_block_elements = ramp_blocks_other_elements();
    // 清除边线信息
    clear_line_info();
    // 得到灰度图像
    ret = get_gray_image();
    // 保护，防止处理空指针
    if(ret == -1)
    {
        max_white.nums = 0;
        max_white.col = 0;
        imgpro_var.midline_error = 0.0f;
        imgpro_var.protect_flag = true;
        printf("\r\n");
        printf("%s:%s:%d,Error: 获取灰度图像指针失败！\r\n", __FILE__, __FUNCTION__, __LINE__);
        printf("\r\n");
        return;
    }

    // 保护，在提边前先做保护检测，异常帧直接丢弃，不进入控制链路。
    out_protect();
    if(imgpro_var.protect_flag == true)
    {
        max_white.nums = 0;
        max_white.col = 0;
        imgpro_var.midline_error = 0.0f;
        return;
    }

    get_color_image(); // 得到彩色图像，逐飞助手看彩图的时候打开就行了
    get_reference_point();

    // 第一遍完整搜线：最长白列 -> 原始边线 -> 滤波边线/中线。
    // 注意：element_process() 依赖 filter_left/right 的 sign，必须放在滤波之后。
    if(!run_lane_scan_once())
    {
        printf("\r\n");
        printf("%s:%s:%d,Error: 未找中线到有效行\r\n", __FILE__, __FUNCTION__, __LINE__);
        printf("\r\n");
        imgpro_var.midline_error = 0.0f;
        imgpro_var.protect_flag = true;
        return;
    }

    // 避障期间保持已有边线和中线，不让环岛/十字后处理覆盖避障控制中线。
    // 非避障状态保持原有元素处理和环岛黑线二次重扫流程。
    if(!obstacle_detect_is_active() && !ramp_block_elements)
    {
        // 元素处理必须在基础滤波边线生成之后执行，否则环岛模块读到的 filter_left/right.sign 全是 0。
        element_process();
        // 元素黑线二次重扫：十字现在已经直接写入 filter_left/right，
        // 这里只保留环岛黑线重扫逻辑。失败则恢复 element_process() 后的备份结果。
        backup_lane_scan_result();
        element_circle_draw_debug_struct dbg = {0};
        element_get_circle_draw_debug(&dbg);
        run_right_enter_blackline_rescan(&dbg);
    }

    // 视觉避障必须在所有元素后处理，确保最终 filter_midline 是控制、屏幕和上位机共同使用的中线。
    if(!ramp_block_elements)
    {
        obstacle_detect_update();
    }

    // 避障期间跳过 element_process()，因此由图像主流程继续推进蜂鸣器的非阻塞关闭计时。
    if(obstacle_active_before)
    {
        beep_update();
    }
}

void update_image_error()
{
    if (imgpro_var.protect_flag)
    {
        imgpro_var.midline_error = 0.0f;
        return;
    }


    
    //误差计算
    // 提供给控制环的核心量：图像加权中线误差。
    //只是用start_line->end_line的误差
    uint8 error_line_num = (uint8)CLAMP(max_white.nums, 0,error_maxw_end);
    imgpro_var.midline_error=calc_midline_error(g_image_info.filter_midline,error_line_num,imgpro_var.lookahead);
}

/**
 * @brief 图像处理接口，包括搜线，元素处理,绕行
 * @note 添加绕行逻辑
 *       通过图像处理，绕行得到的是，关闭正常保护，最下面是否是黑色，边线是否有效，识别到图像当前帧的参考白点，绕行回到赛道的时候是否会触发保护
 */
void image_pro_bypass()
{
    int ret;
    element_circle_draw_debug_struct dbg = {0};
    //状态机是否到左右绕行阶段
    const bool bypass_sm_active = bypass_state_active();
    //清除边线信息
    clear_line_info();
    //得到灰度图像
    ret=get_gray_image(); 
    //ret=get_color_image();
    //保护，防止处理空指针
    if(ret==-1) 
    {
        max_white.nums = 0;
        max_white.col = 0;
        imgpro_var.midline_error = 0.0f;
        imgpro_var.protect_flag=true;
        printf("\r\n");
        printf("%s:%s:%d,Error: 获取灰度图像指针失败！\r\n", __FILE__, __FUNCTION__, __LINE__);
        printf("\r\n");
        return;
    }

//保护
    //绕行时使用其他的保护，不用out_protect
    if (!bypass_sm_active) //状态机是否到绕行阶段，如果没有就正常保护，如果有就停止保护
    {
        out_protect();
    }

    if(imgpro_var.protect_flag==true)
    {
        max_white.nums = 0;
        max_white.col = 0;
        imgpro_var.midline_error = 0.0f;
        return;
    }

//参考白点
    //如果不是绕行状态的话正常更新参考白点
    if (!bypass_sm_active)
    {
        get_reference_point();
    }
    else
    {
        // 阶段1/2：搜线阈值固定为“模型识别瞬间”的锁存值，搜线需要参考白点进行搜线，绕行的时候会出赛道，所以锁存前一帧的参考白点
        imgpro_var.ref_point = s_bypass_ref_point;
        imgpro_var.white_min_point = s_bypass_white_min_point;
        imgpro_var.white_max_point = s_bypass_white_max_point;
    }

//判断最底部分是否是黑色，回到赛道的时候有没有保护的风险
    if (bypass_sm_active)
    {
        //计算五行平均值
        const float cur_bottom_avg = calc_bottom5_avg_full_width();
        s_stage_bottom_black = cur_bottom_avg < s_bypass_white_min_point;
        //保护是否触发标志位，用于回到赛道的时候稳定保护
        s_stage_out_protect_risk = would_trigger_out_protect_this_frame(); 
    }
    else
    {
        s_stage_bottom_black = false;
        s_stage_out_protect_risk = false;
    }

//最长白列搜索
    search_max_white();
    s_stage_center_col_invalid = (max_white.center_col_nums < Min_white_nums-15);
    //判断边线是否无效，最长列长度小于Min_white_nums的时候就认为搜不到边线了
    s_stage_line_invalid = (max_white.nums < Min_white_nums);
    //边线无效
    if(s_stage_line_invalid)
    {
        //状态机是否到左右绕行阶段，如果没有，正常保护
        if (!bypass_sm_active)
        {
            printf("\r\n");
            printf("%s:%s:%d,Error: 未找中线到有效行\r\n", __FILE__, __FUNCTION__, __LINE__);
            printf("\r\n");
            imgpro_var.midline_error = 0.0f;//error清零
            imgpro_var.protect_flag=true;
        }
        //如果判断到了左右绕行，不保护返回状态
        //绕行的时候这里可能有问题，因为定时器可能读取到误差
        // imgpro_var.midline_error = 0.0f;//error清零
        s_stage_edge_valid = false;
        return;
    }

    // 全图搜线，得到边线和中线信息
    search_edge();
    s_stage_edge_valid = true; // 边线状态有效

    // 先生成滤波边线/中线。element_process() 内部使用的是 filter_left/right/filter_midline。
    find_middline(g_image_info.midline, g_image_info.left, g_image_info.right);
    filter_one_edge_median3(g_image_info.filter_left, g_image_info.left, max_white.nums);
    filter_one_edge_median3(g_image_info.filter_right, g_image_info.right, max_white.nums);
    find_middline(g_image_info.midline, g_image_info.filter_left, g_image_info.filter_right);
    smooth_midline_121(g_image_info.filter_midline, g_image_info.midline, max_white.nums);

    // 非模型绕行接管时，正常执行元素处理；环岛判定依赖上述 filter 边线 sign。
    if (!bypass_sm_active)
    {
        element_process();
        backup_lane_scan_result();
        element_get_circle_draw_debug(&dbg);
        run_right_enter_blackline_rescan(&dbg);
    }
}

/**
 * @brief 更新控制环使用的误差
 * @note 添加绕行状态控制
 */
void update_image_error_bypass()
{
    const bool lr_active = model_lr_active();//是否左右绕行
    const bool prev_lr = (s_prev_model_ctrl == 'l' || s_prev_model_ctrl == 'r');//上一次的模型控制是绕行吗
    // 只在模型绕行信号上升沿启动一次状态机，避免同一目标被重复初始化。
    if (lr_active && !prev_lr)//第一次默认prev_lr=false， lr_active=true
    {
        bypass_start_from_model(model_control_access);
    }
    s_prev_model_ctrl = model_control_access;

    //这个和image_pro分开了，所以这个函数也要加保护
    if (imgpro_var.protect_flag)
    {
        imgpro_var.midline_error = 0.0f;
        return;
    }

    if (bypass_state_active())
    {
        // 绕行接管期间：误差完全由状态机输出，不再使用普通搜线中线误差。
        bypass_state_machine_step();
        imgpro_var.midline_error = s_bypass_output_error;
        return;
    }
    if (s_bypass_state == BYPASS_RECOVER)
    {
        bypass_state_machine_step();
    }
    //没有绕行状态正常巡线
    uint8 error_line_num = (uint8)CLAMP(max_white.nums, 0,error_maxw_end);
    imgpro_var.midline_error = calc_midline_error(g_image_info.filter_midline, error_line_num, imgpro_var.lookahead);
}

/**
 * @brief 出界保护，动态保护，得到前五行的平均像素点，阈值小于第一行平均阈值像素点一定的数目的话就直接保护,第一帧的图像直接计算平均阈值
 * @note 保护方式是上一帧计算完图像后，下一帧判断小于平均阈值像素点的个数
 */
// 重置图像保护内部状态。
// 常用场景：上电初始化、手动解除保护后。
void image_protect_reset_state(void)
{
    out_protect_mid_average_thre = 0.0f;
    out_protect_over_frame_count = 0;
    out_protect_ready = false;
}

// 自适应图像异常保护：
// 1）统计底部 5 行平均亮度，作为环境基线。
// 2）统计明显低于基线的暗像素数量。
// 3）连续多帧异常才触发保护，抑制单帧噪声误判。
void out_protect()
{
    int over_count = 0;
    float sum_thre = 0.0f;
    float cur_bottom_avg;
    int dark_thre;

    for(uint8 row = IMG_PROC_HEIGHT - 1; row > IMG_PROC_HEIGHT - 6; row--)
    {
        int thre = 0;
        for(uint8 col = 0; col < IMG_PROC_WIDTH; col++)
        {
            thre += AT(gray_image_proc,row,col);
        }
        sum_thre += (float)thre / IMG_PROC_WIDTH;
    }
    cur_bottom_avg = sum_thre / 5.0f;

    if(!out_protect_ready)
    {
        out_protect_mid_average_thre = cur_bottom_avg;
        out_protect_ready = true;
        return;
    }

    // 暗阈值 = 历史基线 - 20，阈值 20 可按环境光调整。
    dark_thre = (int)(out_protect_mid_average_thre - 20.0f);
    if(dark_thre < 0)
    {
        dark_thre = 0;
    }

    for(uint8 row = IMG_PROC_HEIGHT - 1; row > IMG_PROC_HEIGHT - 6; row--)
    {
        for(uint8 col = 0; col < IMG_PROC_WIDTH; col++)
        {
            if(AT(gray_image_proc,row,col) <= dark_thre)
            {
                over_count++;
            }
        }
    }

    if(over_count >= (IMG_PROC_WIDTH - 60) * 5)
    {
        if(out_protect_over_frame_count < 255)
        {
            out_protect_over_frame_count++;
        }
    }
    else
    {
        out_protect_over_frame_count = 0;
    }

    // 连续 3 帧异常才进入保护。
    if(out_protect_over_frame_count >= 3)
    {
        if(!imgpro_var.protect_flag)
        {
            printf("\r\n");
            printf("%s:%s:%d,Error:图像保护\r\n", __FILE__, __FUNCTION__, __LINE__);
            printf("image_protect triggered, over_count=%d, dark_thre=%d\r\n", over_count, dark_thre);
            printf("\r\n");
        }
        imgpro_var.protect_flag = true;
    }

    if(!imgpro_var.protect_flag)
    {
        // 低通更新基线：跟踪慢变化光照，抑制瞬时噪声。
        out_protect_mid_average_thre = 0.85f * out_protect_mid_average_thre + 0.15f * cur_bottom_avg;
    }
}

 /**
 * @brief 得到灰度图像
 */
int get_gray_image()
{
    // 灰度图像转换，彩图->灰度（原图 320x240）
    uint8* temp_ptr = uvc_dev.get_gray_image_ptr();
    if(temp_ptr == nullptr )
    {
        printf("\r\n");
        printf("%s:%s:%d,Error: 获取灰度图像指针失败！\r\n", __FILE__, __FUNCTION__, __LINE__);
        printf("\r\n");
        gray_image = nullptr;
        gray_image_proc = nullptr;
        return -1;
    }

    // 保留原图灰度给调试/TCP显示使用。
    gray_image = temp_ptr;

    // 每 2 行 2 列下采样成 160x120，小图用于 image.cpp 内搜线算法。
    for (int row = 0; row < IMG_PROC_HEIGHT; ++row)
    {
        const int src_r = row * IMG_PROC_DOWNSAMPLE;
        // 取原图中两行的首地址
        const uint8* src_row0 = temp_ptr + src_r * UVC_WIDTH;
        const uint8* src_row1 = temp_ptr + (src_r + 1) * UVC_WIDTH;
        uint8* dst_row = g_gray_image_proc + row * IMG_PROC_WIDTH;
        for (int col = 0; col < IMG_PROC_WIDTH; ++col)
        {
            const int src_c = col * IMG_PROC_DOWNSAMPLE;
            const int sum = static_cast<int>(src_row0[src_c]) +
                            static_cast<int>(src_row0[src_c + 1]) +
                            static_cast<int>(src_row1[src_c]) +
                            static_cast<int>(src_row1[src_c + 1]);
            dst_row[col] = static_cast<uint8>((sum + 2) >> 2); // 四点均值并四舍五入
        }
    }
    gray_image_proc = g_gray_image_proc;

    // 普通帧默认显示这张 160x120 搜线图。
    // 如果本帧后面进入环岛黑线重扫，会被替换成画完补线后的重扫输入图。
    memcpy(s_debug_scan_gray,
           g_gray_image_proc,
           (size_t)(IMG_PROC_WIDTH * IMG_PROC_HEIGHT) * sizeof(uint8));
    s_debug_scan_gray_valid = true;

    return 0;
}

/**
 * @brief 得到彩色图像
 */
int get_color_image()
{
    uint16* temp_ptr  = uvc_dev.get_rgb_image_ptr();
    if(temp_ptr == nullptr )
    {
        printf("\r\n");
        printf("%s:%s:%d,Error: 获取彩色图像指针失败！\r\n", __FILE__, __FUNCTION__, __LINE__);
        printf("\r\n");
        color_image = nullptr;
        return -1;
    }
    color_image = temp_ptr;
    return 0;
}

/**
 * @brief 清除边线信息
 * @note 左右边线标志位清零,边线和中线可以不清零，因为每次都会更新，我这里清零是为了上位机好观察,memset清零，效率高于for循环
 */
void clear_line_info()
{
    memset(g_image_info.left, 0, sizeof(g_image_info.left));
    memset(g_image_info.filter_left, 0, sizeof(g_image_info.filter_left));
    memset(g_image_info.right, 0, sizeof(g_image_info.right));
    memset(g_image_info.filter_right, 0, sizeof(g_image_info.filter_right));
    memset(g_image_info.midline, 0, sizeof(g_image_info.midline));
    memset(g_image_info.filter_midline, 0, sizeof(g_image_info.filter_midline));
}

/**
 * @brief 得到参考点
 * @note 参考点是根据底部几行的平均亮度计算得到的，作为后续边线搜索的基准，动态调整适应不同光照条件；同时根据参考点计算白点的最大最小值，辅助边线判断。
 */
//先根据图像底部几行的平均亮度，动态估计这一帧里“白色区域”的亮度基准
void get_reference_point()
{
    uint16 count_num=UVC_USE_WIDTH*REFPCNTROW;//保存统计点的数量
    uint32 sum_val=0;//保存所有统计点加起来的和
    for(uint8 row = IMG_PROC_HEIGHT - 1; row > IMG_PROC_HEIGHT -REFPCNTROW- 1; row--)
    {
        for(uint8 col = USE_WIDTH_L; col < USE_WIDTH_R; col++)
        {
            sum_val += AT(gray_image_proc,row,col);
        }
    }
    imgpro_var.ref_point = (uint8)(sum_val/count_num);//计算平均值，得到参考点
    imgpro_var.white_max_point=(uint8)CLAMP(imgpro_var.ref_point*WHITEMAXMUL/10,BLACK_THRESHOLD,255);//根据参考点计算白点最大值
    imgpro_var.white_min_point=(uint8)CLAMP(imgpro_var.ref_point*WHITEMINMUL/10,BLACK_THRESHOLD,255);//根据参考点计算白点最小值
    // ips200.show_string(0, 0, "REF");
    // ips200.show_int(48, 0, imgpro_var.ref_point, 3);
    // ips200.show_string(0,16, "WMAX");
    // ips200.show_int(48, 16, imgpro_var.white_max_point, 3);
    // ips200.show_string(0, 32, "WMIN");
    // ips200.show_int(48, 32, imgpro_var.white_min_point, 3); 
}

/**
 * @brief 搜索最长白列，得到最长白列的行和列
 * @note 从起始行搜索到最后一行，最上面的用于判断元素，转向判断是start_line --> end_line，但是实际上是搜索到5行，因为进行5行的差比和，这个影响不大，最上面的几行可以舍去
 * @check ok，代码没有问题 data:7.07
 */
void search_max_white()
{
    max_white.nums=0;
    max_white.col=0;
    max_white.center_col_nums=0;
    uint8 row_now,row_tmp;
    int16 row_diff;
    //隔行进行最长白列的搜索
    for(uint8 col=Max_white_L_limit;col<=Max_white_R_limit;col+=MAXWOFFSET)
    {
        uint8 white_num=0;
        //最长白列全图搜索
        for(uint8 row=IMG_PROC_HEIGHT-1;row>imgpro_var.diff_sum_mov;row-=MAXWOFFSET)
        {
            row_now=AT(gray_image_proc,row,col);
            row_tmp=AT(gray_image_proc,row-imgpro_var.diff_sum_mov,col);
            if(row_tmp>imgpro_var.white_max_point)
            {
                continue;
            }
            if(row_now<imgpro_var.white_min_point)
            {
                white_num=IMG_PROC_HEIGHT-1-row;
                break;
            }
            row_diff=GRAY_DIFF_SUM((int32)row_now,row_tmp);
            if(row_diff>imgpro_var.mw_diff_thershold)
            {
                white_num=IMG_PROC_HEIGHT-2-row+imgpro_var.diff_sum_mov;
                break;
            }
        }
        if (white_num > max_white.nums)
        {
            max_white.nums = white_num;
            max_white.col  = col;
        }
        if (col == kBypassStage1CenterCol)
        {
            max_white.center_col_nums = white_num;
        }
    }
    // ips200.show_string(0, 48, "MWHT");
    // ips200.show_int(48, 48, max_white.nums, 3); 
    // ips200.show_string(0, 64, "MWCOL");
    // ips200.show_int(48, 64, max_white.col, 3);
}

/**
 * @brief 搜索边线
 */
int search_edge()
{
    uint8 row=IMG_PROC_HEIGHT-1;
    uint8 col_now,col_now_1,col_now_2,col_tmp;
    int16 col_diff;
    uint8 diff=imgpro_var.diff_sum_mov;
    //搜索第一行
    bool start_left_found = false;
    bool start_right_found = false;
    //从左边0开始向右推进，直到找到第一个边界点，或者超过搜索范围
    g_image_info.left[0].pos[0]=USE_WIDTH_L;
    g_image_info.right[0].pos[0]=USE_WIDTH_R-1;
    //搜索左边界，在第一个范围是8-3，然后找到左黑右白的时候就是左边界，然后范围是从左到右的累加的2*8，然后从每个小范围是从右边向左边，比如32-8，来找白跳黑的点
    while(!start_left_found && g_image_info.left[0].pos[0] < USE_WIDTH_R-2*SEARCHWIDTH)
    {
        const int prev_left = g_image_info.left[0].pos[0];
        //第一次窗口的范围是从右到左是 USE_WIDTH_L+SEARCHWIDTH到USE_WIDTH_L + diff，目前也就是8-3，然后减到3的时候如果现在是白点的话，那就是丢线
        int l_col_start = CLAMP(prev_left + SEARCHWIDTH, (int)(USE_WIDTH_L + diff), (int)(USE_WIDTH_R - 1));
        int l_col_end   = CLAMP(prev_left - SEARCHWIDTH, (int)(USE_WIDTH_L + diff), (int)(USE_WIDTH_R - 1));
        if(l_col_start < l_col_end)
        {
            int tmp = l_col_start;
            l_col_start = l_col_end;
            l_col_end = tmp;
        }
        for(int col = l_col_start; col >= l_col_end; col--)
        {            
            col_now = AT(gray_image_proc, row, col);
            col_tmp = AT(gray_image_proc, row, col - diff);
            if(col_now <= imgpro_var.white_min_point)//黑点，说明没有找到边线
            {                
               continue;
            }
            if(col_now>imgpro_var.white_min_point)//白点，说明可能找到了边线
            {
                if(col <= USE_WIDTH_L + diff)
                {
                    start_left_found = true;//这个时候是丢线，先退出
                    g_image_info.left[0].sign=0;
                    break;
                }
                if(col_tmp > imgpro_var.white_min_point)
                {
                    continue;
                }
                col_diff = GRAY_DIFF_SUM((int32)col_now, col_tmp);
                if(col_diff > imgpro_var.diff_thershold)
                {
                    start_left_found = true;
                    g_image_info.left[0].pos[0] = col - diff+2;
                    g_image_info.left[0].sign=1;
                    break;
                }
            }
        }
        if(!start_left_found)
        {
            g_image_info.left[0].pos[0]+=2*SEARCHWIDTH;
        }
    }
    //从右边159开始向左推进，直到找到第一个边界点，或者超过搜索范围
    //现在右边界搜索是窗口从最右边到最左边，每个窗口是从左到右找白黑跳变点
    while(!start_right_found && g_image_info.right[0].pos[0] > USE_WIDTH_L+2*SEARCHWIDTH)
    {
        const int prev_right = g_image_info.right[0].pos[0];
        int r_col_start = CLAMP(prev_right - SEARCHWIDTH, (int)USE_WIDTH_L, (int)(USE_WIDTH_R - 1 - diff));
        int r_col_end   = CLAMP(prev_right + SEARCHWIDTH, (int)USE_WIDTH_L, (int)(USE_WIDTH_R - 1 - diff));
        if(r_col_start > r_col_end)
        {
            int tmp = r_col_start;
            r_col_start = r_col_end;
            r_col_end = tmp;
        }
        for(int col = r_col_start; col <= r_col_end; col++)
        {            
            col_now = AT(gray_image_proc, row, col);
            col_tmp = AT(gray_image_proc, row, col + diff);
            if(col_now <= imgpro_var.white_min_point)//黑点，说明没有找到边线
            {                
               continue;
            }
            if(col_now>imgpro_var.white_min_point)//白点，说明可能找到了边线
            {
                if(col >= USE_WIDTH_R - 1 - diff)
                {
                    start_right_found = true;//这个时候是丢线，先退出
                    g_image_info.right[0].sign=0;
                    break;
                }
                if(col_tmp > imgpro_var.white_min_point)
                {
                    continue;
                }
                col_diff = GRAY_DIFF_SUM((int32)col_now, col_tmp);
                if(col_diff > imgpro_var.diff_thershold)
                {
                    start_right_found = true;
                    g_image_info.right[0].pos[0] = col + diff-2;
                    g_image_info.right[0].sign=1;
                    break;
                }
            }
        }
        if(!start_right_found)
        {
            g_image_info.right[0].pos[0]-=2*SEARCHWIDTH;
        }
    }
    if(g_image_info.left[0].sign==0)  g_image_info.left[0].pos[0]=USE_WIDTH_L;
    if(g_image_info.right[0].sign==0)   g_image_info.right[0].pos[0]=USE_WIDTH_R-1;
    g_image_info.left[0].pos[1]=g_image_info.right[0].pos[1]=row;
    //搜索第一行结束

    //搜索剩下的行
    uint8 max_len = max_white.nums;
    for(uint8 seekline_num = 1; seekline_num < max_len; seekline_num++)
    {
        const int row = (int)(IMG_PROC_HEIGHT-1) - (int)seekline_num;
        int cur_left = USE_WIDTH_L;
        int cur_right = USE_WIDTH_R - 1;
        bool cur_left_found = false;
        bool cur_right_found = false;

        const int prev_left = g_image_info.left[seekline_num - 1].pos[0];
        const int prev_right = g_image_info.right[seekline_num - 1].pos[0];

        // 左边跟踪：先在上一行附近搜索
        int l_col_start = CLAMP(prev_left + SEARCHWIDTH, (int)(USE_WIDTH_L + diff), (int)(USE_WIDTH_R - 2));
        int l_col_end   = CLAMP(prev_left - SEARCHWIDTH, (int)(USE_WIDTH_L + diff), (int)(USE_WIDTH_R - 2));
        if(l_col_start < l_col_end)
        {
            int tmp = l_col_start;
            l_col_start = l_col_end;
            l_col_end = tmp;
        }

        for(int col = l_col_start; col >= l_col_end; col--)
        {
            col_now = AT(gray_image_proc, row, col);
            col_now_1 = AT(gray_image_proc, row, col-1);
            col_now_2 = AT(gray_image_proc, row, col+1);
            col_tmp = AT(gray_image_proc, row, col - diff);
            //边界处理
            if(col <= USE_WIDTH_L + diff)
            {
                cur_left = col-diff+2;
                cur_left_found = true;//这个时候是丢线，搜索到了边界，先退出
                g_image_info.left[seekline_num].sign=0;
                break;
            }
            if(col_tmp > imgpro_var.white_max_point)
            {
                continue;
            }
            //可能会被白赛道上的黑点影响,3个点判断
            if(col_now < imgpro_var.white_min_point 
                && col_now_1 < imgpro_var.white_min_point
                && col_now_2 < imgpro_var.white_min_point)//直接从最长白列开始搜索
            {
                break;
            }
            col_diff = (int16)GRAY_DIFF_SUM((int32)col_now, col_tmp);
            if(col_diff > imgpro_var.diff_thershold)
            {
                cur_left = col - diff+2;
                cur_left_found = true;
                g_image_info.left[seekline_num].sign=1;
                break;
            }
        }

        // 左边回退：局部跟踪失败时，从最长白列开始向左搜索，情况有两种，一种是三点都是黑色的，一周是跟踪上一条边界划定的范围内没有边界
        if(!cur_left_found)
        {
            l_col_start = CLAMP((int)max_white.col, (int)(USE_WIDTH_L + diff), (int)(USE_WIDTH_R - 1));
            l_col_end = (int)USE_WIDTH_L + diff;
            for(int col = l_col_start; col >= l_col_end; col--)
            {
                col_now = AT(gray_image_proc, row, col);
                col_tmp = AT(gray_image_proc, row, col - diff);
                if(col_tmp > imgpro_var.white_max_point)
                {
                    continue;
                }
                if(col_now < imgpro_var.white_min_point)
                {
                    cur_left = col+2;
                    cur_left_found = true;
                    g_image_info.left[seekline_num].sign=1;
                    break;
                }
                col_diff = (int16)GRAY_DIFF_SUM((int32)col_now, col_tmp);
                if(col_diff > imgpro_var.diff_thershold)
                {
                    cur_left = col - diff+2;
                    cur_left_found = true;
                    g_image_info.left[seekline_num].sign=1;
                    break;
                }
            }
        }

        // 右边跟踪：先在上一行附近搜索
        int r_col_start = CLAMP(prev_right - SEARCHWIDTH, (int)(USE_WIDTH_L+1), (int)(USE_WIDTH_R - 1 - diff));
        int r_col_end   = CLAMP(prev_right + SEARCHWIDTH, (int)(USE_WIDTH_L+1), (int)(USE_WIDTH_R - 1 - diff));
        if(r_col_start > r_col_end)
        {
            int tmp = r_col_start;
            r_col_start = r_col_end;
            r_col_end = tmp;
        }

        for(int col = r_col_start; col <= r_col_end; col++)
        {
            col_now = AT(gray_image_proc, row, col);
            col_now_1 = AT(gray_image_proc, row, col-1);
            col_now_2 = AT(gray_image_proc, row, col+1);
            col_tmp = AT(gray_image_proc, row, col + diff);
            //边界处理
            if(col>= USE_WIDTH_R - 1 - diff)
            {
                cur_right = col-diff-2;//这个时候是丢线，先退出
                cur_right_found = true;
                g_image_info.right[seekline_num].sign=0;
                break;
            }
            if(col_tmp > imgpro_var.white_max_point)
            {
                continue;
            }
            //可能会被白赛道上的黑点影响,3个点判断
            if(col_now < imgpro_var.white_min_point                
                && col_now_1 < imgpro_var.white_min_point
                && col_now_2 < imgpro_var.white_min_point)//搜索失败
            {
                break;
            }
            col_diff = (int16)GRAY_DIFF_SUM((int32)col_now, col_tmp);
            if(col_diff > imgpro_var.diff_thershold)
            {
                cur_right = col + diff-2;
                cur_right_found = true;
                g_image_info.right[seekline_num].sign=1;
                break;
            }
        }

        // 右边回退：局部跟踪失败时，从最长白列开始向右搜索
        if(!cur_right_found)
        {
            r_col_start = CLAMP((int)max_white.col, (int)USE_WIDTH_L, (int)(USE_WIDTH_R - 1 - diff));
            r_col_end = (int)USE_WIDTH_R - 1 - diff;
            for(int col = r_col_start; col <= r_col_end; col++)
            {
                col_now = AT(gray_image_proc, row, col);
                col_tmp = AT(gray_image_proc, row, col + diff);
                if(col_now < imgpro_var.white_min_point)
                {
                    cur_right = col-2;
                    cur_right_found = true;
                    g_image_info.right[seekline_num].sign=1;
                    break;
                }
                if(col_tmp > imgpro_var.white_max_point)
                {
                    continue;
                }
                col_diff = (int16)GRAY_DIFF_SUM((int32)col_now, col_tmp);
                if(col_diff > imgpro_var.diff_thershold)
                {
                    cur_right = col + diff-2;
                    cur_right_found = true;
                    g_image_info.right[seekline_num].sign=1;
                    break;
                }
            }
        }

        cur_left = CLAMP(cur_left, USE_WIDTH_L, USE_WIDTH_R - 1);
        cur_right = CLAMP(cur_right, USE_WIDTH_L, USE_WIDTH_R - 1);

        // 极端情况下本行边界交叉，回退到上一行结果
        if(cur_right <= cur_left)
        {
            cur_left = g_image_info.left[seekline_num - 1].pos[0];
            cur_right = g_image_info.right[seekline_num - 1].pos[0];
            cur_left_found = false;
            cur_right_found = false;
        }

        g_image_info.left[seekline_num].pos[0]  = (uint8)cur_left;
        g_image_info.left[seekline_num].pos[1]  = (uint8)row;
        g_image_info.right[seekline_num].pos[0] = (uint8)cur_right;
        g_image_info.right[seekline_num].pos[1] = (uint8)row;

        if(g_image_info.right[seekline_num].sign==0)
        {
            g_image_info.right[seekline_num].pos[0]=USE_WIDTH_R-1;
        }
        if(g_image_info.left[seekline_num].sign==0)
        {
            g_image_info.left[seekline_num].pos[0]=USE_WIDTH_L;
        }
    }
    return 0;
}

/**
 * @brief 得到中线，直接左右边线相加除以2
 * @note 用于测试
 */
void find_middline(uint8 Midline[][2],BoundaryInfo edge1[],BoundaryInfo edge2[])
{
    for(uint8 i=0;i<max_white.nums;i++){
        if(edge1[i].sign == 0U && edge2[i].sign == 0U)
        {
            Midline[i][0] = USE_MIDDLE_VALUE;
        }
        else
        {
            Midline[i][0]=(edge1[i].pos[0]+edge2[i].pos[0])/2;
        }
        Midline[i][1]=edge1[i].pos[1];
    }
}

/**
 * @brief 找出三个整数 a、b、c 中的中位数
 * @note 静态限制作用域，本文件；内联加快函数调用速度
 */
static inline int median3(int a, int b, int c)
{
    if (a > b) { int t = a; a = b; b = t; }
    if (b > c) { int t = b; b = c; c = t; }
    if (a > b) { int t = a; a = b; b = t; }
    return b;
}

/**
 * @brief 3点中值滤波滤除脉冲噪声
 * @note 只滤真实边和预测边，丢线点不滤
 */
void filter_one_edge_median3(BoundaryInfo filter_edge[],BoundaryInfo edge[],uint8 valid_len)
{
    for(uint8 i=0;i<valid_len;i++)
    {
        filter_edge[i].pos[0]=edge[i].pos[0];
        filter_edge[i].pos[1]=edge[i].pos[1];
        filter_edge[i].sign=edge[i].sign;
    }
    for(int i = 1; i < valid_len - 1; i++)
    {
        if(edge[i].sign == 0)
            continue;

        int x0 = edge[i].pos[0];
        int x1 = edge[i].pos[0];
        int x2 = edge[i].pos[0];

        if(edge[i-1].sign != 0) x0=edge[i-1].pos[0];
        if(edge[i+1].sign != 0) x2=edge[i+1].pos[0];
        
        filter_edge[i].pos[0] = (uint8)median3(x0, x1, x2);
    }
}

/**
 * @brief 1-2-1 中线轻量滤波
 */
void smooth_midline_121(uint8 filter_line[][2],uint8 line[][2],uint8 valid_len)
{
    for(int i = 0; i < valid_len; i++)
    {
        filter_line[i][0]= line[i][0];
        filter_line[i][1]= line[i][1];
    }

    for(int i = 1; i < valid_len - 1; i++)
    {
        int val = (int)line[i-1][0] + 2 * line[i][0] + line[i + 1][0];
        val /= 4;
        filter_line[i][0] = (uint8)CLAMP(val, USE_WIDTH_L, USE_WIDTH_R - 1);
    }
}

/**
 * @brief 权重计算，计算加权误差
 * @param Midline[][2] 想要计算error的中线，可以是滤波后的也可以传滤波前的
 * @param nums 计算中线的行数
 * @param lookaheadline 固定的前瞻距离
 * @note 调整lookaheadline来调整error，lookaheadline 固定了，权重就固定了，优化的话可以初始化就构建一个权重数组，这里就不用每一次就计算了
 */
float calc_midline_error(uint8 Midline[][2],uint8 nums,int lookaheadline)
{  
    float weightSum = 0, lineSum = 0;
    float mid_error=0;
    float sigma = 10.0f;//默认高斯参数
    if(nums>5)//最长白列的行数不能小于5
    {
        for(uint8 i=error_maxw_start;i<nums;i++)
        {
            int dis= abs((int)Midline[i][1]-lookaheadline);
            float x = (float)dis / sigma;
            float weight = expf(-0.5f * x * x);
            weightSum += weight;
            lineSum += (float)weight * (Midline[i][0]- USE_MIDDLE_VALUE-1);
        }
        if (weightSum > 1e-4) // 避免除以过小的值
        {
            mid_error = lineSum / weightSum;
            // mid_error=mid_error*0.7+last_mid_error*0.3;//中线error平滑，使用会让响应变迟钝
        }
        else
        {
            mid_error=0;
        }
    }
    else
    {
        mid_error=0;
    }
    mid_error=mid_error/USE_MIDDLE_VALUE;//归一化error
    return mid_error;
}

//写完这个去写控制，可以先进行测试了
