#include "ramp_detect.hpp"

#include "beep.hpp"
#include "car_init.hpp"
#include "d1lb_tof.hpp"
#include "element.hpp"
#include "encoder.hpp"
#include "image.hpp"
#include "imu.hpp"
#include "obstacle_detect.hpp"

#include <math.h>

tof_ramp_state_enum tof_ramp_state = TOF_RAMP_CALIBRATING;
bool tof_ramp_detected = false;
uint8 tof_ramp_suspect_count = 0U;

float tof_distance_rise = 0.0f;
float tof_rise_threshold = 0.0f;
float tof_distance_drop = 0.0f;
float tof_drop_threshold = 0.0f;
float tof_trend_rise = 0.0f;
float tof_filtered_enter_threshold = RAMP_TOF_FILTERED_ENTER_THRESHOLD;
bool tof_rise_ok = false;
bool tof_drop_ok = false;
bool tof_filtered_enter_ok = false;
bool tof_distance_match_ok = false;
bool tof_baseline_near = false;
float ramp_entry_pitch = 0.0f;
float ramp_pitch_relative = 0.0f;
float ramp_arm_encoder_distance = 0.0f;
float ramp_pre_exit_encoder_distance = 0.0f;
bool ramp_pitch_peak_seen = false;

float ramp_left_invalid_ratio = 1.0f;
float ramp_right_invalid_ratio = 1.0f;
bool ramp_left_edge_quality_ok = false;
bool ramp_right_edge_quality_ok = false;
bool ramp_left_line_straight_ok = false;
bool ramp_right_line_straight_ok = false;
bool ramp_enough_long_white_columns = false;
bool visual_ramp_allow = false;

static float s_last_tof_distance = 0.0f;
static bool s_last_tof_distance_valid = false;
static uint8 s_ramp_pitch_near_count = 0U;
static float s_ramp_arm_left_start_total = 0.0f;
static float s_ramp_arm_right_start_total = 0.0f;
static float s_pre_exit_left_start_total = 0.0f;
static float s_pre_exit_right_start_total = 0.0f;

static void ramp_reset_history(void);

static void ramp_clear_tof_match_flags(void)
{
    tof_rise_ok = false;
    tof_drop_ok = false;
    tof_filtered_enter_ok = false;
    tof_distance_match_ok = false;
    tof_baseline_near = false;
}

static void ramp_reset_stage_info(void)
{
    ramp_entry_pitch = 0.0f;
    ramp_pitch_relative = 0.0f;
    ramp_arm_encoder_distance = 0.0f;
    ramp_pre_exit_encoder_distance = 0.0f;
    ramp_pitch_peak_seen = false;
    s_ramp_pitch_near_count = 0U;
    s_pre_exit_left_start_total = 0.0f;
    s_pre_exit_right_start_total = 0.0f;
}

static void ramp_reset_arm_distance(void)
{
    s_ramp_arm_left_start_total = encoder_left_abs_total;
    s_ramp_arm_right_start_total = encoder_right_abs_total;
    ramp_arm_encoder_distance = 0.0f;
}

static bool ramp_arm_distance_ready(void)
{
    const float left_distance =
        encoder_left_abs_total - s_ramp_arm_left_start_total;
    const float right_distance =
        encoder_right_abs_total - s_ramp_arm_right_start_total;
    const float left_use = (left_distance > 0.0f) ? left_distance : 0.0f;
    const float right_use = (right_distance > 0.0f) ? right_distance : 0.0f;
    ramp_arm_encoder_distance = 0.5f * (left_use + right_use);
    return ramp_arm_encoder_distance >= RAMP_ARM_ENCODER_DISTANCE;
}

static void ramp_enter_on_ramp(void)
{
    tof_ramp_state = TOF_RAMP_ON_RAMP;
    tof_ramp_detected = true;
    ramp_entry_pitch = pitch;
    ramp_pitch_relative = 0.0f;
    ramp_pitch_peak_seen = false;
    s_ramp_pitch_near_count = 0U;
    ramp_pre_exit_encoder_distance = 0.0f;
}

static void ramp_enter_pre_exit(void)
{
    tof_ramp_state = TOF_RAMP_PRE_EXIT;
    tof_ramp_detected = true;
    s_pre_exit_left_start_total = encoder_left_abs_total;
    s_pre_exit_right_start_total = encoder_right_abs_total;
    ramp_pre_exit_encoder_distance = 0.0f;
}

static void ramp_finish_exit(void)
{
    tof_ramp_state = TOF_RAMP_FLAT;
    tof_ramp_detected = false;
    tof_ramp_suspect_count = 0U;
    ramp_clear_tof_match_flags();
    ramp_reset_history();
    ramp_reset_stage_info();
    ramp_reset_arm_distance();
}

static bool ramp_fit_segment_slope(const BoundaryInfo *edge,
                                   int row_start,
                                   int row_end,
                                   float *slope_out)
{
    int valid_count = 0;
    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;

    for(int row = row_start; row < row_end; row += RAMP_LINE_FIT_SAMPLE_STEP)
    {
        if(edge[row].sign != 0U)
        {
            const float y = (float)row;
            const float x = (float)edge[row].pos[0];
            sum_y += y;
            sum_x += x;
            sum_yy += y * y;
            sum_yx += y * x;
            valid_count++;
        }
    }

    if(valid_count < 3)
    {
        return false;
    }

    const float denominator =
        (float)valid_count * sum_yy - sum_y * sum_y;
    if(fabsf(denominator) < 1.0e-4f)
    {
        return false;
    }

    *slope_out =
        ((float)valid_count * sum_yx - sum_y * sum_x) / denominator;
    return true;
}

static bool ramp_edge_is_straight(const BoundaryInfo *edge,
                                  int row_start,
                                  int row_end)
{
    int sample_count = 0;
    int valid_count = 0;
    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;

    for(int row = row_start; row < row_end; row += RAMP_LINE_FIT_SAMPLE_STEP)
    {
        sample_count++;
        if(edge[row].sign != 0U)
        {
            const float y = (float)row;
            const float x = (float)edge[row].pos[0];
            sum_y += y;
            sum_x += x;
            sum_yy += y * y;
            sum_yx += y * x;
            valid_count++;
        }
    }

    if(sample_count < RAMP_LINE_FIT_MIN_SAMPLES ||
       valid_count < RAMP_LINE_FIT_MIN_SAMPLES ||
       ((float)valid_count / (float)sample_count) < RAMP_LINE_FIT_MIN_VALID_RATIO)
    {
        return false;
    }

    const float denominator =
        (float)valid_count * sum_yy - sum_y * sum_y;
    if(fabsf(denominator) < 1.0e-4f)
    {
        return false;
    }

    const float slope =
        ((float)valid_count * sum_yx - sum_y * sum_x) / denominator;
    const float intercept =
        (sum_x - slope * sum_y) / (float)valid_count;

    float max_error = 0.0f;
    for(int row = row_start; row < row_end; row += RAMP_LINE_FIT_SAMPLE_STEP)
    {
        if(edge[row].sign != 0U)
        {
            const float expected_x = slope * (float)row + intercept;
            const float error = fabsf((float)edge[row].pos[0] - expected_x);
            if(error > max_error)
            {
                max_error = error;
            }
        }
    }

    if(max_error > RAMP_LINE_FIT_MAX_ERROR)
    {
        return false;
    }

    // 直线前后两段斜率应接近；弯道通常会出现明显的斜率变化。
    const int row_mid = row_start + (row_end - row_start) / 2;
    float first_slope = 0.0f;
    float second_slope = 0.0f;
    if(!ramp_fit_segment_slope(edge, row_start, row_mid, &first_slope) ||
       !ramp_fit_segment_slope(edge, row_mid, row_end, &second_slope))
    {
        return false;
    }

    return fabsf(first_slope - second_slope) <= RAMP_LINE_FIT_MAX_SLOPE_DELTA;
}

static void ramp_reset_history(void)
{
    s_last_tof_distance = 0.0f;
    s_last_tof_distance_valid = false;
    tof_trend_rise = 0.0f;
}

static void ramp_push_tof_history(float value)
{
    if(s_last_tof_distance_valid)
    {
        // 当前值比上一帧大，表示 ToF 测得的距离正在变大。
        tof_trend_rise = value - s_last_tof_distance;
    }
    else
    {
        s_last_tof_distance_valid = true;
        tof_trend_rise = 0.0f;
    }

    s_last_tof_distance = value;
}

void ramp_tof_init(void)
{
    tof_ramp_state = TOF_RAMP_CALIBRATING;
    tof_ramp_detected = false;
    tof_ramp_suspect_count = 0U;
    tof_distance_rise = 0.0f;
    tof_rise_threshold = 0.0f;
    tof_distance_drop = 0.0f;
    tof_drop_threshold = 0.0f;
    tof_filtered_enter_threshold = RAMP_TOF_FILTERED_ENTER_THRESHOLD;
    tof_flat_baseline = 0.0f;
    tof_baseline_ready = (NO_FIND_DEVICE != dl1x_dev_type);
    ramp_clear_tof_match_flags();
    ramp_reset_history();
    ramp_reset_stage_info();
    ramp_reset_arm_distance();

    if(tof_baseline_ready)
    {
        tof_ramp_state = TOF_RAMP_FLAT;
    }
}

void ramp_tof_update_visual_conditions(void)
{
    const int valid_len = (max_white.nums < IMG_PROC_HEIGHT)
                              ? (int)max_white.nums
                              : IMG_PROC_HEIGHT;

    // 去掉最底部和最远端各几行，避免车身区域和消失点影响统计。
    const int row_start = RAMP_VISUAL_IGNORE_ROWS;
    const int row_end = valid_len - RAMP_VISUAL_IGNORE_ROWS;

    if(row_end <= row_start)
    {
        ramp_left_invalid_ratio = 1.0f;
        ramp_right_invalid_ratio = 1.0f;
        ramp_left_edge_quality_ok = false;
        ramp_right_edge_quality_ok = false;
        ramp_left_line_straight_ok = false;
        ramp_right_line_straight_ok = false;
    }
    else
    {
        int left_invalid_count = 0;
        int right_invalid_count = 0;
        const int row_count = row_end - row_start;

        for(int i = row_start; i < row_end; i++)
        {
            if(g_image_info.left[i].sign == 0U)
            {
                left_invalid_count++;
            }
            if(g_image_info.right[i].sign == 0U)
            {
                right_invalid_count++;
            }
        }

        ramp_left_invalid_ratio =
            (float)left_invalid_count / (float)row_count;
        ramp_right_invalid_ratio =
            (float)right_invalid_count / (float)row_count;

        ramp_left_edge_quality_ok =
            ramp_left_invalid_ratio < RAMP_LEFT_INVALID_LIMIT;
        ramp_right_edge_quality_ok =
            ramp_right_invalid_ratio < RAMP_RIGHT_INVALID_LIMIT;

        // 用滤波后的边线做直线拟合，降低单点抖动对判定的影响。
        ramp_left_line_straight_ok =
            ramp_edge_is_straight(g_image_info.filter_left,
                                  row_start,
                                  row_end);
        ramp_right_line_straight_ok =
            ramp_edge_is_straight(g_image_info.filter_right,
                                  row_start,
                                  row_end);
    }

    // Min_white_nums=20 只是最低搜线门槛，坡道视觉条件使用更高的独立阈值。
    ramp_enough_long_white_columns =
        max_white.nums >= RAMP_LONG_WHITE_COLUMN_MIN;

    visual_ramp_allow =
        ramp_left_edge_quality_ok
        && ramp_right_edge_quality_ok
        && ramp_left_line_straight_ok
        && ramp_right_line_straight_ok
        && ramp_enough_long_white_columns;
}

bool ramp_tof_should_sample(void)
{
    if(!tof_baseline_ready)
    {
        return false;
    }

    if(tof_ramp_state == TOF_RAMP_ON_RAMP)
    {
        return true;
    }

    if(tof_ramp_state == TOF_RAMP_PRE_EXIT)
    {
        return false;
    }

    if(element_is_busy() || obstacle_detect_is_active())
    {
        return false;
    }

    if(!ramp_arm_distance_ready())
    {
        return false;
    }

    return true;
}

void ramp_tof_update(bool tof_sample_ready)
{
    if(!tof_baseline_ready)
    {
        tof_ramp_state = TOF_RAMP_CALIBRATING;
        return;
    }

    if(tof_ramp_state == TOF_RAMP_PRE_EXIT)
    {
        const float left_distance =
            encoder_left_abs_total - s_pre_exit_left_start_total;
        const float right_distance =
            encoder_right_abs_total - s_pre_exit_right_start_total;
        const float left_use = (left_distance > 0.0f) ? left_distance : 0.0f;
        const float right_use = (right_distance > 0.0f) ? right_distance : 0.0f;
        ramp_pre_exit_encoder_distance = 0.5f * (left_use + right_use);

        if(ramp_pre_exit_encoder_distance >= RAMP_PRE_EXIT_ENCODER_DISTANCE)
        {
            ramp_finish_exit();
        }
        return;
    }

    if(tof_ramp_state == TOF_RAMP_ON_RAMP)
    {
        ramp_pitch_relative = pitch - ramp_entry_pitch;
        if(ramp_pitch_relative >= RAMP_PITCH_RISE_THRESHOLD)
        {
            ramp_pitch_peak_seen = true;
        }

        if(ramp_pitch_peak_seen &&
           fabsf(ramp_pitch_relative) <= RAMP_PITCH_NEAR_THRESHOLD)
        {
            if(s_ramp_pitch_near_count < RAMP_PITCH_NEAR_CONFIRM_COUNT)
            {
                s_ramp_pitch_near_count++;
            }
        }
        else
        {
            s_ramp_pitch_near_count = 0U;
        }

        if(s_ramp_pitch_near_count >= RAMP_PITCH_NEAR_CONFIRM_COUNT)
        {
            ramp_enter_pre_exit();
            return;
        }

#if RAMP_BEEP_ENABLE
        beep_request_frames(2U);
#endif
    }

    // 环岛或十字处于处理状态时暂停坡道判定，避免特殊元素边线被当成坡道。
    // 退出特殊元素后重新从平地状态开始判断。
    if(tof_ramp_state != TOF_RAMP_ON_RAMP &&
       (element_is_busy() || obstacle_detect_is_active()))
    {
        tof_ramp_state = TOF_RAMP_FLAT;
        tof_ramp_detected = false;
        tof_ramp_suspect_count = 0U;
        ramp_clear_tof_match_flags();
        ramp_reset_history();
        ramp_reset_stage_info();
        return;
    }

    if(!tof_sample_ready)
    {
        return;
    }

    if(!tof_distance_valid)
    {
        return;
    }

    // 入口判据使用 ToF 低通滤波后的绝对距离，不再依赖初始化基准。
    tof_distance_rise = 0.0f;
    tof_rise_threshold = 0.0f;
    tof_distance_drop = 0.0f;
    tof_drop_threshold = 0.0f;
    tof_filtered_enter_threshold = RAMP_TOF_FILTERED_ENTER_THRESHOLD;

    ramp_push_tof_history(tof_distance_filtered);

    tof_rise_ok = false;
    tof_drop_ok = false;
    tof_filtered_enter_ok =
        tof_distance_filtered < tof_filtered_enter_threshold;
    tof_distance_match_ok = tof_filtered_enter_ok;
    tof_baseline_near = false;

    if(tof_ramp_state == TOF_RAMP_FLAT)
    {
        // ToF 滤波距离连续低于入口阈值后确认坡道。
        if(tof_distance_match_ok)
        {
            if(RAMP_CONFIRM_COUNT <= 1)
            {
                // 配置为单次确认时，当前采样直接确认，避免车辆快速通过坡道时漏判。
                ramp_enter_on_ramp();
                tof_ramp_suspect_count = 1U;
#if RAMP_BEEP_ENABLE
                beep_request();
#endif
            }
            else
            {
                tof_ramp_state = TOF_RAMP_SUSPECT;
                tof_ramp_suspect_count = 1U;
            }
        }
        return;
    }

    if(tof_ramp_state == TOF_RAMP_SUSPECT)
    {
        if(tof_distance_match_ok)
        {
            if(tof_ramp_suspect_count < RAMP_CONFIRM_COUNT)
            {
                tof_ramp_suspect_count++;
            }
        }
        else if(tof_ramp_suspect_count > 0U)
        {
            tof_ramp_suspect_count--;
        }

        if(tof_ramp_suspect_count >= RAMP_CONFIRM_COUNT)
        {
            ramp_enter_on_ramp();
#if RAMP_BEEP_ENABLE
            beep_request();
#endif
        }
        else if(tof_ramp_suspect_count == 0U)
        {
            tof_ramp_state = TOF_RAMP_FLAT;
        }
    }
}

bool ramp_speed_limit_active(void)
{
    return tof_ramp_state == TOF_RAMP_ON_RAMP;
}

bool ramp_blocks_other_elements(void)
{
    return tof_ramp_state == TOF_RAMP_ON_RAMP;
}

void ramp_tof_debug_print(void)
{
#if RAMP_DEBUG_SERIAL_ENABLE
    static uint8 sample_count = 0U;

    sample_count++;
    if(sample_count < RAMP_DEBUG_PRINT_EVERY_SAMPLES)
    {
        return;
    }
    sample_count = 0U;

    printf("TOF_MONITOR filtered=%.1f fthr=%.1f tok=%d\r\n",
           tof_distance_filtered,
           tof_filtered_enter_threshold,
           tof_filtered_enter_ok ? 1 : 0);

    printf("RAMP state=%d det=%d raw=%d med=%.1f filt=%.1f fthr=%.1f tok=%d match=%d "
           "pre=%.0f pitch=%.1f peak=%d "
           "Linv=%.2f Rinv=%.2f Lok=%d Rok=%d "
           "Ls=%d Rs=%d white=%d Wok=%d vis=%d cnt=%d\r\n",
           (int)tof_ramp_state,
           tof_ramp_detected ? 1 : 0,
           (int)dl1x_distance_raw,
           tof_distance_median,
           tof_distance_filtered,
           tof_filtered_enter_threshold,
           tof_filtered_enter_ok ? 1 : 0,
           tof_distance_match_ok ? 1 : 0,
           ramp_pre_exit_encoder_distance,
           ramp_pitch_relative,
           ramp_pitch_peak_seen ? 1 : 0,
           ramp_left_invalid_ratio,
           ramp_right_invalid_ratio,
           ramp_left_edge_quality_ok ? 1 : 0,
           ramp_right_edge_quality_ok ? 1 : 0,
           ramp_left_line_straight_ok ? 1 : 0,
           ramp_right_line_straight_ok ? 1 : 0,
           (int)max_white.nums,
           ramp_enough_long_white_columns ? 1 : 0,
           visual_ramp_allow ? 1 : 0,
           (int)tof_ramp_suspect_count);
    fflush(stdout);
#endif
}

void ramp_tof_reset(void)
{
    tof_ramp_detected = false;
    tof_ramp_suspect_count = 0U;
    ramp_clear_tof_match_flags();
    ramp_reset_history();
    ramp_reset_stage_info();
    tof_ramp_state = tof_baseline_ready
                         ? TOF_RAMP_FLAT
                         : TOF_RAMP_CALIBRATING;
}
