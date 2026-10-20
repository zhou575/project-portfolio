#ifndef _RAMP_DETECT_HPP_
#define _RAMP_DETECT_HPP_

#include "zf_common_typedef.hpp"

// 初始调试参数，最终需要结合实测日志调整。
// 车辆运动时允许边线因振动、运动模糊出现一定比例的丢线。
// 数值表示无效行比例上限；0.45 表示最多允许约 45% 的统计行无效。
#define RAMP_LEFT_INVALID_LIMIT       0.20f
#define RAMP_RIGHT_INVALID_LIMIT      0.20f

// 车辆运动时图像容易抖动，降低白列门槛，避免 Wok=0 导致视觉条件失效。
#define RAMP_LONG_WHITE_COLUMN_MIN   90

// 统计边线有效率时，忽略可靠区域两端的行数，避免车身和消失点干扰。
#define RAMP_VISUAL_IGNORE_ROWS       5

// 边线直线拟合参数：允许图像抖动，同时用前后段斜率差区分弯道。
#define RAMP_LINE_FIT_SAMPLE_STEP     4
#define RAMP_LINE_FIT_MIN_SAMPLES     8
#define RAMP_LINE_FIT_MAX_ERROR       12.0f
#define RAMP_LINE_FIT_MIN_VALID_RATIO 0.55f
#define RAMP_LINE_FIT_MAX_SLOPE_DELTA 0.35f

// ToF 滤波后距离小于该值，才认为坡道入口条件成立。
#define RAMP_TOF_FILTERED_ENTER_THRESHOLD 380.0f

// 坡道中阶段：相对进入坡道时的 pitch 出现明显上升，再回到入口附近后进入预退出。
#define RAMP_PITCH_RISE_THRESHOLD     8.0f
#define RAMP_PITCH_NEAR_THRESHOLD     2.0f
#define RAMP_PITCH_NEAR_CONFIRM_COUNT 3U

// 预退出阶段编码器累计距离超过该值后，彻底退出坡道。
#define RAMP_PRE_EXIT_ENCODER_DISTANCE 6000.0f

// 初始化后车辆先行驶一小段距离，再允许坡道寻找，避免静止上电误触发蜂鸣。
#define RAMP_ARM_ENCODER_DISTANCE      0.0f

// 连续满足 ToF 滤波距离门槛的样本数量。
#define RAMP_CONFIRM_COUNT            3

// 已确认坡道后，ToF 连续恢复到非坡道范围多少个采样才允许下一次判定。
#define RAMP_REARM_COUNT              10

// 坡道蜂鸣器开关：0=关闭，1=开启。
#define RAMP_BEEP_ENABLE              1

// 串口实时调试：每个 ToF 新样本打印一次，ToF 周期为 10 ms。
// 打印在主循环执行，不放入 2 ms 中断。
#define RAMP_DEBUG_SERIAL_ENABLE       1
#define RAMP_DEBUG_PRINT_EVERY_SAMPLES 1

enum tof_ramp_state_enum
{
    TOF_RAMP_CALIBRATING = 0,
    TOF_RAMP_FLAT,
    TOF_RAMP_SUSPECT,
    TOF_RAMP_ON_RAMP,
    TOF_RAMP_PRE_EXIT
};

extern tof_ramp_state_enum tof_ramp_state;
extern bool tof_ramp_detected;
extern uint8 tof_ramp_suspect_count;

extern float tof_distance_rise;
extern float tof_rise_threshold;
extern float tof_distance_drop;
extern float tof_drop_threshold;
extern float tof_trend_rise;
extern float tof_filtered_enter_threshold;
extern bool tof_rise_ok;
extern bool tof_drop_ok;
extern bool tof_filtered_enter_ok;
extern bool tof_distance_match_ok;
extern bool tof_baseline_near;
extern float ramp_entry_pitch;
extern float ramp_pitch_relative;
extern float ramp_arm_encoder_distance;
extern float ramp_pre_exit_encoder_distance;
extern bool ramp_pitch_peak_seen;

extern float ramp_left_invalid_ratio;
extern float ramp_right_invalid_ratio;
extern bool ramp_left_edge_quality_ok;
extern bool ramp_right_edge_quality_ok;
extern bool ramp_left_line_straight_ok;
extern bool ramp_right_line_straight_ok;
extern bool ramp_enough_long_white_columns;
extern bool visual_ramp_allow;

void ramp_tof_init(void);
void ramp_tof_update_visual_conditions(void);
bool ramp_tof_should_sample(void);
void ramp_tof_update(bool tof_sample_ready);
bool ramp_speed_limit_active(void);
bool ramp_blocks_other_elements(void);
void ramp_tof_debug_print(void);
void ramp_tof_reset(void);

#endif
