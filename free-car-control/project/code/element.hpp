#ifndef _ELEMENT_HPP_
#define _ELEMENT_HPP_

#include "car_init.hpp"
#include "image.hpp"

#ifndef ELEMENT_ENABLE_DEBUG_API
#define ELEMENT_ENABLE_DEBUG_API   (1)
#endif

typedef enum
{
    ELEMENT_NONE       = 0,
    ELEMENT_CROSSROAD  = 1,
    ELEMENT_ROUNDABOUT = 2,
} element_mode_enum;

typedef enum
{
    CROSS_IDLE     = 0,
    CROSS_DETECTED = 1,
    CROSS_INSIDE   = 2,
} cross_state_enum;

// 采用老环岛状态编号：奇数左环，偶数右环，7 为出环准备
// 这样和你之前屏幕上看的 cs 数值能对得上。
typedef enum
{
    CIRCLE_IDLE         = 0,
    CIRCLE_LEFT_PREP    = 1,
    CIRCLE_RIGHT_PREP   = 2,
    CIRCLE_LEFT_ENTER   = 3,
    CIRCLE_RIGHT_ENTER  = 4,
    CIRCLE_LEFT_INSIDE  = 5,
    CIRCLE_RIGHT_INSIDE = 6,
    CIRCLE_EXIT_PREP    = 7,
} circle_state_enum;

#if (1 == ELEMENT_ENABLE_DEBUG_API)
typedef struct
{
    uint8 mode;
    uint8 cross_state;
    uint8 circle_state;
    uint8 busy;

    uint8 cross_candidate;
    uint8 circle_candidate;
    uint8 cross_line_index;
    uint8 circle_lost_side;
    uint8 valid_rows;

    int16 left_lost_rows;
    int16 right_lost_rows;
    int16 both_lost_rows;
    int16 lane_width_ref;
    int16 last_midline;

    int16 search_stop_line;
    uint8 circle_flag;
    uint8 in_circle_state;
    uint8 left_state;
    uint8 right_state;
    uint8 l_k_state;
    uint8 r_k_state;
    uint8 left_circle_point;
    uint8 right_circle_point;
    float circle_total_left;
    float circle_total_right;

    float dist_left;
    float dist_right;
    float yaw_ref;
    float delta_yaw;
    float current_yaw;
    float circle_yaw;      // 当前全局 yaw，状态机真正使用的角度
    int16 right_enter_leftCount; // RIGHT_ENTER -> INSIDE 时用到的底部左边恢复计数

    uint8 circle_debug_code;

    int16 right_arc_rightCount1;
    int16 right_arc_arcMinCount;
    int16 right_arc_arcMaxCount;
    int16 right_arc_t_line;



    

    // 双角点环岛入口调试导出
    int16 left_down_y;
    int16 left_up_y;
    int16 right_down_y;
    int16 right_up_y;

    uint8 left_two_corner_ok;
    uint8 right_two_corner_ok;

    uint8 rc_left_slope_ok;          // 右环：左边线斜率 OK
    uint8 rc_right_down_slope_ok;    // 右环：右下段斜率 OK
    uint8 rc_left_straight_ok;       // 右环：左边界直线性 OK
    uint8 rc_right_straight_ok;      // 右环：右边界直线性 OK（仅调试）

    uint8 lc_right_slope_ok;         // 左环：右边线斜率 OK
    uint8 lc_left_down_slope_ok;     // 左环：左下段斜率 OK
} element_debug_snapshot_struct;
#endif

// 模板线调试数据：只导出点和线，不在 element 内部做显示
typedef struct
{
    uint8 valid;            // 当前帧是否有有效模板线
    uint8 state;            // 当前环岛状态
    uint8 template_side;    // 1-模板线作为左边界 2-作为右边界

    int16 left_round_point_row;
    int16 right_round_point_row;
    int16 left_round_up_point;
    int16 right_round_up_point;

    int16 template_x1;
    int16 template_y1;
    int16 template_x2;
    int16 template_y2;

    int16 last_point_x[6];
    int16 last_point_y[6];
} element_circle_draw_debug_struct;


// 右圆弧判定调试量
void element_init(void);
void element_reset(void);
uint8 element_get_mode(void);
uint8 element_get_state(void);
bool  element_is_busy(void);
void  element_process(void);

void element_get_circle_draw_debug(element_circle_draw_debug_struct *dbg);

#if (1 == ELEMENT_ENABLE_DEBUG_API)
void element_debug_get_snapshot(element_debug_snapshot_struct *snapshot);
void element_debug_force_reset(void);
#endif

#endif
