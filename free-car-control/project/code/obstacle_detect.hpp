#ifndef _OBSTACLE_DETECT_HPP_
#define _OBSTACLE_DETECT_HPP_

#include "image.hpp"

// 避障总开关：0=关闭避障，1=开启避障。
#define OBSTACLE_DETECT_ENABLE 1

typedef enum
{
    OBS_NONE  = 0,
    OBS_LEFT  = 1,
    OBS_RIGHT = 2,
} obstacle_state_enum;

typedef struct
{
    uint8 state;
    uint8 left_candidate;
    uint8 right_candidate;
    uint8 left_jump;
    uint8 right_jump;
    int16 left_corner;
    int16 right_corner;
    uint8 left_enter_count;
    uint8 right_enter_count;
    uint8 exit_count;
    uint8 jump_threshold;
    uint8 width_shrink_threshold;
    uint8 enter_confirm_frames;
    uint8 exit_confirm_frames;
    uint8 width_shrink_found;
    int16 width_shrink_row;
    int16 width_shrink_value;

    uint8 opposite_fit_ok;
    float opposite_valid_ratio;
    float opposite_rmse;
    float opposite_max_error;
    float opposite_valid_ratio_threshold;
    float opposite_rmse_threshold;
    float opposite_max_error_threshold;

    uint8 exit_fit_ok;
    float exit_valid_ratio;
    float exit_rmse;
    float exit_max_error;
    float exit_encoder_distance_threshold;

    float encoder_distance;
    uint8 exit_distance_ready;
    uint8 exit_upper_jump_found;
    int16 exit_upper_jump_start;
    int16 exit_upper_jump_value;

    int16 control_jump_start;
    int16 control_jump_end;

    uint8 control_midline_mode;
    int16 control_midline_offset;
} obstacle_debug_snapshot_struct;

void obstacle_detect_init(void);
void obstacle_detect_update(void);
bool obstacle_detect_is_active(void);
obstacle_state_enum obstacle_detect_get_state(void);
void obstacle_detect_reset(void);
void obstacle_detect_get_snapshot(obstacle_debug_snapshot_struct *snapshot);

#endif
