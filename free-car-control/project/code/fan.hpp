#ifndef _FAN_HPP_
#define _FAN_HPP_

#include "zf_common_typedef.hpp"

// 0: 完全关闭负压功能（包括上位机通道与起步门控）
// 1: 启用负压功能
#define FAN_ENABLE 0

#if FAN_ENABLE
#define ESC_PATH ZF_PWM_ESC_1
#endif

typedef enum
{
    FAN_STATE_DISABLED = 0,
    FAN_STATE_WAIT_SPEED_VALID = 1,
    FAN_STATE_ESC_INIT = 2,
    FAN_STATE_RAMP_UP = 3,
    FAN_STATE_STABILIZE = 4,
    FAN_STATE_READY = 5,
    FAN_STATE_RAMP_DOWN = 6,
    FAN_STATE_FAULT = 7,
} fan_state_enum;

void fan_init(void);
// speed_valid 参数仅为兼容旧调用保留；负压只由上位机远程使能控制。
void fan_update_2ms(bool speed_valid);
void fan_set_remote_enable(bool enable);
bool fan_is_ready(void);
bool fan_feature_enabled(void);
int fan_get_current_duty(void);
fan_state_enum fan_get_state(void);
bool fan_has_fault(void);
void fan_cleanup_on_exit(void);

#endif
