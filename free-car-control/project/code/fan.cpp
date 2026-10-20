#include "fan.hpp"

#include "car_init.hpp"
#include "zf_driver_delay.hpp"

#include <cstdio>

#if FAN_ENABLE

namespace
{
constexpr int kFanDutyMin = 500;
constexpr int kFanDutyTargetDefault = 800;
constexpr int kFanDutyStep = 5;
constexpr int kFanStepPeriodMs = 30;
constexpr int kFanEscInitMs = 3000;
constexpr int kFanStabilizeMs = 1000;
constexpr int kFanExitHoldMs = 80;
constexpr int kFanCleanupStep = 20;
constexpr int kFanCleanupStepMs = 20;

struct pwm_info g_fan_pwm_info = {};
zf_driver_pwm g_fan_pwm(ESC_PATH);

fan_state_enum g_fan_state = FAN_STATE_DISABLED;
bool g_fan_remote_enable = false;
bool g_fan_ready = false;
bool g_fan_fault = false;
bool g_fan_pwm_info_valid = false;
int g_fan_current_duty = kFanDutyMin;
int g_fan_target_duty = kFanDutyTargetDefault;
int g_fan_state_elapsed_ms = 0;
int g_fan_step_elapsed_ms = 0;

static inline int fan_output_max_duty()
{
    if (!g_fan_pwm_info_valid || g_fan_pwm_info.duty_max == 0U)
    {
        return kFanDutyTargetDefault;
    }
    const int duty_max = static_cast<int>(g_fan_pwm_info.duty_max);
    return (duty_max < kFanDutyMin) ? kFanDutyMin : duty_max;
}

static inline int fan_clamp_duty(int duty)
{
    const int duty_max = fan_output_max_duty();
    if (duty < kFanDutyMin)
    {
        duty = kFanDutyMin;
    }
    if (duty > duty_max)
    {
        duty = duty_max;
    }
    return duty;
}

static void fan_apply_duty(int duty)
{
    g_fan_current_duty = fan_clamp_duty(duty);
    g_fan_pwm.set_duty(static_cast<uint16>(g_fan_current_duty));
}

static void fan_enter_state(fan_state_enum next_state)
{
    g_fan_state = next_state;
    g_fan_state_elapsed_ms = 0;
    g_fan_step_elapsed_ms = 0;
    g_fan_ready = (next_state == FAN_STATE_READY);
}

static bool fan_request_active(void)
{
    return g_fan_remote_enable && !g_fan_fault;
}

static void fan_enter_fault(const char* reason)
{
    g_fan_fault = true;
    g_fan_ready = false;
    fan_apply_duty(kFanDutyMin);
    fan_enter_state(FAN_STATE_FAULT);
    std::printf("fan fault: %s\r\n", reason);
}
} // namespace

void fan_init(void)
{
    g_fan_pwm.get_dev_info(&g_fan_pwm_info);
    g_fan_pwm_info_valid = true;
    g_fan_target_duty = fan_clamp_duty(kFanDutyTargetDefault);
    std::printf("fan pwm freq = %u Hz\r\n", g_fan_pwm_info.freq);
    std::printf("fan pwm duty_max = %u\r\n", g_fan_pwm_info.duty_max);
    if (g_fan_pwm_info.duty_max < static_cast<uint32>(kFanDutyMin))
    {
        fan_enter_fault("duty_max < 500");
        return;
    }
    fan_apply_duty(kFanDutyMin);
    fan_enter_state(FAN_STATE_DISABLED);
}

void fan_update_2ms(bool speed_valid)
{
    // 保留参数以兼容现有调用接口，但负压启动不再受任何速度条件控制。
    (void)speed_valid;

    if (!g_fan_pwm_info_valid || g_fan_fault)
    {
        return;
    }

    g_fan_state_elapsed_ms += 2;
    g_fan_step_elapsed_ms += 2;

    const bool request_active = fan_request_active();

    switch (g_fan_state)
    {
        case FAN_STATE_DISABLED:
        {
            fan_apply_duty(kFanDutyMin);
            if (g_fan_remote_enable)
            {
                fan_enter_state(FAN_STATE_ESC_INIT);
            }
        } break;

        case FAN_STATE_WAIT_SPEED_VALID:
        {
            fan_apply_duty(kFanDutyMin);
            if (!g_fan_remote_enable)
            {
                fan_enter_state(FAN_STATE_DISABLED);
            }
            else
            {
                fan_enter_state(FAN_STATE_ESC_INIT);
            }
        } break;

        case FAN_STATE_ESC_INIT:
        {
            fan_apply_duty(kFanDutyMin);
            if (!request_active)
            {
                fan_enter_state(FAN_STATE_DISABLED);
            }
            else if (g_fan_state_elapsed_ms >= kFanEscInitMs)
            {
                fan_enter_state(FAN_STATE_RAMP_UP);
            }
        } break;

        case FAN_STATE_RAMP_UP:
        {
            if (!request_active)
            {
                fan_enter_state(FAN_STATE_RAMP_DOWN);
                break;
            }
            if (g_fan_step_elapsed_ms >= kFanStepPeriodMs)
            {
                g_fan_step_elapsed_ms = 0;
                if (g_fan_current_duty < g_fan_target_duty)
                {
                    fan_apply_duty(g_fan_current_duty + kFanDutyStep);
                }
            }
            if (g_fan_current_duty >= g_fan_target_duty)
            {
                fan_enter_state(FAN_STATE_STABILIZE);
            }
        } break;

        case FAN_STATE_STABILIZE:
        {
            fan_apply_duty(g_fan_target_duty);
            if (!request_active)
            {
                fan_enter_state(FAN_STATE_RAMP_DOWN);
            }
            else if (g_fan_state_elapsed_ms >= kFanStabilizeMs)
            {
                fan_enter_state(FAN_STATE_READY);
            }
        } break;

        case FAN_STATE_READY:
        {
            fan_apply_duty(g_fan_target_duty);
            if (!request_active)
            {
                fan_enter_state(FAN_STATE_RAMP_DOWN);
            }
        } break;

        case FAN_STATE_RAMP_DOWN:
        {
            if (g_fan_step_elapsed_ms >= kFanStepPeriodMs)
            {
                g_fan_step_elapsed_ms = 0;
                if (g_fan_current_duty > kFanDutyMin)
                {
                    fan_apply_duty(g_fan_current_duty - kFanDutyStep);
                }
            }
            if (g_fan_current_duty <= kFanDutyMin)
            {
                fan_enter_state(FAN_STATE_DISABLED);
            }
        } break;

        case FAN_STATE_FAULT:
        {
            fan_apply_duty(kFanDutyMin);
        } break;
    }
}

void fan_set_remote_enable(bool enable)
{
    g_fan_remote_enable = enable;
}

bool fan_is_ready(void)
{
    return g_fan_ready;
}

bool fan_feature_enabled(void)
{
    return true;
}

int fan_get_current_duty(void)
{
    return g_fan_current_duty;
}

fan_state_enum fan_get_state(void)
{
    return g_fan_state;
}

bool fan_has_fault(void)
{
    return g_fan_fault;
}

void fan_cleanup_on_exit(void)
{
    if (!g_fan_pwm_info_valid)
    {
        return;
    }

    while (g_fan_current_duty > kFanDutyMin)
    {
        fan_apply_duty(g_fan_current_duty - kFanCleanupStep);
        system_delay_ms(kFanCleanupStepMs);
    }
    fan_apply_duty(kFanDutyMin);
    system_delay_ms(kFanExitHoldMs);
}

#else

void fan_init(void) {}
void fan_update_2ms(bool) {}
void fan_set_remote_enable(bool) {}
bool fan_is_ready(void) { return true; }
bool fan_feature_enabled(void) { return false; }
int fan_get_current_duty(void) { return 500; }
fan_state_enum fan_get_state(void) { return FAN_STATE_DISABLED; }
bool fan_has_fault(void) { return false; }
void fan_cleanup_on_exit(void) {}

#endif
