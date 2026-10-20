#include "beep.hpp"
#include "car_init.hpp"

// 默认响声保持帧数。element_process() 每帧调用 beep_update()，因此这里不是毫秒延时。
#define BEEP_ENABLE                 (0)
#define BEEP_DEFAULT_HOLD_FRAMES    (3U)
#define BEEP_PULSE_GAP_FRAMES       (2U)

static uint8 s_beep_hold_frames = 0U;
static uint8 s_beep_pulses_remaining = 0U;
static uint8 s_beep_gap_frames = 0U;
static bool  s_beep_pulse_active = false;

void beep_request_frames(uint8 hold_frames)
{
#if !BEEP_ENABLE
    (void)hold_frames;
    beep_reset();
    return;
#else
    if(hold_frames == 0U)
    {
        hold_frames = BEEP_DEFAULT_HOLD_FRAMES;
    }

    s_beep_pulse_active = false;
    s_beep_pulses_remaining = 0U;
    s_beep_gap_frames = 0U;
    s_beep_hold_frames = hold_frames;
    beep.set_level(1);
#endif
}

void beep_request(void)
{
    beep_request_frames(BEEP_DEFAULT_HOLD_FRAMES);
}

void beep_request_pulses(uint8 pulse_count)
{
#if !BEEP_ENABLE
    (void)pulse_count;
    beep_reset();
    return;
#else
    if(pulse_count == 0U)
    {
        return;
    }

    s_beep_pulse_active = true;
    s_beep_pulses_remaining = pulse_count;
    s_beep_gap_frames = 0U;
    s_beep_hold_frames = BEEP_DEFAULT_HOLD_FRAMES;
    beep.set_level(1);
#endif
}

void beep_update(void)
{
#if !BEEP_ENABLE
    beep_reset();
    return;
#else
    if(s_beep_pulse_active)
    {
        if(s_beep_hold_frames > 0U)
        {
            beep.set_level(1);
            s_beep_hold_frames--;
            if(s_beep_hold_frames == 0U)
            {
                beep.set_level(0);
                if(s_beep_pulses_remaining > 0U)
                {
                    s_beep_pulses_remaining--;
                }
                if(s_beep_pulses_remaining > 0U)
                {
                    s_beep_gap_frames = BEEP_PULSE_GAP_FRAMES;
                }
                else
                {
                    s_beep_pulse_active = false;
                }
            }
            return;
        }

        if(s_beep_gap_frames > 0U)
        {
            beep.set_level(0);
            s_beep_gap_frames--;
            if(s_beep_gap_frames == 0U)
            {
                s_beep_hold_frames = BEEP_DEFAULT_HOLD_FRAMES;
                beep.set_level(1);
            }
            return;
        }
    }

    if(s_beep_hold_frames > 0U)
    {
        beep.set_level(1);
        s_beep_hold_frames--;

        if(s_beep_hold_frames == 0U)
        {
            beep.set_level(0);
        }
    }
    else
    {
        beep.set_level(0);
    }
#endif
}

void beep_reset(void)
{
    s_beep_hold_frames = 0U;
    s_beep_pulses_remaining = 0U;
    s_beep_gap_frames = 0U;
    s_beep_pulse_active = false;
    beep.set_level(0);
}
