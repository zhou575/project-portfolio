#ifndef _BEEP_HPP_
#define _BEEP_HPP_

#include "zf_common_typedef.hpp"

// 非阻塞蜂鸣器接口：
// - beep_request()：请求响一下，默认保持若干图像帧；
// - beep_update()：每帧调用一次，自动关蜂鸣器；
// - beep_reset()：强制关闭蜂鸣器并清空保持计数。
void beep_request(void);
void beep_request_frames(uint8 hold_frames);
void beep_request_pulses(uint8 pulse_count);
void beep_update(void);
void beep_reset(void);

#endif
