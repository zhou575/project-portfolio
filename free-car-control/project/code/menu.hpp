#ifndef _MENU_HPP_
#define _MENU_HPP_

#include "zf_common_typedef.hpp"
#include <stdio.h>

#define KEY_NUMBER              4       //按键数目
#define KEY_RELEASE_LEVEL       1       //释放电平是1
#define KEY_LONG_PRESS_PERIOD   1000    //长按判定时间,单位ms
#define KEY_MAX_SHOCK_PERIOD    20      //短按消抖时间,单位ms

typedef enum
{
    KEY_RELEASE = 0,                                                                // 按键释放状态
    KEY_SHORT_PRESS,                                                            // 按键短按状态
    KEY_LONG_PRESS,                                                             // 按键长按状态
}key_state_enum;

key_state_enum key_get_state(uint8 index);
void key_scanner (uint32 scanner_period);
void test_key(void);
void key_use(void);
void key_use_bypass(void);

#endif
