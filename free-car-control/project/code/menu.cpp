#include "car_init.hpp"
#include "menu.hpp"
#include "image.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "lq_ncnn.hpp"

//按键列表
zf_driver_gpio* key_dev[KEY_NUMBER] = {&key_1, &key_2, &key_3, &key_4};
static uint32 key_press_time[KEY_NUMBER] = {0};             // 按下持续时间（按扫描次数累加）
static key_state_enum key_state[KEY_NUMBER] = {KEY_RELEASE};
// 这个数组用于防止长按事件重复触发
static uint8 key_long_reported[KEY_NUMBER] = {0};

/**
 * @brief 按键扫描函数
 * @param scanner_period 扫描周期，单位 ms
 * @note 需要定期调用，建议放在定时器回调中。
 *       根据按键状态持续时间区分短按和长按，更新 key_state 数组。
 *       长按事件只上报一次，直到按键释放后才会再次上报长按事件。
 *       需要配合 key_get_state() 和 key_use() 来读取和处理按键事件。
 */
void key_scanner (uint32 scanner_period)
{
    if(scanner_period == 0)
    {
        return;
    }
    for(uint8 i = 0; i < KEY_NUMBER; i ++)
    {
        uint8 level = key_dev[i]->get_level();

        if(KEY_RELEASE_LEVEL != level)// 按键按下
        {
            key_press_time[i] ++;
            if(KEY_LONG_PRESS_PERIOD / scanner_period <= key_press_time[i])
            {
                // 只上报一次长按事件
                if(!key_long_reported[i])
                {
                    key_state[i] = KEY_LONG_PRESS;
                    key_long_reported[i] = 1;
                }
            }
        }
        else// 按键释放
        {
            if((!key_long_reported[i])&& (KEY_MAX_SHOCK_PERIOD / scanner_period <= key_press_time[i]))
            {
                key_state[i] = KEY_SHORT_PRESS;
            }
            key_press_time[i] = 0;
            key_long_reported[i] = 0;
        }
    }
}

/**
 * @brief 获取按键事件状态
 * @param index 按键索引，范围 0 ~ KEY_NUMBER-1
 * @return key_state_enum 按键事件状态：KEY_RELEASE、KEY_SHORT_PRESS、KEY_LONG_PRESS
 * @note 调用后会清除对应按键的事件状态，
 */
key_state_enum key_get_state(uint8 index)
{
    if(index >= KEY_NUMBER)
    {
        return KEY_RELEASE;
    }

    key_state_enum state = key_state[index];
    key_state[index] = KEY_RELEASE;
    return state;
}

/**
 * @brief 处理按键事件
 * @note 目前实现了 KEY1 的短按事件，清除保护标志，并重置图像保护基线。可以根据需要扩展其他按键事件的处理
 *       临时使用，后面有菜单了就不用了
 */
void key_use()
{
    if(key_get_state(0)==KEY_SHORT_PRESS)
    {
        // 先暂时拉起保护，避免复位过程中控制环读取到中间态。
        imgpro_var.protect_flag=true;
        encoder_clear();
        imgpro_var.protect_flag=false;
        image_protect_reset_state();
        motor_protect_reset_state();
        printf("protect reset by key1\r\n");
    }
}

/**
 * @brief 绕行状态按键
 */
void key_use_bypass()
{
    if(key_get_state(0)==KEY_SHORT_PRESS)
    {
        // 先暂时拉起保护，避免复位过程中控制环读取到中间态。
        imgpro_var.protect_flag=true;
        encoder_clear();
        model_encoder_clear_increment();
        image_protect_reset_state();
        image_bypass_reset_state();
        motor_protect_reset_state();
        lq_ncnn_reset_control_state();
        imgpro_var.midline_error = 0.0f;
        imgpro_var.protect_flag=false;
        printf("protect reset by key1\r\n");
    }

}
/**
 * @brief 用于按键测试,测试代码能不能用
 */
void test_key()
{
    for(int i = 0; i < KEY_NUMBER; i++)
    {
        key_state_enum state = key_get_state(i);
        if(state == KEY_SHORT_PRESS)
        {
            printf("KEY_%d short press\r\n", i + 1);
        }
        else if(state == KEY_LONG_PRESS)
        {
            printf("KEY_%d long press\r\n", i + 1);
        }
    }
}
