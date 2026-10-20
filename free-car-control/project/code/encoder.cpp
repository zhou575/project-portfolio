#include "car_init.hpp"
#include "encoder.hpp"

// 编码器模块输出滤波后的轮速量，供电机内环使用。

float f_encoder_left,f_encoder_right;
float pre_encoder_left,pre_encoder_right;
float model_encoder_integral=0;
float model_l_integral,model_r_integral;
float encoder_left_abs_total = 0.0f;
float encoder_right_abs_total = 0.0f;
float encoder_left_sum_total = 0.0f;
float encoder_right_sum_total = 0.0f;

uint8_t encoder_first_flag = 1;

static inline float encoder_absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}

/**
 * @brief 编码器计数清零
 * @note 调用设备文件接口清零编码器计数，建议在系统初始化
 *       编码器文件初始化可能会有残留值，需要开始进行清零，避免第一次读取到异常值。
 */
void encoder_clear()
{
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();
}

void encoder_distance_reset_total(void)
{
    encoder_left_abs_total = 0.0f;
    encoder_right_abs_total = 0.0f;
    encoder_left_sum_total = 0.0f;
    encoder_right_sum_total = 0.0f;
}

/**
 * @brief 获取编码器的读值，并且进行滤波
 * @param filter_alpha 滤波系数,范围0-1,1相当于没有滤波
 * @note 对第一次进行处理，保证第一次读取的是正常值
 */
// 读取增量编码器计数，清零硬件计数器，再做一阶滤波。
void filter_encoder_get(float filter_alpha)
{
    float raw_right = float(-encoder_dir_1.get_count());
    float raw_left = float(encoder_dir_2.get_count());
    
    encoder_dir_1.clear_count();
    encoder_dir_2.clear_count();

    // 保护无效滤波系数，避免输出异常。
    if(filter_alpha<0.0f||filter_alpha>1.0f) return;
    
    // 首次采样直接使用原始值，初始化滤波状态。
    if (encoder_first_flag)
    {
        f_encoder_left  = raw_left;
        f_encoder_right = raw_right;
        encoder_first_flag = 0;
    }
    else
    {
        f_encoder_left  = filter_alpha * raw_left  + (1.0f - filter_alpha) * pre_encoder_left;
        f_encoder_right = filter_alpha * raw_right + (1.0f - filter_alpha) * pre_encoder_right;
    }

    pre_encoder_left  = f_encoder_left;
    pre_encoder_right = f_encoder_right;

    encoder_left_abs_total += encoder_absf_local(f_encoder_left);
    encoder_right_abs_total += encoder_absf_local(f_encoder_right);
    encoder_left_sum_total += f_encoder_left;
    encoder_right_sum_total += f_encoder_right;
}

// 编码器积分，可以用于元素判断中
void model_encoder_increment(void)
{
    model_l_integral+=f_encoder_left;
    model_r_integral+=f_encoder_right;
    model_encoder_integral=( model_l_integral+ model_r_integral)/2;
}

void model_encoder_clear_increment(void)
{
    model_encoder_integral=model_l_integral=model_r_integral=0;
}
