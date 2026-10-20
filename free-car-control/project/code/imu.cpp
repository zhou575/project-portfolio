#include "car_init.hpp"
#include "imu.hpp"
#include "image.hpp"
#include "motor.hpp"
#include "lq_ncnn.hpp"

// IMU 数据处理链路：
// 原始陀螺仪 -> 零偏补偿 -> 低通滤波 -> 死区处理 -> 输出给控制器的归一化量。

float Y,Z;
// int16 imu_acc_x,imu_acc_y,imu_acc_z;
int16 raw_gyro_x,raw_gyro_y,raw_gyro_z;
float gyro_y,gyro_z,f_gyro_y,f_gyro_z;
float yaw,model_yaw,pitch;
float gyro_z_for_yaw = 0.0f;
static constexpr float YAW_GYRO_SCALE_K = 1.0f;
float test_use_angle;
float motor_control_gyro_z;

/**
 * @brief 读取 y 轴原始角速度
 */
static inline void get_raw_gyroy ()
{
    raw_gyro_y = imu_dev.get_gyro_y();
}

/**
 * @brief 读取 z 轴原始角速度
 */
static inline void get_raw_gyroz ()
{
    raw_gyro_z = imu_dev.get_gyro_z();
}

/**
 * @brief 除去陀螺仪的角速度零飘
 * @note 上电静止采样 100 次，估计陀螺仪零偏。
 */
void gyroOffsetInit(void)
{
    uint8 i;
    Y = 0;
    Z = 0;

    // 获取陀螺仪数据100次并求平均值，减去偏移
    for (i = 0; i < 100; i++)
    {
        get_raw_gyroy();
        get_raw_gyroz(); // 获取陀螺仪数据
        Y += (float)raw_gyro_y;
        Z += (float)raw_gyro_z;
        system_delay_ms(2); // 延时2ms
    }
    Y /= 100.0f;
    Z /= 100.0f;
}

/**
 * @brief 读取z轴角速度
 * @param filter_alpha 滤波系数,范围0-1,1相当于没有滤波
 * @note 毛刺裁剪，一阶低通,将 Z 轴原始值换算到物理量，并做一阶低通滤波
 */
static inline void get_filter_gyro_z(float filter_alpha)
{
    get_raw_gyroz();
    gyro_z_for_yaw = (((float)raw_gyro_z - Z) / 16.4f) * YAW_GYRO_SCALE_K;
    gyro_z = ((float)raw_gyro_z - Z) / 16.4f;
    gyro_z = CLAMP(gyro_z, -GYRO_MAX_VALUE, GYRO_MAX_VALUE);
    f_gyro_z=filter_alpha*f_gyro_z+(1.0f-filter_alpha)*gyro_z;
}

/**
 * @brief 读取y轴角速度
 * @param filter_alpha 滤波系数,范围0-1,1相当于没有滤波
 * @note 毛刺裁剪，一阶低通,将 Y 轴原始值换算到物理量，并做一阶低通滤波
 */
static inline void get_filter_gyro_y(float filter_alpha){
    get_raw_gyroy();
    gyro_y = ((float)raw_gyro_y - Y) / 16.4;
    gyro_y = CLAMP(gyro_y, -GYRO_MAX_VALUE, GYRO_MAX_VALUE);
    f_gyro_y=filter_alpha*f_gyro_y+(1.0f-filter_alpha)*gyro_y;
}

void Control_yaw(float c_gyro_z,float time)
{
    float yaw_dead=c_gyro_z*time;
    if(fabsf(yaw_dead)>=0.01)
    {
        yaw+=yaw_dead;
    }
}

/**
 * @brief 偏航角积分
 * @param c_gyro_z 当前处理后的角速度值
 * @param time 积分时间
 * @note 对 Z 轴角速度积分得到 yaw，微小增量忽略以减小漂移噪声
 */
//这个实际测量一下
void Control_model_yaw(float c_gyro_z,float time)
{
    if(!(is_model_infer_complete && (model_control_access == 'l' || model_control_access == 'r')))
    {
        model_yaw = 0.0f;
        return;
    }
    model_yaw+=c_gyro_z*time;
    // float yaw_dead=c_gyro_z*time;
    // if(fabsf(yaw_dead)>=0.01)
    // {
    //     model_yaw+=yaw_dead;
    // }
}
/**
 * @brief 偏航角测试
 * @param c_gyro_z 当前处理后的角速度值
 * @param time 积分时间
 * @note 对 Z 轴角速度积分得到 yaw，微小增量忽略以减小漂移噪声
 */
void Control_yaw_test(float c_gyro_z,float time)
{
    float yaw_dead=c_gyro_z*time;
    if(fabsf(yaw_dead)>=0.01)
    {
        // 统一约定：右转为正、左转为负（当前陀螺仪右转是负角速度）
        model_yaw+=yaw_dead;
    } 
}

/**
 * @brief 俯仰角积分
 * @param c_gyro_y 当前处理后的角速度值
 * @param time 积分时间
 * @note 对 Y 轴角速度积分得到 pitch，微小增量忽略以减小漂移噪声
 */
static inline void Control_pitch(float c_gyro_y,float time)
{
    float pitch_dead=c_gyro_y*time;
    if(fabsf(pitch_dead)>=0.0)
    {
        pitch+=pitch_dead;
    }
}   

/**
 * @brief 提供给控制器的主接口
 * @param deadband 陀螺仪死区
 * @param time 积分时间
 * @note 死区用于控制，积分不用传入的死区,放定时器里
 *       右转gyro为正，左转gyro为负
 *       返回滤波+死区处理后的 Z 轴归一化角速度（范围约为 [-1,1]）
 */
void gyro_process(float deadband)
{
    get_filter_gyro_z(0.7f);
    get_filter_gyro_y(0.7f);
    Control_yaw(gyro_z_for_yaw, 0.002f);
    Control_pitch(f_gyro_y, 0.002f);
    if(fabsf(f_gyro_z)<deadband)
    {
        motor_control_gyro_z=0;
    }
    else
    {
        motor_control_gyro_z=f_gyro_z;
    }
}
