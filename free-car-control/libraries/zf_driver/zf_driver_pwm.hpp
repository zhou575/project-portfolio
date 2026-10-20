/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#ifndef __ZF_DRIVER_PWM_HPP__
#define __ZF_DRIVER_PWM_HPP__

#include "zf_driver_file_buffer.hpp"
#include "zf_common_typedef.hpp"


// PWM设备节点 (电调/电机/舵机)
#define ZF_PWM_ESC_1               "/dev/zf_pwm_esc_1"
#define ZF_PWM_MOTOR_1             "/dev/zf_pwm_motor_1"
#define ZF_PWM_MOTOR_2             "/dev/zf_pwm_motor_2"
#define ZF_PWM_SERVO_1             "/dev/zf_pwm_servo_1"

// GPIO电机设备节点
#define ZF_GPIO_MOTOR_1            "/dev/zf_gpio_motor_1"
#define ZF_GPIO_MOTOR_2            "/dev/zf_gpio_motor_2"

// PWM设备信息结构体
struct pwm_info
{
    uint32 freq;       // PWM频率
    uint32 duty;       // PWM占空比
    uint32 duty_max;   // PWM占空比最大值
    uint32 duty_ns;    // PWM高电平时间（纳秒）
    uint32 period_ns;  // PWM周期时间（纳秒）
    uint32 clk_freq;   // 时钟频率
};

class zf_driver_pwm : public zf_driver_file_buffer
{
private:
    zf_driver_pwm(const zf_driver_pwm&) = delete;
    zf_driver_pwm& operator=(const zf_driver_pwm&) = delete;

public:
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 path  PWM设备文件路径
// 参数说明 flags 文件打开标志位，默认O_RDWR读写模式
// 返回参数 无
// 使用示例 zf_driver_pwm pwm_obj("/dev/pwm");
// 备注信息 继承文件缓存操作类，实现PWM设备数据读写控制
//-------------------------------------------------------------------------------------------------------------------
    zf_driver_pwm(const char* path, int flags = O_RDWR);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取PWM设备信息数据
// 参数说明 pwm_info PWM信息结构体指针，用于存储读取的数据
// 返回参数 无
// 使用示例 struct pwm_info info; pwm_obj.get_dev_info(&info);
// 备注信息 二进制读取PWM设备信息，指针为空则不执行读取操作
//-------------------------------------------------------------------------------------------------------------------
    void get_dev_info(pwm_info *pwm_info);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置PWM输出占空比值
// 参数说明 duty  PWM占空比值，数据类型uint16
// 返回参数 无
// 使用示例 pwm_obj.set_duty(500);
// 备注信息 二进制写入占空比数值到PWM设备文件，直接控制输出占空比
//-------------------------------------------------------------------------------------------------------------------
    void set_duty(uint16 duty);

};

#endif