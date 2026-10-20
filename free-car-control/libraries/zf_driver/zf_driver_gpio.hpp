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

#ifndef __ZF_DRIVER_GPIO_HPP__
#define __ZF_DRIVER_GPIO_HPP__

#include "zf_driver_file_buffer.hpp"
#include "zf_common_typedef.hpp"

// GPIO蜂鸣器设备节点
#define ZF_GPIO_BEEP               "/dev/zf_gpio_beep"

// GPIO霍尔检测设备节点
#define ZF_GPIO_HALL_DETECTION     "/dev/zf_gpio_hall_detection"

// GPIO按键设备节点
#define ZF_GPIO_KEY_1              "/dev/zf_gpio_key_1"
#define ZF_GPIO_KEY_2              "/dev/zf_gpio_key_2"
#define ZF_GPIO_KEY_3              "/dev/zf_gpio_key_3"
#define ZF_GPIO_KEY_4              "/dev/zf_gpio_key_4"

// GPIO电机设备节点
#define ZF_GPIO_MOTOR_1            "/dev/zf_gpio_motor_1"
#define ZF_GPIO_MOTOR_2            "/dev/zf_gpio_motor_2"

class zf_driver_gpio : public zf_driver_file_buffer
{
private:
    zf_driver_gpio(const zf_driver_gpio&) = delete;
    zf_driver_gpio& operator=(const zf_driver_gpio&) = delete;

public:
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 path  GPIO设备文件路径
// 参数说明 flags 文件打开标志位，默认O_RDWR读写模式
// 返回参数 无
// 使用示例 zf_driver_gpio gpio_obj("/dev/gpio_pin");
// 备注信息 继承文件缓存操作类，实现GPIO电平读写控制
//-------------------------------------------------------------------------------------------------------------------
    zf_driver_gpio(const char* path, int flags = O_RDWR);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 设置GPIO电平状态
// 参数说明 level  GPIO电平值 0-低电平  1-高电平
// 返回参数 无
// 使用示例 gpio_obj.gpio_set_level(1);
// 备注信息 数值转ASCII码写入设备文件，0->0x30,1->0x31
//-------------------------------------------------------------------------------------------------------------------
    void set_level(uint8 level);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取GPIO当前电平状态
// 参数说明 无
// 返回参数 uint8 GPIO电平值 0-低电平  1-高电平
// 使用示例 uint8 val = gpio_obj.gpio_get_level();
// 备注信息 从设备文件读取ASCII码并直接返回对应数值
//-------------------------------------------------------------------------------------------------------------------
    uint8 get_level(void);

};

#endif