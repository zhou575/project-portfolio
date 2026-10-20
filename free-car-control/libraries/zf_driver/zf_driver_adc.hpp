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
* 文件名称          zf_driver_adc
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#ifndef __ZF_DRIVER_ADC_HPP__
#define __ZF_DRIVER_ADC_HPP__

#include "zf_common_typedef.hpp"


#define ADC_CH0_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage0_raw"
#define ADC_CH1_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage1_raw"
#define ADC_CH2_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage2_raw"
#define ADC_CH3_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage3_raw"
#define ADC_CH4_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage4_raw"
#define ADC_CH5_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage5_raw"
#define ADC_CH6_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage6_raw"
#define ADC_CH7_PATH                   "/sys/bus/iio/devices/iio:device0/in_voltage7_raw"
#define ADC_SCALE_PATH                 "/sys/bus/iio/devices/iio:device0/in_voltage_scale"

// 继承关系保留，兼容原有框架，内部不再使用父类文件指针
class zf_driver_adc {
private:
    zf_driver_adc(const zf_driver_adc&) = delete;            // 禁止拷贝构造
    zf_driver_adc& operator=(const zf_driver_adc&) = delete; // 禁止赋值重载

    int     fd_convert;     // ADC原始值文件描述符 (对应原ADC通道路径)
    int     fd_scale;       // ADC校准系数文件描述符
    char    read_buf[64];   // 全局读取缓存，复用提升效率，防止栈内存反复申请
    const   char *adc_path; // 保存ADC通道路径

public:
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 adc_path  ADC通道文件路径
// 参数说明 mode     兼容传参，无实际作用，保留为了不修改调用代码
// 返回参数 无
// 使用示例 zf_driver_adc battery_adc(ADC_CH7_PATH);
// 备注信息 初始化打开ADC原始值文件+校准系数文件，双文件描述符只打开一次，永久复用无重复开销
//-------------------------------------------------------------------------------------------------------------------
    zf_driver_adc(const char* adc_path, const char* mode = "r");

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用无需手动调用
// 备注信息 自动关闭双文件描述符(fd_convert + fd_scale)，无句柄泄漏/内存泄漏，资源自动释放
//-------------------------------------------------------------------------------------------------------------------
    ~zf_driver_adc(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取ADC通道原始数值
// 参数说明 无
// 返回参数 uint16 转换后的ADC原始整型数值，读取失败返回0
// 使用示例 adc_reg = battery_adc.convert();
// 备注信息 使用lseek重置文件偏移量，保证每次读取最新实时值，彻底解决值重复问题
//-------------------------------------------------------------------------------------------------------------------
    uint16 convert(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取ADC比例校准系数
// 参数说明 无
// 返回参数 float 转换后的浮点型比例系数值，读取失败返回0.0f
// 使用示例 adc_scale = battery_adc.get_scale();
// 备注信息 固定读取ADC_SCALE_PATH路径，重置偏移量，读取稳定无脏数据
//-------------------------------------------------------------------------------------------------------------------
    float get_scale(void);
};

#endif