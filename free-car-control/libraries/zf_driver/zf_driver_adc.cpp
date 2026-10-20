/*********************************************************************************************************************
* 配套实现文件 请勿修改函数名和接口，仅修改内部实现逻辑
* 双文件描述符：fd_convert(ADC原始值) + fd_scale(校准系数)
* 纯Linux系统IO调用，无C标准库文件指针，无缓冲问题，嵌入式最优方案
********************************************************************************************************************/
#include "zf_driver_adc.hpp"
#include <cstdlib>
#include <cstring>
#include <cstdio>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 adc_path  ADC通道文件路径
// 参数说明 mode     文件打开模式，兼容传参无实际作用，保留以兼容调用
// 返回参数 无
// 使用示例 zf_driver_adc battery_adc(ADC_CH7_PATH);
// 备注信息 初始化打开ADC原始值和校准系数文件，双文件描述符仅打开一次，全程复用无重复开销
//-------------------------------------------------------------------------------------------------------------------
zf_driver_adc::zf_driver_adc(const char* adc_path, const char* mode)
    : fd_convert(-1),
      fd_scale(-1),
      adc_path(adc_path)
{
    memset(read_buf, 0x00, sizeof(read_buf));
    // 1. 只读方式打开 ADC原始值 文件
    if(NULL != adc_path)
    {
        fd_convert = open(adc_path, O_RDONLY);
    }
    // 2. 只读方式打开 ADC校准系数 文件
    fd_scale = open(ADC_SCALE_PATH, O_RDONLY);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用无需手动调用
// 备注信息 自动关闭双文件描述符释放资源，无文件句柄内存泄漏风险
//-------------------------------------------------------------------------------------------------------------------
zf_driver_adc::~zf_driver_adc(void)
{
    // 关闭ADC原始值文件描述符
    if(fd_convert >= 0)
    {
        close(fd_convert);
        fd_convert = -1;
    }
    // 关闭ADC校准系数文件描述符
    if(fd_scale >= 0)
    {
        close(fd_scale);
        fd_scale = -1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取ADC通道原始数值
// 参数说明 无
// 返回参数 uint16 转换后的ADC原始整型数值，读取失败返回0
// 使用示例 adc_reg = battery_adc.convert();
// 备注信息 读取前重置文件偏移量，保证每次读取最新实时数据，无重复值问题
//-------------------------------------------------------------------------------------------------------------------
uint16 zf_driver_adc::convert(void)
{
    uint16 adc_reg = 0;
    if(fd_convert < 0) return adc_reg;  // 文件描述符无效，返回0

    lseek(fd_convert, 0, SEEK_SET);     // 重置文件偏移量到开头，必加！解决值重复问题
    ssize_t read_len = read(fd_convert, read_buf, sizeof(read_buf) - 1);
    if(read_len > 0)                    // 读取成功
    {
        read_buf[read_len] = '\0';      // 手动补字符串结束符，系统调用不会自动补
        adc_reg = (uint16)atoi(read_buf);
    }
    memset(read_buf, 0x00, sizeof(read_buf));
    return adc_reg;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取ADC比例校准系数
// 参数说明 无
// 返回参数 float 转换后的浮点型比例系数值，读取失败返回0.0f
// 使用示例 adc_scale = battery_adc.get_scale();
// 备注信息 固定读取ADC_SCALE_PATH路径，重置偏移量，读取稳定无脏数据
//-------------------------------------------------------------------------------------------------------------------
float zf_driver_adc::get_scale(void)
{
    float scale = 0.0f;
    if(fd_scale < 0) return scale;      // 文件描述符无效，返回0.0f

    lseek(fd_scale, 0, SEEK_SET);       // 重置文件偏移量到开头
    ssize_t read_len = read(fd_scale, read_buf, sizeof(read_buf) - 1);
    if(read_len > 0)                    // 读取成功
    {
        read_buf[read_len] = '\0';      // 手动补字符串结束符
        scale = atof(read_buf);
    }
    memset(read_buf, 0x00, sizeof(read_buf));
    return scale;
}