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

#ifndef __ZF_DRIVER_ENCODER_HPP__
#define __ZF_DRIVER_ENCODER_HPP__

#include "zf_driver_file_buffer.hpp"
#include "zf_common_typedef.hpp"

// 正交编码器设备节点，默认使用该节点
#define ZF_ENCODER_QUAD_1          "/dev/zf_encoder_quad_1"
#define ZF_ENCODER_QUAD_2          "/dev/zf_encoder_quad_2"

// 方向编码器设备节点，如果需要使用该节点，请修改对应的设备树文件后重新编译内核
#define ZF_ENCODER_DIR_1           "/dev/zf_encoder_dir_1"
#define ZF_ENCODER_DIR_2           "/dev/zf_encoder_dir_2"

class zf_driver_encoder : public zf_driver_file_buffer
{
private:
    zf_driver_encoder(const zf_driver_encoder&) = delete;
    zf_driver_encoder& operator=(const zf_driver_encoder&) = delete;

public:
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 path  设备文件路径
// 参数说明 flags 文件打开标识位，默认O_RDWR读写模式
// 返回参数 无
// 使用示例 zf_driver_encoder encoder_obj("/dev/encoder");
// 备注信息 继承文件缓存操作类，实现编码器设备文件读写
//-------------------------------------------------------------------------------------------------------------------
    zf_driver_encoder(const char* path, int flags = O_RDWR);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取编码器计数值
// 参数说明 无
// 返回参数 int16 编码器当前计数值
// 使用示例 int16 cnt = encoder_obj.get_count();
// 备注信息 读取设备文件数据并转换为整型计数值
//-------------------------------------------------------------------------------------------------------------------
    int16 get_count(void);

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 清零编码器计数值
// 参数说明 无
// 返回参数 无
// 使用示例 encoder_obj.clear_count();
// 备注信息 向设备文件写入0值，完成编码器计数清零操作
//-------------------------------------------------------------------------------------------------------------------
    void clear_count(void);

};

#endif