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
* 文件名称          zf_device_imu
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
* 2025-12-30        修正           1.路径替换为宏定义 2.设备类型适配枚举体 3.补全缺失头文件
********************************************************************************************************************/

#include "zf_device_imu.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <unistd.h>

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 无
// 返回参数 无
// 使用示例 zf_device_imu imu_dev;
//-------------------------------------------------------------------------------------------------------------------
zf_device_imu::zf_device_imu(void) : zf_driver_file_string(NULL, "r")
{
    imu_type = DEV_NO_FIND;     // 枚举体初始化
    // 所有传感器文件句柄初始化为无效值-1
    fd_acc_x = -1; fd_acc_y = -1; fd_acc_z = -1;
    fd_gyro_x = -1; fd_gyro_y = -1; fd_gyro_z = -1;
    fd_mag_x = -1; fd_mag_y = -1; fd_mag_z = -1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用
//-------------------------------------------------------------------------------------------------------------------
zf_device_imu::~zf_device_imu(void)
{
    imu_close_all_fd();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 关闭所有打开的文件句柄
// 参数说明 无
// 返回参数 无
// 使用示例 内部调用
//-------------------------------------------------------------------------------------------------------------------
void zf_device_imu::imu_close_all_fd(void)
{
    if(fd_acc_x > 0) close(fd_acc_x);
    if(fd_acc_y > 0) close(fd_acc_y);
    if(fd_acc_z > 0) close(fd_acc_z);
    if(fd_gyro_x > 0) close(fd_gyro_x);
    if(fd_gyro_y > 0) close(fd_gyro_y);
    if(fd_gyro_z > 0) close(fd_gyro_z);
    if(fd_mag_x > 0) close(fd_mag_x);
    if(fd_mag_y > 0) close(fd_mag_y);
    if(fd_mag_z > 0) close(fd_mag_z);

    // 句柄重置为无效值
    fd_acc_x = -1; fd_acc_y = -1; fd_acc_z = -1;
    fd_gyro_x = -1; fd_gyro_y = -1; fd_gyro_z = -1;
    fd_mag_x = -1; fd_mag_y = -1; fd_mag_z = -1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 通过文件句柄读取传感器原始数据并转int16
// 参数说明 fd - 已打开的传感器文件句柄
// 返回参数 int16 转换后的传感器原始值，句柄无效/未初始化返回0
// 使用示例 内部调用
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::imu_read_fd_data(int fd)
{
    if(imu_type == DEV_NO_FIND || fd < 0) return 0;
    char str[20] = {0};
    
    lseek(fd, 0, SEEK_SET);          // 关键：偏移到文件头，保证读最新值
    if(read(fd, str, sizeof(str)-1) <= 0) return 0;
    
    return atoi(str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 IMU设备初始化【核心流程：写1初始化→读该文件获型号→按型号开文件】
// 参数说明 无
// 返回参数 imu_device_type_enum 识别到的IMU设备类型，值对应枚举体
// 使用示例 imu_device_type_enum type = imu_dev.init();
//-------------------------------------------------------------------------------------------------------------------
imu_device_type_enum zf_device_imu::init(void)
{
    int fd_event = -1;
    char read_buf[10] = {0};
    int read_val = DEV_NO_FIND;

    // 第一步：写入1 执行IMU硬件初始化 【路径替换为宏定义】
    fd_event = open(IMU_EVENT_PATH, O_RDWR);  // 读写模式打开：先写后读
    if(fd_event < 0)
    {
        printf("IMU open event file fail: %s, errno:%d\r\n", IMU_EVENT_PATH, errno);
        imu_type = DEV_NO_FIND;
        return imu_type;
    }
    // 写入字符'1'完成初始化，符合sysfs写入规范
    if(write(fd_event, "1", 1) != 1)
    {
        printf("IMU write 1 to event file fail, errno:%d\r\n", errno);
        close(fd_event);
        imu_type = DEV_NO_FIND;
        return imu_type;
    }

    // 第二步：读取该文件的值 获取设备型号
    lseek(fd_event, 0, SEEK_SET);  // 写后读，必须偏移到文件头
    if(read(fd_event, read_buf, sizeof(read_buf)-1) > 0)
    {
        read_val = atoi(read_buf); // 字符串转整型，直接映射枚举值
    }
    close(fd_event); // 立即关闭事件文件句柄，无泄漏
    fd_event = -1;

    // 校验读取的型号值有效性，赋值到设备类型
    if(read_val == DEV_IMU660RA || read_val == DEV_IMU660RB || read_val == DEV_IMU963RA)
    {
        imu_type = (imu_device_type_enum)read_val;
        // 打印数值+对应枚举名称
        switch(imu_type)
        {
            case DEV_IMU660RA:
                printf("IMU init success, type: %d (DEV_IMU660RA)\r\n", imu_type);
                break;
            case DEV_IMU660RB:
                printf("IMU init success, type: %d (DEV_IMU660RB)\r\n", imu_type);
                break;
            case DEV_IMU963RA:
                printf("IMU init success, type: %d (DEV_IMU963RA)\r\n", imu_type);
                break;
            default:
                printf("IMU init success, type: %d (UNKNOWN)\r\n", imu_type);
                break;
        }
    }
    else
    {
        printf("IMU read invalid type val: %d, init fail\r\n", read_val);
        imu_type = DEV_NO_FIND;
        return imu_type;
    }

    // 第三步：按型号动态打开对应传感器文件 【只打开一次】 【路径替换为宏定义】
    // 所有型号都默认打开 加速度+陀螺仪 6个文件，这是基础功能
    fd_acc_x  = open(IMU_ACC_X_PATH,  O_RDONLY);
    fd_acc_y  = open(IMU_ACC_Y_PATH,  O_RDONLY);
    fd_acc_z  = open(IMU_ACC_Z_PATH,  O_RDONLY);
    fd_gyro_x = open(IMU_GYRO_X_PATH, O_RDONLY);
    fd_gyro_y = open(IMU_GYRO_Y_PATH, O_RDONLY);
    fd_gyro_z = open(IMU_GYRO_Z_PATH, O_RDONLY);

    // 只有 IMU963RA 才打开磁力计3个文件，660系列不打开，节省资源
    if(imu_type == DEV_IMU963RA)
    {
        fd_mag_x  = open(IMU_MAG_X_PATH,  O_RDONLY);
        fd_mag_y  = open(IMU_MAG_Y_PATH,  O_RDONLY);
        fd_mag_z  = open(IMU_MAG_Z_PATH,  O_RDONLY);
    }

    // 第四步：校验文件句柄有效性
    if(imu_type == DEV_IMU660RA || imu_type == DEV_IMU660RB)
    {
        // 660系列只校验加速度+陀螺仪
        if(fd_acc_x<0 || fd_acc_y<0 || fd_acc_z<0 || fd_gyro_x<0 || fd_gyro_y<0 || fd_gyro_z<0)
        {
            printf("IMU open accel/gyro file error\r\n");
            imu_close_all_fd();
            imu_type = DEV_NO_FIND;
        }
    }
    else if(imu_type == DEV_IMU963RA)
    {
        // 963系列校验所有9个文件
        if(fd_acc_x<0 || fd_acc_y<0 || fd_acc_z<0 || fd_gyro_x<0 || fd_gyro_y<0 || fd_gyro_z<0 || 
           fd_mag_x<0 || fd_mag_y<0 || fd_mag_z<0)
        {
            printf("IMU open all sensor file error\r\n");
            imu_close_all_fd();
            imu_type = DEV_NO_FIND;
        }
    }

    return imu_type;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前识别到的IMU设备类型
// 参数说明 无
// 返回参数 imu_device_type_enum 设备类型枚举值
// 使用示例 imu_device_type_enum type = imu_dev.get_dev_type();
//-------------------------------------------------------------------------------------------------------------------
imu_device_type_enum zf_device_imu::get_dev_type(void)
{
    return imu_type;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取加速度X轴原始数据
// 参数说明 无
// 返回参数 int16 加速度X轴原始值，未初始化返回0
// 使用示例 int16 ax = imu_dev.get_acc_x();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_acc_x(void) { return imu_read_fd_data(fd_acc_x); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取加速度Y轴原始数据
// 参数说明 无
// 返回参数 int16 加速度Y轴原始值，未初始化返回0
// 使用示例 int16 ay = imu_dev.get_acc_y();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_acc_y(void) { return imu_read_fd_data(fd_acc_y); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取加速度Z轴原始数据
// 参数说明 无
// 返回参数 int16 加速度Z轴原始值，未初始化返回0
// 使用示例 int16 az = imu_dev.get_acc_z();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_acc_z(void) { return imu_read_fd_data(fd_acc_z); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取角速度X轴原始数据
// 参数说明 无
// 返回参数 int16 角速度X轴原始值，未初始化返回0
// 使用示例 int16 gx = imu_dev.get_gyro_x();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_gyro_x(void) { return imu_read_fd_data(fd_gyro_x); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取角速度Y轴原始数据
// 参数说明 无
// 返回参数 int16 角速度Y轴原始值，未初始化返回0
// 使用示例 int16 gy = imu_dev.get_gyro_y();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_gyro_y(void) { return imu_read_fd_data(fd_gyro_y); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取角速度Z轴原始数据
// 参数说明 无
// 返回参数 int16 角速度Z轴原始值，未初始化返回0
// 使用示例 int16 gz = imu_dev.get_gyro_z();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_gyro_z(void) { return imu_read_fd_data(fd_gyro_z); }

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取磁力计X轴原始数据
// 参数说明 无
// 返回参数 int16 磁力计X轴原始值，660系列返回0，未初始化返回0
// 使用示例 int16 mx = imu_dev.get_mag_x();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_mag_x(void)
{
    if(imu_type != DEV_IMU963RA) return 0;
    return imu_read_fd_data(fd_mag_x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取磁力计Y轴原始数据
// 参数说明 无
// 返回参数 int16 磁力计Y轴原始值，660系列返回0，未初始化返回0
// 使用示例 int16 my = imu_dev.get_mag_y();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_mag_y(void)
{
    if(imu_type != DEV_IMU963RA) return 0;
    return imu_read_fd_data(fd_mag_y);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取磁力计Z轴原始数据
// 参数说明 无
// 返回参数 int16 磁力计Z轴原始值，660系列返回0，未初始化返回0
// 使用示例 int16 mz = imu_dev.get_mag_z();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_imu::get_mag_z(void)
{
    if(imu_type != DEV_IMU963RA) return 0;
    return imu_read_fd_data(fd_mag_z);
}