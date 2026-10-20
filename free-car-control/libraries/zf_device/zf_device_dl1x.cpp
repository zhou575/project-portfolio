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
* 2025-12-30        修改           枚举名 dl1x_type → dl1x_device_type_enum
********************************************************************************************************************/

#include "zf_device_dl1x.hpp"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 构造函数
// 参数说明 无
// 返回参数 无
// 使用示例 zf_device_dl1x dl1x_dev;
//-------------------------------------------------------------------------------------------------------------------
zf_device_dl1x::zf_device_dl1x(void) : zf_driver_file_string(NULL, "r")
{
    dl1x_dev_type = NO_FIND_DEVICE;  
    fd_distance = -1;                // 距离文件句柄初始化为无效值
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 析构函数
// 参数说明 无
// 返回参数 无
// 使用示例 自动调用
//-------------------------------------------------------------------------------------------------------------------
zf_device_dl1x::~zf_device_dl1x(void)
{
    dl1x_close_all_fd();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 关闭所有打开的文件句柄
// 参数说明 无
// 返回参数 无
// 使用示例 内部调用
//-------------------------------------------------------------------------------------------------------------------
void zf_device_dl1x::dl1x_close_all_fd(void)
{
    if(fd_distance > 0) close(fd_distance);
    fd_distance = -1;  // 重置为无效值，防止重复关闭
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 通过文件句柄读取传感器原始数据并转int16
// 参数说明 fd - 已打开的传感器文件句柄
// 返回参数 int16 转换后的传感器原始值，句柄无效/未初始化返回0
// 使用示例 内部调用
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_dl1x::dl1x_read_fd_data(int fd)
{
    if(dl1x_dev_type == NO_FIND_DEVICE || fd < 0) return 0;  // 枚举常量未变，无需修改
    char str[20] = {0};
    
    lseek(fd, 0, SEEK_SET);          // 关键：偏移到文件头，保证读最新值
    if(read(fd, str, sizeof(str)-1) <= 0) return 0;
    
    return atoi(str);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 DL1X设备初始化【核心流程：写1初始化→读该文件获型号→打开距离文件】
// 参数说明 无
// 返回参数 enum dl1x_device_type_enum 识别到的DL1X设备类型，值对应枚举定义
// 使用示例 enum dl1x_device_type_enum type = dl1x_dev.init();
//-------------------------------------------------------------------------------------------------------------------
enum dl1x_device_type_enum zf_device_dl1x::init(void)  // 【同步修改：返回值类型更新】
{
    int fd_event = -1;
    char read_buf[10] = {0};
    int read_val = NO_FIND_DEVICE;

    // 第一步：写入1 执行DL1X硬件初始化
    fd_event = open(DL1X_EVENT_PATH, O_RDWR);  // 读写模式打开：先写后读
    if(fd_event < 0)
    {
        printf("DL1X open event file fail: %s, errno:%d\r\n", DL1X_EVENT_PATH, errno);
        dl1x_dev_type = NO_FIND_DEVICE;       
        return dl1x_dev_type;
    }
    // 写入字符'1'完成初始化，符合sysfs写入规范
    if(write(fd_event, "1", 1) != 1)
    {
        printf("DL1X write 1 to event file fail, errno:%d\r\n", errno);
        close(fd_event);
        dl1x_dev_type = NO_FIND_DEVICE;       
        return dl1x_dev_type;
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
    if(read_val == ZF_DEVICE_DL1A || read_val == ZF_DEVICE_DL1B)
    {
        dl1x_dev_type = (enum dl1x_device_type_enum)read_val;  // 【核心修改：强制转换类型同步更新】
        // 打印数值+对应枚举名称
        switch(dl1x_dev_type)  // switch判断的变量类型已同步更新，case常量不变
        {
            case ZF_DEVICE_DL1A:
                printf("DL1X init success, type: %d (ZF_DEVICE_DL1A)\r\n", dl1x_dev_type);
                break;
            case ZF_DEVICE_DL1B:
                printf("DL1X init success, type: %d (ZF_DEVICE_DL1B)\r\n", dl1x_dev_type);
                break;
            default:
                printf("DL1X init success, type: %d (UNKNOWN)\r\n", dl1x_dev_type);
                break;
        }
    }
    else
    {
        printf("DL1X read invalid type val: %d, init fail\r\n", read_val);
        dl1x_dev_type = NO_FIND_DEVICE;
        return dl1x_dev_type;
    }

    // 第三步：打开距离数据文件
    fd_distance = open(DL1X_DISTANCE_PATH, O_RDONLY);
    if(fd_distance < 0)
    {
        printf("DL1X open distance file error, errno:%d\r\n", errno);
        dl1x_close_all_fd();
        dl1x_dev_type = NO_FIND_DEVICE;
    }

    return dl1x_dev_type;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 获取当前识别到的DL1X设备类型
// 参数说明 无
// 返回参数 enum dl1x_device_type_enum 设备类型枚举值
// 使用示例 enum dl1x_device_type_enum type = dl1x_dev.get_dev_type();
//-------------------------------------------------------------------------------------------------------------------
enum dl1x_device_type_enum zf_device_dl1x::get_dev_type(void)  // 【同步修改：返回值类型更新】
{
    return dl1x_dev_type; 
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 读取测距传感器原始距离数据
// 参数说明 无
// 返回参数 int16 距离原始值，未初始化返回0
// 使用示例 int16 distance = dl1x_dev.get_distance();
//-------------------------------------------------------------------------------------------------------------------
int16 zf_device_dl1x::get_distance(void)
{
    return dl1x_read_fd_data(fd_distance);
}