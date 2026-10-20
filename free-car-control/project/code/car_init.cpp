#include "car_init.hpp"
#include "imu.hpp"
#include "image.hpp"
#include "car_debug.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "lq_ncnn.hpp"
#include "fan.hpp"
#include "ramp_detect.hpp"

// 全局硬件实例在本文件定义，在头文件通过 extern 对外引用。
// 这样可以集中管理设备对象，避免重复初始化。

struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;
enum dl1x_device_type_enum dl1x_dev_type;  
uint8 send_gray_image[UVC_HEIGHT][UVC_WIDTH];//传输到逐飞助手要拷贝的灰度图像
uint16 send_color_image[UVC_HEIGHT][UVC_WIDTH];//传输到逐飞助手要拷贝彩色的图像
uint16 send_color_image_120[IMG_PROC_HEIGHT][IMG_PROC_WIDTH];//传输到逐飞助手要拷贝彩色图像（160x120）

//按键
zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);
zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);
zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);
zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);
//电机  1是右电机，2是左电机
zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);
//编码器
zf_driver_encoder encoder_dir_1(ENCODER_DIR_1_PATH);
zf_driver_encoder encoder_dir_2(ENCODER_DIR_2_PATH);
//陀螺仪
zf_device_imu imu_dev;
//蜂鸣器
zf_driver_gpio  beep(BEEP_PATH, O_RDWR);
//屏幕
zf_device_ips200 ips200;
//tof
zf_device_dl1x dl1x_dev;
//摄像头
zf_device_uvc uvc_dev;


//TCP
zf_driver_tcp_client tcp_client_dev;
//定时器
zf_driver_pit pit_use;//定义了,运行代码的时候日志就会输出

/**
 * @brief 注册清理函数
 * @note 处理 Ctrl+C 中断，SIGINT 处理函数：将 Ctrl+C 转为正常退出，以便触发 cleanup()。
 */
void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

/**
 * @brief 注册SIGINT信号的处理函数
 * @note 收到Ctrl+C，之后先运行sigint_handler，然后exit(0)之后运行cleanup
 *       atexit 注册的清理流程：停止周期任务并强制电机 PWM 归零。
 */
void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    pit_use.stop();
    // 显式释放 ncnn 资源，避免进程退出阶段出现静态析构顺序问题。
    lq_ncnn_deinit();
    printf("程序异常退出，执行清理操作\n");
    fan_cleanup_on_exit();
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);    
}

/**
 * @brief 图像初始化函数，摄像头初始化独立封装，便于与其他外设问题分离排查。
 */
void camera_init()
{
    if (uvc_dev.init(UVC_PATH) < 0)
    {
        std::cerr << "摄像头初始化失败！请检查设备节点或摄像头连接" << std::endl;
    }
}

/**
 * @brief tof初始函数
 * @note TOF 初始化在当前主流程中为可选项，独立出来便于开关。
 */
void tof_init()
{
    // DL1X传感器初始化
    dl1x_dev_type = dl1x_dev.init();

    // 判断DL1X初始化结果
    if(NO_FIND_DEVICE == dl1x_dev_type)
    {
        printf("错误：未找到DL1X设备，请检查硬件连接！\r\n");
    }
    else
    {
        printf("DL1X 初始化成功！识别到：");
        if(ZF_DEVICE_DL1A == dl1x_dev_type) printf("ZF_DEVICE_DL1A\r\n");
        else if(ZF_DEVICE_DL1B == dl1x_dev_type) printf("ZF_DEVICE_DL1B\r\n");
    }
}

/**
 * @brief tcp发送数据到缓冲区
 * @note 放在定时器里面，发送关键变量到助手端示波器显示，便于观察变量变化趋势和调试。seekfree 助手发送适配层：通过 tcp_client 发送数据。
 */
uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data(buf, len);
}

/**
 * @brief tcp从缓冲区读取数据
 * @note 放在定时器里面，发送关键变量到助手端示波器显示，便于观察变量变化趋势和调试。seekfree 助手接收适配层：通过 tcp_client 读取数据。
 */
uint32 tcp_read_wrap( uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}

/**
 * @brief TCP,WIFI初始化，用wifi连接上位机，与逐飞助手进行通信
 * @note 建立 TCP 连接并绑定助手协议回调。成功连接服务器后，注册助手接口回调函数，供助手协议层调用发送/接收函数进行通信。
 */
void tcp_init()
{
    if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");
    }
    else
    {
        printf("tcp_client error\r\n");
    }
    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);
}

/**
 * @brief tcp传输灰度图像大小和类型初始化 120*160
 * @note 注册灰度图流格式给助手端显示，配置摄像头信息：指定摄像头类型为灰度摄像头MT9V03X、图像缓冲区地址、图像宽高
 */
void tcp_send_gray_image_init()
{
    // 边界发送使用搜线坐标系：160x120。
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, send_gray_image[0], IMG_PROC_WIDTH, IMG_PROC_HEIGHT);
}

/**
 * @brief tcp传输彩色图像大小和类型初始化 120*160
 * @note 注册 RGB565 图流格式给助手端显示，配置摄像头信息：指定摄像头类型为灰度摄像头MT9V03X、图像缓冲区地址、图像宽高
 */
void tcp_send_color_image_init()
{
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, send_color_image_120[0], IMG_PROC_WIDTH, IMG_PROC_HEIGHT);
}

/**
 * @brief tcp传输彩色图像大小和类型初始化 240*320
 * @note 注册 RGB565 图流格式给助手端显示，缓冲区直接使用整幅彩图发送缓存。
 */
void tcp_send_color_image_init_240_320()
{
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660, send_color_image[0], UVC_WIDTH, UVC_HEIGHT);
}

/**
 * @brief tcp传输边界信息初始化，一共可以传输三条线
 * @param dot_len 三条线的长度
 * @note 初始化不尽行传参，默认传输左右边线和最长白列，需要传输其他的线自己改
 */
void tcp_send_boundary_init(int16 dot_num,uint8* x1,uint8* x2,uint8* x3,uint8* y1,uint8* y2,uint8* y3)
{
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, dot_num, x1, x2, x3, y1, y2, y3);
}

/**
 * @brief 小车设备总体初始化
 * @note  编码器的值可以直接读写文件获取
 */
// 系统上电初始化顺序：
// 1）初始化传感器/屏幕/摄像头/网络。
// 2）初始化算法参数。
// 3）注册安全退出程序。
void car_init()
{
    //外设初始化
    imu_dev.init();//陀螺仪
    ips200.init(FB_PATH);//屏幕
    tof_init();
    camera_init();//摄像头

    tcp_init();//tcp
    // if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_GRAY)
    // {
    //     tcp_send_gray_image_init();//逐飞助手灰度图像传输初始化
    // }
    // else
    // {
    //     tcp_send_color_image_init();//逐飞助手彩色图像传输初始化
    // }
    // tcp_send_boundary_init(IMG_PROC_HEIGHT,
    //                        tcp_left_boundary_x,
    //                        tcp_right_boundary_x,
    //                        tcp_middle_line_x,
    //                        tcp_left_boundary_y,
    //                        tcp_right_boundary_y,
    //                        tcp_middle_line_y);//逐飞助手边界传输初始化
    
    //电机pwm
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);
    
    //参数初始化
    gyroOffsetInit();//陀螺仪去零飘
    image_pro_init();//图像处理变量初始化
    motor_par_init();//电机参数初始化
    fan_init();
    encoder_clear();//编码器计数清零,防止闭环突然冲一下
    ramp_tof_init();//初始化坡道ToF状态，不再采集平地基准
    
    atexit(cleanup);
    signal(SIGINT, sigint_handler);
}
