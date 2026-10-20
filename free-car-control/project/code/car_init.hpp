#ifndef _CAR_INIT_HPP_
#define _CAR_INIT_HPP_

#include "zf_common_headfile.hpp"
#include "image.hpp"

///////////宏////////////
//电机
#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1
#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2

//编码器
#define ENCODER_DIR_1_PATH           ZF_ENCODER_DIR_1
#define ENCODER_DIR_2_PATH           ZF_ENCODER_DIR_2

//按键
#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4

//蜂鸣器
#define BEEP_PATH        ZF_GPIO_BEEP

//摄像头
#define UVC_PATH       "/dev/video0"  // 摄像头设备节点（根据实际情况修改）

//tcp
#define SERVER_IP "192.168.3.178"
#define PORT 8086

///////////宏////////////

void car_init(void);
void tof_init(void);

///////////变量//////////
//电机
extern zf_driver_gpio  drv8701e_dir_1;
extern zf_driver_gpio  drv8701e_dir_2;
extern zf_driver_pwm   drv8701e_pwm_1;
extern zf_driver_pwm   drv8701e_pwm_2;
//编码器
extern zf_driver_encoder encoder_dir_1;
extern zf_driver_encoder encoder_dir_2;
//陀螺仪
extern zf_device_imu imu_dev;
//按键
extern zf_driver_gpio  key_1;
extern zf_driver_gpio  key_2;
extern zf_driver_gpio  key_3;
extern zf_driver_gpio  key_4;
//蜂鸣器
extern zf_driver_gpio  beep;
//屏幕
extern zf_device_ips200 ips200;
//tof
extern zf_device_dl1x dl1x_dev;
extern enum dl1x_device_type_enum dl1x_dev_type; 
//摄像头
extern zf_device_uvc uvc_dev;

//定时器
extern zf_driver_pit pit_use;

//逐飞助手传图
extern uint8 send_gray_image[UVC_HEIGHT][UVC_WIDTH];
extern uint16 send_color_image[UVC_HEIGHT][UVC_WIDTH];
extern uint16 send_color_image_120[IMG_PROC_HEIGHT][IMG_PROC_WIDTH];
///////////变量//////////

#endif 
