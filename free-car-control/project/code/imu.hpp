#ifndef _IMU_HPP_
#define _IMU_HPP_

#include "zf_common_typedef.hpp"
#include <stdio.h>

//2ms定时器改成4500，其他时间再算
//计算方法 2ms内我让车最多转90度，那么4500=90/0.02
//实际这个最大值是10000,4500并不是归一化而是根据角度计算出来的
//但是实际车转向并不是很快，所以一般乘以10不然不好给pid，见motor文件
#define GYRO_MAX_VALUE 1000.0f



extern float yaw,model_yaw,pitch,test_use_angle,motor_control_gyro_z;
extern float gyro_y,gyro_z,f_gyro_y,f_gyro_z;

void gyroOffsetInit(void);
void gyro_process(float deadband);
void Control_yaw(float c_gyro_z,float time);
void Control_model_yaw(float c_gyro_z,float time);
void Control_yaw_test(float c_gyro_z,float time);

#endif
