#ifndef _MOTOR_HPP_
#define _MOTOR_HPP_

#include "zf_common_typedef.hpp"
#include <stdio.h>

#define MOTOR_PWM_MAX 9000

typedef struct node1{
    float set_num;//设定速度 
    float actual_num;//实际速度
    float error;//现在偏差  
    float error_last;//上一个偏差 
    float error_Llast;
    float integral;
    float kp,ki,kd;//增量式比例，积分，微分参数   
}Use_pid;

typedef struct {
    //编码器速度
    float expect_Speed;
    float l_motor_speed;
    float r_motor_speed;
    // float round_speed;//会用到的

    //PWM速度
    float l_motor_pwm;
    float r_motor_pwm;
    // uint16 fan_pwm;//风扇的转速

    //方向环参数
    float dir_kp;
    float dir_kd;  
    float dir_kz;
    float bypass_dir_kp;
    float bypass_dir_kz;
    
    //期望角速度
    float expect_gyroz;

    float diff_add_gain;//左右差速比，这个根据调试的时候非给
    float diff_sub_gain;//左右差速比，这个根据调试的时候非给
    float diff_out;
    float speed_score;
    float speed_target_debug;
    float manual_speed_target;
}Speed_Dir_info;

typedef struct {
    float speed_max;//速度决策
    float speed_min;
    float speed_model_red;
    float gyro_deadband;
    
    float speed_err_max;
    float white_use_min;
    float white_ctrl_min;
    float white_ctrl_max;
    float weight_white;
    float weight_err;
    float weight_gyro;

    float startup_min;
    float startup_step_up;
    uint8 startup_exit_encoder_frames;

    float speed_deadband;
    float speed_step_up;
    float speed_step_down;
    uint16 brushless_use_duty;
    uint16 brushless_min_duty;
}Speed_limit;

extern Use_pid l_motor,r_motor;
extern Speed_Dir_info speed_dir_info;
extern Speed_limit speed_limit;
extern bool s_startup_active; // 起步阶段状态，全局可见，便于调试观察
extern bool motor_speed_decision_enable_flag;

void motor_par_init(void);
void motor_speed_decision_set_enable(bool enable);
void motor_use();
void motor_use_bypass(float time);
void motor_turn_test(float time);
void motor_speed_control(void);
void motor_hold_stop(void);
void motor_protect_reset_state(void);
void speed_uniform_acceleration(float target_speed,
                                float speed_max,
                                float speed_step_up,
                                float speed_step_down,
                                float speed_deadband);//最小速度默认是0

void motor_close_loop_test(void);

#endif 
