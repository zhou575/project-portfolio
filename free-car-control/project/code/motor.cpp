#include "car_init.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "image.hpp"
#include "imu.hpp"
#include "car_debug.hpp"
#include "lq_ncnn.hpp"
#include "ramp_detect.hpp"

Use_pid l_motor={0};
Use_pid r_motor={0};

Speed_Dir_info speed_dir_info={0};
Speed_limit speed_limit={0};
bool s_startup_active = true; // 起步状态全局变量，便于上位机/示波器调试查看
bool motor_speed_decision_enable_flag = true;
static bool s_motor_stall_protect_flag = false;
static int s_motor_stall_stop_time = 0;

void run_wheel_loop();

/**
 * @brief 电机参数初始化
 */
void motor_par_init()
{
    //实际上要调的几个参数
    //速度决策的四个
    //目前匀速最大可以跑45
    speed_limit.brushless_min_duty=500;
    speed_limit.brushless_use_duty=800;
    speed_limit.speed_min=0;//42;
    speed_limit.speed_max=0;//65;
    speed_limit.startup_step_up = 0.25f;          // 起步阶段最大加速步进
    speed_limit.speed_step_up=0.25f;    // 巡航阶段每周期最大加速步进
    //外环的两个
    speed_dir_info.dir_kp=  0.0f;//1.3
    speed_dir_info.dir_kz=  0.0f;//2

    speed_dir_info.diff_add_gain= 1.0f;   //外侧轮加速系数
    speed_dir_info.diff_sub_gain= 1.0f;  //内侧轮减速系数

    //PID参数如果调的好的话，速度肯定可以更快的
    //左电机闭环参数
    l_motor.kp=225;//180;//220;//150
    l_motor.ki=45;//16;//20;//15
    l_motor.kd=80;//240;//104;//305
    //右电机闭环参数
    r_motor.kp=225;//176;//220;//115
    r_motor.ki=45;//17;//20;//12
    r_motor.kd=80;//240;//104;//202

    //期望速度
    speed_dir_info.expect_Speed=0;
    // 当前使用“图像比例 + 陀螺补偿 + 速度内环”的控制思路
    speed_dir_info.bypass_dir_kp= 0.8f;
    speed_dir_info.bypass_dir_kz= 0.8f;
    //速度决策速度参数
    speed_limit.speed_model_red=20;
    //陀螺仪死区，减少轻微抖动
    speed_limit.gyro_deadband=0.1f;
    // 速度决策加权参数
    speed_limit.speed_err_max=0.25f;
    speed_limit.white_use_min=20.0f;   // 最长白列有效控制下限
    speed_limit.white_ctrl_min=100.0f;  // 白列速度分段下限
    speed_limit.white_ctrl_max=115.0f; // 白列速度分段上限
    speed_limit.weight_white=0.3f;     // 白列权重
    speed_limit.weight_err=0.4f;       // 误差权重
    speed_limit.weight_gyro=0.3f;      // 陀螺权重

    speed_limit.speed_deadband=0.2f;   // 速度接近目标时的死区
    speed_limit.speed_step_down=20.0f;  // 每周期最大减速步进（减速更快）

    // 起步阶段参数：起步从 0 开始，慢加速防翘头
    speed_limit.startup_min = 0.0f;              // 起步阶段最小速度
    speed_limit.startup_exit_encoder_frames = 3; // 连续满足编码器阈值的帧数后退出起步
    /*-----------------------------
     * PWM
     *-----------------------------*/
    speed_dir_info.l_motor_pwm = 0.0f;
    speed_dir_info.r_motor_pwm = 0.0f;
    speed_dir_info.speed_score = 0.0f;
    speed_dir_info.speed_target_debug = 0.0f;
    speed_dir_info.manual_speed_target = 0.0f;

    //输出必要参数
    printf("\r\n");
    printf("speed_control: \r\n");
    printf("speed_min:%.2f\r\n", speed_limit.speed_min);
    printf("speed_max:%.2f\r\n", speed_limit.speed_max);
    printf("\r\n");

    motor_protect_reset_state();
}

/**
 * @brief 电机pwm设置函数
 * @param l_pwm 左电机pwm
 * @param r_pwm 右电机pwm
 * @note 1是右电机，2是左电机
 */
void motor_pwm_set(float l_pwm,float r_pwm)
{
    if(r_pwm >= 0.0){
        drv8701e_dir_1.set_level(1);// DIR输出高电平
        drv8701e_pwm_1.set_duty((uint16)r_pwm);// 计算占空比
    }else{
        drv8701e_dir_1.set_level(0);// DIR输出低电平
        drv8701e_pwm_1.set_duty((uint16)(-r_pwm));// 计算占空比
    }
    if(l_pwm >= 0.0){
        drv8701e_dir_2.set_level(1);// DIR输出高电平
        drv8701e_pwm_2.set_duty((uint16)l_pwm);// 计算占空比
    }else{
        drv8701e_dir_2.set_level(0);// DIR输出低电平
        drv8701e_pwm_2.set_duty((uint16)(-l_pwm));// 计算占空比
    }
}

/**
 * @brief 电机堵转保护
 * @note 4个控制周期堵转直接保护
 */
// 保护态硬停：
// 1）清零目标量和控制中间量。
// 2）清空 PID 历史误差。
// 3）立即下发 0 PWM，确保轮子立刻停下。
static inline void protect_force_stop()
{
    speed_dir_info.expect_Speed = 0.0f;
    speed_dir_info.l_motor_speed = 0.0f;
    speed_dir_info.r_motor_speed = 0.0f;
    speed_dir_info.expect_gyroz = 0.0f;
    speed_dir_info.diff_out = 0.0f;

    speed_dir_info.l_motor_pwm = 0.0f;
    speed_dir_info.r_motor_pwm = 0.0f;

    l_motor.error = l_motor.error_last = l_motor.error_Llast = 0.0f;
    r_motor.error = r_motor.error_last = r_motor.error_Llast = 0.0f;

    motor_pwm_set(0.0f, 0.0f);
}

// 保护态闭环减速停：
// 1）清零外层目标量和差速中间量。
// 2）左右轮目标速度直接置 0。
// 3）保留当前 PWM 和 PID 历史，让增量式 PID 自己把车刹停。
static inline void protect_pid_stop()
{
    speed_dir_info.expect_Speed = 0.0f;
    speed_dir_info.l_motor_speed = 0.0f;
    speed_dir_info.r_motor_speed = 0.0f;
    speed_dir_info.expect_gyroz = 0.0f;
    speed_dir_info.diff_out = 0.0f;

    run_wheel_loop();
}

void motor_hold_stop(void)
{
    //speed_dir_info.expect_Speed = 0.0f;
    speed_dir_info.l_motor_speed = 0.0f;
    speed_dir_info.r_motor_speed = 0.0f;
    speed_dir_info.expect_gyroz = 0.0f;
    speed_dir_info.diff_out = 0.0f;
    speed_dir_info.l_motor_pwm = 0.0f;
    speed_dir_info.r_motor_pwm = 0.0f;
    speed_dir_info.speed_score = 0.0f;
    speed_dir_info.speed_target_debug = 0.0f;
    l_motor.error = l_motor.error_last = l_motor.error_Llast = 0.0f;
    r_motor.error = r_motor.error_last = r_motor.error_Llast = 0.0f;
    s_startup_active = true;
    motor_pwm_set(0.0f, 0.0f);
}

void motor_speed_decision_set_enable(bool enable)
{
    motor_speed_decision_enable_flag = enable;

    if (enable)
    {
        s_startup_active = true;
        speed_dir_info.speed_score = 0.0f;
        speed_dir_info.speed_target_debug = 0.0f;
    }
    else
    {
        if ((speed_dir_info.manual_speed_target <= 1e-6f) &&
            (speed_dir_info.expect_Speed > 1e-6f))
        {
            speed_dir_info.manual_speed_target = speed_dir_info.expect_Speed;
        }

        speed_dir_info.speed_score = 0.0f;
        speed_dir_info.speed_target_debug = speed_dir_info.manual_speed_target;
    }

    printf("motor speed decision %s\r\n", motor_speed_decision_enable_flag ? "ENABLED" : "DISABLED");
}

void motor_protect_reset_state(void)
{
    s_motor_stall_protect_flag = false;
    s_motor_stall_stop_time = 0;
}

// 电机堵转保护：
// 在有明确目标速度时，若 PWM 很高但编码器速度很低，则判定可能堵转。
// 只有持续一段时间异常才触发，避免短时波动误判。
// 速度低为了安全，还是判断堵转
static inline void motor_protect()
{
    // 速度门限：目标速度过低时不做堵转判定。
    // const bool running_fast_enough = fabsf(speed_dir_info.expect_Speed) >= 10.0f;
    const bool left_stall = (fabsf(speed_dir_info.l_motor_pwm) >= 8000.0f) && (fabsf(f_encoder_left) < 10.0f);
    const bool right_stall = (fabsf(speed_dir_info.r_motor_pwm) >= 8000.0f) && (fabsf(f_encoder_right) < 10.0f);

    if(imgpro_var.protect_flag || s_motor_stall_protect_flag)
    {
        s_motor_stall_stop_time = 0;
        return;
    }

    if(/*running_fast_enough && */(left_stall || right_stall))
    {
        s_motor_stall_stop_time++;
    }
    else
    {
        s_motor_stall_stop_time = 0;
    }

    // 2ms 周期下，100 次约等于 200ms 持续异常。
    if(s_motor_stall_stop_time >= 100)
    {
        printf("\r\n");
        printf("ERROR_INFO: motor.cpp : motor_protect\r\n");
        printf("motor_protect triggered, pwm_l=%.1f pwm_r=%.1f enc_l=%.1f enc_r=%.1f\r\n",
               speed_dir_info.l_motor_pwm, speed_dir_info.r_motor_pwm, f_encoder_left, f_encoder_right);
        printf("\r\n");
        s_motor_stall_protect_flag = true;
        s_motor_stall_stop_time = 0;
    }
}

/**
 * @brief 增量式pid控制
 * @param motor 传入电机参数
 * @param now 编码器现在的速度
 * @param expand_speed 期望的速度
 * @return increment_num 本此增量式pid得到的值
 * @note static 限制作用域(本文件)，inline内联加快函数调用速度
 */
static inline float PID_increment(Use_pid* motor,float now,float expand_speed)
{
    float increment_num=0;
    motor->error=expand_speed - now;//
    increment_num = motor->kp * (motor->error-motor->error_last)+motor->ki * motor->error+motor->kd * (motor->error-2*motor->error_last+motor->error_Llast);//增量计算公式  
    increment_num = CLAMP(increment_num, -8000.0f, 8000.0f);
    // 更新误差历史以供下一次计算使用  
    motor->error_Llast = motor->error_last;  
    motor->error_last = motor->error;
    return increment_num;
}

// static inline float calc_error_gain_from_white(uint8 white_num)
// {
//     float white_len = (float)CLAMP((int)white_num, 0, IMG_PROC_HEIGHT);

//     if (white_len >= 95.0f)
//     {
//         return 1.0f;
//     }
//     if (white_len <= 60.0f)
//     {
//         return 1.25f;
//     }

//     float t = (100.0f - white_len) / 35.0f;
//     t = CLAMP(t, 0.0f, 1.0f);

//     float s = t * t * (3.0f - 2.0f * t);

//     return 1.0f + (1.25f - 1.0f) * s;
// }


static inline float calc_error_gain_from_white(uint8 white_num)
{
    const float WHITE_LOW  = 90.0f;
    const float WHITE_HIGH = 110.0f;

    const float GAIN_MIN = 0.7f;
    const float GAIN_MAX = 1.4f;

    const float SLOPE_POWER = 2.0f;

    float white_len =
        (float)CLAMP((int)white_num, 0, IMG_PROC_HEIGHT);

    // 白列较长：使用最小误差增益
    if (white_len >= WHITE_HIGH)
    {
        return GAIN_MIN;
    }

    // 白列较短：使用最大误差增益
    if (white_len <= WHITE_LOW)
    {
        return GAIN_MAX;
    }

    /*
     * white_len = WHITE_HIGH 时，t = 0
     * white_len = WHITE_LOW  时，t = 1
     */
    float t =
        (WHITE_HIGH - white_len) /
        (WHITE_HIGH - WHITE_LOW);

    t = CLAMP(t, 0.0f, 1.0f);

    /*
     * 可调斜率的对称 S 曲线：
     *
     *                 t^k
     * s = ---------------------------
     *       t^k + (1 - t)^k
     */
    float t_power = powf(t, SLOPE_POWER);
    float inv_power = powf(1.0f - t, SLOPE_POWER);

    float denominator = t_power + inv_power;

    float s;
    if (denominator > 1e-6f)
    {
        s = t_power / denominator;
    }
    else
    {
        s = t;
    }

    return GAIN_MIN + (GAIN_MAX - GAIN_MIN) * s;
}

// static inline float calc_error_gain_from_white(uint8 white_num)
// {
//     const float WHITE_LOW  = 85.0f;
//     const float WHITE_HIGH = 110.0f;
//     const float GAIN_MIN   = 0.75f;
//     const float GAIN_MAX   = 1.5f;

//     float white_len =
//         (float)CLAMP((int)white_num, 0, IMG_PROC_HEIGHT);

//     // 最长白列较长：使用最小误差增益
//     if (white_len >= WHITE_HIGH)
//     {
//         return GAIN_MIN;
//     }

//     // 最长白列较短：使用最大误差增益
//     if (white_len <= WHITE_LOW)
//     {
//         return GAIN_MAX;
//     }

//     // 将 white_len 从 [120, 70] 映射到 t 的 [0, 1]
//     float t =
//         (WHITE_HIGH - white_len) /
//         (WHITE_HIGH - WHITE_LOW);

//     t = CLAMP(t, 0.0f, 1.0f);

//     // Smoothstep：使两端变化平缓
//     float s = t * t * (3.0f - 2.0f * t);

//     // 将 s 从 [0, 1] 映射到增益 [0.5, 2.0]
//     return GAIN_MIN + (GAIN_MAX - GAIN_MIN) * s;
// }

/**
 * @brief 左右轮子闭环
 */
// 速度内环：依据编码器反馈更新增量式 PID，并输出电机 PWM。
void run_wheel_loop()
{
    // speed_dir_info.l_motor_speed=speed_dir_info.r_motor_speed= speed_dir_info.expect_Speed;
    //PID
    speed_dir_info.l_motor_pwm+=PID_increment(&l_motor,f_encoder_left,speed_dir_info.l_motor_speed);
    speed_dir_info.r_motor_pwm+=PID_increment(&r_motor,f_encoder_right,speed_dir_info.r_motor_speed);
    //限幅
    speed_dir_info.l_motor_pwm=CLAMP(speed_dir_info.l_motor_pwm,-MOTOR_PWM_MAX,MOTOR_PWM_MAX);
    speed_dir_info.r_motor_pwm=CLAMP(speed_dir_info.r_motor_pwm,-MOTOR_PWM_MAX,MOTOR_PWM_MAX);
    
    motor_pwm_set(speed_dir_info.l_motor_pwm,speed_dir_info.r_motor_pwm);
}

/**
 * @brief 图像比例差速 + 陀螺补偿测试
 * @note 保留原函数名，实际控制思路改为：
 *       1. 图像误差直接按比例生成 steer_output
 *       2. 陀螺仪角速度换算为弧度制后作为补偿项直接加入左右轮目标速度
 *       3. 最终由左右轮增量式速度 PID 完成闭环
 */
void three_cascade_test(float image_err, float dir_kp_use, float dir_kz_use)
{
    //p  z都是1-2左右
    const float base_speed = speed_dir_info.expect_Speed;
    const bool lr_bypass_active =
        is_model_infer_complete &&
        (model_control_access == 'l' || model_control_access == 'r');
    const float error_gain = lr_bypass_active ? 1.0f : calc_error_gain_from_white(max_white.nums);
    const float error_use = image_err * error_gain;
    const float normalized_error = CLAMP(error_use, -1.0f, 1.0f);
    // const float normalized_error = CLAMP(image_err, -1.0f, 1.0f);
    float steer_output = base_speed * (normalized_error * dir_kp_use);
    const float steer_limit = fabsf(base_speed) * 1.07f;
    const float gyro_rad = motor_control_gyro_z * 0.01745329252f;//转化为弧度制 //范围-10到10左右
    const float gyro_comp = dir_kz_use * gyro_rad;

    steer_output = CLAMP(steer_output, -steer_limit, steer_limit);

    // 兼容现有调试变量：记录本周期图像转向输出和陀螺补偿量。
    speed_dir_info.diff_out = steer_output;
    speed_dir_info.expect_gyroz = gyro_comp;

    if(steer_output < 0.0f)
    {
        // speed_dir_info.l_motor_speed = base_speed + steer_output * speed_dir_info.diff_sub_gain + gyro_comp;
        // speed_dir_info.r_motor_speed = base_speed - steer_output * speed_dir_info.diff_add_gain - gyro_comp;
        speed_dir_info.l_motor_speed = base_speed + (steer_output + gyro_comp) * speed_dir_info.diff_sub_gain ;
        speed_dir_info.r_motor_speed = base_speed - (steer_output + gyro_comp) * speed_dir_info.diff_add_gain;
    }
    else
    {
        // speed_dir_info.l_motor_speed = base_speed + steer_output * speed_dir_info.diff_add_gain + gyro_comp;
        // speed_dir_info.r_motor_speed = base_speed - steer_output * speed_dir_info.diff_sub_gain - gyro_comp;
        speed_dir_info.l_motor_speed = base_speed + (steer_output + gyro_comp) * speed_dir_info.diff_add_gain;
        speed_dir_info.r_motor_speed = base_speed - (steer_output + gyro_comp) * speed_dir_info.diff_sub_gain;
    }
    
    run_wheel_loop();
}

//真正的三串
// void three_cascade_test(float image_err, float dir_kp_use)
// {
//     static float last_img_err=0;
//     float d_image_err;
//     float diff_raw;

//     //得到基础速度
//     const float base_speed=speed_dir_info.expect_Speed;
//     const float error_gain = calc_error_gain_from_white(max_white.nums);
//     const float error_use = image_err * error_gain;
//     const float normalized_error = CLAMP(error_use, -1.0f, 1.0f);
//     //转化为弧度制
//     const float gyro_rad = motor_control_gyro_z * 0.01745329252f;//大概在-17-17左右，一般不会大于10
//     //图像误差微分
//     d_image_err = normalized_error - last_img_err;
//     last_img_err = normalized_error;
//     //外环
//     speed_dir_info.expect_gyroz = speed_dir_info.dir_kp * normalized_error + speed_dir_info.dir_kd * d_image_err;
//     //角速度限幅
//     speed_dir_info.expect_gyroz=CLAMP(speed_dir_info.expect_gyroz,-10,10);
//     //内环
//     imu_pid.error=speed_dir_info.expect_gyroz + gyro_rad;  //必须是左转gyro_rad为负数
//     diff_raw=imu_pid.kp*(imu_pid.error)+imu_pid.kd*(imu_pid.error-imu_pid.error_last);
//     imu_pid.error_last=imu_pid.error;
//     //差速幅值限幅
//     const float steer_limit = fabsf(base_speed) * 1.07f;
//     speed_dir_info.diff_out = CLAMP(diff_raw, -steer_limit, steer_limit);

//     //得到左右轮的期望速度，前面已经对差速进行限幅，这里不再限幅
//     if(speed_dir_info.diff_out>0)
//     {
//         //加的多，减的不变
//         speed_dir_info.l_motor_speed = base_speed + speed_dir_info.diff_add_gain*speed_dir_info.diff_out;
//         speed_dir_info.r_motor_speed = base_speed - speed_dir_info.diff_sub_gain*speed_dir_info.diff_out;
//     }
//     else
//     {
//         speed_dir_info.l_motor_speed = base_speed + speed_dir_info.diff_sub_gain*speed_dir_info.diff_out;
//         speed_dir_info.r_motor_speed = base_speed - speed_dir_info.diff_add_gain*speed_dir_info.diff_out;
//     }
//     //分配左右轮目标速度
//     run_wheel_loop();
// }

//ok ,目前应该可以做到匀加速
void speed_uniform_acceleration(float target_speed,
                                float speed_max,
                                float speed_step_up,
                                float speed_step_down,
                                float speed_deadband)
{
    // 统一加减速函数：
    // 1) 目标速度先限幅。
    // 2) 加速按 step_up 慢慢抬升，减速按 step_down 快速下降。
    // 3) 进入死区后直接贴合目标，消除细小抖动。
    target_speed = CLAMP(target_speed, 0, speed_max);

    float current_speed = speed_dir_info.expect_Speed;
    const float diff = target_speed - current_speed;
    const float dead = CLAMP(speed_deadband, 0.0f, 10.0f);
    const float step_up = CLAMP(speed_step_up, 0.0f, 10.0f);
    const float step_down = CLAMP(speed_step_down, 0.0f, 10.0f);

    if (diff > dead)//目标速度和目前速度的差别要大于死区，才进行加减速
    {
        // 需要加速：每次最多 +step_up，慢加速防翘头。
        const float add = (diff < step_up) ? diff : step_up;
        current_speed += add;
    }
    else if (diff < -dead)
    {
        // 需要减速：每次最多 -step_down，快速减速防冲出。
        const float sub = ((-diff) < step_down) ? (-diff) : step_down;
        current_speed -= sub;
    }
    else
    {
        // 已接近目标：直接贴合目标速度，减少小抖动。
        current_speed = target_speed;
    }

    current_speed = CLAMP(current_speed, 0, speed_max);
    speed_dir_info.expect_Speed = current_speed;
}

static float motor_get_manual_speed_target(void)
{
    float target_speed = speed_dir_info.manual_speed_target;
    if (target_speed <= 1e-6f)
    {
        if (speed_limit.speed_min > 1e-6f)
        {
            target_speed = speed_limit.speed_min;
        }
        else
        {
            target_speed = speed_limit.speed_max;
        }
    }
    return CLAMP(target_speed, 0.0f, 100.0f);
}

static void motor_speed_control_manual(void)
{
    const float target_speed = motor_get_manual_speed_target();
    if (target_speed <= 1e-6f)
    {
        speed_dir_info.expect_Speed = 0.0f;
        speed_dir_info.speed_score = 0.0f;
        speed_dir_info.speed_target_debug = 0.0f;
        return;
    }

    float speed_max_use = speed_limit.speed_max;
    if (speed_max_use < target_speed)
    {
        speed_max_use = target_speed;
    }

    float step_up_use = speed_limit.speed_step_up;
    if (step_up_use <= 1e-6f)
    {
        step_up_use = speed_limit.startup_step_up;
    }

    speed_dir_info.speed_score = 0.0f;
    speed_dir_info.speed_target_debug = target_speed;
    s_startup_active = false;
    speed_uniform_acceleration(target_speed,
                               speed_max_use,
                               step_up_use,
                               speed_limit.speed_step_down,
                               speed_limit.speed_deadband);
}

//ok
//有必要分为起步和加减速，因为最小速度一定不是0，是弯道的速度，起步的速度是0
//感觉也没必要，我只需要限制每一步的current速度就行了
//重新读一下，感觉起步的时候有问题啊
void motor_speed_control()
{
//状态解锁没有问题
    // 特殊元素减速状态：
    // red_found 触发进入；模型推理完成状态从 true->false 时退出。
    static bool s_red_slowdown_active = false;
    static bool s_prev_model_infer_complete = false;
    const bool cur_model_infer_complete = is_model_infer_complete;
    if (model_red_found && !s_red_slowdown_active)
    {
        s_red_slowdown_active = true;//红块减速
    }
    //锁定
    if(cur_model_infer_complete)
    {
        s_prev_model_infer_complete=true;//推理完成
    }
    //解锁
    if (s_prev_model_infer_complete)
    {
        if(!cur_model_infer_complete)
        {
            s_prev_model_infer_complete=false;
            s_red_slowdown_active = false;
        }
    }
//状态解锁没有问题

    // 起步状态机：
    // s_startup_active=true 时使用 startup_min / startup_step_up；
    // 满足编码器连续阈值后切到巡航 speed_min / speed_step_up。
    static uint8 s_startup_encoder_ok_cnt = 0;

    float speed_min = speed_limit.speed_min;
    float speed_max = speed_limit.speed_max;

    // 防止上位机参数把最小/最大速度顺序改反：若 speed_min > speed_max，则直接返回。
    if (speed_min > speed_max)
    {
        speed_dir_info.expect_Speed = 0.0f;
        return;
    }

    // 速度总开关：当 speed_min=0 或 speed_max=0 时，强制期望速度为 0。
    if (fabsf(speed_min) <= 1e-6f 
        || fabsf(speed_max) <= 1e-6f)
    {
        s_startup_active = true; //起步状态
        s_startup_encoder_ok_cnt = 0;
        speed_dir_info.expect_Speed = 0.0f;
        return;
    }

    // 若当前速度已回到接近 0（比如保护后重新起跑），自动重新进入起步阶段。
    if (!s_startup_active && fabsf(speed_dir_info.expect_Speed) < 1e-3f)
    {
        s_startup_active = true;
        s_startup_encoder_ok_cnt = 0;
    }

    //大于三帧的时候起步状态切换为运行
    if (s_startup_active)
    {
        const float enc_abs_avg = 0.5f * (fabsf(f_encoder_left) + fabsf(f_encoder_right));
        const float startup_exit_th = CLAMP(speed_min - 2.0f, 0.0f, 100.0f);
        //比较的帧数，如果是0的话就是一帧
        const uint8 need_frames = (speed_limit.startup_exit_encoder_frames == 0U) ? 1U : speed_limit.startup_exit_encoder_frames;

        //编码器实际速度连续达到阈值，就退出起步
        if (enc_abs_avg >= startup_exit_th)
        {
            if (s_startup_encoder_ok_cnt < 255U) { s_startup_encoder_ok_cnt++; }
        }
        else
        {
            s_startup_encoder_ok_cnt = 0U;
        }

        if (s_startup_encoder_ok_cnt >= need_frames)
        {
            s_startup_active = false;
            s_startup_encoder_ok_cnt = 0;
        }
    }

    const float startup_min = CLAMP(speed_limit.startup_min, 0.0f, speed_max);
    float step_up_use = s_startup_active ? speed_limit.startup_step_up : speed_limit.speed_step_up;//这个可以从0.1开始调，直道不翘头为止
    if (step_up_use <= 1e-6f)
    {
        step_up_use = speed_limit.speed_step_up;
    }
    // 双保险：min_use 不能大于 speed_max。
    float min_use = s_startup_active ? startup_min : speed_min;
    if (min_use > speed_max) //一般不会出现这种情况
    {
        speed_dir_info.expect_Speed = 0.0f;
        return;
    }
//到这里最小速度切换，起步速度切换没有问题

    float target_speed = min_use;

    if (s_red_slowdown_active)
    {
        target_speed = speed_limit.speed_model_red;
        speed_dir_info.speed_score = 0.0f;
        speed_dir_info.speed_target_debug = target_speed;
        speed_uniform_acceleration(target_speed,
                                   speed_max,
                                   step_up_use,
                                   speed_limit.speed_step_down,
                                   speed_limit.speed_deadband);
        return;
    }


//打分机制没问题
    // 1) 最长白列评分（只使用 20~100 的白列区间做速度控制）。
    float white_len = (float)CLAMP((int)max_white.nums, 0, IMG_PROC_HEIGHT);
    if (white_len < speed_limit.white_use_min)
    {
        white_len = speed_limit.white_use_min;
    }
    float score_white = 0.0f;
    if (white_len < speed_limit.white_ctrl_min)
    {
        score_white = 0.0f;
    }
    else if (white_len <= speed_limit.white_ctrl_max)
    {
        const float denom = speed_limit.white_ctrl_max - speed_limit.white_ctrl_min;
        if (denom > 1e-6f)
        {
            score_white = (white_len - speed_limit.white_ctrl_min) / denom;
        }
    }
    else
    {
        score_white = 1.0f;
    }

    // 2) 图像误差评分（误差越小，分数越高,测试的误差应该在0.2左右速度就要减小到最小了）。
    const float err_abs = fabsf(imgpro_var.midline_error);
    const float err_score_norm = CLAMP(err_abs / speed_limit.speed_err_max, 0.0f, 1.0f);
    const float score_err = 1.0f - err_score_norm;

    // 3) 陀螺评分（角速度越小，分数越高）。
    //除以1000之后一般的大弯最大不超过0.3
    const float motor_gyro_z = motor_control_gyro_z / 1000;
    const float gyro_norm = fabsf(motor_gyro_z);
    const float score_gyro = 1.0f - CLAMP(gyro_norm /0.2 , 0.0f, 1.0f);

    // 加权归一化。
    const float w_white = CLAMP(speed_limit.weight_white, 0.0f, 1.0f);
    const float w_err = CLAMP(speed_limit.weight_err, 0.0f, 1.0f);
    const float w_gyro = CLAMP(speed_limit.weight_gyro, 0.0f, 1.0f);
    const float w_sum = w_white + w_err + w_gyro;
    float score = 0.0f;
    if (w_sum > 1e-6f)
    {
        //防止给错w_sum加起来不是1
        score = (w_white * score_white + w_err * score_err + w_gyro * score_gyro) / w_sum;
    }
    score = CLAMP(score, 0.0f, 1.0f);
//打分机制没问题

    //举个例子，现在min=20，max=50，最长白列是90，得到的分数是0.5*5/25=0.1,如果误差比较小是0.1,得到的分数是0.4*(1-0.1/0.25)=0.16,角速度也比较小是0.1，得到的分数是0.1*(1-0.9)=0.09,那么现在的速度就是20+30*(0.1+0.16+0.09)=30.5s
    target_speed = min_use + score * (speed_max - min_use);
    if(ramp_speed_limit_active() && target_speed > 30.0f)
    {
        target_speed = 30.0f;
    }
    speed_dir_info.speed_score = score;
    speed_dir_info.speed_target_debug = target_speed;
    speed_uniform_acceleration(target_speed,
                               speed_max,
                               step_up_use,
                               speed_limit.speed_step_down,
                               speed_limit.speed_deadband);
}

/**
 * @brief 用于电机控制测试
 * @note 限幅确定了之后使用motor_control接口，目前调试这个作为主要接口使用
 */
// 目前控制总入口（由 2ms 定时回调调用）：
// 1）读取编码器。
// 2）执行保护判定。
// 3）若已保护则硬停并返回。
// 4）否则执行正常控制器。
void motor_use()
{
    filter_encoder_get(0.4);
    motor_protect();

    if(s_motor_stall_protect_flag)
    {
        protect_force_stop();
        return;
    }

    if(imgpro_var.protect_flag==true)
    {
        // protect_force_stop();
        protect_pid_stop();
        return;
    }
     //motor_speed_control();//速度决策
    gyro_process(speed_limit.gyro_deadband);
    three_cascade_test(imgpro_var.midline_error, speed_dir_info.dir_kp, speed_dir_info.dir_kz);
}

/**
 * @brief 调节速度环pid用
 * @note speed_dir_info.l_motor_speed,speed_dir_info.r_motor_speed作为左右轮的期望速度
 */
void motor_close_loop_test()
{
    filter_encoder_get(0.4);
    run_wheel_loop();
}

/**
 * @brief: 转向测试，测试绕行转向灵敏度，给一个固定误差
 * @note 固定一个角度,角度大小可以自己定,转到固定角度后就停止
 */
void motor_turn_test(float time)
{
    filter_encoder_get(0.4);
    motor_protect();

    if(s_motor_stall_protect_flag)
    {
        protect_force_stop();
        return;
    }

    if(imgpro_var.protect_flag==true)
    {
        // protect_force_stop();
        protect_pid_stop();
        return;
    }
    gyro_process(speed_limit.gyro_deadband);
    Control_yaw_test(motor_control_gyro_z,time);

    if(model_yaw >= fabs(test_use_angle))
    {
        imgpro_var.midline_error=0;
    }
    three_cascade_test(imgpro_var.midline_error, speed_dir_info.dir_kp,speed_dir_info.dir_kz);
}

/**
 * @brief 根据当前模型绕行状态选择外环 P 参数
 * @note 不改动 dir_kp / bypass_dir_kp 本体值，只返回本周期控制使用值
 */
static inline float select_dir_kp_for_control(void)
{
    const bool bypass_active =
        is_model_infer_complete &&
        (model_control_access == 'l' || model_control_access == 'r');

    return bypass_active ? speed_dir_info.bypass_dir_kp : speed_dir_info.dir_kp;
}

/**
 * @brief 根据当前模型绕行状态选择外环 z 参数
 * @note 不改动 dir_kz / bypass_dir_kz 本体值，只返回本周期控制使用值
 */
static inline float select_dir_kz_for_control(void)
{
    const bool bypass_active =
        is_model_infer_complete &&
        (model_control_access == 'l' || model_control_access == 'r');

    return bypass_active ? speed_dir_info.bypass_dir_kz : speed_dir_info.dir_kz;
}

/**
 * @brief 三串控制+绕行逻辑
 * @note 增加绕行编码器积分
 */
void motor_use_bypass(float time)
{
    static uint8 speed_ctrl_div2_cnt = 0; // 速度决策分频计数：每2次调用执行1次

    filter_encoder_get(0.4);//编码器采集
    motor_protect();//堵转保护

    if(s_motor_stall_protect_flag)
    {
        protect_force_stop();
        return;
    }

    //保护
    if(imgpro_var.protect_flag==true)
    {
        // protect_force_stop();
        protect_pid_stop();
        return;
    }

    //速度决策（每2次运行一次）4ms一次，改成5ms的话就5ms一次
    // speed_ctrl_div2_cnt++;
    // if (speed_ctrl_div2_cnt >= 2U)
    // {
    //     speed_ctrl_div2_cnt = 0U;
    //     motor_speed_control();
    // }

    //绕行编码器积分
    if(is_model_infer_complete &&
       (model_control_access == 'l' || model_control_access == 'r' || model_control_access == 'g'))
    {
        model_encoder_increment();
    }
    else
    {
        model_encoder_clear_increment();
    }

    //陀螺仪角速度处理和三串级控制
    gyro_process(speed_limit.gyro_deadband);
    Control_model_yaw(motor_control_gyro_z,time);

    const float dir_kp_use = select_dir_kp_for_control();
    const float dir_kz_use = select_dir_kz_for_control();
    three_cascade_test(imgpro_var.midline_error, dir_kp_use,dir_kz_use);
}
