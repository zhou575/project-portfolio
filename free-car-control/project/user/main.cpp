/*********************************************************************************************************************
* 修改记录
* 日期            作者           备注
* 2026-3-27       zhy            V3.0
********************************************************************************************************************/

//数据集采集用 red_roi_debug_test_video
//模型读图片验证用 lq_ncnn_batch_infer_picture_dir

#include "car_init.hpp"
#include "car_debug.hpp"
#include "image.hpp"
#include "motor.hpp"
#include "imu.hpp"
#include "encoder.hpp"
#include "menu.hpp"
#include "red_roi.hpp"
#include "lq_ncnn.hpp"
#include "fan.hpp"
#include "d1lb_tof.hpp"
#include "ramp_detect.hpp"

#include <chrono>
#include <thread>

#define RING_DEBUG_MOTOR 1

// 编译时打开后运行纯 ToF 终端测试，不启动摄像头和电机控制。
// 构建命令：cmake -S project/user -B project/out -DTOF_DISTANCE_TEST=ON
#ifndef TOF_DISTANCE_TEST
#define TOF_DISTANCE_TEST 0
#endif

// key_scanner 按 10ms 设计，而 PIT 回调是 2ms 一次。
// 这里累计 5 个 2ms 周期后再扫描一次按键。
int key_time10=0;

// 2ms 实时控制回调：
// 1）按固定节拍扫描按键
// 2）执行电机闭环控制
// 3）处理上位机在线调参数据
//定时器只看控制就行了，参数更改和示波器还是放在主函数里面
void pit_callback(void)
{
#if TOF_DISTANCE_TEST
    // ToF 测试模式不执行按键、电机和其他实时控制逻辑。
    return;
#endif

    if(key_time10>=4)
    {
        key_scanner(10);
        key_time10=0;
    }
    else key_time10++;

    //Oscill_param_speed();
    Oscill_param_dir();
    
    // const bool speed_valid =
    //     // ((speed_limit.speed_min > 0.0f) &&
    //     //  (speed_limit.speed_max > 0.0f) &&
    //     //  (speed_limit.speed_min <= speed_limit.speed_max)) ||
    //     (speed_dir_info.expect_Speed > 0.0f);
    // fan_update_2ms(speed_valid);
#if RING_DEBUG_MOTOR
    //motor_use();
    motor_use_bypass(0.002);
    // if (fan_is_ready())
    // {
    //     motor_use();//无绕行控制
    // }
    // else
    // {
    //     motor_hold_stop();
    // }
#else
    // 电机调试关闭时仍保持传感器数据链更新，但不允许输出驱动 PWM。
    gyro_process(speed_limit.gyro_deadband);
    filter_encoder_get(0.4f);
    //motor_hold_stop();
#endif
    // // motor_close_loop_test();
    //Oscill(); 

        //gyro_process(speed_limit.gyro_deadband);
    // Oscill_param_dir();
    // 控制算法必须固定周期运行，PID 才稳定。
    // motor_use_bypass(0.002);//有绕行控制
    
    // 在线参数更新放在同一控制节拍内，避免时序漂移。
    // motor_turn_test(0.002);
    // Oscill_param_turn_test();
    // Oscill_param_spid();
}

#if TOF_DISTANCE_TEST

int main(int, char**)
{
    tof_init();
    if(NO_FIND_DEVICE == dl1x_dev_type)
    {
        printf("ToF test aborted: DL1X device not found.\r\n");
        return 1;
    }

    printf("ToF test started: sampling in main loop, target period 10 ms.\r\n");
    printf("Keep the sensor still and observe the distance values.\r\n");

    pit_use.init_ms(2, pit_callback);

    while(1)
    {
        tof_get_data();
        if(tof_process_new_data())
        {
            if(tof_distance_valid)
            {
                printf("TOF raw=%d median=%.2f filtered=%.2f\r\n",
                       (int)dl1x_distance_raw,
                       tof_distance_median,
                       tof_distance_filtered);
            }
            else
            {
                printf("TOF raw=%d invalid\r\n", (int)dl1x_distance_raw);
            }
            fflush(stdout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(TOF_SAMPLE_PERIOD_MS));
    }
}

#else

int main(int, char**)
{
    // 初始化所有外设、算法参数和退出清理回调
    car_init();
    if (lq_ncnn_init() != 0)
    {
        std::cerr << "NCNN模型初始化失败" << std::endl;
    }

    // 启动 2ms 周期中断，驱动实时控制链路
    // 定时器CPU占用率 6-8%
    pit_use.init_ms(2, pit_callback);
    while(1)
    {
        // 图像刷新放在主循环：每轮先等待新帧。
        // 图像采集CPU占用率30-38%,耗时4.8-6.8ms

        int8_t refresh_ret = uvc_dev.wait_image_refresh();
        if (refresh_ret != 0)
        {
            std::cerr << "图像刷新失败，跳过当前帧" << std::endl;
            continue;
        }

        cv::Mat src_img = uvc_dev.get_frame_mjpg();

    // 正常搜线无绕行
    //image_pro();
    // ToF 采样和坡道判定放到主循环，不占用 2 ms 控制中断。
    tof_pro();
    //   ramp_tof_debug_print();

    image_pro_bypass();
    model_use_bypass(src_img);//模型识别到了，当前帧就开始控制了
    update_image_error_bypass();
    key_use_bypass();
    //  tof_pro();
        // model_use(src_img);
        //update_image_error();
        // key_use();
    // ips200_main_use();
        
        //  printf("yaw : %f ", yaw);
        // 搜线+绕行
        // image_pro_bypass();
        // model_use_bypass(src_img);//模型识别到了，当前帧就开始控制了
        // update_image_error_bypass();
        // key_use_bypass();

        // red_roi_debug_test_video(src_img);222
        // Oscill_param_img();
         //Oscill();
         //tcp_main_use();
        
        
          //ips200_main_use();
    }
}

#endif

/*
//图像转换，耗时约0.4ms，放心使用
uint16* rgb_image = uvc_dev.get_rgb_image_ptr();
uint8* gray_image = uvc_dev.get_gray_image_ptr();

// 模型测试,批量读取图片然后验证，自动读取文件夹中的图片，如果只是测试的话，把下面代码直接放在While(1)前面，while(1)可以注释掉
const int batch_infer_ret = lq_ncnn_batch_infer_picture_dir("picture");
if (batch_infer_ret != 0)
{
    std::cerr << "批量推理失败，请检查 picture 目录或图片格式" << std::endl;
}

//最长白列测试，只得到最长白列，用于看白列是否有效，以及白列的值
image_max_white_test();
 */



// 时间测量
/*
    static int count=0;
    static double elapsed_sum_ms = 0.0;
    count++;
    auto start_time = std::chrono::steady_clock::now(); 
    
    //要测量的代码

    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> elapsed = current_time - start_time;       
    elapsed_sum_ms += elapsed.count();
    if(count >= 100)
    {
        printf("use_time_avg_100 = %f ms.\r\n", elapsed_sum_ms / count);
        elapsed_sum_ms = 0.0;
        count=0;
    }
*/
