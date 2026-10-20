#ifndef _D1LB_TOF_HPP_ 
#define _D1LB_TOF_HPP_

#include <stdio.h>
#include "zf_common_typedef.hpp"

// ==================== ToF 运行参数 ====================

// 正常运行阶段的 ToF 参考采样周期。10 ms 对应 100 Hz。
// 现在 ToF 采集放在主循环/图像处理链路中，不再由 2 ms 中断分频触发。
#define TOF_SAMPLE_PERIOD_MS       10

// ToF 有效距离范围，单位与 DL1X 返回值一致。
// 超出范围的数据会被丢弃，不进入滤波器。
#define TOF_VALID_MIN              20
#define TOF_VALID_MAX              2000

// 中值滤波窗口长度。越大越稳定，但响应坡道越慢。
#define TOF_MEDIAN_SIZE            2

// 一阶低通系数，范围 0～1。
// 越大响应越快，越小越平滑。
#define TOF_FILTER_ALPHA           0.5f

// 初始化阶段平地标定的有效样本数量。
#define TOF_CALIBRATION_COUNT      40

// 初始化阶段两次标定采样之间的间隔，单位 ms。
#define TOF_CALIBRATION_INTERVAL_MS 5

// 标定最多尝试次数，防止传感器持续返回无效值时无限阻塞。
#define TOF_CALIBRATION_MAX_ATTEMPTS 400

// 全局共享的最新 TOF 原始值，由 tof_get_data() 更新。
extern volatile int16 dl1x_distance_raw;

// 每次完成一次新的 ToF 读取后置位，由主循环消费后清零。
extern volatile bool tof_new_data;

// ToF 滤波结果，单位与 DL1X 原始距离值一致。
extern float tof_distance_median;
extern float tof_distance_filtered;
extern bool tof_distance_valid;
// 历史变量名保留；当前坡道逻辑中表示 ToF 设备可用，不再表示平地基准已标定。
extern float tof_flat_baseline;
extern bool tof_baseline_ready;

// 若 TOF 设备可用，则刷新 dl1x_distance_raw。

void tof_get_data(void);
bool tof_process_new_data(void);
void tof_pro(void);
bool tof_flat_calibrate_blocking(void);

#endif
