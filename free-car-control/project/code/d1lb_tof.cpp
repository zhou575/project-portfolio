#include "car_init.hpp"
#include "d1lb_tof.hpp"
#include "ramp_detect.hpp"

// 保存最近一次 DL1X 原始测距值。
volatile int16 dl1x_distance_raw = 0;
volatile bool tof_new_data = false;
float tof_distance_median = 0.0f;
float tof_distance_filtered = 0.0f;
bool tof_distance_valid = false;
float tof_flat_baseline = 0.0f;
bool tof_baseline_ready = false;

static int16 tof_median_buffer[TOF_MEDIAN_SIZE] = {0};
static uint8 tof_median_count = 0;
static uint8 tof_median_index = 0;
static bool tof_filter_initialized = false;

/**
 * @brief tcp读取原始数据
 * @note 放在主循环里读取，不放到 2 ms 中断里。
 */
void tof_get_data(void)
{
    // 仅读取DL1X距离数据，存入全局变量（主循环负责打印，避免回调内耗时）
    if(NO_FIND_DEVICE != dl1x_dev_type)  // 仅当设备初始化成功时读取
    {
        dl1x_distance_raw = dl1x_dev.get_distance();
        tof_new_data = true;
    }
}

/**
 * @brief 消费一次新的 ToF 数据并完成滤波
 * @note 只能放在主循环，不放入 2 ms PIT 回调。
 */
bool tof_process_new_data(void)
{
    if(!tof_new_data)
    {
        return false;
    }

    // 先保存本次数据，再清除标志，避免主循环重复处理同一个样本。
    const int16 raw_distance = dl1x_distance_raw;
    tof_new_data = false;
    tof_distance_valid = false;

    // 有效范围过滤，超出范围的数据不进入滤波器。
    if(raw_distance < TOF_VALID_MIN || raw_distance > TOF_VALID_MAX)
    {
        return false;
    }

    tof_median_buffer[tof_median_index] = raw_distance;
    tof_median_index = (uint8)((tof_median_index + 1) % TOF_MEDIAN_SIZE);
    if(tof_median_count < TOF_MEDIAN_SIZE)
    {
        tof_median_count++;
    }

    // 对当前窗口排序，取中间值，去除偶发跳点。
    int16 sorted[TOF_MEDIAN_SIZE];
    for(uint8 i = 0; i < tof_median_count; i++)
    {
        sorted[i] = tof_median_buffer[i];
    }
    for(uint8 i = 1; i < tof_median_count; i++)
    {
        const int16 value = sorted[i];
        int8 j = (int8)i - 1;
        while(j >= 0 && sorted[j] > value)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = value;
    }

    tof_distance_median = (float)sorted[tof_median_count / 2];

    // 第一个有效中值直接作为低通初值，避免从 0 缓慢爬升。
    if(!tof_filter_initialized)
    {
        tof_distance_filtered = tof_distance_median;
        tof_filter_initialized = true;
    }
    else
    {
        tof_distance_filtered =
            TOF_FILTER_ALPHA * tof_distance_median
            + (1.0f - TOF_FILTER_ALPHA) * tof_distance_filtered;
    }

    tof_distance_valid = true;
    return true;
}

/**
 * @brief 主循环里的 ToF 处理入口
 * @note 先更新视觉调试量；坡道入口当前只使用 ToF 滤波距离判定。
 */
void tof_pro(void)
{
    ramp_tof_update_visual_conditions();

    bool tof_sample_ready = false;
    if(ramp_tof_should_sample())
    {
        tof_get_data();
        tof_sample_ready = tof_process_new_data();
    }

    ramp_tof_update(tof_sample_ready);
}

/**
 * @brief 初始化阶段阻塞采集平地基准
 * @note ToF 已完成硬件初始化后调用，车辆必须静止放在平地。
 */
bool tof_flat_calibrate_blocking(void)
{
    if(NO_FIND_DEVICE == dl1x_dev_type)
    {
        tof_baseline_ready = false;
        return false;
    }

    int32 valid_sum = 0;
    uint16 valid_count = 0;

    printf("TOF flat calibration start...\r\n");

    for(uint16 attempt = 0;
        attempt < TOF_CALIBRATION_MAX_ATTEMPTS && valid_count < TOF_CALIBRATION_COUNT;
        attempt++)
    {
        const int16 distance = dl1x_dev.get_distance();

        if(distance >= TOF_VALID_MIN && distance <= TOF_VALID_MAX)
        {
            valid_sum += distance;
            valid_count++;
        }

        system_delay_ms(TOF_CALIBRATION_INTERVAL_MS);
    }

    if(valid_count == 0)
    {
        tof_baseline_ready = false;
        printf("TOF flat calibration failed: no valid samples\r\n");
        return false;
    }

    tof_flat_baseline = (float)valid_sum / (float)valid_count;
    tof_baseline_ready = (valid_count >= TOF_CALIBRATION_COUNT);

    // 运行采样开始时从平地基准重新建立滤波初值。
    tof_distance_median = tof_flat_baseline;
    tof_distance_filtered = tof_flat_baseline;
    tof_distance_valid = true;
    tof_new_data = false;

    if(tof_baseline_ready)
    {
        printf("TOF flat baseline ready: %.2f (%d samples)\r\n",
               tof_flat_baseline,
               (int)valid_count);
    }
    else
    {
        printf("TOF flat calibration incomplete: %d/%d samples\r\n",
               (int)valid_count,
               TOF_CALIBRATION_COUNT);
    }

    return tof_baseline_ready;
}
