# ToF + 视觉联合坡道检测：Codex 任务上下文与实现说明

## 1. 文档用途

本文件用于把当前关于“智能车坡道检测”的讨论完整交接给 Codex。

请 Codex 在读取本文件后，结合当前智能车工程代码，完成坡道检测模块的设计、接入和调试输出。不要自行恢复本文件已经明确删除的视觉条件，也不要把 ToF 坡道判断简化为单个固定距离阈值。

---

## 2. 当前硬件与基础代码

### 2.1 硬件

- 主控平台：LS2K0300。
- 测距模块：逐飞 DL1X 系列，例程可自动识别 DL1A 或 DL1B。
- 接口：I2C。
- ToF 安装方式：安装在车头，向前下方倾斜，测量车前方地面。
- 推荐安装高度：约 8～15 cm。
- 推荐俯角：约 25°～45°。
- ToF 用于检测前方地面高度变化，不是正对前方检测墙体。

### 2.2 已有例程

当前已有 `main.cpp` ToF 例程，其主要结构如下：

1. 创建 `zf_device_dl1x dl1x_dev`。
2. 使用 `dl1x_dev.init()` 初始化并识别 DL1A/DL1B。
3. 使用 PIT 定时器周期调用回调函数。
4. 回调中执行：

```cpp
dl1x_distance_raw = dl1x_dev.get_distance();
```

5. 原例程采样周期为 100 ms。
6. 原例程主循环只负责打印距离。

目标是把该例程从“打印测试代码”改造成可接入智能车主工程的坡道检测模块。

---

## 3. 坡道检测总体思路

检测采用：

```text
ToF 测距为主判断
+
视觉条件作为场景允许条件
+
连续多帧确认
+
状态机锁定
```

核心原则：

- ToF 负责判断前方地面距离是否出现符合上坡的变化。
- 视觉只负责判断当前是否处于“边线完整的长直道”。
- 视觉不直接证明前方一定是坡道。
- 不要求视觉逻辑非常严格，避免视觉条件过多导致漏判。
- 禁止仅使用 `tof_distance < 固定阈值` 判断坡道。
- 禁止只用一帧数据确认坡道。

---

## 4. 已观察到的坡道图像特征

车辆逐渐靠近坡道时，图像中曾观察到以下现象：

1. 图像远端赛道宽度逐渐增大。
2. 左右边线逐渐趋于平行。
3. 坡道入口附近可能出现横向分界线。
4. 横向分界线随车辆接近向图像下方移动。
5. 左右边线可能在相近行出现斜率变化。
6. 中线横坐标变化不明显，不适合作为主要坡道特征。
7. 亮度变化受环境影响较大，不适合作为核心条件。

曾讨论过：

```cpp
upper_ratio = upper_lane_width / lower_lane_width;
```

以及：

```cpp
ratio_abnormal
```

但最终决定：**不使用 `ratio_abnormal` 作为当前坡道视觉条件。**

不要在实现时自行重新加入该条件，除非用户后续明确要求。

---

## 5. 最终确定的视觉辅助条件

视觉只用于确认：

```text
当前左右边线基本可靠
并且
当前场景像长直道
```

最终视觉条件为：

```cpp
bool visual_ramp_allow =
    left_edge_quality_ok
    && right_edge_quality_ok
    && enough_long_white_columns;
```

### 5.1 `left_edge_quality_ok`

表示左边线在指定检测区域内无效点比例没有超过允许阈值。

推荐：

```cpp
left_edge_quality_ok =
    left_invalid_ratio < LEFT_INVALID_LIMIT;
```

初始阈值可以从：

```cpp
LEFT_INVALID_LIMIT = 0.15f;
```

开始调试。

注意：

- 左右边线必须分别统计。
- 不要把左右两侧无效点合并后只计算一个总比例。
- 统计范围应使用可靠图像区域，不建议统计消失点附近和图像最底部。
- “边线有效”最好依据工程已有的原始边线有效标志，不要把人为补线默认当作原始可靠边线。

### 5.2 `right_edge_quality_ok`

与左边线逻辑一致：

```cpp
right_edge_quality_ok =
    right_invalid_ratio < RIGHT_INVALID_LIMIT;
```

初始阈值可以从：

```cpp
RIGHT_INVALID_LIMIT = 0.15f;
```

开始调试。

### 5.3 `enough_long_white_columns`

原始想法是使用“最长白列不能太短”确认长直道。

实现时不要只依赖单独一个最大白列，因为单列噪声、过曝或错误二值化可能造成虚假的超长白列。

推荐使用以下任一已有工程特征：

方案 A，优先使用工程中已有的“最长白列”定义：

```cpp
enough_long_white_columns =
    longest_white_column >= LONG_WHITE_COLUMN_MIN;
```

如果当前工程已经稳定使用该变量判断直道，可以先保持原逻辑，避免扩大修改范围。

方案 B，如果工程中没有成熟逻辑，则改为统计中央区域中满足长度要求的白列数量：

```cpp
enough_long_white_columns =
    long_white_column_count >= MIN_LONG_WHITE_COLUMNS;
```

视觉只是辅助，因此阈值不应过严。

---

## 6. ToF 测距判断逻辑

### 6.1 平地与坡道时的距离变化

ToF 向前下方照射地面。

正常平地：

```text
测距值在某个平地基准附近稳定波动
```

车辆接近上坡：

```text
光束提前打到抬高的坡面
测距值相对平地基准逐渐减小
```

因此主要判断量为：

```cpp
distance_drop =
    tof_flat_baseline - tof_distance_filtered;
```

当 `distance_drop` 足够大时，说明当前测距明显短于平地基准。

---

## 7. ToF 采样要求

### 7.1 采样周期

原例程为 100 ms，建议改为：

```cpp
#define TOF_SAMPLE_PERIOD_MS 50
```

即约 20 Hz。

先使用 50 ms 调试，不要一开始设置过快。

### 7.2 PIT 回调职责

PIT 回调只做两件事：

1. 读取距离。
2. 设置新数据标志。

示例：

```cpp
volatile int16 tof_distance_raw = 0;
volatile bool tof_new_data = false;

void dl1x_pit_callback(void)
{
    if(NO_FIND_DEVICE != dl1x_dev_type)
    {
        tof_distance_raw = dl1x_dev.get_distance();
        tof_new_data = true;
    }
}
```

禁止在 PIT 回调中执行：

- 大量打印；
- 排序；
- 状态机；
- 图像处理；
- 耗时逻辑。

---

## 8. ToF 数据滤波

建议使用两级滤波。

### 8.1 有效范围过滤

示例初始范围：

```cpp
#define TOF_VALID_MIN 20
#define TOF_VALID_MAX 2000
```

实际范围应结合 DL1X 返回单位和实测数据调整。

无效数据：

```cpp
if(raw_distance < TOF_VALID_MIN ||
   raw_distance > TOF_VALID_MAX)
{
    return;
}
```

### 8.2 5 点中值滤波

作用：

- 去除偶发跳点；
- 避免一个异常值直接触发坡道。

建议窗口：

```cpp
#define TOF_MEDIAN_SIZE 5
```

### 8.3 一阶低通滤波

在中值滤波后增加：

```cpp
filtered =
    alpha * median
    + (1.0f - alpha) * previous_filtered;
```

建议初值：

```cpp
alpha = 0.3f;
```

`alpha` 越大响应越快，越小越平滑。

---

## 9. 平地基准标定

### 9.1 标定方式

车辆上电时放在正常平地直道，保持静止，采集一段时间得到：

```cpp
tof_flat_baseline
```

建议：

```cpp
#define TOF_CALIBRATION_COUNT 40
```

采样周期 50 ms 时，40 次约为 2 秒。

计算：

```cpp
tof_flat_baseline =
    valid_sample_sum / valid_sample_count;
```

标定阶段：

- 只接受有效测距数据。
- 不进行坡道检测。
- 应打印标定完成后的基准值。
- 标定前整车不应立即进入坡道状态。

### 9.2 基准缓慢更新

正常平地状态下，如果当前滤波距离与基准很接近，可以很慢地更新基准：

```cpp
tof_flat_baseline =
    0.995f * tof_flat_baseline
    + 0.005f * tof_distance_filtered;
```

但只有在：

```cpp
abs(tof_distance_filtered - tof_flat_baseline)
    < BASELINE_UPDATE_WINDOW
```

时才允许更新。

进入疑似坡道或确认坡道后，禁止继续更新平地基准，否则会把坡道值学习成新的平地。

---

## 10. ToF 坡道条件

建议定义：

```cpp
float distance_drop =
    tof_flat_baseline - tof_distance_filtered;
```

动态阈值：

```cpp
float drop_threshold =
    max(TOF_DROP_MIN,
        tof_flat_baseline * TOF_DROP_RATIO);
```

建议初始值：

```cpp
#define TOF_DROP_MIN   20
#define TOF_DROP_RATIO 0.10f
```

注意：这些数字只用于初始测试，最终必须依据真实测距日志调整。

基础距离下降条件：

```cpp
bool distance_drop_ok =
    distance_drop > drop_threshold;
```

---

## 11. 距离下降趋势

为了区分坡道和单帧噪声，初次进入疑似状态时需要检查短时间趋势。

保存最近若干次滤波距离，例如 4 个采样：

```cpp
history[0] = 当前值;
history[3] = 较早值;
```

计算：

```cpp
trend_drop =
    history[3] - history[0];
```

当：

```cpp
trend_drop > TOF_TREND_DROP_MIN
```

说明距离在最近约 150～200 ms 内总体减小。

建议初值：

```cpp
#define TOF_TREND_DROP_MIN 5
```

趋势条件只用于从普通状态进入疑似状态。

进入疑似状态后，不要求每一帧继续严格下降，因为车辆靠近坡道后，测距可能短暂稳定。

---

## 12. 状态机

建议状态：

```cpp
enum tof_ramp_state_enum
{
    TOF_RAMP_CALIBRATING = 0,
    TOF_RAMP_FLAT,
    TOF_RAMP_SUSPECT,
    TOF_RAMP_CONFIRMED
};
```

### 12.1 `TOF_RAMP_CALIBRATING`

- 完成平地基准标定。
- 不输出坡道确认。

### 12.2 `TOF_RAMP_FLAT`

进入疑似状态需要同时满足：

```cpp
distance_drop_ok
&& distance_trend_ok
&& visual_ramp_allow
```

### 12.3 `TOF_RAMP_SUSPECT`

连续多次满足：

```cpp
distance_drop_ok
&& visual_ramp_allow
```

才确认坡道。

建议：

```cpp
#define TOF_RAMP_CONFIRM_COUNT 3
```

计数方式可以采用加一减一，而不是某一帧失败立即清零：

```cpp
if(condition)
{
    suspect_count++;
}
else if(suspect_count > 0)
{
    suspect_count--;
}
```

### 12.4 `TOF_RAMP_CONFIRMED`

确认后：

```cpp
tof_ramp_detected = true;
```

该状态应锁定，不要仅因为 ToF 距离恢复就立即退出。

原因：

- 车辆驶上坡道后，传感器与坡面相对关系会改变；
- 测距可能重新接近平地基准；
- 如果自动退出，坡道控制会在上坡途中失效。

退出应由整车更高层状态机决定。

---

## 13. 视觉与 ToF 最终关系

视觉条件：

```cpp
bool visual_ramp_allow =
    left_edge_quality_ok
    && right_edge_quality_ok
    && enough_long_white_columns;
```

ToF 候选：

```cpp
bool tof_ramp_candidate =
    distance_drop_ok
    && distance_trend_ok;
```

首次疑似：

```cpp
bool ramp_candidate =
    tof_ramp_candidate
    && visual_ramp_allow;
```

连续确认后：

```cpp
tof_ramp_detected = true;
```

视觉不是坡道主判断，不需要恢复：

```cpp
ratio_abnormal
```

也不强制加入：

```cpp
ratio_increasing
midline_curve_small
horizontal_line_detected
```

除非后续实测证明当前方案误判严重，并且用户明确要求增加条件。

---

## 14. 与障碍物检测的冲突

当前视觉条件只确认“边线完整的长直道”，不能区分：

- 上坡；
- 长直道中央的低矮障碍物。

因此若工程中已有障碍物检测，优先级建议为：

```cpp
if(obstacle_detected)
{
    // 优先进入避障状态
}
else if(tof_ramp_detected)
{
    // 进入坡道状态
}
```

不要让坡道检测覆盖已有避障元素。

如果赛道规则保证坡道区域不存在低矮障碍物，可以先使用当前简化方案。

---

## 15. 坡道确认后的控制建议

确认坡道后建议由整车状态机控制：

- 适当降低目标速度，避免高速冲坡。
- 根据电机实际表现进行动力补偿。
- 限制过大的方向输出，避免坡面图像畸变导致剧烈打舵。
- 保留中线参与方向控制。
- 对短暂丢线增加容忍。
- 使用编码器、IMU、固定行驶距离或更高层赛道状态确认坡道结束。

坡道结束后调用：

```cpp
tof_ramp_reset();
```

示例：

```cpp
void tof_ramp_reset(void)
{
    tof_ramp_detected = false;
    tof_ramp_state = TOF_RAMP_FLAT;
    suspect_count = 0;
}
```

如果 `suspect_count` 是函数内部静态变量，需要重构为模块变量，确保复位函数可以清零。

---

## 16. 建议模块接口

如果工程允许新增文件，建议创建：

```text
project/code/ramp_detect.cpp
project/code/ramp_detect.h
```

### `ramp_detect.h` 建议接口

```cpp
#ifndef RAMP_DETECT_H
#define RAMP_DETECT_H

#include "zf_common_headfile.hpp"

enum tof_ramp_state_enum
{
    TOF_RAMP_CALIBRATING = 0,
    TOF_RAMP_FLAT,
    TOF_RAMP_SUSPECT,
    TOF_RAMP_CONFIRMED
};

extern volatile int16 tof_distance_raw;
extern float tof_distance_filtered;
extern float tof_flat_baseline;
extern bool tof_baseline_ready;
extern bool tof_ramp_detected;
extern tof_ramp_state_enum tof_ramp_state;

bool ramp_tof_init(void);
void ramp_tof_update(bool visual_ramp_allow);
void ramp_tof_reset(void);

#endif
```

如果工程已有统一传感器文件或元素状态机，不要为了模块化大规模重构。优先最小修改接入现有结构。

---

## 17. 推荐主循环调用方式

图像处理完成后得到：

```cpp
bool left_edge_quality_ok;
bool right_edge_quality_ok;
bool enough_long_white_columns;
```

组合：

```cpp
bool visual_ramp_allow =
    left_edge_quality_ok
    && right_edge_quality_ok
    && enough_long_white_columns;
```

调用：

```cpp
ramp_tof_update(visual_ramp_allow);
```

状态优先级示例：

```cpp
if(obstacle_detected)
{
    // 保持原避障逻辑
}
else if(tof_ramp_detected)
{
    // 设置坡道状态或坡道速度策略
}
else
{
    // 原普通巡线逻辑
}
```

---

## 18. 调试输出要求

在正式启用控制前，先只采集日志。

建议周期打印：

```cpp
printf(
    "TOF raw=%d filtered=%.2f baseline=%.2f "
    "drop=%.2f trend=%.2f visual=%d "
    "left_ok=%d right_ok=%d white_ok=%d "
    "state=%d detected=%d\r\n",
    tof_distance_raw,
    tof_distance_filtered,
    tof_flat_baseline,
    distance_drop,
    trend_drop,
    visual_ramp_allow,
    left_edge_quality_ok,
    right_edge_quality_ok,
    enough_long_white_columns,
    tof_ramp_state,
    tof_ramp_detected
);
```

不要每个主循环都打印，避免终端刷屏影响实时性。可以每 100～200 ms 打印一次。

---

## 19. 实测流程

至少采集以下场景：

### 19.1 静止平地

记录：

- 原始值范围；
- 滤波值范围；
- 标定基准；
- 异常跳点数量。

### 19.2 平地正常行驶

记录：

- 车身振动造成的波动；
- 加速、刹车时的距离变化；
- 是否会误进入疑似坡道。

### 19.3 逐渐接近坡道

记录：

- 距离从何时开始下降；
- 最大下降量；
- 趋势值；
- 视觉允许条件是否持续成立；
- 距离坡道多远时能够确认。

### 19.4 驶上坡道

记录：

- 确认后距离是否重新上升；
- 状态锁定是否正常；
- 控制是否会中途退出坡道模式。

### 19.5 长直道障碍物

如果赛道存在障碍物，记录：

- 障碍物是否造成相同距离下降；
- 避障状态是否拥有更高优先级；
- 是否发生坡道误判。

---

## 20. 阈值调试原则

初始建议：

```cpp
TOF_SAMPLE_PERIOD_MS   = 50
TOF_MEDIAN_SIZE        = 5
TOF_CALIBRATION_COUNT  = 40
TOF_DROP_MIN           = 20
TOF_DROP_RATIO         = 0.10f
TOF_TREND_DROP_MIN     = 5
TOF_RAMP_CONFIRM_COUNT = 3
LEFT_INVALID_LIMIT     = 0.15f
RIGHT_INVALID_LIMIT    = 0.15f
```

这些值不是最终标准。

调节顺序：

1. 先确认安装稳定。
2. 再确认原始数据有效。
3. 再调整滤波。
4. 再统计平地波动。
5. 再设置 `TOF_DROP_MIN`。
6. 再设置趋势阈值。
7. 最后调整连续确认次数。

不要通过无限增强滤波掩盖机械安装松动、反射材料不稳定或传感器视场错误。

---

## 21. Codex 修改约束

Codex 执行时必须遵守：

1. 先阅读当前工程中图像边线有效标志、最长白列变量、元素状态机和速度决策位置。
2. 尽量复用工程现有变量和数据，避免重复计算。
3. 不修改与坡道无关的十字、环岛、斑马线、避障逻辑。
4. 不删除原有功能。
5. 不把 `ratio_abnormal` 恢复到视觉判断中。
6. 不把视觉条件做得过严。
7. ToF 回调只采样，不执行复杂逻辑。
8. 使用新数据标志，避免主循环拿同一个 ToF 样本重复累计确认次数。
9. 坡道确认后状态锁定，由上层逻辑复位。
10. 提供清晰调试打印。
11. 修改后列出：
    - 修改文件；
    - 新增变量；
    - 新增函数；
    - 状态机接入点；
    - 需要用户实测调整的参数。
12. 如果当前工程已有同名变量或坡道逻辑，应先说明冲突，再采用最小范围修改。
13. 编译前检查类型：
    - 工程可能使用 `uint8`、`int16` 等逐飞类型；
    - 不要混用不存在的类型；
    - 注意回调线程与主线程共享变量需要 `volatile` 或安全的数据传递方式。

---

## 22. Codex 可直接执行的任务描述

请读取当前智能车工程和本文件，然后完成以下任务：

```text
在不影响现有十字、环岛、斑马线、避障和普通巡线逻辑的前提下，
基于现有 DL1X ToF 例程实现 ToF + 视觉联合坡道检测。

视觉条件仅为：
left_edge_quality_ok
&& right_edge_quality_ok
&& enough_long_white_columns

不要使用 upper_ratio 或 ratio_abnormal。

ToF 以 50 ms 周期采样，回调只读取数据并设置新数据标志。
主循环中完成有效值过滤、5 点中值滤波、一阶低通滤波、
上电平地标定、相对基准下降量计算、短时间下降趋势判断、
连续多帧确认和状态机锁定。

状态至少包括：
CALIBRATING、FLAT、SUSPECT、CONFIRMED。

首次从 FLAT 进入 SUSPECT 时要求：
距离下降明显
&& 短时间总体下降
&& 视觉允许。

SUSPECT 连续 3 个新 ToF 样本满足：
距离下降明显
&& 视觉允许
后进入 CONFIRMED。

CONFIRMED 后不要仅依赖距离恢复自动退出，
由整车更高层状态机调用 reset 函数退出。

优先复用当前工程已有的边线有效率和最长白列变量。
若没有成熟变量，再新增最小必要计算。
增加低频调试输出，打印原始距离、滤波距离、平地基准、
下降量、趋势、视觉条件、状态和确认结果。

完成后给出修改说明和可调参数列表。
```

---

## 23. 当前最终结论

当前已确定的设计是：

```cpp
visual_ramp_allow =
    left_edge_quality_ok
    && right_edge_quality_ok
    && enough_long_white_columns;
```

最终坡道确认依赖：

```text
ToF 相对平地基准明显下降
+
初次进入时存在短时间下降趋势
+
视觉允许
+
连续多个新 ToF 样本成立
```

视觉只是辅助 ToF，不承担独立识别坡道的职责。
