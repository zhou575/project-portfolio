# 智能车视觉避障方案：Codex 实现说明

> 用途：将本说明交给 Codex，让其在目标智能车工程中实现视觉避障。  
> 目标：保持工程其他元素逻辑不变，只新增或替换避障相关代码。

## 1. 核心目标

实现一套基于左右赛道边线的视觉避障逻辑：

1. 正常状态下，中线仍由左右边线相加除以 2 得到。
2. 检测到左侧障碍后，进入左避障状态：
   - 不再使用左右相加除以 2；
   - 只使用正常的右边线；
   - 由“右边线减去固定宽度”生成避障控制中线。
3. 检测到右侧障碍后，进入右避障状态：
   - 不再使用左右相加除以 2；
   - 只使用正常的左边线；
   - 由“左边线加上固定宽度”生成避障控制中线。
4. 避障期间持续观察障碍物所在侧的边线。
5. 当障碍侧边线在图像下半部分整体连续，并且所有有效点能够共同拟合成同一条直线，连续若干帧成立后退出避障。

退出判断强调“整体拟合”：有障碍时，障碍侧通常由正常赛道边线段和障碍物边缘段组成，两段斜率不同，整体拟合误差较大；障碍通过后，图像下半部分恢复为一条完整赛道边线，整体拟合误差变小。

## 2. 修改范围约束

Codex 必须遵守：

- 只修改避障相关逻辑及必要的中线生成接口。
- 不修改环岛、十字、斑马线、坡道、停车、速度决策等其他元素逻辑。
- 修改后的最终控制中线必须同时用于舵机控制、屏幕显示和上位机显示/发送。
- 如果原工程的 `fit_line()` 或其他函数会在避障补线后重新覆盖 `Midline[]`，必须调整调用顺序。
- 不改变现有数组含义、图像坐标系和边线搜索结果。
- 所有新增数组访问必须做边界检查。
- 若原工程已有 `bizhang_lfind`、`bizhang_rfind`，可以兼容保留，但内部建议以单一状态机为准。

## 3. 预计已有数据

```cpp
LeftPos[i][0];     // 左边线 x
LeftPos[i][1];     // 左边线 y
RightPos[i][0];    // 右边线 x
RightPos[i][1];    // 右边线 y
LeftSign[y];       // 左边线有效标志
RightSign[y];      // 右边线有效标志
Midline[i][0];     // 中线 x
Midline[i][1];     // 中线 y
max_white.nums;    // 当前有效搜索点数
UVC_USE_WIDTH;
UVC_USE_HEIGHT;
```

通常数组下标越小越靠近图像下方、越靠近车辆。接入前必须确认目标工程是否符合这一关系。

## 4. 状态机

```cpp
enum ObstacleState
{
    OBS_NONE = 0,
    OBS_LEFT,      // 左侧有障碍，车辆从右侧绕
    OBS_RIGHT      // 右侧有障碍，车辆从左侧绕
};

static ObstacleState obstacle_state = OBS_NONE;
```

若要兼容旧变量：

```text
OBS_NONE  -> bizhang_lfind=0, bizhang_rfind=0
OBS_LEFT  -> bizhang_lfind=1, bizhang_rfind=0
OBS_RIGHT -> bizhang_lfind=0, bizhang_rfind=1
```

## 5. 推荐宏参数

```cpp
// 障碍候选搜索范围
#define OBS_SEARCH_START_INDEX          5
#define OBS_SEARCH_END_INDEX           70

// 局部跳变检测
#define OBS_JUMP_ROWS                   5
#define OBS_JUMP_THRESHOLD             15

// 跳变前后局部平滑性
#define OBS_SECOND_DIFF_LIMIT           6

// 对侧正常边线质量
#define OBS_OPPOSITE_VALID_RATIO     0.90f
#define OBS_OPPOSITE_MIN_POINTS        25
#define OBS_OPPOSITE_RMSE             2.5f
#define OBS_OPPOSITE_MAX_ERROR        5.0f

// 进入避障连续帧
#define OBS_ENTER_CONFIRM_FRAMES        2

// 避障中线
#define NORMAL_HALF_LANE_WIDTH         32
#define OBSTACLE_SHIFT                 12
#define OBSTACLE_EDGE_TO_MID \
        (NORMAL_HALF_LANE_WIDTH - OBSTACLE_SHIFT)

// 退出判断范围
#define OBS_EXIT_CHECK_START_INDEX      0
#define OBS_EXIT_CHECK_END_INDEX       35

// 退出时障碍侧整体直线质量
#define OBS_EXIT_VALID_RATIO         0.90f
#define OBS_EXIT_MIN_POINTS            20
#define OBS_EXIT_RMSE                 2.5f
#define OBS_EXIT_MAX_ERROR            5.0f
#define OBS_EXIT_CONFIRM_FRAMES         3
```

`OBSTACLE_EDGE_TO_MID` 是正常侧边线到避障控制中线的距离，不是完整赛道宽度。

例如半赛道宽 32、希望避障中线横移 12，则：

```text
OBSTACLE_EDGE_TO_MID = 32 - 12 = 20
```

左障碍时：

```cpp
Midline = RightEdge - 20;
```

右障碍时：

```cpp
Midline = LeftEdge + 20;
```

## 6. 点有效性函数

```cpp
static inline bool obstacle_line_point_valid(
    int16 line[][2],
    const uint8 *line_sign,
    int index,
    int nums)
{
    if(index < 0 || index >= nums)
        return false;

    int y = line[index][1];

    if(y < 0 || y >= UVC_USE_HEIGHT)
        return false;

    return line_sign[y] != 0;
}
```

请按目标工程实际类型调整 `int16`、`uint8`。

## 7. 障碍侧局部平滑判断

采用二阶差分：

```cpp
diff2 = line[i + 2][0]
      - 2 * line[i + 1][0]
      + line[i][0];
```

参考函数：

```cpp
static bool obstacle_edge_segment_smooth(
    int16 line[][2],
    const uint8 *line_sign,
    int start,
    int end,
    int nums,
    int diff_limit)
{
    if(start < 0 || end + 2 >= nums || start > end)
        return false;

    for(int i = start; i <= end; i++)
    {
        if(!obstacle_line_point_valid(line, line_sign, i, nums) ||
           !obstacle_line_point_valid(line, line_sign, i + 1, nums) ||
           !obstacle_line_point_valid(line, line_sign, i + 2, nums))
        {
            return false;
        }

        int diff2 = line[i + 2][0]
                  - 2 * line[i + 1][0]
                  + line[i][0];

        if(abs(diff2) > diff_limit)
            return false;
    }

    return true;
}
```

障碍角点本身不参与平滑判断。推荐检查：

```text
跳变下方：i-5 到 i-2
跳变上方：i+5 到 i+8
```

## 8. 左障碍候选检测

左障碍表现为左边线向赛道内部，也就是向右明显跳变：

```cpp
jump = LeftPos[i + OBS_JUMP_ROWS][0]
     - LeftPos[i][0];
```

要求：

```cpp
jump >= OBS_JUMP_THRESHOLD
```

在搜索区中选择跳变量最大的合法候选，不要找到第一个就 `break`。

```cpp
static int find_left_obstacle_corner(int nums, int *best_jump_out)
{
    int best_corner = -1;
    int best_jump = 0;

    int search_end = nums - 11;
    if(search_end > OBS_SEARCH_END_INDEX)
        search_end = OBS_SEARCH_END_INDEX;

    if(search_end < OBS_SEARCH_START_INDEX)
    {
        if(best_jump_out) *best_jump_out = 0;
        return -1;
    }

    for(int i = OBS_SEARCH_START_INDEX; i <= search_end; i++)
    {
        bool points_valid = true;

        for(int k = 0; k <= OBS_JUMP_ROWS; k++)
        {
            if(!obstacle_line_point_valid(
                    LeftPos, LeftSign, i + k, nums))
            {
                points_valid = false;
                break;
            }
        }

        if(!points_valid)
            continue;

        int jump = LeftPos[i + OBS_JUMP_ROWS][0]
                 - LeftPos[i][0];

        if(jump < OBS_JUMP_THRESHOLD)
            continue;

        bool lower_smooth = obstacle_edge_segment_smooth(
            LeftPos, LeftSign,
            i - 5, i - 2,
            nums, OBS_SECOND_DIFF_LIMIT);

        bool upper_smooth = obstacle_edge_segment_smooth(
            LeftPos, LeftSign,
            i + 5, i + 8,
            nums, OBS_SECOND_DIFF_LIMIT);

        if(!lower_smooth || !upper_smooth)
            continue;

        if(jump > best_jump)
        {
            best_jump = jump;
            best_corner = i + OBS_JUMP_ROWS;
        }
    }

    if(best_jump_out)
        *best_jump_out = best_jump;

    return best_corner;
}
```

## 9. 右障碍候选检测

右障碍表现为右边线向左明显跳变：

```cpp
jump = RightPos[i][0]
     - RightPos[i + OBS_JUMP_ROWS][0];
```

参考：

```cpp
static int find_right_obstacle_corner(int nums, int *best_jump_out)
{
    int best_corner = -1;
    int best_jump = 0;

    int search_end = nums - 11;
    if(search_end > OBS_SEARCH_END_INDEX)
        search_end = OBS_SEARCH_END_INDEX;

    if(search_end < OBS_SEARCH_START_INDEX)
    {
        if(best_jump_out) *best_jump_out = 0;
        return -1;
    }

    for(int i = OBS_SEARCH_START_INDEX; i <= search_end; i++)
    {
        bool points_valid = true;

        for(int k = 0; k <= OBS_JUMP_ROWS; k++)
        {
            if(!obstacle_line_point_valid(
                    RightPos, RightSign, i + k, nums))
            {
                points_valid = false;
                break;
            }
        }

        if(!points_valid)
            continue;

        int jump = RightPos[i][0]
                 - RightPos[i + OBS_JUMP_ROWS][0];

        if(jump < OBS_JUMP_THRESHOLD)
            continue;

        bool lower_smooth = obstacle_edge_segment_smooth(
            RightPos, RightSign,
            i - 5, i - 2,
            nums, OBS_SECOND_DIFF_LIMIT);

        bool upper_smooth = obstacle_edge_segment_smooth(
            RightPos, RightSign,
            i + 5, i + 8,
            nums, OBS_SECOND_DIFF_LIMIT);

        if(!lower_smooth || !upper_smooth)
            continue;

        if(jump > best_jump)
        {
            best_jump = jump;
            best_corner = i + OBS_JUMP_ROWS;
        }
    }

    if(best_jump_out)
        *best_jump_out = best_jump;

    return best_corner;
}
```

左右必须分别判断方向，不要简单对差值取绝对值。

## 10. 整体直线拟合

统一拟合：

```text
x = a*y + b
```

不要拟合 `y = a*x + b`，因为赛道边线可能接近竖直。

```cpp
struct ObstacleLineFitResult
{
    bool ok;
    int total_points;
    int valid_points;
    float valid_ratio;
    float a;
    float b;
    float rmse;
    float max_error;
};
```

参考实现：

```cpp
static ObstacleLineFitResult fit_edge_as_whole_line(
    int16 line[][2],
    const uint8 *line_sign,
    int start,
    int end,
    int nums,
    float min_valid_ratio,
    int min_valid_points,
    float max_rmse,
    float max_point_error)
{
    ObstacleLineFitResult result = {};

    if(start < 0) start = 0;
    if(end >= nums) end = nums - 1;
    if(start > end) return result;

    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;

    for(int i = start; i <= end; i++)
    {
        result.total_points++;

        if(!obstacle_line_point_valid(line, line_sign, i, nums))
            continue;

        float x = (float)line[i][0];
        float y = (float)line[i][1];

        sum_x += x;
        sum_y += y;
        sum_yy += y * y;
        sum_yx += y * x;
        result.valid_points++;
    }

    if(result.total_points <= 0 ||
       result.valid_points < min_valid_points)
        return result;

    result.valid_ratio =
        (float)result.valid_points /
        (float)result.total_points;

    if(result.valid_ratio < min_valid_ratio)
        return result;

    float denominator =
        result.valid_points * sum_yy
        - sum_y * sum_y;

    if(fabs(denominator) < 1e-6f)
        return result;

    result.a =
        (result.valid_points * sum_yx - sum_y * sum_x)
        / denominator;

    result.b =
        (sum_x - result.a * sum_y)
        / result.valid_points;

    float square_error_sum = 0.0f;
    int error_count = 0;

    for(int i = start; i <= end; i++)
    {
        if(!obstacle_line_point_valid(line, line_sign, i, nums))
            continue;

        float expected_x = result.a * line[i][1] + result.b;
        float error = fabs(line[i][0] - expected_x);

        square_error_sum += error * error;
        if(error > result.max_error)
            result.max_error = error;

        error_count++;
    }

    if(error_count <= 0)
        return result;

    result.rmse = sqrt(square_error_sum / error_count);

    result.ok =
        result.rmse <= max_rmse &&
        result.max_error <= max_point_error;

    return result;
}
```

## 11. 对侧正常边线确认

左侧疑似障碍时，右边线必须整体连续且为一条直线。

右侧疑似障碍时，左边线必须整体连续且为一条直线。

```cpp
left_candidate =
    left_corner >= 0 &&
    right_fit.ok;

right_candidate =
    right_corner >= 0 &&
    left_fit.ok;
```

这用于排除弯道、环岛、十字和整体搜线错误。

## 12. 左右同时出现候选

如果：

```cpp
left_candidate && right_candidate
```

本帧不进入避障：

```cpp
if(left_candidate && right_candidate)
{
    left_candidate = false;
    right_candidate = false;
}
```

不要设置左侧优先或右侧优先。

## 13. 连续两帧确认进入

```cpp
static int left_enter_count = 0;
static int right_enter_count = 0;
```

```cpp
if(left_candidate && !right_candidate)
    left_enter_count++;
else
    left_enter_count = 0;

if(right_candidate && !left_candidate)
    right_enter_count++;
else
    right_enter_count = 0;
```

```cpp
if(left_enter_count >= OBS_ENTER_CONFIRM_FRAMES)
{
    obstacle_state = OBS_LEFT;
    left_enter_count = 0;
    right_enter_count = 0;
}
else if(right_enter_count >= OBS_ENTER_CONFIRM_FRAMES)
{
    obstacle_state = OBS_RIGHT;
    left_enter_count = 0;
    right_enter_count = 0;
}
```

只在 `OBS_NONE` 状态执行进入检测。并且只在当前未处于环岛、十字等冲突元素时允许进入避障。

## 14. 中线生成

正常状态保持原逻辑：

```cpp
Midline[i][0] =
    (LeftPos[i][0] + RightPos[i][0]) / 2;
```

左避障，只相信右边线：

```cpp
Midline[i][0] =
    RightPos[i][0] - OBSTACLE_EDGE_TO_MID;
```

右避障，只相信左边线：

```cpp
Midline[i][0] =
    LeftPos[i][0] + OBSTACLE_EDGE_TO_MID;
```

进入避障后，不要再额外修改 PID 目标点。避障路径只通过最终 `Midline[]` 体现。

正常侧偶尔丢一行时，可继承上一行中线；所有 x 坐标要限制在图像范围内。

## 15. 退出避障判断

左避障期间：

- 检查左边线；
- 取图像下半部分对应的完整检查区；
- 将所有有效点一起拟合为 `x=a*y+b`；
- 有效比例、有效点数、RMSE、最大误差均满足；
- 连续 3 帧成立后退出。

右避障完全镜像。

不需要额外判断边线恢复到固定位置。关键是“图像下半部分障碍侧的所有有效点整体能否由同一条直线描述”。

```cpp
static int obstacle_exit_count = 0;
```

```cpp
static bool obstacle_side_lower_half_is_whole_line(int nums)
{
    int check_end = OBS_EXIT_CHECK_END_INDEX;
    if(check_end >= nums)
        check_end = nums - 1;

    if(check_end < OBS_EXIT_CHECK_START_INDEX)
        return false;

    ObstacleLineFitResult result = {};

    if(obstacle_state == OBS_LEFT)
    {
        result = fit_edge_as_whole_line(
            LeftPos, LeftSign,
            OBS_EXIT_CHECK_START_INDEX,
            check_end,
            nums,
            OBS_EXIT_VALID_RATIO,
            OBS_EXIT_MIN_POINTS,
            OBS_EXIT_RMSE,
            OBS_EXIT_MAX_ERROR);
    }
    else if(obstacle_state == OBS_RIGHT)
    {
        result = fit_edge_as_whole_line(
            RightPos, RightSign,
            OBS_EXIT_CHECK_START_INDEX,
            check_end,
            nums,
            OBS_EXIT_VALID_RATIO,
            OBS_EXIT_MIN_POINTS,
            OBS_EXIT_RMSE,
            OBS_EXIT_MAX_ERROR);
    }
    else
    {
        return false;
    }

    return result.ok;
}
```

```cpp
static void obstacle_update_exit(int nums)
{
    if(obstacle_state == OBS_NONE)
    {
        obstacle_exit_count = 0;
        return;
    }

    bool exit_candidate =
        obstacle_side_lower_half_is_whole_line(nums);

    if(exit_candidate)
        obstacle_exit_count++;
    else
        obstacle_exit_count = 0;

    if(obstacle_exit_count >= OBS_EXIT_CONFIRM_FRAMES)
    {
        obstacle_state = OBS_NONE;
        obstacle_exit_count = 0;
    }
}
```

检查范围不能太短。必须足以同时覆盖正常赛道边线段和障碍物边缘段，否则可能只看到其中一段而提前退出。首轮使用数组索引 `0～35`。

## 16. 推荐每帧调用顺序

```cpp
void image_pro(void)
{
    red_gray();

    seek_start_Edge();
    find_max_white(...);
    seek_lr_edges(...);

    int nums = max_white.nums;

    if(obstacle_state == OBS_NONE)
        obstacle_detect_enter(nums);

    if(obstacle_state != OBS_NONE)
        obstacle_update_exit(nums);

    if(obstacle_state == OBS_NONE)
        build_normal_midline(nums);
    else if(obstacle_state == OBS_LEFT)
        build_left_obstacle_midline(nums);
    else if(obstacle_state == OBS_RIGHT)
        build_right_obstacle_midline(nums);

    // 后续中线平滑、舵机控制、屏幕和上位机均使用最终 Midline[]
}
```

如果某个原函数内部会重新执行 `(LeftPos+RightPos)/2`，必须拆开或调整顺序，避免覆盖避障中线。

## 17. 调试输出

第一阶段先低速或静态调试。输出：

```text
left_corner
right_corner
left_jump
right_jump
left_candidate
right_candidate
left_enter_count
right_enter_count
obstacle_state
opposite_valid_ratio
opposite_rmse
opposite_max_error
exit_valid_ratio
exit_rmse
exit_max_error
obstacle_exit_count
```

建议终端格式：

```cpp
printf(
    "[OBS] state=%d "
    "Lc=%d Lj=%d Rc=%d Rj=%d "
    "Lcan=%d Rcan=%d "
    "Lcnt=%d Rcnt=%d exit=%d\n",
    obstacle_state,
    left_corner,
    left_jump,
    right_corner,
    right_jump,
    left_candidate,
    right_candidate,
    left_enter_count,
    right_enter_count,
    obstacle_exit_count
);
```

屏幕和上位机还应显示检测角点、状态和最终控制中线。

## 18. 首轮调参顺序

1. 固定车辆，逐渐移动左右障碍物。
2. 观察 `left_jump`、`right_jump`。
3. 调 `OBS_JUMP_THRESHOLD`：
   - 识别太晚：从 15 降到 12～14；
   - 正常直道误判：提高到 17～20。
4. 检查对侧边线拟合值。
5. 确认连续两帧后正确进入左右状态。
6. 开启低速控制。
7. 调 `OBSTACLE_SHIFT`：
   - 离障碍太近则增加；
   - 离另一侧边线太近则减小。
8. 最后调退出检查范围、RMSE 和最大误差。

不要一次同时修改全部参数。

## 19. 验收标准

### 正常直道

- 状态保持 `OBS_NONE`。
- 中线仍为左右相加除以 2。
- 不频繁误触发。

### 左障碍逐渐靠近

- 左边线出现方向正确的局部跳变。
- 右边线整体正常。
- 连续两帧后进入 `OBS_LEFT`。
- 中线只由右边线减固定宽度生成。
- 屏幕中线与控制中线一致。
- 障碍仍在图像下半部分时不得提前退出。
- 左边线下半部分整体恢复为一条直线并连续三帧后退出。

### 右障碍逐渐靠近

完全镜像。

### 弯道

- 另一侧边线不再满足整体直线条件。
- 不进入避障。

### 左右同时异常

- 不随便选择某一侧。
- 保持当前状态或交给其他元素逻辑。

### 安全性

- 所有数组访问不得越界。
- `nums` 不足时放弃本帧判断。
- 拟合分母接近零时返回失败。
- 有效点不足时不能判为直线。
- 不允许 NaN 继续参与控制。

## 20. Codex 交付要求

完成后请输出：

1. 修改了哪些文件。
2. 每个文件修改了什么。
3. 新增宏及默认值。
4. 新增状态变量。
5. 进入避障的完整条件。
6. 左右避障中线公式。
7. 退出避障的完整条件。
8. 调用顺序是否会覆盖最终中线。
9. 编译结果。
10. 建议优先观察的调试变量。

如果无法确认目标工程中某个变量的含义，应先搜索引用和调用链，不要凭变量名猜测，也不要修改无关逻辑。

## 21. 最终逻辑摘要

```text
正常状态
  ↓
一侧边线出现方向正确的局部大跳变
  ↓
跳变前后两段分别平滑
  ↓
另一侧边线连续且整体为一条直线
  ↓
左右候选不能同时成立
  ↓
连续两帧确认
  ↓
进入左避障或右避障
  ↓
不再使用左右相加除以 2
  ↓
使用正常侧边线 ± 固定宽度重建控制中线
  ↓
持续整体拟合障碍侧图像下半部分边线
  ↓
障碍侧所有有效点整体可以拟合为唯一一条直线
  ↓
连续三帧成立
  ↓
退出避障
  ↓
恢复正常双边中线
```
