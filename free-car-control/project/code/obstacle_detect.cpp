#include "car_init.hpp"
#include "obstacle_detect.hpp"
#include "element.hpp"
#include "encoder.hpp"
#include "beep.hpp"

#include <cmath>
#include <cstring>

// 首轮参数按视觉避障交接说明设置，实际运行时优先根据调试页调整。

// 障碍候选搜索区域。数组下标 0 通常靠近图像下方/车辆，数值越大越靠近远方。
// 从 5 开始是为了给跳变前的平滑检查预留 i-5 到 i-2 的空间。
#define OBS_SEARCH_START_INDEX       5   // 障碍角点候选搜索的起始数组下标
#define OBS_SEARCH_END_INDEX        100  // 两个跳变点候选搜索的最大起始数组下标

// 局部跳变检测参数。
#define OBS_JUMP_ROWS               5   // 前后比较的数组间隔：比较 line[i] 与 line[i+5]
#define OBS_JUMP_THRESHOLD          10 // 最小横向跳变像素；低于此值不认为是障碍边缘
#define OBS_SECOND_DIFF_LIMIT        6  // 跳变前后局部二阶差分最大值，用于检查两段边线是否平滑
#define OBS_MIN_OBSTACLE_SEGMENT_ROWS  8  // 两个反向跳变之间至少间隔的数组行数
#define OBS_MAX_OBSTACLE_SEGMENT_ROWS 60  // 两个反向跳变之间允许的最大间隔

// 障碍前置门槛：先看赛道总宽度是否从上往下出现一次明显变窄。
// 只有这一门槛成立，才继续搜索单侧障碍的双跳变特征。
#define OBS_WIDTH_SHRINK_THRESHOLD  10  // 上方宽度减下方宽度至少 10 像素，才认为出现了变窄突变

// 进入避障前，对“障碍对侧正常边线”做整体直线拟合的质量要求。
// 拟合区间只取图像下半部分，避免远处透视和上半区局部扰动放大误判。
#define OBS_OPPOSITE_VALID_RATIO     0.90f // 对侧边线有效点比例至少 90%
#define OBS_OPPOSITE_MIN_POINTS      25    // 对侧参与拟合的有效点最少数量
#define OBS_OPPOSITE_RMSE             2.5f // 对侧边线拟合均方根误差上限（像素）
#define OBS_OPPOSITE_MAX_ERROR        5.0f // 对侧单个有效点允许的最大拟合误差（像素）

#define OBS_ENTER_CONFIRM_FRAMES      2   // 障碍候选连续成立多少帧后进入避障
#define OBSTACLE_BEEP_ENABLE          1   // 避障确认时只触发一次提示音。

// 进入避障后，车辆至少累计行驶这么多编码器计数，才允许检查退出条件。
// 该值是首轮调试值，需根据实际编码器计数和障碍物长度现场调整。
#define OBS_EXIT_MIN_ENCODER_DISTANCE 3200.0f

// 避障控制中线参数：直接使用安全侧边线向赛道内部偏移 10 像素。
// 左侧障碍使用右边线 - 10；右侧障碍使用左边线 + 10。
#define OBSTACLE_EDGE_TO_MID         10

// 退出避障时，不再做整段边线有效率/拟合判断，而是看“上跳变点”是否还存在。
// 当上跳变点在最近一次位置附近消失若干帧后，认为已经绕过障碍，可退出避障。
#define OBS_EXIT_UPPER_SEARCH_WINDOW  12  // 上跳变点存在性检查的上下搜索窗口（数组行）
#define OBS_EXIT_CONFIRM_FRAMES        3  // 上跳变点连续消失多少帧后恢复正常中线

// 串口实时打印退出关键量：只在避障状态下按固定帧率打印，方便现场看 gate/上跳变是否消失。
#define OBS_EXIT_DEBUG_SERIAL_ENABLE       1
#define OBS_EXIT_DEBUG_PRINT_EVERY_FRAMES  4

struct obstacle_line_fit_result
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

static obstacle_state_enum s_state = OBS_NONE;
static int s_left_enter_count = 0;
static int s_right_enter_count = 0;
static int s_exit_count = 0;
static float s_encoder_left_start_total = 0.0f;
static float s_encoder_right_start_total = 0.0f;
static float s_encoder_distance = 0.0f;
static bool s_exit_distance_ready = false;
static int s_control_jump_start = -1;
static int s_control_jump_end = -1;

static int s_left_corner = -1;
static int s_right_corner = -1;
static int s_left_jump = 0;
static int s_right_jump = 0;
static bool s_width_shrink_found = false;
static int s_width_shrink_row = -1;
static int s_width_shrink_value = 0;
static bool s_left_candidate = false;
static bool s_right_candidate = false;
static bool s_exit_upper_jump_found = false;
static int s_exit_upper_jump_start = -1;
static int s_exit_upper_jump_value = 0;

static obstacle_line_fit_result s_opposite_fit = {};
static obstacle_line_fit_result s_exit_fit = {};

static void obstacle_exit_debug_print(void)
{
#if OBS_EXIT_DEBUG_SERIAL_ENABLE
    static uint8 frame_count = 0U;

    if(s_state == OBS_NONE)
    {
        frame_count = 0U;
        return;
    }

    frame_count++;
    if(frame_count < OBS_EXIT_DEBUG_PRINT_EVERY_FRAMES)
        return;
    frame_count = 0U;

    const char *side = (s_state == OBS_LEFT) ? "L" : "R";
    // printf("OBS_EXIT state=%d side=%s gate=%d dist=%.0f ctrl=%d-%d "
    //        "cnt=%d up=%d uj=%d jump=%d win=%d\r\n",
    //        (int)s_state,
    //        side,
    //        s_exit_distance_ready ? 1 : 0,
    //        s_encoder_distance,
    //        s_control_jump_start,
    //        s_control_jump_end,
    //        s_exit_count,
    //        s_exit_upper_jump_found ? 1 : 0,
    //        s_exit_upper_jump_start,
    //        s_exit_upper_jump_value,
    //        OBS_EXIT_UPPER_SEARCH_WINDOW);
#endif
}

static void set_control_jump_range(int entry_start, int exit_start, int nums)
{
    if(entry_start < 0 || exit_start < 0 || nums <= 0)
        return;

    // entry_start 是下方进入拐点，exit_start 是上方离开拐点的跳变起点。
    // 控制接管范围覆盖两个真实拐点之间的完整障碍边缘段。
    s_control_jump_start = entry_start;
    s_control_jump_end = exit_start + OBS_JUMP_ROWS;
    if(s_control_jump_start < 0) s_control_jump_start = 0;
    if(s_control_jump_end >= nums) s_control_jump_end = nums - 1;

    if(s_control_jump_start > s_control_jump_end)
    {
        s_control_jump_start = -1;
        s_control_jump_end = -1;
    }
}

static void obstacle_start_encoder_gate(void)
{
    s_encoder_left_start_total = encoder_left_abs_total;
    s_encoder_right_start_total = encoder_right_abs_total;
    s_encoder_distance = 0.0f;
    s_exit_distance_ready = false;

#if OBSTACLE_BEEP_ENABLE
    // 进入左/右避障时只触发一次提示音，不在后续每帧重复触发。
    beep_request();
#endif
}

static void obstacle_update_encoder_distance(void)
{
    float left_distance = encoder_left_abs_total - s_encoder_left_start_total;
    float right_distance = encoder_right_abs_total - s_encoder_right_start_total;

    // 编码器总量如果被外部复位，当前帧不使用负增量。
    if(left_distance < 0.0f) left_distance = 0.0f;
    if(right_distance < 0.0f) right_distance = 0.0f;

    s_encoder_distance = 0.5f * (left_distance + right_distance);
    s_exit_distance_ready = s_encoder_distance >= OBS_EXIT_MIN_ENCODER_DISTANCE;
}

static inline int clamp_col_local(int x)
{
    if(x < USE_WIDTH_L) return USE_WIDTH_L;
    if(x >= USE_WIDTH_R) return USE_WIDTH_R - 1;
    return x;
}

static inline int valid_nums(void)
{
    int nums = (int)max_white.nums;
    if(nums < 0) nums = 0;
    if(nums > IMG_PROC_HEIGHT) nums = IMG_PROC_HEIGHT;
    return nums;
}

static inline int obstacle_opposite_fit_end(int nums)
{
    if(nums <= 0)
        return -1;

    int end = nums / 2;
    if(end >= nums) end = nums - 1;
    return end;
}

static inline int clamp_int_local(int v, int low, int high)
{
    if(v < low) return low;
    if(v > high) return high;
    return v;
}

static bool line_point_valid(const BoundaryInfo line[], int index, int nums)
{
    if(line == nullptr || index < 0 || index >= nums || index >= IMG_PROC_HEIGHT)
        return false;

    const int y = (int)line[index].pos[1];
    return line[index].sign != 0U && y >= 0 && y < IMG_PROC_HEIGHT;
}

static bool edge_segment_smooth(const BoundaryInfo line[], int start, int end, int nums)
{
    if(start < 0 || end + 2 >= nums || start > end)
        return false;

    for(int i = start; i <= end; ++i)
    {
        if(!line_point_valid(line, i, nums) ||
           !line_point_valid(line, i + 1, nums) ||
           !line_point_valid(line, i + 2, nums))
        {
            return false;
        }

        const int diff2 = (int)line[i + 2].pos[0]
                        - 2 * (int)line[i + 1].pos[0]
                        + (int)line[i].pos[0];
        if(std::abs(diff2) > OBS_SECOND_DIFF_LIMIT)
            return false;
    }
    return true;
}

static bool get_oriented_jump(const BoundaryInfo line[], int start, int nums,
                              bool left_side, int *jump_out)
{
    if(!line_point_valid(line, start, nums) ||
       !line_point_valid(line, start + OBS_JUMP_ROWS, nums))
        return false;

    const int x1 = (int)line[start].pos[0];
    const int x2 = (int)line[start + OBS_JUMP_ROWS].pos[0];
    // 左边线向右为进入障碍；右边线向左为进入障碍。
    const int jump = left_side ? (x2 - x1) : (x1 - x2);
    if(jump_out != nullptr) *jump_out = jump;
    return true;
}

static bool get_lane_width_if_valid(int index, int nums, int *width_out)
{
    if(width_out == nullptr)
        return false;

    if(!line_point_valid(g_image_info.left, index, nums) ||
       !line_point_valid(g_image_info.right, index, nums))
    {
        return false;
    }

    *width_out = (int)g_image_info.right[index].pos[0]
               - (int)g_image_info.left[index].pos[0];
    return true;
}

static bool find_lane_width_shrink(int nums, int *row_out, int *shrink_out)
{
    const int search_end = (nums - 1 < OBS_SEARCH_END_INDEX) ? (nums - 1) : OBS_SEARCH_END_INDEX;
    const int upper_start = search_end;
    const int upper_end = OBS_SEARCH_START_INDEX + OBS_JUMP_ROWS;
    if(upper_start < upper_end)
        return false;

    for(int upper = upper_start; upper >= upper_end; --upper)
    {
        const int lower = upper - OBS_JUMP_ROWS;
        int upper_width = 0;
        int lower_width = 0;

        // 只统计“这一行左右两边都有效”的真实赛道宽度；有任一侧无效就直接跳过。
        if(!get_lane_width_if_valid(upper, nums, &upper_width) ||
           !get_lane_width_if_valid(lower, nums, &lower_width))
        {
            continue;
        }

        const int shrink = upper_width - lower_width;
        if(shrink >= OBS_WIDTH_SHRINK_THRESHOLD)
        {
            if(row_out != nullptr) *row_out = lower;
            if(shrink_out != nullptr) *shrink_out = shrink;
            return true;
        }
    }

    return false;
}

static bool find_obstacle_pair(const BoundaryInfo line[], int nums, bool left_side,
                               int *entry_start_out, int *exit_start_out,
                               int *entry_jump_out, int *exit_jump_out)
{
    int best_entry_start = -1;
    int best_exit_start = -1;
    int best_entry_jump = 0;
    int best_exit_jump = 0;
    int best_score = 0;
    const int search_end = (nums - 11 < OBS_SEARCH_END_INDEX) ? nums - 11 : OBS_SEARCH_END_INDEX;

    for(int i = OBS_SEARCH_START_INDEX; i <= search_end; ++i)
    {
        int entry_jump = 0;
        if(!get_oriented_jump(line, i, nums, left_side, &entry_jump) ||
           entry_jump < OBS_JUMP_THRESHOLD)
            continue;

        // 进入拐点之前的正常边线必须平滑。
        if(!edge_segment_smooth(line, i - OBS_JUMP_ROWS, i - 2, nums))
            continue;

        const int exit_search_start = i + OBS_MIN_OBSTACLE_SEGMENT_ROWS;
        const int exit_search_end = (exit_search_start + OBS_MAX_OBSTACLE_SEGMENT_ROWS < search_end)
                                  ? exit_search_start + OBS_MAX_OBSTACLE_SEGMENT_ROWS
                                  : search_end;
        for(int j = exit_search_start; j <= exit_search_end; ++j)
        {
            int exit_oriented_jump = 0;
            if(!get_oriented_jump(line, j, nums, left_side, &exit_oriented_jump))
                continue;

            // 障碍上方离开拐点必须向外反向跳变。
            if(exit_oriented_jump > -OBS_JUMP_THRESHOLD)
                continue;

            const int exit_jump = -exit_oriented_jump;
            if(!edge_segment_smooth(line, i + OBS_JUMP_ROWS, j - 2, nums) ||
               !edge_segment_smooth(line, j + OBS_JUMP_ROWS, j + OBS_JUMP_ROWS + 3, nums))
                continue;

            const int score = entry_jump + exit_jump;
            if(score > best_score)
            {
                best_score = score;
                best_entry_start = i;
                best_exit_start = j;
                best_entry_jump = entry_jump;
                best_exit_jump = exit_jump;
            }
        }
    }

    if(best_entry_start < 0)
        return false;

    if(entry_start_out != nullptr) *entry_start_out = best_entry_start;
    if(exit_start_out != nullptr) *exit_start_out = best_exit_start;
    if(entry_jump_out != nullptr) *entry_jump_out = best_entry_jump;
    if(exit_jump_out != nullptr) *exit_jump_out = best_exit_jump;
    return true;
}

static bool find_upper_exit_jump_near(const BoundaryInfo line[], int nums, bool left_side,
                                      int reference_start, int *exit_start_out, int *exit_jump_out)
{
    int best_exit_start = -1;
    int best_exit_jump = 0;
    const int search_end = (nums - 1 - OBS_JUMP_ROWS < OBS_SEARCH_END_INDEX)
                         ? (nums - 1 - OBS_JUMP_ROWS)
                         : OBS_SEARCH_END_INDEX;
    if(search_end < OBS_SEARCH_START_INDEX)
        return false;

    int start = reference_start - OBS_EXIT_UPPER_SEARCH_WINDOW;
    int end = reference_start + OBS_EXIT_UPPER_SEARCH_WINDOW;
    start = clamp_int_local(start, OBS_SEARCH_START_INDEX, search_end);
    end = clamp_int_local(end, OBS_SEARCH_START_INDEX, search_end);
    if(start > end)
        return false;

    for(int j = start; j <= end; ++j)
    {
        int exit_oriented_jump = 0;
        if(!get_oriented_jump(line, j, nums, left_side, &exit_oriented_jump))
            continue;

        if(exit_oriented_jump > -OBS_JUMP_THRESHOLD)
            continue;

        const int exit_jump = -exit_oriented_jump;
        if(exit_jump > best_exit_jump)
        {
            best_exit_jump = exit_jump;
            best_exit_start = j;
        }
    }

    if(best_exit_start < 0)
        return false;

    if(exit_start_out != nullptr) *exit_start_out = best_exit_start;
    if(exit_jump_out != nullptr) *exit_jump_out = best_exit_jump;
    return true;
}

static obstacle_line_fit_result fit_edge_as_whole_line(const BoundaryInfo line[], int start, int end, int nums,
                                                       float min_valid_ratio, int min_valid_points,
                                                       float max_rmse, float max_point_error)
{
    obstacle_line_fit_result result = {};
    if(start < 0) start = 0;
    if(end >= nums) end = nums - 1;
    if(start > end) return result;

    float sum_y = 0.0f;
    float sum_x = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;

    for(int i = start; i <= end; ++i)
    {
        ++result.total_points;
        if(!line_point_valid(line, i, nums)) continue;

        const float x = (float)line[i].pos[0];
        const float y = (float)line[i].pos[1];
        sum_x += x;
        sum_y += y;
        sum_yy += y * y;
        sum_yx += y * x;
        ++result.valid_points;
    }

    if(result.total_points <= 0 || result.valid_points < min_valid_points)
        return result;

    result.valid_ratio = (float)result.valid_points / (float)result.total_points;
    if(result.valid_ratio < min_valid_ratio)
        return result;

    const float denominator = result.valid_points * sum_yy - sum_y * sum_y;
    if(std::fabs(denominator) < 1e-6f)
        return result;

    result.a = (result.valid_points * sum_yx - sum_y * sum_x) / denominator;
    result.b = (sum_x - result.a * sum_y) / (float)result.valid_points;

    float square_error_sum = 0.0f;
    for(int i = start; i <= end; ++i)
    {
        if(!line_point_valid(line, i, nums)) continue;
        const float expected_x = result.a * (float)line[i].pos[1] + result.b;
        const float error = std::fabs((float)line[i].pos[0] - expected_x);
        square_error_sum += error * error;
        if(error > result.max_error) result.max_error = error;
    }

    result.rmse = std::sqrt(square_error_sum / (float)result.valid_points);
    result.ok = result.rmse <= max_rmse && result.max_error <= max_point_error;
    return result;
}

static void reset_frame_debug(void)
{
    // 避障期间保留上一帧跳变范围，当前帧重新找到时再更新。
    // 这样短暂丢线不会让控制中线突然接管整条图像。
    if(s_state == OBS_NONE)
    {
        s_left_corner = -1;
        s_right_corner = -1;
        s_left_jump = 0;
        s_right_jump = 0;
        s_width_shrink_row = -1;
        s_width_shrink_value = 0;
        s_exit_upper_jump_start = -1;
        s_exit_upper_jump_value = 0;
    }
    s_width_shrink_found = false;
    s_left_candidate = false;
    s_right_candidate = false;
    s_exit_upper_jump_found = false;
    s_opposite_fit = {};
    s_exit_fit = {};
}

static void build_obstacle_midline(void)
{
    const int nums = valid_nums();
    if(nums <= 0) return;

    const bool left_obstacle = s_state == OBS_LEFT;
    const BoundaryInfo *normal_edge = left_obstacle ? g_image_info.filter_right : g_image_info.filter_left;
    const BoundaryInfo *raw_normal_edge = left_obstacle ? g_image_info.right : g_image_info.left;
    const int offset = OBSTACLE_EDGE_TO_MID;
    const int sign = left_obstacle ? -1 : 1;

    for(int i = 0; i < nums; ++i)
    {
        int x;
        int y = (int)g_image_info.filter_left[i].pos[1];
        const bool in_obstacle_jump_range =
            s_control_jump_start >= 0 &&
            s_control_jump_end >= s_control_jump_start &&
            i >= s_control_jump_start && i <= s_control_jump_end;

        if(in_obstacle_jump_range && line_point_valid(raw_normal_edge, i, nums))
        {
            x = (int)normal_edge[i].pos[0] + sign * offset;
            y = (int)normal_edge[i].pos[1];
        }
        else
        {
            // 两个跳变点范围之外恢复正常中线：左右滤波边线相加除以 2。
            if(g_image_info.filter_left[i].sign == 0U &&
               g_image_info.filter_right[i].sign == 0U)
            {
                x = USE_MIDDLE_VALUE;
            }
            else
            {
                x = ((int)g_image_info.filter_left[i].pos[0]
                   + (int)g_image_info.filter_right[i].pos[0]) / 2;
            }
        }

        x = clamp_col_local(x);
        if(y < 0 || y >= IMG_PROC_HEIGHT) y = i;

        g_image_info.midline[i][0] = (uint8)x;
        g_image_info.midline[i][1] = (uint8)y;
        g_image_info.filter_midline[i][0] = (uint8)x;
        g_image_info.filter_midline[i][1] = (uint8)y;
    }
}

static void update_entry_candidate(int nums)
{
    if(element_is_busy())
        return;

    s_width_shrink_found = find_lane_width_shrink(nums, &s_width_shrink_row, &s_width_shrink_value);
    if(!s_width_shrink_found)
    {
        s_left_enter_count = 0;
        s_right_enter_count = 0;
        return;
    }

    const int opposite_fit_end = obstacle_opposite_fit_end(nums);
    int left_exit_start = -1;
    int right_exit_start = -1;
    int left_exit_jump = 0;
    int right_exit_jump = 0;
    const bool left_pair_found = find_obstacle_pair(
        g_image_info.left, nums, true,
        &s_left_corner, &left_exit_start, &s_left_jump, &left_exit_jump);
    const bool right_pair_found = find_obstacle_pair(
        g_image_info.right, nums, false,
        &s_right_corner, &right_exit_start, &s_right_jump, &right_exit_jump);

    s_opposite_fit = fit_edge_as_whole_line(
        s_left_corner >= 0 ? g_image_info.right : g_image_info.left,
        0, opposite_fit_end, nums,
        OBS_OPPOSITE_VALID_RATIO, OBS_OPPOSITE_MIN_POINTS,
        OBS_OPPOSITE_RMSE, OBS_OPPOSITE_MAX_ERROR);

    const obstacle_line_fit_result left_opposite_fit = fit_edge_as_whole_line(
        g_image_info.right, 0, opposite_fit_end, nums,
        OBS_OPPOSITE_VALID_RATIO, OBS_OPPOSITE_MIN_POINTS,
        OBS_OPPOSITE_RMSE, OBS_OPPOSITE_MAX_ERROR);
    const obstacle_line_fit_result right_opposite_fit = fit_edge_as_whole_line(
        g_image_info.left, 0, opposite_fit_end, nums,
        OBS_OPPOSITE_VALID_RATIO, OBS_OPPOSITE_MIN_POINTS,
        OBS_OPPOSITE_RMSE, OBS_OPPOSITE_MAX_ERROR);

    s_left_candidate = left_pair_found && left_opposite_fit.ok;
    s_right_candidate = right_pair_found && right_opposite_fit.ok;
    if(s_left_candidate && !s_right_candidate) s_opposite_fit = left_opposite_fit;
    if(s_right_candidate && !s_left_candidate) s_opposite_fit = right_opposite_fit;

    if(s_left_candidate && s_right_candidate)
    {
        s_left_candidate = false;
        s_right_candidate = false;
    }

    if(s_left_candidate)
    {
        ++s_left_enter_count;
    }
    else
    {
        s_left_enter_count = 0;
    }

    if(s_right_candidate)
    {
        ++s_right_enter_count;
    }
    else
    {
        s_right_enter_count = 0;
    }

    if(s_left_enter_count >= OBS_ENTER_CONFIRM_FRAMES)
    {
        s_state = OBS_LEFT;
        s_left_enter_count = 0;
        s_right_enter_count = 0;
        set_control_jump_range(s_left_corner, left_exit_start, nums);
        obstacle_start_encoder_gate();
    }
    else if(s_right_enter_count >= OBS_ENTER_CONFIRM_FRAMES)
    {
        s_state = OBS_RIGHT;
        s_left_enter_count = 0;
        s_right_enter_count = 0;
        set_control_jump_range(s_right_corner, right_exit_start, nums);
        obstacle_start_encoder_gate();
    }
}

static void update_active_jump_range(int nums)
{
    int entry_start = -1;
    int exit_start = -1;
    int entry_jump = 0;
    int exit_jump = 0;

    if(s_state == OBS_LEFT)
    {
        if(find_obstacle_pair(g_image_info.left, nums, true,
                              &entry_start, &exit_start,
                              &entry_jump, &exit_jump))
        {
            s_left_corner = entry_start;
            s_left_jump = entry_jump;
            set_control_jump_range(entry_start, exit_start, nums);
        }
    }
    else if(s_state == OBS_RIGHT)
    {
        if(find_obstacle_pair(g_image_info.right, nums, false,
                              &entry_start, &exit_start,
                              &entry_jump, &exit_jump))
        {
            s_right_corner = entry_start;
            s_right_jump = entry_jump;
            set_control_jump_range(entry_start, exit_start, nums);
        }
    }
}

static void update_exit_candidate(int nums)
{
    update_active_jump_range(nums);
    obstacle_update_encoder_distance();

    // 先走过设定距离，再开始检查“上跳变点是否消失”。
    if(!s_exit_distance_ready)
    {
        s_exit_count = 0;
        return;
    }

    const bool left_obstacle = s_state == OBS_LEFT;
    const BoundaryInfo *obstacle_edge = left_obstacle ? g_image_info.left : g_image_info.right;
    const int reference_exit_start = s_control_jump_end >= 0
                                   ? (s_control_jump_end - OBS_JUMP_ROWS)
                                   : (left_obstacle ? s_left_corner : s_right_corner);

    s_exit_upper_jump_found = find_upper_exit_jump_near(
        obstacle_edge, nums, left_obstacle, reference_exit_start,
        &s_exit_upper_jump_start, &s_exit_upper_jump_value);

    if(!s_exit_upper_jump_found)
        ++s_exit_count;
    else
        s_exit_count = 0;

    if(s_exit_count >= OBS_EXIT_CONFIRM_FRAMES)
    {
        s_state = OBS_NONE;
        s_exit_count = 0;
        s_control_jump_start = -1;
        s_control_jump_end = -1;
        s_exit_upper_jump_start = -1;
        s_exit_upper_jump_value = 0;
    }
}

void obstacle_detect_init(void)
{
    obstacle_detect_reset();
}

void obstacle_detect_reset(void)
{
    s_state = OBS_NONE;
    s_left_enter_count = 0;
    s_right_enter_count = 0;
    s_exit_count = 0;
    s_encoder_left_start_total = encoder_left_abs_total;
    s_encoder_right_start_total = encoder_right_abs_total;
    s_encoder_distance = 0.0f;
    s_exit_distance_ready = false;
    s_control_jump_start = -1;
    s_control_jump_end = -1;
    reset_frame_debug();
}

bool obstacle_detect_is_active(void)
{
#if (OBSTACLE_DETECT_ENABLE == 0)
    return false;
#else
    return s_state != OBS_NONE;
#endif
}

obstacle_state_enum obstacle_detect_get_state(void)
{
    return s_state;
}

void obstacle_detect_update(void)
{
#if (OBSTACLE_DETECT_ENABLE == 0)
    // 避障关闭时不检测、不进入避障状态，也不修改控制中线。
    reset_frame_debug();
    return;
#endif

    reset_frame_debug();
    const int nums = valid_nums();
    if(nums < OBS_OPPOSITE_MIN_POINTS)
    {
        if(s_state != OBS_NONE) build_obstacle_midline();
        obstacle_exit_debug_print();
        return;
    }

    if(s_state == OBS_NONE)
        update_entry_candidate(nums);
    else
        update_exit_candidate(nums);

    if(s_state != OBS_NONE)
        build_obstacle_midline();

    obstacle_exit_debug_print();
}

void obstacle_detect_get_snapshot(obstacle_debug_snapshot_struct *snapshot)
{
    if(snapshot == nullptr) return;
    std::memset(snapshot, 0, sizeof(*snapshot));
    snapshot->state = (uint8)s_state;
    snapshot->left_candidate = s_left_candidate ? 1U : 0U;
    snapshot->right_candidate = s_right_candidate ? 1U : 0U;
    snapshot->left_jump = (uint8)CLAMP(s_left_jump, 0, 255);
    snapshot->right_jump = (uint8)CLAMP(s_right_jump, 0, 255);
    snapshot->left_corner = (int16)s_left_corner;
    snapshot->right_corner = (int16)s_right_corner;
    snapshot->left_enter_count = (uint8)CLAMP(s_left_enter_count, 0, 255);
    snapshot->right_enter_count = (uint8)CLAMP(s_right_enter_count, 0, 255);
    snapshot->exit_count = (uint8)CLAMP(s_exit_count, 0, 255);
    snapshot->jump_threshold = (uint8)CLAMP(OBS_JUMP_THRESHOLD, 0, 255);
    snapshot->width_shrink_threshold = (uint8)CLAMP(OBS_WIDTH_SHRINK_THRESHOLD, 0, 255);
    snapshot->enter_confirm_frames = (uint8)CLAMP(OBS_ENTER_CONFIRM_FRAMES, 0, 255);
    snapshot->exit_confirm_frames = (uint8)CLAMP(OBS_EXIT_CONFIRM_FRAMES, 0, 255);
    snapshot->width_shrink_found = s_width_shrink_found ? 1U : 0U;
    snapshot->width_shrink_row = (int16)s_width_shrink_row;
    snapshot->width_shrink_value = (int16)s_width_shrink_value;
    snapshot->opposite_fit_ok = s_opposite_fit.ok ? 1U : 0U;
    snapshot->opposite_valid_ratio = s_opposite_fit.valid_ratio;
    snapshot->opposite_rmse = s_opposite_fit.rmse;
    snapshot->opposite_max_error = s_opposite_fit.max_error;
    snapshot->opposite_valid_ratio_threshold = OBS_OPPOSITE_VALID_RATIO;
    snapshot->opposite_rmse_threshold = OBS_OPPOSITE_RMSE;
    snapshot->opposite_max_error_threshold = OBS_OPPOSITE_MAX_ERROR;
    snapshot->exit_fit_ok = s_exit_upper_jump_found ? 1U : 0U;
    snapshot->exit_valid_ratio = (float)s_exit_upper_jump_start;
    snapshot->exit_rmse = (float)s_exit_upper_jump_value;
    snapshot->exit_max_error = (float)OBS_EXIT_UPPER_SEARCH_WINDOW;
    snapshot->exit_encoder_distance_threshold = OBS_EXIT_MIN_ENCODER_DISTANCE;
    snapshot->encoder_distance = s_encoder_distance;
    snapshot->exit_distance_ready = s_exit_distance_ready ? 1U : 0U;
    snapshot->exit_upper_jump_found = s_exit_upper_jump_found ? 1U : 0U;
    snapshot->exit_upper_jump_start = (int16)s_exit_upper_jump_start;
    snapshot->exit_upper_jump_value = (int16)s_exit_upper_jump_value;
    snapshot->control_jump_start = (int16)s_control_jump_start;
    snapshot->control_jump_end = (int16)s_control_jump_end;
    snapshot->control_midline_mode = (uint8)s_state;
    snapshot->control_midline_offset = (int16)OBSTACLE_EDGE_TO_MID;
}
