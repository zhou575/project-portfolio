
#include "element.hpp"
#include "image.hpp"

#include <math.h>
#include <string.h>

#include "encoder.hpp"
#include "imu.hpp"

#include "beep.hpp"

// ================================================================================================
// 参数区
// ================================================================================================
#define ELEMENT_MIN_VALID_ROWS                (12U)
#define ELEMENT_BASE_WIDTH_MIN                (20)
#define ELEMENT_BASE_WIDTH_MAX                (140)
#define ELEMENT_ENABLE_ROUNDABOUT             (1)
#define ELEMENT_ENABLE_CROSSROAD              (1)

#define ELEMENT_JUMP_LIMIT                    (20)
#define ELEMENT_MID_JUMP_LIMIT                (18)
#define ELEMENT_MIN_LANE_WIDTH                (24)
#define ELEMENT_MAX_LANE_WIDTH                (150)

#define OLD_CIRCLE_DIST1                      (24000.0f)   //19500
#define OLD_CIRCLE_DIST2                      (18000.0f)
#define OLD_CIRCLE_SEARCH_STOP_MIN           (60)
#define OLD_CIRCLE_SEARCH_STOP_EARLY_MIN     (55)
#define OLD_RC_EARLY_DY_MIN                  (24)
#define OLD_RC_EARLY_RIGHT_LOST_MIN          (13)
#define OLD_CIRCLE_LEFT_LOST_MIN             (20)
#define OLD_CIRCLE_RIGHT_LOST_MIN            (8)
#define OLD_CIRCLE_OTHER_LOST_MAX            (25)

// 右环入口：右上/右下角点判据
#define OLD_RC_DY_MIN                        (10)
#define OLD_RC_DY_MAX                        (80)
#define OLD_RC_X_GAP_MIN                     (5)

// 右环入口斜率判据
#define OLD_RC_LEFT_SLOPE_THR                (-3.4f)
#define OLD_RC_LEFT_SLOPE_DELTA              (2.9f)
#define OLD_RC_RIGHT_SLOPE_THR               (0.7f)
#define OLD_RC_RIGHT_SLOPE_DELTA             (2.3f)

// 右环入口右边线预筛：右侧在入环早期经常丢线，不能要求 5 个固定采样行全部有效。
// 这里只要求当前可见右边界中存在若干段满足右下斜出趋势的线段。
#define OLD_RC_RIGHT_PRE_SLOPE_THR           (0.45f)
#define OLD_RC_RIGHT_PRE_SLOPE_DELTA         (4.55f)
#define OLD_RC_RIGHT_PRE_DY_MIN              (8)
#define OLD_RC_RIGHT_PRE_DY_MAX              (42)
#define OLD_RC_RIGHT_PRE_HIT_MIN             (2)

// 右环入口左边稳定边：也改成可见线段斜率判据，避免固定 5 行采样过严。
#define OLD_RC_LEFT_VISIBLE_DY_MIN           (8)
#define OLD_RC_LEFT_VISIBLE_DY_MAX           (42)
#define OLD_RC_LEFT_VISIBLE_HIT_MIN          (2)

// 左环入口：左上/左下角点判据
#define OLD_LC_DY_MIN                        (10)
#define OLD_LC_DY_MAX                        (80)
#define OLD_LC_X_GAP_MIN                     OLD_RC_X_GAP_MIN

// 左环上角点仍使用旧的短距离突变判据；右环上角点改为“短竖线段下端 + 下方右下斜出”判据。
#define OLD_CIRCLE_UP_TIP_JUMP_MIN           (15)
#define OLD_CIRCLE_UP_TIP_STABLE_MAX         (8)
#define OLD_CIRCLE_UP_TIP_SEARCH_MARGIN      (4)

// 右环 RU：识别右上方短红线段，RU 取短竖线段的最下端。
#define OLD_RU_VERT_X_MAX                    (5)
#define OLD_RU_VERT_LEN_MIN                  (2)
#define OLD_RU_VERT_LEN_MAX                  (20)

// 右环 RU 简化判据：上方两行近似竖直，下方两行向右偏移
#define OLD_RU_SIMPLE_X_DIFF_MAX             (5)
#define OLD_RU_SIMPLE_RIGHT_SHIFT_MIN        (5)


// RU 搜索允许向 old_search_end 上方多扫一段，避免刚露出的短红线被跳过
#define OLD_RU_SCAN_UP_MARGIN                (16)

// 短竖线下方寻找斜边，不再只看固定 +4 行
#define OLD_RU_SLANT_SEARCH_DY_MIN           (3)
#define OLD_RU_SLANT_SEARCH_DY_MAX           (18)
#define OLD_RU_SLANT_HIT_MIN                 (1)

#define OLD_RU_TO_SLANT_RIGHT_MIN            (4)
#define OLD_RU_TO_SLANT_RIGHT_MAX            (45)
// 右环 RD：RD 必须绑定在 RU 的右下方，防止下角点被其他右边界吸走。
#define OLD_RD_FROM_RU_DY_MIN                (14)
#define OLD_RD_FROM_RU_DY_MAX                (80)
#define OLD_RD_FROM_RU_X_MIN                 (5)
#define OLD_RD_FROM_RU_X_MAX                 (70)
#define OLD_RD_STABLE_N                      (3)
#define OLD_RD_STABLE_X_MAX                  (8)

// PREP 阶段角点缓存保护：当前帧角点抖动过大时不刷新，用上一帧缓存继续补线。
#define OLD_CORNER_X_REFRESH_MAX             (6)
#define OLD_CIRCLE_DOWN_HOLD_MAX             (5U)
#define OLD_CIRCLE_UP_HOLD_MAX               (8U)

// 环岛所有补线端点统一防跳变：候选端点横纵都大跳时沿用上一帧；可信变化按每帧限速平滑。
#define OLD_CIRCLE_REPAIR_REJECT_X           (18)
#define OLD_CIRCLE_REPAIR_REJECT_Y           (14)
#define OLD_CIRCLE_REPAIR_STEP_X             (4)
#define OLD_CIRCLE_REPAIR_STEP_Y             (3)

#define CIRCLE_REPAIR_SLOT_LEFT_PREP         (0U)
#define CIRCLE_REPAIR_SLOT_LEFT_ENTER        (1U)
#define CIRCLE_REPAIR_SLOT_LEFT_INSIDE       (2U)
#define CIRCLE_REPAIR_SLOT_RIGHT_PREP        (3U)
#define CIRCLE_REPAIR_SLOT_RIGHT_ENTER       (4U)
#define CIRCLE_REPAIR_SLOT_RIGHT_INSIDE      (5U)
#define CIRCLE_REPAIR_SLOT_EXIT_PREP         (6U)
#define CIRCLE_REPAIR_LINE_SLOT_NUM          (7U)

// 左环入口斜率判据
// 左环时：右边线通常是稳定边，右边线斜率为正
#define OLD_LC_RIGHT_SLOPE_THR               (0.7f)
#define OLD_LC_RIGHT_SLOPE_DELTA             (2.3f)

// 左环时：左下段边线是丢边侧的下方有效段，镜像右环，斜率为负
#define OLD_LC_LEFT_SLOPE_THR                (-3.0f)
#define OLD_LC_LEFT_SLOPE_DELTA              (2.3f)

// 下角点保护：防止右上/左上短线段被误判成下角点
#define OLD_RC_DOWN_Y_MIN                    (45)
#define OLD_LC_DOWN_Y_MIN                    (45)

#define OLD_CIRCLE_DOWN_JUMP_MAX             (18)
#define OLD_CIRCLE_DOWN_CONTINUE_N           (5)
#define OLD_CIRCLE_DOWN_FIT_MAX              (14)
#define OLD_CIRCLE_DOWN_SMOOTH_MAX           (8)
#define OLD_CIRCLE_UP_FAR_MIN                (16)

// 边界直线拟合：连续有效率放宽到 60%，拟合点按固定行距抽样。
#define OLD_LINE_FIT_SAMPLE_STEP             (4)
#define OLD_LINE_FIT_MIN_VALID_RATIO         (0.6f)
#define OLD_LINE_FIT_MIN_SAMPLES             (4)


// 右环 PREP 补线保护：RU 必须是上方短红线点，不能落到斜边中段
#define OLD_RU_PREP_MAX_OFFSET_FROM_SEARCH_END   (28)
#define OLD_RU_PREP_DY_MIN                       (38)
#define OLD_RU_PREP_DY_MAX                       (110)
#define OLD_RU_PREP_X_GAP_MIN                    (6)

// 右环 PREP 固定补线端点：
// 上端点 = 之前蓝色圆圈标出的尖端点
// 下端点 = 现在蓝色点标出的固定位置
#define OLD_RC_PREP_FIXED_UP_X                   (90)
#define OLD_RC_PREP_FIXED_UP_Y                   (5)
#define OLD_RC_PREP_FIXED_DOWN_X                 (120)
#define OLD_RC_PREP_FIXED_DOWN_Y                 (76)

// 左环 PREP 固定补线端点：按右环固定点做水平镜像。
#define OLD_LC_PREP_FIXED_UP_X                   ((USE_WIDTH_R - 1) - OLD_RC_PREP_FIXED_UP_X)
#define OLD_LC_PREP_FIXED_UP_Y                   OLD_RC_PREP_FIXED_UP_Y
#define OLD_LC_PREP_FIXED_DOWN_X                 ((USE_WIDTH_R - 1) - OLD_RC_PREP_FIXED_DOWN_X)
#define OLD_LC_PREP_FIXED_DOWN_Y                 OLD_RC_PREP_FIXED_DOWN_Y

// 左环提前入态阈值：镜像右环右丢线提前入态。
#define OLD_LC_EARLY_LEFT_LOST_MIN               OLD_RC_EARLY_RIGHT_LOST_MIN

// 右环 EXIT_PREP 阶段真实黑线上端点搜索放宽参数：
// 允许 y 周围 4 行中有少量丢点，只要候选点附近仍形成一段较平滑的右边界即可。
#define OLD_EXIT_PREP_EDGE_NEAR1_MAX             (8)
#define OLD_EXIT_PREP_EDGE_NEAR2_MAX             (10)
#define OLD_EXIT_PREP_EDGE_NEAR_HIT_MIN          (3)
#define OLD_EXIT_PREP_EDGE_REFRESH_X_MAX         (18)
#define OLD_EXIT_PREP_BLACK_EDGE_DX_TO_BOTTOM   (34)

// OLD_RC_DOWN_Y_MIN / OLD_LC_DOWN_Y_MIN：
// 下角点不能跑到太上面。你现在假下角点经常在 20~30 行，所以先设 45。

// OLD_CIRCLE_DOWN_JUMP_MAX：
// 下角点不能相对上一帧突然向上跳很多。

// OLD_CIRCLE_DOWN_CONTINUE_N：
// 下角点下方必须连续 5 行有效，避免短碎边误判。

// OLD_CIRCLE_DOWN_FIT_MAX：
// 当前点和下方 5 行的最大横向差不能太大。

// OLD_CIRCLE_DOWN_SMOOTH_MAX：
// 下方连续边界本身要平滑。

// OLD_CIRCLE_UP_FAR_MIN：
// 下角点上方如果不是丢线，也必须有明显跳变。

// ================================================================================================
// 十字参数区
// ================================================================================================
// 初检测：连续 3 帧满足“同时丢线 > 30”或“左右各自丢线都 > 20”。
#define CROSS_ENTRY_BOTH_LOST_MIN             (30)
#define CROSS_ENTRY_SIDE_LOST_MIN             (20)
#define CROSS_ENTRY_CONFIRM_FRAMES            (3U)
#define CROSS_ENTRY_SKIP_BOTTOM_ROWS          (30)
#define CROSS_BEEP_ENABLE                     (0)

// 角点阶段：两侧下角点和至少一侧上角点齐全则开始补线；角点不足退出暂时关闭。
#define CROSS_CORNER_MIN_FOR_KEEP             (2U)
#define CROSS_CORNER_FAIL_FRAMES              (3U)
#define CROSS_CORNER_FAIL_EXIT_ENABLE         (0)

// 角点检测：进入十字时用严格阈值；补线保持期间用稍松阈值。
#define CROSS_STRICT_UP_JUMP_MIN              (18)
#define CROSS_LOOSE_UP_JUMP_MIN               (10)
// 先找下角点：有下角点时从其上方 10 行开始找上角点；
// 下角点消失时从图像最下方可安全计算 i-4 的位置开始。
#define CROSS_UP_FROM_LOWER_GAP               (10)
#define CROSS_UP_SEARCH_BOTTOM_IDX            (4)
// 本帧下角点消失时，上角点只从底部搜到边界索引 60。
#define CROSS_UP_NO_LOWER_SEARCH_END_IDX      (60)
// 上角点起向上共 7 个点（包含上角点）必须连续且近似直/斜线；
// 每点最大允许 2 像素拟合误差。
#define CROSS_UP_ABOVE_LINE_POINTS            (7)
#define CROSS_UP_ABOVE_LINE_ERR_MAX           (2)
// 下角点只允许出现在处理图像 y=50～最下方的范围内（包含端点）。
#define CROSS_DOWN_SEARCH_Y_MIN               (50)
#define CROSS_DOWN_SEARCH_Y_MAX               (IMG_PROC_HEIGHT - 1)

// 十字退出：检查补线前原始边界在图像 y=80～100 的恢复情况。
// 至少覆盖 15 个检查行，任意一侧有效率达到 75% 即认为下部边线基本恢复。
#define CROSS_EXIT_BOTTOM_Y_MIN               (80)
#define CROSS_EXIT_BOTTOM_Y_MAX               (100)
#define CROSS_EXIT_BOTTOM_MIN_CHECKED_ROWS    (15)
#define CROSS_EXIT_BOTTOM_VALID_PERCENT       (75)

// 缺失上角点时，用下角点下方 10 个扫描行的同侧边界点拟合斜率。
#define CROSS_LOWER_SLOPE_FIT_ROWS            (10)
#define CROSS_LOWER_SLOPE_K_ABS_MAX           (7.0f)
#define CROSS_UPPER_SLOPE_ABOVE_ROWS          (5)

// ================================================================================================
// 状态区
// ================================================================================================
static element_mode_enum  s_element_mode      = ELEMENT_NONE;
static cross_state_enum   s_cross_state       = CROSS_IDLE;
static circle_state_enum  s_circle_state      = CIRCLE_IDLE;

static bool   s_element_busy                  = false;
static bool   s_circle_flag                   = false;
static bool   s_in_circle                     = false;

static uint8  s_cross_candidate               = 0;
static uint8  s_circle_candidate              = 0;
static uint8  s_cross_confirm_frames          = 0;
static uint8  s_circle_confirm_frames         = 0;
static uint8  s_cross_line_index              = 0;
static uint8  s_cross_lost_frames             = 0U;
static uint8  s_cross_exit_frames             = 0U;
static uint8  s_cross_bottom_recover_frames   = 0U;
// 本帧补线前锁存的原始丢线统计；十字判断和调试显示都只读这三个值。
static int16  s_cross_raw_left_lost_count     = 0;
static int16  s_cross_raw_right_lost_count    = 0;
static int16  s_cross_raw_both_lost_count     = 0;
static int16  s_cross_bottom_left_valid       = 0;
static int16  s_cross_bottom_right_valid      = 0;
static int16  s_cross_bottom_checked_rows     = 0;
static uint8  s_circle_lost_side              = 0;   // 1-左丢边 2-右丢边

// 十字角点：只保存当前帧补线前检测结果，不跨帧沿用。
typedef struct
{
    uint8 valid;
    int x;
    int y;
    int idx;
} cross_corner_current_struct;

typedef struct
{
    uint8 found;
    int x;
    int y;
    int idx;
} cross_corner_detect_struct;

// 十字补线模型：仅保存当前帧实际生成的完整线段，供本帧绘制和调试。
typedef struct
{
    uint8 valid;
    float slope;
    int x1;
    int y1;
    int x2;
    int y2;
} cross_patch_model_struct;

static cross_corner_current_struct s_cross_lu = {0U, 0, 0, 0};
static cross_corner_current_struct s_cross_ld = {0U, 0, 0, 0};
static cross_corner_current_struct s_cross_ru = {0U, 0, 0, 0};
static cross_corner_current_struct s_cross_rd = {0U, 0, 0, 0};
static cross_patch_model_struct s_cross_left_patch_model  = {0U, 0.0f, 0, 0, 0, 0};
static cross_patch_model_struct s_cross_right_patch_model = {0U, 0.0f, 0, 0, 0, 0};


static float s_circle_yaw_start = 0.0f;
static uint8 s_circle_yaw_active = 0U;

// 右圆弧判定过程调试量
static int s_dbg_right_arc_rightCount1 = 0;
static int s_dbg_right_arc_arcMinCount = 0;
static int s_dbg_right_arc_arcMaxCount = 0;
static int s_dbg_right_arc_t_line      = 0;

// 环岛角点当前帧调试量：用于屏幕/上位机显示“这一帧真正识别到的点”，
// 不再用缓存点冒充当前点，避免 RD/RU 显示误导。
static int   s_dbg_right_down_y_current       = 0;
static int   s_dbg_right_up_y_current         = 0;
static uint8 s_dbg_right_two_corner_ok_current = 0U;
static uint8 s_dbg_rc_left_slope_ok_current   = 0U;
static uint8 s_dbg_rc_right_slope_ok_current  = 0U;
static uint8 s_dbg_rc_left_straight_ok_current = 0U;
static uint8 s_dbg_rc_right_straight_ok_current = 0U;
static int   s_dbg_left_down_y_current        = 0;
static int   s_dbg_left_up_y_current          = 0;
static uint8 s_dbg_left_two_corner_ok_current = 0U;
static uint8 s_dbg_lc_right_slope_ok_current  = 0U;
static uint8 s_dbg_lc_left_slope_ok_current   = 0U;

static int    s_lane_width_ref                = 80;
static int    s_last_valid_midline            = USE_MIDDLE_VALUE;

static float  s_distance_ref_left  = 0.0f;
static float  s_distance_ref_right = 0.0f;
static float  s_circle_enter_ref_left  = 0.0f;
static float  s_circle_enter_ref_right = 0.0f;
static float  s_yaw_ref  = 0.0f;
static uint8  s_circle_debug_code = 0;

static int    s_half_road_width[UVC_HEIGHT];
static uint8  s_left_repair_mask[UVC_HEIGHT];
static uint8  s_right_repair_mask[UVC_HEIGHT];

// 旧环岛视图
static int   s_old_left_pos[UVC_HEIGHT];
static int   s_old_right_pos[UVC_HEIGHT];
static int   s_old_midline[UVC_HEIGHT];
static uint8 s_old_left_sign[UVC_HEIGHT];
static uint8 s_old_right_sign[UVC_HEIGHT];

static int   s_old_row_begin                 = start_line;
static int   s_old_search_end                = end_line;
static int   s_old_search_stop_line          = 0;
static int   s_old_left_lost_count           = 0;
static int   s_old_right_lost_count          = 0;
static int   s_old_both_lost_count           = 0;



static uint8 s_old_circle_state_flag         = 0U;
static uint8 s_old_in_circle_state           = 0U;
static uint8 s_left_circle_point             = 0U;
static uint8 s_right_circle_point            = 0U;
static uint8 s_left_state                    = 0U;
static uint8 s_right_state                   = 0U;
static uint8 s_l_k_state                     = 0U;
static uint8 s_r_k_state                     = 0U;
static int   s_left_round_point              = 0;
static int   s_left_round_point_row          = start_line;
static int   s_left_round_up_point           = 0;
static int   s_left_round_up_point_x         = 0;
static int   s_right_round_point             = USE_WIDTH_R - 1;
static int   s_right_round_point_row         = start_line;
static int   s_right_round_up_point          = 0;
static int   s_right_round_up_point_x        = 0;
static uint8 s_right_round_down_hold_cnt     = 0U;
static uint8 s_right_round_up_hold_cnt       = 0U;
static uint8 s_right_inside_down_lost_frames = 0U;

// 右环 EXIT_PREP 阶段真实黑线的上端点缓存。
// 当前帧找到就更新；当前帧找不到就沿用上一帧；
// 如果从来没找到过，就用图像中间行右边界点兜底。
static uint8 s_right_exit_prep_line_valid    = 0U;
static int   s_right_exit_prep_line_x        = 0;
static int   s_right_exit_prep_line_y        = 0;

// 左环 EXIT_PREP 阶段真实黑线的上端点缓存。
// 左环出环时使用左边界，逻辑与右环镜像。
static uint8 s_left_exit_prep_line_valid     = 0U;
static int   s_left_exit_prep_line_x         = 0;
static int   s_left_exit_prep_line_y         = 0;

static int   s_last_point[6][2]              = {{0}};

// 环岛补线端点防跳变缓存。每条补线两个端点分别缓存。
static uint8 s_circle_repair_pt_valid[CIRCLE_REPAIR_LINE_SLOT_NUM][2] = {{0U}};
static int   s_circle_repair_pt_x[CIRCLE_REPAIR_LINE_SLOT_NUM][2]     = {{0}};
static int   s_circle_repair_pt_y[CIRCLE_REPAIR_LINE_SLOT_NUM][2]     = {{0}};


// 右环 EXIT_PREP 阶段新增竖直补线缓存点。
// 找到新点就更新；当前帧找不到就沿用上一帧。
static uint8 s_left_round_down_hold_cnt      = 0U;
static uint8 s_left_round_up_hold_cnt        = 0U;

// 模板线只读调试导出数据
static uint8 s_circle_tpl_valid              = 0U;
static uint8 s_circle_tpl_side               = 0U;   // 1-左边界 2-右边界
static int   s_circle_tpl_x1                 = 0;
static int   s_circle_tpl_y1                 = 0;
static int   s_circle_tpl_x2                 = 0;
static int   s_circle_tpl_y2                 = 0;

// ================================================================================================
// 前置声明
// ================================================================================================
static void circle_template_debug_clear(void);
static void circle_template_debug_set(uint8 side, int x1, int y1, int x2, int y2);
static void rebuild_midline_from_current_edges(void);
static int line_x_at_y(int x1, int y1, int x2, int y2, int y);
static uint8 old_left_down_corner_valid_now(int y);

// ================================================================================================
// 工具函数
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     获取当前元素处理有效行数。
// 使用方法     内部在访问 filter_left/filter_right/filter_midline 等数组前调用，用于统一取得 max_white.nums。
// 接口属性     内部接口；参数形式：element_valid_rows(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline uint8 element_valid_rows(void)
{
    return max_white.nums;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断边界数组下标是否在有效行范围内。
// 使用方法     内部在读取左右边界、中线或行号前调用，防止越界访问。
// 接口属性     内部接口；参数形式：row_valid(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline bool row_valid(int idx)
{
    return (idx >= 0 && idx < (int)element_valid_rows());
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     将列坐标限制在图像可用横向范围内。
// 使用方法     内部在写入边界点或补线点前调用，避免坐标越过 USE_WIDTH_L/USE_WIDTH_R。
// 接口属性     内部接口；参数形式：clamp_col(int col)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int clamp_col(int col)
{
    return (int)CLAMP(col, USE_WIDTH_L, USE_WIDTH_R - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据数组下标换算默认图像 y 坐标。
// 使用方法     内部在当前行没有可靠坐标时调用，用 start_line - idx 生成兜底行号。
// 接口属性     内部接口；参数形式：default_row_y(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int default_row_y(int idx)
{
    int y = start_line - idx;
    if(y < 0) y = 0;
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     从现有中线或左右边界中提取当前下标对应的图像 y 坐标。
// 使用方法     内部在补线、重建中线和调试导出时调用，优先使用已有有效坐标。
// 接口属性     内部接口；参数形式：edge_row_y_from_existing(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int edge_row_y_from_existing(int idx)
{
    int y = 0;
    if(row_valid(idx))
    {
        if(g_image_info.filter_midline[idx][1] != 0) y = g_image_info.filter_midline[idx][1];
        else if(g_image_info.filter_left[idx].pos[1] != 0) y = g_image_info.filter_left[idx].pos[1];
        else if(g_image_info.filter_right[idx].pos[1] != 0) y = g_image_info.filter_right[idx].pos[1];
        else y = default_row_y(idx);
    }
    return y;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取指定下标处的左边界 x 坐标。
// 使用方法     内部用于边界判断、宽度计算和补线，非法下标返回左侧默认边界。
// 接口属性     内部接口；参数形式：left_pos(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int left_pos(int idx)
{
    return row_valid(idx) ? (int)g_image_info.filter_left[idx].pos[0] : USE_WIDTH_L;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取指定下标处的右边界 x 坐标。
// 使用方法     内部用于边界判断、宽度计算和补线，非法下标返回右侧默认边界。
// 接口属性     内部接口；参数形式：right_pos(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int right_pos(int idx)
{
    return row_valid(idx) ? (int)g_image_info.filter_right[idx].pos[0] : (USE_WIDTH_R - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取指定下标处的中线 x 坐标。
// 使用方法     内部用于中线连续性、拟合和调试输出，非法下标返回图像中心。
// 接口属性     内部接口；参数形式：mid_pos(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int mid_pos(int idx)
{
    return row_valid(idx) ? (int)g_image_info.filter_midline[idx][0] : USE_MIDDLE_VALUE;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取指定下标处左边界有效标志。
// 使用方法     内部用于判断左边界是否由扫线得到或可参与拟合。
// 接口属性     内部接口；参数形式：left_sign(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline uint8 left_sign(int idx)
{
    return row_valid(idx) ? g_image_info.filter_left[idx].sign : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取指定下标处右边界有效标志。
// 使用方法     内部用于判断右边界是否由扫线得到或可参与拟合。
// 接口属性     内部接口；参数形式：right_sign(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline uint8 right_sign(int idx)
{
    return row_valid(idx) ? g_image_info.filter_right[idx].sign : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算指定下标处的当前赛道宽度。
// 使用方法     内部在边界恢复、单边估计和元素判定中调用。
// 接口属性     内部接口；参数形式：lane_width_at(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int lane_width_at(int idx)
{
    int width = right_pos(idx) - left_pos(idx);
    if(width < ELEMENT_MIN_LANE_WIDTH) width = ELEMENT_MIN_LANE_WIDTH;
    if(width > ELEMENT_MAX_LANE_WIDTH) width = ELEMENT_MAX_LANE_WIDTH;
    return width;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     写入指定下标处的左边界坐标和有效标志。
// 使用方法     内部在补线或边界强制修正后调用，同时写入 x/y 和 sign。
// 接口属性     内部接口；参数形式：set_left_edge(int idx, int col, uint8 sign)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline void set_left_edge(int idx, int col, uint8 sign)
{
    if(!row_valid(idx)) return;
    g_image_info.filter_left[idx].pos[0] = (uint8)clamp_col(col);
    g_image_info.filter_left[idx].pos[1] = (uint8)edge_row_y_from_existing(idx);
    g_image_info.filter_left[idx].sign   = sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     写入指定下标处的右边界坐标和有效标志。
// 使用方法     内部在补线或边界强制修正后调用，同时写入 x/y 和 sign。
// 接口属性     内部接口；参数形式：set_right_edge(int idx, int col, uint8 sign)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline void set_right_edge(int idx, int col, uint8 sign)
{
    if(!row_valid(idx)) return;
    g_image_info.filter_right[idx].pos[0] = (uint8)clamp_col(col);
    g_image_info.filter_right[idx].pos[1] = (uint8)edge_row_y_from_existing(idx);
    g_image_info.filter_right[idx].sign   = sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     按指定图像行号写入左边界坐标。
// 使用方法     十字模板线逐行投影时调用，保证数组中的 y 与模板线使用的 y 完全一致。
// 接口属性     内部接口；参数形式：set_left_edge_at_y(int idx, int col, int y, uint8 sign)。
// 备注说明     这里不能再次从中线或旧边界反推 y，否则会产生“黑色模板线”和彩色边界点错位。
//-------------------------------------------------------------------------------------------------------------------
static inline void set_left_edge_at_y(int idx, int col, int y, uint8 sign)
{
    if(!row_valid(idx)) return;
    g_image_info.filter_left[idx].pos[0] = (uint8)clamp_col(col);
    g_image_info.filter_left[idx].pos[1] = (uint8)CLAMP(y, 0, UVC_HEIGHT - 1);
    g_image_info.filter_left[idx].sign   = sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     按指定图像行号写入右边界坐标。
// 使用方法     十字模板线逐行投影时调用，保证数组中的 y 与模板线使用的 y 完全一致。
// 接口属性     内部接口；参数形式：set_right_edge_at_y(int idx, int col, int y, uint8 sign)。
// 备注说明     这里不能再次从中线或旧边界反推 y，否则会产生“黑色模板线”和彩色边界点错位。
//-------------------------------------------------------------------------------------------------------------------
static inline void set_right_edge_at_y(int idx, int col, int y, uint8 sign)
{
    if(!row_valid(idx)) return;
    g_image_info.filter_right[idx].pos[0] = (uint8)clamp_col(col);
    g_image_info.filter_right[idx].pos[1] = (uint8)CLAMP(y, 0, UVC_HEIGHT - 1);
    g_image_info.filter_right[idx].sign   = sign;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     写入指定下标处的中线坐标。
// 使用方法     内部在边界补线、重新计算中线或中线平滑后调用。
// 接口属性     内部接口；参数形式：set_midline(int idx, int col)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline void set_midline(int idx, int col)
{
    if(!row_valid(idx)) return;
    g_image_info.filter_midline[idx][0] = (uint8)clamp_col(col);
    g_image_info.filter_midline[idx][1] = (uint8)edge_row_y_from_existing(idx);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     清空本帧补线标记数组。
// 使用方法     每帧元素处理开始前调用，使后续平滑函数能区分原始扫线点和本帧补线点。
// 接口属性     内部接口；参数形式：clear_repair_mask(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void clear_repair_mask(void)
{
    memset(s_left_repair_mask, 0, sizeof(s_left_repair_mask));
    memset(s_right_repair_mask, 0, sizeof(s_right_repair_mask));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     寻找图像底部附近最可靠的中线坐标。
// 使用方法     内部在缺失中线或初始化参考中线时调用，找不到时返回图像中心。
// 接口属性     内部接口；参数形式：get_bottom_valid_midline(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int get_bottom_valid_midline(void)
{
    uint8 rows = element_valid_rows();
    uint8 sample = (rows > 6U) ? 6U : rows;
    int sum = 0;
    int cnt = 0;

    for(uint8 i = 0; i < sample; ++i)
    {
        int m = mid_pos((int)i);
        if(m >= USE_WIDTH_L && m <= USE_WIDTH_R - 1)
        {
            sum += m;
            cnt++;
        }
    }

    if(cnt > 0) return sum / cnt;
    return s_last_valid_midline;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     构建不同图像行对应的半赛道宽度查表。
// 使用方法     初始化时调用一次，为单边丢线估计另一侧边界提供宽度参考。
// 接口属性     内部接口；参数形式：build_half_road_width_table(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void build_half_road_width_table(void)
{
    uint8 rows = element_valid_rows();
    int sum = 0;
    int cnt = 0;
    int fallback = s_lane_width_ref / 2;

    if(fallback < ELEMENT_MIN_LANE_WIDTH / 2) fallback = ELEMENT_MIN_LANE_WIDTH / 2;

    int half_y = s_old_row_begin / 2;

    for(uint8 i = 0; i < rows; ++i)
    {
        if(edge_row_y_from_existing((int)i) < half_y) continue;

        if(left_sign(i) && right_sign(i))
        {
            int width = lane_width_at(i);
            if(width >= ELEMENT_BASE_WIDTH_MIN && width <= ELEMENT_BASE_WIDTH_MAX)
            {
                sum += width;
                cnt++;
            }
        }
    }

    if(cnt > 0) s_lane_width_ref = sum / cnt;
    if(s_lane_width_ref < ELEMENT_MIN_LANE_WIDTH) s_lane_width_ref = ELEMENT_MIN_LANE_WIDTH;
    if(s_lane_width_ref > ELEMENT_MAX_LANE_WIDTH) s_lane_width_ref = ELEMENT_MAX_LANE_WIDTH;

    for(uint8 i = 0; i < rows; ++i)
    {
        if(left_sign(i) && right_sign(i)) s_half_road_width[i] = lane_width_at(i) / 2;
        else s_half_road_width[i] = (cnt > 0) ? (s_lane_width_ref / 2) : fallback;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据中线估计左边界位置。
// 使用方法     内部在左边界缺失但中线可用时调用。
// 接口属性     内部接口；参数形式：estimate_left_from_mid(int idx, int mid)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int estimate_left_from_mid(int idx, int mid)
{
    int half_width = s_half_road_width[idx];
    if(half_width <= 0) half_width = s_lane_width_ref / 2;
    return clamp_col(mid - half_width);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据中线估计右边界位置。
// 使用方法     内部在右边界缺失但中线可用时调用。
// 接口属性     内部接口；参数形式：estimate_right_from_mid(int idx, int mid)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int estimate_right_from_mid(int idx, int mid)
{
    int half_width = s_half_road_width[idx];
    if(half_width <= 0) half_width = s_lane_width_ref / 2;
    return clamp_col(mid + half_width);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据当前行已有边界和历史中线微调半宽估计值。
// 使用方法     内部在单边估计时调用，避免固定半宽导致弯道处估计突变。
// 接口属性     内部接口；参数形式：adjust_half_width_for_single_edge(int idx, int half, bool use_left_edge)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int adjust_half_width_for_single_edge(int idx, int half, bool use_left_edge)
{
    if(half < ELEMENT_MIN_LANE_WIDTH / 2) half = ELEMENT_MIN_LANE_WIDTH / 2;
    if(half > ELEMENT_MAX_LANE_WIDTH / 2) half = ELEMENT_MAX_LANE_WIDTH / 2;

    // 右环阶段：如果右边丢线、只剩左边，用满半宽会把中线推得太右
    if(use_left_edge &&
       (s_circle_state == CIRCLE_RIGHT_PREP ||
        s_circle_state == CIRCLE_RIGHT_ENTER ||
        s_circle_state == CIRCLE_RIGHT_INSIDE))
    {
        half = (int)(half * 1.50f);
    }

    // 左环阶段镜像处理：如果左边丢线、只剩右边，用满半宽会把中线推得太左
    if((!use_left_edge) &&
       (s_circle_state == CIRCLE_LEFT_PREP ||
        s_circle_state == CIRCLE_LEFT_ENTER ||
        s_circle_state == CIRCLE_LEFT_INSIDE))
    {
        half = (int)(half * 1.50f);
    }

    if(half < ELEMENT_MIN_LANE_WIDTH / 2) half = ELEMENT_MIN_LANE_WIDTH / 2;
    return half;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算 float 类型绝对值。
// 使用方法     内部用于斜率、距离和拟合误差判断，避免依赖额外库函数。
// 接口属性     内部接口；参数形式：absf_local(float x)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float absf_local(float x)
{
    return (x >= 0.0f) ? x : -x;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     获取指定下标处用于中线估计的局部半宽。
// 使用方法     内部综合当前行宽度、查表宽度和有效性后返回较稳定的半宽。
// 接口属性     内部接口；参数形式：get_local_half_width_for_mid(int idx)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int get_local_half_width_for_mid(int idx)
{
    // 先用当前行真实双边宽度
    if(row_valid(idx) && left_sign(idx) && right_sign(idx))
    {
        int half = lane_width_at(idx) / 2;
        if(half < ELEMENT_MIN_LANE_WIDTH / 2) half = ELEMENT_MIN_LANE_WIDTH / 2;
        if(half > ELEMENT_MAX_LANE_WIDTH / 2) half = ELEMENT_MAX_LANE_WIDTH / 2;
        return half;
    }

    // 再向附近找最近的“双边有效行”，优先用局部真实宽度
    for(int d = 1; d <= 6; ++d)
    {
        int up = idx - d;
        int dn = idx + d;

        if(row_valid(up) && left_sign(up) && right_sign(up))
        {
            int half = lane_width_at(up) / 2;
            if(half < ELEMENT_MIN_LANE_WIDTH / 2) half = ELEMENT_MIN_LANE_WIDTH / 2;
            if(half > ELEMENT_MAX_LANE_WIDTH / 2) half = ELEMENT_MAX_LANE_WIDTH / 2;
            return half;
        }

        if(row_valid(dn) && left_sign(dn) && right_sign(dn))
        {
            int half = lane_width_at(dn) / 2;
            if(half < ELEMENT_MIN_LANE_WIDTH / 2) half = ELEMENT_MIN_LANE_WIDTH / 2;
            if(half > ELEMENT_MAX_LANE_WIDTH / 2) half = ELEMENT_MAX_LANE_WIDTH / 2;
            return half;
        }
    }

    // 还找不到，就退回当前表里的半宽
    if(row_valid(idx) && s_half_road_width[idx] > 0)
    {
        return s_half_road_width[idx];
    }

    // 最后再退到全局参考
    int half = s_lane_width_ref / 2;
    if(half < ELEMENT_MIN_LANE_WIDTH / 2) half = ELEMENT_MIN_LANE_WIDTH / 2;
    if(half > ELEMENT_MAX_LANE_WIDTH / 2) half = ELEMENT_MAX_LANE_WIDTH / 2;
    return half;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算当前偏航角相对于阶段参考角的变化量。
// 使用方法     环岛、十字等状态机内部用于判断车辆旋转进度。
// 接口属性     内部接口；参数形式：delta_yaw_value(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float delta_yaw_value(void)
{
    return yaw - s_yaw_ref;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     记录当前 yaw 作为新阶段参考角。
// 使用方法     进入需要角度累计判断的阶段时调用。
// 接口属性     内部接口；参数形式：capture_yaw_ref(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void capture_yaw_ref(void)
{
    s_yaw_ref = yaw;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     缓存当前帧中线，供下一帧连续性和平滑使用。
// 使用方法     每帧边界处理后调用，不直接改变当前控制输出。
// 接口属性     内部接口；参数形式：update_last_midline(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void update_last_midline(void)
{
    uint8 rows = element_valid_rows();
    if(rows == 0U) return;
    s_last_valid_midline = clamp_col(get_bottom_valid_midline());
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     统计图像底部左右边界同时恢复的连续行数。
// 使用方法     内部用于判断十字或特殊元素是否已经离开。
// 接口属性     内部接口；参数形式：count_bottom_recover_rows(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int count_bottom_recover_rows(void)
{
    uint8 rows = element_valid_rows();
    uint8 sample = (rows > 6U) ? 6U : rows;
    int count = 0;
    for(uint8 i = 0; i < sample; ++i)
    {
        if(left_sign(i) && right_sign(i)) count++;
    }
    return count;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     记录当前编码器距离作为阶段距离参考。
// 使用方法     进入需要里程判断的状态时调用。
// 接口属性     内部接口；参数形式：capture_stage_distance_ref(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void capture_stage_distance_ref(void)
{
    s_distance_ref_left  = encoder_left_abs_total;
    s_distance_ref_right = encoder_right_abs_total;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     记录进入环岛阶段时的编码器距离参考。
// 使用方法     环岛状态机中用于判断入环后的累计行驶距离。
// 接口属性     内部接口；参数形式：capture_circle_enter_distance_ref(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void capture_circle_enter_distance_ref(void)
{
    s_circle_enter_ref_left  = encoder_left_abs_total;
    s_circle_enter_ref_right = encoder_right_abs_total;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算左编码器相对阶段参考的行驶距离。
// 使用方法     状态机内部用于按距离推进阶段。
// 接口属性     内部接口；参数形式：stage_distance_left(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float stage_distance_left(void)
{
    return encoder_left_abs_total - s_distance_ref_left;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算右编码器相对阶段参考的行驶距离。
// 使用方法     状态机内部用于按距离推进阶段。
// 接口属性     内部接口；参数形式：stage_distance_right(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float stage_distance_right(void)
{
    return encoder_right_abs_total - s_distance_ref_right;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算左右编码器相对阶段参考的平均/合成距离。
// 使用方法     状态机内部用于距离阈值判断。
// 接口属性     内部接口；参数形式：stage_distance_sum(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float stage_distance_sum(void)
{
    return stage_distance_left() + stage_distance_right();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算左编码器相对环岛进入参考的距离。
// 使用方法     环岛状态机内部用于入环距离判断。
// 接口属性     内部接口；参数形式：circle_enter_distance_left(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float circle_enter_distance_left(void)
{
    return encoder_left_abs_total - s_circle_enter_ref_left;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算右编码器相对环岛进入参考的距离。
// 使用方法     环岛状态机内部用于入环距离判断。
// 接口属性     内部接口；参数形式：circle_enter_distance_right(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float circle_enter_distance_right(void)
{
    return encoder_right_abs_total - s_circle_enter_ref_right;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算环岛进入后的左右编码器合成距离。
// 使用方法     环岛 ENTER/INSIDE 阶段用于判断车辆推进程度。
// 接口属性     内部接口；参数形式：circle_enter_distance_sum(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float circle_enter_distance_sum(void)
{
    return circle_enter_distance_left() + circle_enter_distance_right();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算环岛过程中的累计 yaw 变化量。
// 使用方法     环岛状态机根据左右环方向使用该值判断进入、内部和退出时机。
// 接口属性     内部接口；参数形式：circle_yaw_value(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static float circle_yaw_value(void)
{
    if(!s_circle_yaw_active)
    {
        return 0.0f;
    }
    return yaw - s_circle_yaw_start;
}// ================================================================================================
// 十字逻辑：当前强制关闭，避免抢占环岛
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     强制关闭十字处理相关状态。
// 使用方法     在进入环岛或需要排除十字干扰时调用，避免两个元素状态机互相影响。
// 接口属性     内部接口；参数形式：cross_force_disable_internal(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void cross_force_disable_internal(void)
{
    s_cross_candidate = 0U;
    s_cross_confirm_frames = 0U;
    s_cross_line_index = 0U;
    s_cross_lost_frames = 0U;
    s_cross_bottom_recover_frames = 0U;
    s_cross_state = CROSS_IDLE;

    s_cross_lu = {0U, 0, 0, 0};
    s_cross_ld = {0U, 0, 0, 0};
    s_cross_ru = {0U, 0, 0, 0};
    s_cross_rd = {0U, 0, 0, 0};
    s_cross_left_patch_model  = {0U, 0.0f, 0, 0, 0, 0};
    s_cross_right_patch_model = {0U, 0.0f, 0, 0, 0, 0};

    if(s_element_mode == ELEMENT_CROSSROAD)
    {
        s_element_mode = ELEMENT_NONE;
        s_element_busy = false;
    }
}

// ================================================================================================
// 中线重建
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据当前左右边界重新生成中线。
// 使用方法     补线或强制边界修正后调用，使控制使用的中线与最终边界一致。
// 接口属性     内部接口；参数形式：rebuild_midline_from_current_edges(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void rebuild_midline_from_current_edges(void)
{
    uint8 rows = element_valid_rows();
    if(rows == 0U) return;

    int prev_mid = s_last_valid_midline;

    for(uint8 i = 0; i < rows; ++i)
    {
        int m = prev_mid;

        // ENTER 阶段：不再用“单边 + 半宽”补中线，
        // 直接使用当前数组里已有的左右边线位置求中点。
if(s_circle_state == CIRCLE_LEFT_ENTER  ||
   s_circle_state == CIRCLE_RIGHT_ENTER ||
   s_circle_state == CIRCLE_LEFT_INSIDE ||
   s_circle_state == CIRCLE_RIGHT_INSIDE ||
    s_circle_state == CIRCLE_EXIT_PREP)
{
    // ENTER 和 INSIDE 阶段统一处理：
    // 直接使用当前数组里的左右边线位置求中点，
    // 不再走“单边 + 半宽”补中线逻辑。
    int l = g_image_info.filter_left[i].pos[0];
    int r = g_image_info.filter_right[i].pos[0];
    if(!left_sign(i) && !right_sign(i)) m = USE_MIDDLE_VALUE;
    else m = (l + r) / 2;
}
        else
        {
            bool l_ok = (left_sign(i) != 0U);
            bool r_ok = (right_sign(i) != 0U);
            int l = left_pos(i);
            int r = right_pos(i);
            bool left_virtual = false;
            bool right_virtual = false;

            // 十字补线只作为本帧重构中线时使用的虚拟边界：
            // 不写 filter_left/right，也不改变原始左右边界的 sign。
            if(s_cross_state == CROSS_INSIDE)
            {
                const int y = edge_row_y_from_existing(i);
                if(s_cross_left_patch_model.valid &&
                   y >= s_cross_left_patch_model.y1 &&
                   y <= s_cross_left_patch_model.y2)
                {
                    l = line_x_at_y(s_cross_left_patch_model.x1,
                                    s_cross_left_patch_model.y1,
                                    s_cross_left_patch_model.x2,
                                    s_cross_left_patch_model.y2,
                                    y);
                    l_ok = true;
                    left_virtual = true;
                }
                if(s_cross_right_patch_model.valid &&
                   y >= s_cross_right_patch_model.y1 &&
                   y <= s_cross_right_patch_model.y2)
                {
                    r = line_x_at_y(s_cross_right_patch_model.x1,
                                    s_cross_right_patch_model.y1,
                                    s_cross_right_patch_model.x2,
                                    s_cross_right_patch_model.y2,
                                    y);
                    r_ok = true;
                    right_virtual = true;
                }
            }

            // 当前行只要有一侧使用虚拟补线，就和另一侧原始边界直接求中点；
            // 即使另一侧原始 sign==0，也不再使用“虚拟线 +/- 半路宽”。
            if(left_virtual || right_virtual)
            {
                m = (l + r) / 2;
            }
            else if(l_ok || r_ok)
            {
                // 单边有效时也直接使用当前左右坐标求平均，不使用半路宽。
                m = (l + r) / 2;
            }
            else
            {
                // 两侧都无效时固定使用图像中心，不沿用上一行。
                m = USE_MIDDLE_VALUE;
            }
        }

        m = clamp_col(m);
        set_midline(i, m);
        prev_mid = m;
    }
}

// ================================================================================================
// 旧视图导入与判环辅助
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     返回旧版环岛逻辑使用的底部起始行号。
// 使用方法     旧环岛识别函数内部调用，用于兼容原 row_begin 坐标体系。
// 接口属性     内部接口；参数形式：old_row_begin(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int old_row_begin(void) { return s_old_row_begin; }
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     返回旧版环岛逻辑使用的上方搜索终止行号。
// 使用方法     旧环岛角点搜索和斜率判断内部调用。
// 接口属性     内部接口；参数形式：old_search_end(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int old_search_end(void) { return s_old_search_end; }
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断旧版 y 行号是否处在可搜索范围内。
// 使用方法     旧环岛逻辑读取旧视图数组前调用。
// 接口属性     内部接口；参数形式：old_row_valid(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline bool old_row_valid(int y) { return (y >= 0 && y < UVC_HEIGHT); }

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     将旧版图像 y 行号转换为当前数组下标。
// 使用方法     旧环岛识别和角点搜索内部调用。
// 接口属性     内部接口；参数形式：old_to_idx(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int old_to_idx(int y)
{
    int idx = old_row_begin() - y;
    if(idx < 0) idx = 0;
    if(idx >= (int)element_valid_rows()) idx = (int)element_valid_rows() - 1;
    return idx;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取旧版 y 行对应的左边界 x 坐标。
// 使用方法     旧环岛判断中按图像行号访问左边界时调用。
// 接口属性     内部接口；参数形式：old_left_pos_at(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int old_left_pos_at(int y)  { return old_row_valid(y) ? s_old_left_pos[y]  : USE_WIDTH_L; }
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取旧版 y 行对应的右边界 x 坐标。
// 使用方法     旧环岛判断中按图像行号访问右边界时调用。
// 接口属性     内部接口；参数形式：old_right_pos_at(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int old_right_pos_at(int y) { return old_row_valid(y) ? s_old_right_pos[y] : (USE_WIDTH_R - 1); }
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取旧版 y 行对应的左边界有效标志。
// 使用方法     旧环岛判断中判断左边界是否可用时调用。
// 接口属性     内部接口；参数形式：old_left_sign_at(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline uint8 old_left_sign_at(int y)  { return old_row_valid(y) ? s_old_left_sign[y]  : 0U; }
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     读取旧版 y 行对应的右边界有效标志。
// 使用方法     旧环岛判断中判断右边界是否可用时调用。
// 接口属性     内部接口；参数形式：old_right_sign_at(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline uint8 old_right_sign_at(int y) { return old_row_valid(y) ? s_old_right_sign[y] : 0U; }

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     把当前边界数据导入旧环岛算法使用的视图数组。
// 使用方法     每帧环岛识别前调用，使旧逻辑能继续按 y 行号方式处理。
// 接口属性     内部接口；参数形式：circle_import_old_view(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_import_old_view(void)
{
    uint8 rows = element_valid_rows();
    int last_y = start_line - ((rows > 0U) ? (int)rows - 1 : 0);
    if(last_y < end_line) last_y = end_line;

    s_old_row_begin = start_line;
    s_old_search_end = last_y;
    s_old_search_stop_line = s_old_row_begin - s_old_search_end;

    s_old_left_lost_count = 0;
    s_old_right_lost_count = 0;
    s_old_both_lost_count = 0;

    for(int y = 0; y < UVC_HEIGHT; ++y)
    {
        s_old_left_pos[y] = USE_WIDTH_L;
        s_old_right_pos[y] = USE_WIDTH_R - 1;
        s_old_midline[y] = USE_MIDDLE_VALUE;
        s_old_left_sign[y] = 0U;
        s_old_right_sign[y] = 0U;
    }

    for(uint8 i = 0; i < rows; ++i)
    {
        int y = edge_row_y_from_existing(i);
        if(!old_row_valid(y)) continue;
        s_old_left_pos[y] = left_pos(i);
        s_old_right_pos[y] = right_pos(i);
        s_old_midline[y] = mid_pos(i);
        s_old_left_sign[y] = left_sign(i);
        s_old_right_sign[y] = right_sign(i);
    }

    for(int y = s_old_row_begin - 1; y >= s_old_search_end; --y)
    {
        bool l_ok = old_left_sign_at(y) != 0U;
        bool r_ok = old_right_sign_at(y) != 0U;
        if(!l_ok) s_old_left_lost_count++;
        if(!r_ok) s_old_right_lost_count++;
        if(!l_ok && !r_ok) s_old_both_lost_count++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断指定边界线段是否连续且满足最小有效比例。
// 使用方法     旧环岛斜率、直线性和角点筛选内部调用。
// 接口属性     内部接口；参数形式：old_is_line_continuous(const int *pos, const uint8 *sign, int start, int end, float max_diff, float min_valid_ratio)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_is_line_continuous(const int *pos, const uint8 *sign, int start, int end, float max_diff, float min_valid_ratio)
{
    if(start < end) { int t = start; start = end; end = t; }
    if(start >= UVC_HEIGHT) start = UVC_HEIGHT - 1;
    if(end < 5) end = 5;

    int total_points = start - end + 1;
    if(total_points <= 0) return 0U;

    int valid_count = 0;
    int prev_valid_i = -1;
    for(int i = start; i >= end; --i)
    {
        if(sign[i])
        {
            valid_count++;
            if(prev_valid_i >= 0)
            {
                int row_diff = prev_valid_i - i;
                if(row_diff <= 0) row_diff = 1;
                int pos_diff = pos[prev_valid_i] - pos[i];
                float normalized = (float)((pos_diff >= 0) ? pos_diff : -pos_diff) / (float)row_diff;
                if(normalized > max_diff) return 0U;
            }
            prev_valid_i = i;
        }
    }

    if(((float)valid_count / (float)total_points) < min_valid_ratio) return 0U;
    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断指定边界线段是否近似直线。
// 使用方法     旧环岛入口判定和辅助边筛选时调用。
// 接口属性     内部接口；参数形式：old_is_line_straight(const int *pos, const uint8 *sign, int start, int end, float max_error)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_is_line_straight(const int *pos, const uint8 *sign, int start, int end, float max_error)
{
    if(start < end) { int t = start; start = end; end = t; }
    if(start >= UVC_HEIGHT) start = UVC_HEIGHT - 1;
    if(end < 5) end = 5;

    const int sample_step = OLD_LINE_FIT_SAMPLE_STEP;
    int sample_count = 0;
    int valid_count = 0;
    float sum_x = 0.0f, sum_y = 0.0f, sum_xy = 0.0f, sum_xx = 0.0f;
    for(int i = start; i >= end; i -= sample_step)
    {
        sample_count++;
        if(sign[i])
        {
            sum_x += (float)i;
            sum_y += (float)pos[i];
            sum_xy += (float)(i * pos[i]);
            sum_xx += (float)(i * i);
            valid_count++;
        }
    }
    if((sample_count < OLD_LINE_FIT_MIN_SAMPLES) ||
       (valid_count < OLD_LINE_FIT_MIN_SAMPLES) ||
       ((float)valid_count / (float)sample_count < OLD_LINE_FIT_MIN_VALID_RATIO))
    {
        return 0U;
    }

    float denominator = valid_count * sum_xx - sum_x * sum_x;
    if(fabsf(denominator) < 1e-4f) return 0U;

    float a = (valid_count * sum_xy - sum_x * sum_y) / denominator;
    float b = (sum_y - a * sum_x) / (float)valid_count;
    float max_dev = 0.0f;

    for(int i = start; i >= end; i -= sample_step)
    {
        if(sign[i])
        {
            float expected = a * (float)i + b;
            float dev = fabsf((float)pos[i] - expected);
            if(dev > max_dev) max_dev = dev;
        }
    }
    return (max_dev <= max_error) ? 1U : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     同时判断边界线段的直线性和连续性。
// 使用方法     旧环岛入口预筛中用于过滤不稳定边界。
// 接口属性     内部接口；参数形式：old_is_line_straight_and_continuous(const int *pos, const uint8 *sign, int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_is_line_straight_and_continuous(const int *pos, const uint8 *sign, int start, int end)
{
    if(!old_is_line_continuous(pos, sign, start, end, 2.6f, 0.6f)) return 0U;
    return old_is_line_straight(pos, sign, start, end, 1.4f);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断指定边界线段斜率是否满足阈值范围。
// 使用方法     旧环岛左右入口斜率条件中调用。
// 接口属性     内部接口；参数形式：old_judge_slope(int start, int end, const int *positions, const uint8 *sign, float thr, float delta)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static bool old_judge_slope(int start, int end, const int *positions, const uint8 *sign, float thr, float delta)
{
    if(start >= UVC_HEIGHT) start = UVC_HEIGHT - 1;
    if(end < 5) end = 5;
    if((end + 40) >= UVC_HEIGHT) return false;
    int end10 = end + 10;
    int end20 = end + 25;
    int end30 = end + 40;
    if(!sign[start] || !sign[end] || !sign[end10] || !sign[end20] || !sign[end30]) return false;

    float d4 = (float)(positions[start] - positions[end]);
    float d1 = (float)(positions[start] - positions[end10]);
    float d2 = (float)(positions[start] - positions[end20]);
    float d3 = (float)(positions[start] - positions[end30]);
    if(fabsf(d4) < 1e-4f || fabsf(d1) < 1e-4f || fabsf(d2) < 1e-4f || fabsf(d3) < 1e-4f) return false;

    float k4 = (float)(start - end) / d4;
    float k1 = (float)(start - end10) / d1;
    float k2 = (float)(start - end20) / d2;
    float k3 = (float)(start - end30) / d3;
    return (k4 > thr && k3 > thr && k2 > thr && k1 > thr &&
            k4 < thr + delta && k3 < thr + delta &&
            k2 < thr + delta && k1 < thr + delta);
}

// 可见线段斜率判据：用于环岛丢边侧。
// 和 old_judge_slope 的区别：不要求固定的 5 个采样行全部有效，
// 而是在当前可见边界中寻找若干段满足斜率范围的线段。
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在可见边界中寻找满足斜率条件的局部线段。
// 使用方法     旧环岛在丢线较多时使用，用于替代整段边界斜率判断。
// 接口属性     内部接口；参数形式：old_judge_visible_slope_segment(int start, int end, const int *positions, const uint8 *sign, float thr, float delta, int dy_min, int dy_max, int hit_need)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_judge_visible_slope_segment(int start,
                                             int end,
                                             const int *positions,
                                             const uint8 *sign,
                                             float thr,
                                             float delta,
                                             int dy_min,
                                             int dy_max,
                                             int hit_need)
{
    if(start >= UVC_HEIGHT) start = UVC_HEIGHT - 1;
    if(end < 5) end = 5;
    if(dy_min < 3) dy_min = 3;
    if(dy_max < dy_min) dy_max = dy_min;
    if(hit_need < 1) hit_need = 1;

    if(start <= end + dy_min) return 0U;

    int hit = 0;

    for(int y_high = end; y_high <= start - dy_min; ++y_high)
    {
        if(!old_row_valid(y_high) || !sign[y_high]) continue;

        for(int dy = dy_min; dy <= dy_max; dy += 4)
        {
            int y_low = y_high + dy;
            if(y_low > start) break;
            if(!old_row_valid(y_low) || !sign[y_low]) continue;

            float dx = (float)(positions[y_low] - positions[y_high]);
            if(fabsf(dx) < 1e-4f) continue;

            float k = (float)(y_low - y_high) / dx;
            if(k > thr && k < thr + delta)
            {
                hit++;
                break;
            }
        }

        if(hit >= hit_need) return 1U;
    }

    return 0U;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算 int 类型绝对值。
// 使用方法     内部用于坐标差、跳变和阈值判断。
// 接口属性     内部接口；参数形式：iabs_local(int x)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int iabs_local(int x)
{
    return (x >= 0) ? x : -x;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     按最大步长限制整数坐标变化。
// 使用方法     补线端点平滑时调用，防止端点单帧跳变。
// 接口属性     内部接口；参数形式：limit_step_i(int cur, int target, int step)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static inline int limit_step_i(int cur, int target, int step)
{
    int d = target - cur;

    if(d > step)  return cur + step;
    if(d < -step) return cur - step;

    return target;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     清空环岛补线端点平滑缓存。
// 使用方法     环岛重置或退出时调用，防止上一轮环岛缓存影响下一轮。
// 接口属性     内部接口；参数形式：circle_repair_smooth_clear(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_repair_smooth_clear(void)
{
    memset(s_circle_repair_pt_valid, 0, sizeof(s_circle_repair_pt_valid));
    memset(s_circle_repair_pt_x,     0, sizeof(s_circle_repair_pt_x));
    memset(s_circle_repair_pt_y,     0, sizeof(s_circle_repair_pt_y));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断指定补线端点是否存在历史缓存。
// 使用方法     环岛补线端点找不到时，用于决定能否沿用上一帧。
// 接口属性     内部接口；参数形式：circle_repair_point_has_cache(uint8 slot, uint8 point_id)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 circle_repair_point_has_cache(uint8 slot, uint8 point_id)
{
    if(slot >= CIRCLE_REPAIR_LINE_SLOT_NUM) return 0U;
    if(point_id > 1U) return 0U;

    return s_circle_repair_pt_valid[slot][point_id];
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对单个环岛补线端点进行防跳变和平滑更新。
// 使用方法     PREP/ENTER/INSIDE 等阶段得到候选端点后调用，返回最终可用端点。
// 接口属性     内部接口；参数形式：circle_repair_point_smooth_update(uint8 slot, uint8 point_id, uint8 have_target, int target_x, int target_y, int *out_x, int *out_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 circle_repair_point_smooth_update(uint8 slot,
                                               uint8 point_id,
                                               uint8 have_target,
                                               int target_x,
                                               int target_y,
                                               int *out_x,
                                               int *out_y)
{
    if(out_x == NULL || out_y == NULL) return 0U;
    if(slot >= CIRCLE_REPAIR_LINE_SLOT_NUM) return 0U;
    if(point_id > 1U) return 0U;

    if(have_target == 0U)
    {
        if(s_circle_repair_pt_valid[slot][point_id] == 0U)
        {
            return 0U;
        }

        *out_x = s_circle_repair_pt_x[slot][point_id];
        *out_y = s_circle_repair_pt_y[slot][point_id];
        return 1U;
    }

    target_x = clamp_col(target_x);

    if(target_y < old_search_end()) target_y = old_search_end();
    if(target_y > old_row_begin())  target_y = old_row_begin();

    if(s_circle_repair_pt_valid[slot][point_id] == 0U)
    {
        s_circle_repair_pt_valid[slot][point_id] = 1U;
        s_circle_repair_pt_x[slot][point_id] = target_x;
        s_circle_repair_pt_y[slot][point_id] = target_y;
    }
    else
    {
        int dx = iabs_local(target_x - s_circle_repair_pt_x[slot][point_id]);
        int dy = iabs_local(target_y - s_circle_repair_pt_y[slot][point_id]);

        // 横向和纵向同时大跳，认为当前候选端点不可信，直接沿用上一帧。
        if((dx > OLD_CIRCLE_REPAIR_REJECT_X) &&
           (dy > OLD_CIRCLE_REPAIR_REJECT_Y))
        {
            // keep last point
        }
        else
        {
            // 候选端点可信，但仍限制每帧最大移动量，避免补线端点突然跳变。
            s_circle_repair_pt_x[slot][point_id] = limit_step_i(
                s_circle_repair_pt_x[slot][point_id],
                target_x,
                OLD_CIRCLE_REPAIR_STEP_X);

            s_circle_repair_pt_y[slot][point_id] = limit_step_i(
                s_circle_repair_pt_y[slot][point_id],
                target_y,
                OLD_CIRCLE_REPAIR_STEP_Y);
        }
    }

    *out_x = s_circle_repair_pt_x[slot][point_id];
    *out_y = s_circle_repair_pt_y[slot][point_id];
    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对一条环岛补线的两个端点同时做平滑更新。
// 使用方法     环岛补线导出前调用，使整条模板线端点连续变化。
// 接口属性     内部接口；参数形式：circle_repair_line_smooth_update(uint8 slot, uint8 have_p1, int x1, int y1, uint8 have_p2, int x2, int y2, int *out_x1, int *out_y1, int *out_x2, int *out_y2)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 circle_repair_line_smooth_update(uint8 slot,
                                              uint8 have_p1,
                                              int x1,
                                              int y1,
                                              uint8 have_p2,
                                              int x2,
                                              int y2,
                                              int *out_x1,
                                              int *out_y1,
                                              int *out_x2,
                                              int *out_y2)
{
    uint8 p1_ok = circle_repair_point_smooth_update(slot, 0U, have_p1, x1, y1, out_x1, out_y1);
    uint8 p2_ok = circle_repair_point_smooth_update(slot, 1U, have_p2, x2, y2, out_x2, out_y2);

    return (uint8)((p1_ok != 0U) && (p2_ok != 0U));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在候选点缺失时使用固定兜底点或历史缓存生成稳定端点。
// 使用方法     环岛补线下端点需要固定到图像边角或沿用缓存时调用。
// 接口属性     内部接口；参数形式：circle_repair_point_smooth_or_fixed(uint8 slot, uint8 point_id, uint8 have_target, int target_x, int target_y, int fixed_x, int fixed_y, int *out_x, int *out_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 circle_repair_point_smooth_or_fixed(uint8 slot,
                                                  uint8 point_id,
                                                  uint8 have_target,
                                                  int target_x,
                                                  int target_y,
                                                  int fixed_x,
                                                  int fixed_y,
                                                  int *out_x,
                                                  int *out_y)
{
    /*
     * 固定兜底点只用于“该阶段该端点从进入到现在一次真实点都没找到”的情况。
     * 注意：固定点不写入平滑缓存。
     * 这样一旦后续找到了真实端点，会直接用真实端点初始化缓存，
     * 不会从固定点慢慢限速移动过去。
     */
    if(circle_repair_point_smooth_update(slot,
                                         point_id,
                                         have_target,
                                         target_x,
                                         target_y,
                                         out_x,
                                         out_y))
    {
        return 1U;
    }

    if(have_target != 0U) return 0U;
    if(circle_repair_point_has_cache(slot, point_id)) return 0U;

    fixed_x = clamp_col(fixed_x);
    if(fixed_y < old_search_end()) fixed_y = old_search_end();
    if(fixed_y > old_row_begin())  fixed_y = old_row_begin();

    *out_x = fixed_x;
    *out_y = fixed_y;
    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对补线两个端点执行“候选优先、缓存次之、固定兜底”的平滑策略。
// 使用方法     环岛 PREP/INSIDE 阶段构建模板线时调用。
// 接口属性     内部接口；参数形式：circle_repair_line_smooth_or_fixed(uint8 slot, uint8 have_p1, int x1, int y1, int fixed_x1, int fixed_y1, uint8 have_p2, int x2, int y2, int fixed_x2, int fixed_y2, int *out_x1, int *out_y1, int *out_x2, int *out_y2)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 circle_repair_line_smooth_or_fixed(uint8 slot,
                                                uint8 have_p1,
                                                int x1,
                                                int y1,
                                                int fixed_x1,
                                                int fixed_y1,
                                                uint8 have_p2,
                                                int x2,
                                                int y2,
                                                int fixed_x2,
                                                int fixed_y2,
                                                int *out_x1,
                                                int *out_y1,
                                                int *out_x2,
                                                int *out_y2)
{
    uint8 p1_ok = circle_repair_point_smooth_or_fixed(slot,
                                                      0U,
                                                      have_p1,
                                                      x1,
                                                      y1,
                                                      fixed_x1,
                                                      fixed_y1,
                                                      out_x1,
                                                      out_y1);

    uint8 p2_ok = circle_repair_point_smooth_or_fixed(slot,
                                                      1U,
                                                      have_p2,
                                                      x2,
                                                      y2,
                                                      fixed_x2,
                                                      fixed_y2,
                                                      out_x2,
                                                      out_y2);

    return (uint8)((p1_ok != 0U) && (p2_ok != 0U));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在右边界中寻找单调性变化位置。
// 使用方法     旧环岛右侧角点搜索中用于定位边界突变点。
// 接口属性     内部接口；参数形式：old_monotonicity_change_right(int row_begin, int row_end, uint8 highThreshold, uint8 lowThreshold, uint8 step)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_monotonicity_change_right(int row_begin, int row_end, uint8 highThreshold, uint8 lowThreshold, uint8 step)
{
    for(int i = row_begin - step; i > row_end + step; --i)
    {
        int diff1 = old_right_pos_at(i - step) - old_right_pos_at(i);
        int diff2 = old_right_pos_at(i + step) - old_right_pos_at(i);
        if(diff1 > highThreshold && diff2 < lowThreshold && old_right_sign_at(i) && old_right_sign_at(i + step))
        {
            int line = i + 1;
            if(line > row_begin) line = row_begin;
            return line;
        }
    }
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在左边界中寻找单调性变化位置。
// 使用方法     旧环岛左侧角点搜索中用于定位边界突变点。
// 接口属性     内部接口；参数形式：old_monotonicity_change_left(int row_begin, int row_end, uint8 highThreshold, uint8 lowThreshold, uint8 step)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_monotonicity_change_left(int row_begin, int row_end, uint8 highThreshold, uint8 lowThreshold, uint8 step)
{
    for(int i = row_begin - step; i > row_end + step; --i)
    {
        int diff1 = old_left_pos_at(i) - old_left_pos_at(i - step);
        int diff2 = old_left_pos_at(i) - old_left_pos_at(i + step);
        if(diff1 > highThreshold && diff2 < lowThreshold && old_left_sign_at(i) && old_left_sign_at(i + step))
        {
            int line = i + 1;
            if(line > row_begin) line = row_begin;
            return line;
        }
    }
    return 0;
}

// static uint8 old_l_round_get_arc(int start, int end)
// {
//     if(s_old_left_lost_count >= (int)(0.9f * (float)(old_row_begin() - old_search_end()))) return 0U;
//     if(start >= old_row_begin()) start = old_row_begin();
//     if(end <= 5) end = 5;
//     if(start <= end) return 0U;

//     uint8 leftCount1 = 0U;
//     int i = start;
//     for(; i > end + 1; --i)
//     {
//         if(!old_left_sign_at(i)) leftCount1++;
//         else leftCount1 = 0U;
//         if(old_left_sign_at(i - 1) && old_left_sign_at(i - 2) && !old_left_sign_at(i)) break;
//     }

//     uint8 arcMaxCount = 0U, arcMinCount = 0U;
//     int arcMax = 0;
//     for(i = i - 1; i > end + 1; --i)
//     {
//         if(old_left_sign_at(i) && arcMax <= old_left_pos_at(i)) { arcMax = old_left_pos_at(i); arcMaxCount++; }
//         if(old_left_sign_at(i) && arcMax > old_left_pos_at(i)) arcMinCount++;
//         if(old_left_sign_at(i) && !old_left_sign_at(i - 1) && !old_left_sign_at(i - 2)) break;
//     }

//     int t = old_monotonicity_change_left(start, end, 3U, 8U, 4U);
//     return (leftCount1 > 15U && arcMaxCount > 5U && arcMinCount > 4U && t) ? 1U : 0U;
// }

// static uint8 old_r_round_get_arc(int start, int end)
// {
//     if(s_old_right_lost_count >= (int)(0.9f * (float)(old_row_begin() - old_search_end())))
//     {
//         s_dbg_right_arc_rightCount1 = 0;
//         s_dbg_right_arc_arcMinCount = 0;
//         s_dbg_right_arc_arcMaxCount = 0;
//         s_dbg_right_arc_t_line      = 0;
//         return 0U;
//     }

//     if(start >= old_row_begin()) start = old_row_begin();
//     if(end <= 5) end = 5;
//     if(start <= end)
//     {
//         s_dbg_right_arc_rightCount1 = 0;
//         s_dbg_right_arc_arcMinCount = 0;
//         s_dbg_right_arc_arcMaxCount = 0;
//         s_dbg_right_arc_t_line      = 0;
//         return 0U;
//     }

//     uint8 rightCount1 = 0U;
//     int i = start;
//     for(; i > end + 1; --i)
//     {
//         if(!old_right_sign_at(i)) rightCount1++;
//         else rightCount1 = 0U;

//         if(!old_right_sign_at(i) && old_right_sign_at(i - 1) && old_right_sign_at(i - 2))
//         {
//             break;
//         }
//     }

//     uint8 arcMaxCount = 0U, arcMinCount = 0U;
//     int arcMin = USE_WIDTH_R - 1;
//     for(i = i - 1; i > end + 1; --i)
//     {
//         if(old_right_sign_at(i) && arcMin >= old_right_pos_at(i))
//         {
//             arcMin = old_right_pos_at(i);
//             arcMinCount++;
//         }

//         if(old_right_sign_at(i) && arcMin < old_right_pos_at(i))
//         {
//             arcMaxCount++;
//         }

//         if(old_right_sign_at(i) && !old_right_sign_at(i - 1) && !old_right_sign_at(i - 2))
//         {
//             break;
//         }
//     }

//     int t = old_monotonicity_change_right(start, end, 2U, 10U, 3U);

//     // ---- 调试量导出 ----
//     s_dbg_right_arc_rightCount1 = (int)rightCount1;
//     s_dbg_right_arc_arcMinCount = (int)arcMinCount;
//     s_dbg_right_arc_arcMaxCount = (int)arcMaxCount;
//     s_dbg_right_arc_t_line      = t;

//     return (rightCount1 > 5U && arcMinCount > 2U /**&& arcMaxCount > 2U **/&& t) ? 1U : 0U;
// }



//-------------------------------------------------------------------------------------------------------------------
// 函数作用     原始搜索左下角点候选行。
// 使用方法     旧左环/右环辅助判断中调用，只负责找候选，不做完整稳定性约束。
// 接口属性     内部接口；参数形式：old_find_left_down_corner_raw(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_left_down_corner_raw(int start, int end)
{
    if(start > old_row_begin()) start = old_row_begin();
    if(end < 8) end = 8;

    for(int y = start - OLD_CIRCLE_DOWN_CONTINUE_N; y > end + 8; --y)
    {
        if(y < OLD_LC_DOWN_Y_MIN) break;

        if(!old_left_sign_at(y)) continue;

        // 1. 下方必须有较长连续左边界
        uint8 down_ok = 1U;
        for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
        {
            if(!old_left_sign_at(y + k))
            {
                down_ok = 0U;
                break;
            }
        }
        if(!down_ok) continue;

        int x0 = old_left_pos_at(y);

        // 2. 当前点必须和下方长边界整体接近
        int max_down_diff = 0;
        for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
        {
            int d = iabs_local(x0 - old_left_pos_at(y + k));
            if(d > max_down_diff) max_down_diff = d;
        }
        if(max_down_diff > OLD_CIRCLE_DOWN_FIT_MAX) continue;

        // 3. 下方边界本身必须平滑
        uint8 down_smooth = 1U;
        for(int k = 1; k < OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
        {
            int d = iabs_local(old_left_pos_at(y + k) - old_left_pos_at(y + k + 1));
            if(d > OLD_CIRCLE_DOWN_SMOOTH_MAX)
            {
                down_smooth = 0U;
                break;
            }
        }
        if(!down_smooth) continue;

        // 4. 上方允许：连续丢线，或者和当前点明显跳变
        int lost_cnt = 0;
        for(int k = 1; k <= 4; ++k)
        {
            if(!old_left_sign_at(y - k)) lost_cnt++;
        }

        bool up_lost = (lost_cnt >= 2);

        bool up_far = false;
        for(int k = 1; k <= 4; ++k)
        {
            if(old_left_sign_at(y - k))
            {
                int d = iabs_local(x0 - old_left_pos_at(y - k));
                if(d >= OLD_CIRCLE_UP_FAR_MIN)
                {
                    up_far = true;
                    break;
                }
            }
        }

        if(up_lost || up_far)
        {
            return y;
        }
    }

    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     搜索稳定的左下角点。
// 使用方法     旧环岛入口判断中调用，可根据上角点约束下角点位置。
// 接口属性     内部接口；参数形式：old_find_left_down_corner_stable(int start, int end, int up_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_left_down_corner_stable(int start, int end, int up_y)
{
    int cand = old_find_left_down_corner_raw(start, end);

    if(up_y > 0)
    {
        if(cand > 0)
        {
            int dy = cand - up_y;
            if(dy < OLD_LC_DY_MIN || dy > OLD_LC_DY_MAX)
            {
                cand = 0;
            }
        }

        if(cand <= 0 &&
           s_left_round_point_row >= OLD_LC_DOWN_Y_MIN &&
           old_left_down_corner_valid_now(s_left_round_point_row))
        {
            int dy_cache = s_left_round_point_row - up_y;
            if(dy_cache >= OLD_LC_DY_MIN && dy_cache <= OLD_LC_DY_MAX)
            {
                return s_left_round_point_row;
            }
        }

        if(cand <= 0)
        {
            return 0;
        }
    }
    else
    {
        if(cand <= 0)
        {
            if(s_left_round_point_row >= OLD_LC_DOWN_Y_MIN &&
               old_left_down_corner_valid_now(s_left_round_point_row))
            {
                return s_left_round_point_row;
            }
            return 0;
        }
    }

    if(s_left_round_point_row >= OLD_LC_DOWN_Y_MIN &&
       old_left_down_corner_valid_now(s_left_round_point_row))
    {
        int diff = cand - s_left_round_point_row;

        if(diff < -OLD_CIRCLE_DOWN_JUMP_MAX)
        {
            return s_left_round_point_row;
        }
    }

    return cand;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     搜索稳定的左下角点。
// 使用方法     旧环岛入口判断中调用，可根据上角点约束下角点位置。
// 接口属性     内部接口；参数形式：old_find_left_down_corner_stable(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_left_down_corner_stable(int start, int end)
{
    int cand = old_find_left_down_corner_raw(start, end);

    if(cand <= 0)
    {
        return (s_left_round_point_row > 0) ? s_left_round_point_row : 0;
    }

    if(s_left_round_point_row > 0)
    {
        int diff = iabs_local(cand - s_left_round_point_row);

        if(diff > OLD_CIRCLE_DOWN_JUMP_MAX)
        {
            return s_left_round_point_row;
        }
    }

    return cand;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在左边界恢复段中搜索左上角点。
// 使用方法     左环入口和补线阶段调用，用于确定上端补线点。
// 接口属性     内部接口；参数形式：old_find_left_up_corner_in_recover_segment(int start, int end, int left_down_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_left_up_corner_in_recover_segment(int start,
                                                      int end,
                                                      int left_down_y)
{
    (void)start;
    (void)end;
    (void)left_down_y;

    int scan_top = 2;
    int scan_bottom = UVC_HEIGHT / 2;

    if(scan_bottom > UVC_HEIGHT - 3)
    {
        scan_bottom = UVC_HEIGHT - 3;
    }

    for(int y = scan_top; y < scan_bottom; ++y)
    {
        if(!old_left_sign_at(y - 2)) continue;
        if(!old_left_sign_at(y - 1)) continue;
        if(!old_left_sign_at(y))     continue;
        if(!old_left_sign_at(y + 1)) continue;
        if(!old_left_sign_at(y + 2)) continue;

        int x0  = old_left_pos_at(y);
        int xu1 = old_left_pos_at(y - 1);
        int xu2 = old_left_pos_at(y - 2);
        int xd1 = old_left_pos_at(y + 1);
        int xd2 = old_left_pos_at(y + 2);

        bool upper_vertical_ok =
            (iabs_local(xu1 - x0) < OLD_RU_SIMPLE_X_DIFF_MAX) &&
            (iabs_local(xu2 - x0) < OLD_RU_SIMPLE_X_DIFF_MAX);

        bool lower_left_shift_ok =
            ((x0 - xd1) > OLD_RU_SIMPLE_RIGHT_SHIFT_MIN) &&
            ((x0 - xd2) > OLD_RU_SIMPLE_RIGHT_SHIFT_MIN);

        if(upper_vertical_ok && lower_left_shift_ok)
        {
            s_left_round_up_point = y;
            s_left_round_up_point_x = x0;
            s_left_round_up_hold_cnt = 0U;
            return y;
        }
    }

    if(s_left_round_up_point > 0)
    {
        if(s_left_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
        {
            s_left_round_up_hold_cnt++;
        }

        return s_left_round_up_point;
    }

    return 0;
}




//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断左环上、下角点组合是否满足几何约束。
// 使用方法     左环入口确认时调用，防止角点距离或方向异常造成误判。
// 接口属性     内部接口；参数形式：old_left_circle_two_corner_ok(int up_y, int down_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_left_circle_two_corner_ok(int up_y, int down_y)
{
    if(up_y <= 0 || down_y <= 0) return 0U;
    if(up_y >= down_y) return 0U;

    int dy = down_y - up_y;
    if(dy < OLD_LC_DY_MIN || dy > OLD_LC_DY_MAX) return 0U;

    if(!old_left_sign_at(up_y)) return 0U;
    if(!old_left_sign_at(down_y)) return 0U;

    int up_x = old_left_pos_at(up_y);
    int down_x = old_left_pos_at(down_y);

    if(iabs_local(down_x - up_x) < OLD_LC_X_GAP_MIN) return 0U;

    return 1U;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     按旧算法搜索左侧突变点。
// 使用方法     旧环岛或遗留逻辑内部使用，用于兼容原角点判断。
// 接口属性     内部接口；参数形式：old_left_point(int start, int end, int highValue, int lowValue)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_left_point(int start, int end, int highValue, int lowValue)
{
    if(start > old_row_begin()) start = old_row_begin();
    if(end < 10) end = 10;
    int i = 0;
    bool upSure = false, downSure = false;
    for(i = end; i < start - 1; ++i)
    {
        if(old_left_pos_at(i) - old_left_pos_at(i + 2) > highValue) { upSure = true; break; }
    }
    for(int j = start; j > i + 1; --j)
    {
        if(old_left_pos_at(j) - old_left_pos_at(j - 2) > lowValue) { downSure = true; break; }
    }
    if(upSure && downSure) return 1U;
    if(!upSure && !downSure) return 0U;
    int begin = CLAMP(start, end, 90);
    return old_judge_slope(begin, end, s_old_right_pos, s_old_right_sign, 0.7f, 2.3f) ? 1U : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     按旧算法搜索右侧突变点。
// 使用方法     旧环岛或遗留逻辑内部使用，用于兼容原角点判断。
// 接口属性     内部接口；参数形式：old_right_point(int start, int end, int highValue, int lowValue)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_point(int start, int end, int highValue, int lowValue)
{
    if(start > old_row_begin()) start = old_row_begin();
    if(end < 10) end = 10;
    int i = 0;
    bool upSure = false, downSure = false;
    for(i = end; i < start - 1; ++i)
    {
        if(old_right_pos_at(i + 2) - old_right_pos_at(i) > highValue) { upSure = true; break; }
    }
    for(int j = start; j > i + 1; --j)
    {
        if(old_right_pos_at(j - 2) - old_right_pos_at(j) > lowValue) { downSure = true; break; }
    }
    if(upSure && downSure) return 1U;
    if(!upSure && !downSure) return 0U;
    return old_judge_slope(start, end, s_old_left_pos, s_old_left_sign, -3.0f, 2.3f) ? 1U : 0U;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     计算两行边界点之间的斜率并判断是否满足阈值。
// 使用方法     旧环岛入口和局部斜边验证中调用。
// 接口属性     内部接口；参数形式：old_judge_slope_two_rows(int y_low, int y_high, const int *pos, const uint8 *sign, float thr, float delta)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_judge_slope_two_rows(int y_low,
                                      int y_high,
                                      const int *pos,
                                      const uint8 *sign,
                                      float thr,
                                      float delta)
{
    if(y_low <= y_high) return 0U;

    if(y_low >= UVC_HEIGHT) y_low = UVC_HEIGHT - 1;
    if(y_high < 5) y_high = 5;

    if(!old_row_valid(y_low) || !old_row_valid(y_high)) return 0U;
    if(!sign[y_low] || !sign[y_high]) return 0U;

    float dx = (float)(pos[y_low] - pos[y_high]);
    if(fabsf(dx) < 1e-4f) return 0U;

    float k = (float)(y_low - y_high) / dx;

    return (k > thr && k < thr + delta) ? 1U : 0U;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证右下角点上方过渡是否合理。
// 使用方法     右环下角点候选确认时调用，降低误判。
// 接口属性     内部接口；参数形式：old_right_down_corner_upper_transition_ok(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_down_corner_upper_transition_ok(int y)
{
    if(y <= 4) return 0U;
    if(!old_row_valid(y)) return 0U;
    if(!old_right_sign_at(y)) return 0U;

    int x0 = old_right_pos_at(y);

    // 真正的右下角点应当是“下方连续边界”的上端：
    // 它的上方不能继续保持同一条稳定竖直边。
    // 允许两类情况：上方丢线；或者上方相对当前点已经明显偏离/进入斜线段。
    int lost_cnt = 0;
    int above_same_cnt = 0;
    bool up_far = false;

    for(int k = 1; k <= 4; ++k)
    {
        if(!old_row_valid(y - k) || !old_right_sign_at(y - k))
        {
            lost_cnt++;
            continue;
        }

        int d = iabs_local(x0 - old_right_pos_at(y - k));
        if(d <= OLD_RD_STABLE_X_MAX)
        {
            above_same_cnt++;
        }
        if(d >= OLD_CIRCLE_UP_FAR_MIN)
        {
            up_far = true;
        }
    }

    if(lost_cnt >= 2) return 1U;
    if(up_far) return 1U;

    // 上方有效但不再和当前点保持同 x，也可认为是稳定段起点。
    // 这比只看 OLD_CIRCLE_UP_FAR_MIN 更适合“斜线接稳定线”的下角点。
    if(above_same_cnt <= 1) return 1U;

    return 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     原始搜索右下角点候选行。
// 使用方法     旧右环逻辑中调用，只负责候选定位。
// 接口属性     内部接口；参数形式：old_find_right_down_corner_raw(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_right_down_corner_raw(int start, int end)
{
    if(start > old_row_begin()) start = old_row_begin();
    if(end < 8) end = 8;

    for(int y = start - OLD_CIRCLE_DOWN_CONTINUE_N; y > end + 8; --y)
    {
        // 下角点必须在图像相对靠下区域，避免被上方短线段吸走
        if(y < OLD_RC_DOWN_Y_MIN) break;

        if(!old_right_sign_at(y)) continue;

        // 1. 下方必须有较长连续右边界
        uint8 down_ok = 1U;
        for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
        {
            if(!old_right_sign_at(y + k))
            {
                down_ok = 0U;
                break;
            }
        }
        if(!down_ok) continue;

        int x0 = old_right_pos_at(y);

        // 2. 当前点必须和下方长边界整体接近
        int max_down_diff = 0;
        for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
        {
            int d = iabs_local(x0 - old_right_pos_at(y + k));
            if(d > max_down_diff) max_down_diff = d;
        }
        if(max_down_diff > OLD_CIRCLE_DOWN_FIT_MAX) continue;

        // 3. 下方边界本身必须平滑，防止短碎边误判
        uint8 down_smooth = 1U;
        for(int k = 1; k < OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
        {
            int d = iabs_local(old_right_pos_at(y + k) - old_right_pos_at(y + k + 1));
            if(d > OLD_CIRCLE_DOWN_SMOOTH_MAX)
            {
                down_smooth = 0U;
                break;
            }
        }
        if(!down_smooth) continue;

        // 4. 上方必须有“连续段起点”的特征，不能把普通连续右边界当下角点。
        if(old_right_down_corner_upper_transition_ok(y))
        {
            return y;
        }
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断当前右下角点候选是否即时有效。
// 使用方法     右环入口和 PREP 缓存刷新前调用。
// 接口属性     内部接口；参数形式：old_right_down_corner_valid_now(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_down_corner_valid_now(int y)
{
    if(y <= 0) return 0U;
    if(y < OLD_RC_DOWN_Y_MIN) return 0U;
    if(y + OLD_CIRCLE_DOWN_CONTINUE_N >= UVC_HEIGHT) return 0U;
    if(!old_row_valid(y)) return 0U;
    if(!old_right_sign_at(y)) return 0U;

    // 1. 下方必须连续有效
    for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
    {
        if(!old_row_valid(y + k)) return 0U;
        if(!old_right_sign_at(y + k)) return 0U;
    }

    int x0 = old_right_pos_at(y);

    // 2. 当前点和下方连续段不能差太远
    int max_down_diff = 0;
    for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
    {
        int d = iabs_local(x0 - old_right_pos_at(y + k));
        if(d > max_down_diff) max_down_diff = d;
    }

    if(max_down_diff > OLD_CIRCLE_DOWN_FIT_MAX) return 0U;

    // 3. 下方连续段本身要平滑
    for(int k = 1; k < OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
    {
        int d = iabs_local(old_right_pos_at(y + k) - old_right_pos_at(y + k + 1));
        if(d > OLD_CIRCLE_DOWN_SMOOTH_MAX) return 0U;
    }

    // 4. 还必须保留“上方丢线/跳变”的下角点特征。
    // 仅有下方连续边界不够，否则普通右边界中段也会被显示成 RD。
    if(!old_right_down_corner_upper_transition_ok(y)) return 0U;

    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     搜索稳定的右下角点。
// 使用方法     右环入口判断中调用，可结合右上角点约束位置。
// 接口属性     内部接口；参数形式：old_find_right_down_corner_stable(int start, int end, int up_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_right_down_corner_stable(int start, int end, int up_y)
{
    int cand = old_find_right_down_corner_raw(start, end);

    // 如果已经找到 RU，则 RD 必须落在 RU 右下方的合理纵向范围内。
    // 这一步是为了避免 RU 在 6 行附近时，RD 却跳到 100 行这种明显不成对的情况。
    if(up_y > 0)
    {
        if(cand > 0)
        {
            int dy = cand - up_y;
            if(dy < OLD_RC_DY_MIN || dy > OLD_RC_DY_MAX)
            {
                cand = 0;
            }
        }

        if(cand <= 0 &&
           s_right_round_point_row >= OLD_RC_DOWN_Y_MIN &&
           old_right_down_corner_valid_now(s_right_round_point_row))
        {
            int dy_cache = s_right_round_point_row - up_y;
            if(dy_cache >= OLD_RC_DY_MIN && dy_cache <= OLD_RC_DY_MAX)
            {
                return s_right_round_point_row;
            }
        }

        if(cand <= 0)
        {
            return 0;
        }
    }
    else
    {
        // 当前帧没找到：只有缓存点在当前帧仍满足完整下角点特征时才沿用。
        // 不能只看行号，否则普通右边界连续段会长期污染 RD。
        if(cand <= 0)
        {
            if(s_right_round_point_row >= OLD_RC_DOWN_Y_MIN &&
               old_right_down_corner_valid_now(s_right_round_point_row))
            {
                return s_right_round_point_row;
            }
            return 0;
        }
    }

    // 防止候选下角点相对上一帧突然向上跳很多。
    if(s_right_round_point_row >= OLD_RC_DOWN_Y_MIN &&
       old_right_down_corner_valid_now(s_right_round_point_row))
    {
        int diff = cand - s_right_round_point_row;
        if(diff < -OLD_CIRCLE_DOWN_JUMP_MAX)
        {
            return s_right_round_point_row;
        }
    }

    return cand;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在右边界恢复段中搜索右上角点。
// 使用方法     右环入口和补线阶段调用，用于确定上端补线点。
// 接口属性     内部接口；参数形式：old_find_right_up_corner_in_recover_segment(int start, int end, int right_down_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_right_up_corner_in_recover_segment(int start,
                                                       int end,
                                                       int right_down_y)
{
    (void)start;
    (void)end;
    (void)right_down_y;

    /*
     * 右环 RU 简化判据：
     * 从图像最上方往下搜到图像中部。
     *
     * 对候选行 y：
     * 1. y-2、y-1、y 都必须有右边界；
     * 2. y-2、y-1 的 x 与 y 行 x 的差值都小于 5，说明上方是近似竖直短红线；
     * 3. y+1、y+2 都必须有右边界；
     * 4. y+1、y+2 的 x 相对 y 行 x 都大于 5，说明下方开始向右斜出；
     * 5. 满足后，直接把 y 行右边界点作为 RU，并写入缓存；
     * 6. 当前帧没搜到，就沿用上一帧缓存的 RU。
     */

    int scan_top = 2;
    int scan_bottom = UVC_HEIGHT / 2;

    // 保证 y + 2 不越界
    if(scan_bottom > UVC_HEIGHT - 3)
    {
        scan_bottom = UVC_HEIGHT - 3;
    }

    for(int y = scan_top; y < scan_bottom; ++y)
    {
        if(!old_right_sign_at(y - 2)) continue;
        if(!old_right_sign_at(y - 1)) continue;
        if(!old_right_sign_at(y))     continue;
        if(!old_right_sign_at(y + 1)) continue;
        if(!old_right_sign_at(y + 2)) continue;

        int x0  = old_right_pos_at(y);
        int xu1 = old_right_pos_at(y - 1);
        int xu2 = old_right_pos_at(y - 2);
        int xd1 = old_right_pos_at(y + 1);
        int xd2 = old_right_pos_at(y + 2);

        // 上方两行与候选行基本同 x：认为还在短红线竖直段
        bool upper_vertical_ok =
            (iabs_local(xu1 - x0) < OLD_RU_SIMPLE_X_DIFF_MAX) &&
            (iabs_local(xu2 - x0) < OLD_RU_SIMPLE_X_DIFF_MAX);

        // 下方两行相对候选行明显向右：认为开始进入右下斜线段
        bool lower_right_shift_ok =
            ((xd1 - x0) > OLD_RU_SIMPLE_RIGHT_SHIFT_MIN) &&
            ((xd2 - x0) > OLD_RU_SIMPLE_RIGHT_SHIFT_MIN);

        if(upper_vertical_ok && lower_right_shift_ok)
        {
            s_right_round_up_point = y;
            s_right_round_up_point_x = x0;
            s_right_round_up_hold_cnt = 0U;

            return y;
        }
    }

    // 当前帧没搜到：如果之前搜到过，就继续沿用上一帧 RU
    if(s_right_round_up_point > 0)
    {
        if(s_right_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
        {
            s_right_round_up_hold_cnt++;
        }

        return s_right_round_up_point;
    }

    return 0;
}



// 右环 ENTER 阶段专用：当前帧重新寻找补线上端点，不使用 PREP/RU 缓存。
// 逻辑：从上往下找右边界出现明显跳变的第一处；
//      取“跳变前这一行”的右边界点作为补线上端点。
// 保护：如果新点相对上一帧 s_last_point[3] 跳变太大，则沿用上一帧点。
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在右环 ENTER 阶段搜索当前帧补线上端点。
// 使用方法     右环 ENTER 补线时调用，找不到时由缓存/兜底逻辑处理。
// 接口属性     内部接口；参数形式：old_find_right_enter_up_corner_current(int *out_x)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_right_enter_up_corner_current(int *out_x)
{
    if(out_x == NULL) return 0;
    *out_x = 0;

    const int ENTER_UP_JUMP_MIN      = 6;    // 下一行相对本行向右跳超过这个值，认为出现跳变
    const int ENTER_UP_NEAR_MAX      = 5;    // 上两行与本行接近，保证本行之前边界较稳定
    const int ENTER_UP_REFRESH_X_MAX = 12;   // 帧间 x 最大允许刷新幅度
    const int ENTER_UP_REFRESH_Y_MAX = 18;   // 帧间 y 最大允许刷新幅度

    int scan_top = old_search_end() + 2;
    if(scan_top < 2) scan_top = 2;

    int scan_bottom = UVC_HEIGHT / 2;
    if(scan_bottom > old_row_begin() - 2)
    {
        scan_bottom = old_row_begin() - 2;
    }

    for(int y = scan_top; y < scan_bottom; ++y)
    {
        if(!old_row_valid(y - 2)) continue;
        if(!old_row_valid(y - 1)) continue;
        if(!old_row_valid(y))     continue;
        if(!old_row_valid(y + 1)) continue;

        if(!old_right_sign_at(y - 2)) continue;
        if(!old_right_sign_at(y - 1)) continue;
        if(!old_right_sign_at(y))     continue;
        if(!old_right_sign_at(y + 1)) continue;

        int x0  = old_right_pos_at(y);
        int xu1 = old_right_pos_at(y - 1);
        int xu2 = old_right_pos_at(y - 2);
        int xd1 = old_right_pos_at(y + 1);

        /*
         * 上两行和本行接近：说明跳变前的右边界比较稳定；
         * 下一行相对本行明显向右：说明从这里开始出现右边界跳变；
         * 所以取本行右边界点 (x0, y) 作为 ENTER 补线上端点。
         */
        if((iabs_local(xu1 - x0) <= ENTER_UP_NEAR_MAX) &&
           (iabs_local(xu2 - x0) <= ENTER_UP_NEAR_MAX) &&
           ((xd1 - x0) >= ENTER_UP_JUMP_MIN))
        {
            /*
             * 帧间保护：
             * 如果上一帧已有 ENTER 上端点，新点不能突然跳太远。
             * 否则继续沿用上一帧，避免补线抖动。
             */
            if(s_last_point[3][1] > 0)
            {
                int last_x = s_last_point[3][0];
                int last_y = s_last_point[3][1];

                if((iabs_local(x0 - last_x) > ENTER_UP_REFRESH_X_MAX) ||
                   (iabs_local(y  - last_y) > ENTER_UP_REFRESH_Y_MAX))
                {
                    *out_x = last_x;
                    return last_y;
                }
            }

            *out_x = x0;
            return y;
        }
    }

    // 当前帧没找到跳变点：沿用上一帧 ENTER 上端点。
    if(s_last_point[3][1] > 0)
    {
        *out_x = s_last_point[3][0];
        return s_last_point[3][1];
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据左上角点约束搜索对应的左下角点。
// 使用方法     左环入口判断中调用，保证上下角点属于同一个环岛结构。
// 接口属性     内部接口；参数形式：old_find_left_down_corner_from_up(int start, int end, int up_y, int up_x)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_left_down_corner_from_up(int start,
                                             int end,
                                             int up_y,
                                             int up_x)
{
    if(up_y <= 0) return 0;
    if(up_x <= 0) return 0;

    if(start > old_row_begin()) start = old_row_begin();
    if(end < 5) end = 5;

    int scan_top = up_y + OLD_RD_FROM_RU_DY_MIN;
    int scan_bottom = up_y + OLD_RD_FROM_RU_DY_MAX;

    if(scan_top < OLD_LC_DOWN_Y_MIN) scan_top = OLD_LC_DOWN_Y_MIN;
    if(scan_bottom > start - OLD_RD_STABLE_N - 1)
    {
        scan_bottom = start - OLD_RD_STABLE_N - 1;
    }

    if(scan_bottom <= scan_top) return 0;

    for(int y = scan_top; y <= scan_bottom; ++y)
    {
        if(!old_left_sign_at(y)) continue;

        int x0 = old_left_pos_at(y);
        int dx = up_x - x0;

        if(dx < OLD_RD_FROM_RU_X_MIN) continue;
        if(dx > OLD_RD_FROM_RU_X_MAX) continue;

        uint8 stable = 1U;
        for(int k = 1; k <= OLD_RD_STABLE_N; ++k)
        {
            if(!old_left_sign_at(y + k))
            {
                stable = 0U;
                break;
            }

            int xk = old_left_pos_at(y + k);
            if(iabs_local(xk - x0) > OLD_RD_STABLE_X_MAX)
            {
                stable = 0U;
                break;
            }
        }
        if(!stable) continue;

        int above_same = 0;
        for(int k = 1; k <= 3; ++k)
        {
            if(old_left_sign_at(y - k) &&
               iabs_local(old_left_pos_at(y - k) - x0) <= OLD_RD_STABLE_X_MAX)
            {
                above_same++;
            }
        }
        if(above_same >= 2) continue;

        return y;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     在左环 ENTER 阶段搜索当前帧补线上端点。
// 使用方法     左环 ENTER 补线时调用，找不到时由缓存/兜底逻辑处理。
// 接口属性     内部接口；参数形式：old_find_left_enter_up_corner_current(int *out_x)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_left_enter_up_corner_current(int *out_x)
{
    if(out_x == NULL) return 0;
    *out_x = 0;

    const int ENTER_UP_JUMP_MIN      = 6;
    const int ENTER_UP_NEAR_MAX      = 5;
    const int ENTER_UP_REFRESH_X_MAX = 12;
    const int ENTER_UP_REFRESH_Y_MAX = 18;

    int scan_top = old_search_end() + 2;
    if(scan_top < 2) scan_top = 2;

    int scan_bottom = UVC_HEIGHT / 2;
    if(scan_bottom > old_row_begin() - 2)
    {
        scan_bottom = old_row_begin() - 2;
    }

    for(int y = scan_top; y < scan_bottom; ++y)
    {
        if(!old_row_valid(y - 2)) continue;
        if(!old_row_valid(y - 1)) continue;
        if(!old_row_valid(y))     continue;
        if(!old_row_valid(y + 1)) continue;

        if(!old_left_sign_at(y - 2)) continue;
        if(!old_left_sign_at(y - 1)) continue;
        if(!old_left_sign_at(y))     continue;
        if(!old_left_sign_at(y + 1)) continue;

        int x0  = old_left_pos_at(y);
        int xu1 = old_left_pos_at(y - 1);
        int xu2 = old_left_pos_at(y - 2);
        int xd1 = old_left_pos_at(y + 1);

        if((iabs_local(xu1 - x0) <= ENTER_UP_NEAR_MAX) &&
           (iabs_local(xu2 - x0) <= ENTER_UP_NEAR_MAX) &&
           ((x0 - xd1) >= ENTER_UP_JUMP_MIN))
        {
            if(s_last_point[2][1] > 0)
            {
                int last_x = s_last_point[2][0];
                int last_y = s_last_point[2][1];

                if((iabs_local(x0 - last_x) > ENTER_UP_REFRESH_X_MAX) ||
                   (iabs_local(y  - last_y) > ENTER_UP_REFRESH_Y_MAX))
                {
                    *out_x = last_x;
                    return last_y;
                }
            }

            *out_x = x0;
            return y;
        }
    }

    if(s_last_point[2][1] > 0)
    {
        *out_x = s_last_point[2][0];
        return s_last_point[2][1];
    }

    return 0;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据右上角点约束搜索对应的右下角点。
// 使用方法     右环入口判断中调用，保证上下角点属于同一个环岛结构。
// 接口属性     内部接口；参数形式：old_find_right_down_corner_from_up(int start, int end, int up_y, int up_x)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int old_find_right_down_corner_from_up(int start,
                                              int end,
                                              int up_y,
                                              int up_x)
{
    if(up_y <= 0) return 0;
    if(up_x <= 0) return 0;

    if(start > old_row_begin()) start = old_row_begin();
    if(end < 5) end = 5;

    int scan_top = up_y + OLD_RD_FROM_RU_DY_MIN;
    int scan_bottom = up_y + OLD_RD_FROM_RU_DY_MAX;

    if(scan_top < OLD_RC_DOWN_Y_MIN) scan_top = OLD_RC_DOWN_Y_MIN;
    if(scan_bottom > start - OLD_RD_STABLE_N - 1)
    {
        scan_bottom = start - OLD_RD_STABLE_N - 1;
    }

    if(scan_bottom <= scan_top) return 0;

    /*
     * RD 不能脱离 RU 单独找。它必须位于 RU 的右下方，且从 RD 往下有一段
     * 稳定连续右边界；同时 RD 上方不能已经是一段同 x 的稳定竖线，避免取到稳定段中部。
     */
    for(int y = scan_top; y <= scan_bottom; ++y)
    {
        if(!old_right_sign_at(y)) continue;

        int x0 = old_right_pos_at(y);
        int dx = x0 - up_x;

        if(dx < OLD_RD_FROM_RU_X_MIN) continue;
        if(dx > OLD_RD_FROM_RU_X_MAX) continue;

        uint8 stable = 1U;
        for(int k = 1; k <= OLD_RD_STABLE_N; ++k)
        {
            if(!old_right_sign_at(y + k))
            {
                stable = 0U;
                break;
            }

            int xk = old_right_pos_at(y + k);
            if(iabs_local(xk - x0) > OLD_RD_STABLE_X_MAX)
            {
                stable = 0U;
                break;
            }
        }
        if(!stable) continue;

        int above_same = 0;
        for(int k = 1; k <= 3; ++k)
        {
            if(old_right_sign_at(y - k) &&
               iabs_local(old_right_pos_at(y - k) - x0) <= OLD_RD_STABLE_X_MAX)
            {
                above_same++;
            }
        }
        if(above_same >= 2) continue;

        return y;
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断当前左下角点候选是否即时有效。
// 使用方法     左环入口和 PREP 缓存刷新前调用。
// 接口属性     内部接口；参数形式：old_left_down_corner_valid_now(int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_left_down_corner_valid_now(int y)
{
    if(y <= 0) return 0U;
    if(y < OLD_LC_DOWN_Y_MIN) return 0U;
    if(y + OLD_CIRCLE_DOWN_CONTINUE_N >= UVC_HEIGHT) return 0U;
    if(!old_row_valid(y)) return 0U;
    if(!old_left_sign_at(y)) return 0U;

    for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
    {
        if(!old_row_valid(y + k)) return 0U;
        if(!old_left_sign_at(y + k)) return 0U;
    }

    int x0 = old_left_pos_at(y);

    int max_down_diff = 0;
    for(int k = 1; k <= OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
    {
        int d = iabs_local(x0 - old_left_pos_at(y + k));
        if(d > max_down_diff) max_down_diff = d;
    }

    if(max_down_diff > OLD_CIRCLE_DOWN_FIT_MAX) return 0U;

    for(int k = 1; k < OLD_CIRCLE_DOWN_CONTINUE_N; ++k)
    {
        int d = iabs_local(old_left_pos_at(y + k) - old_left_pos_at(y + k + 1));
        if(d > OLD_CIRCLE_DOWN_SMOOTH_MAX) return 0U;
    }

    return 1U;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断右环上、下角点组合是否满足几何约束。
// 使用方法     右环入口确认时调用，防止角点距离或方向异常造成误判。
// 接口属性     内部接口；参数形式：old_right_circle_two_corner_ok(int up_y, int down_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_circle_two_corner_ok(int up_y, int down_y)
{
    if(up_y <= 0 || down_y <= 0) return 0U;
    if(up_y >= down_y) return 0U;

    int dy = down_y - up_y;
    if(dy < OLD_RC_DY_MIN || dy > OLD_RC_DY_MAX) return 0U;

    if(!old_right_sign_at(up_y)) return 0U;
    if(!old_right_sign_at(down_y)) return 0U;

    int up_x = old_right_pos_at(up_y);
    int down_x = old_right_pos_at(down_y);

    if(iabs_local(down_x - up_x) < OLD_RC_X_GAP_MIN) return 0U;

    return 1U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证右环右下段边界斜率是否满足入环特征。
// 使用方法     右环入口综合判定中调用。
// 接口属性     内部接口；参数形式：old_right_down_slope_for_right_circle(int start, int right_down_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_down_slope_for_right_circle(int start, int right_down_y)
{
    if(right_down_y <= 0) return 0U;

    int y_low = start - 2;
    if(y_low > old_row_begin()) y_low = old_row_begin();

    // 往上找一个右边界有效的下方点
    while(y_low > right_down_y + 3)
    {
        if(old_right_sign_at(y_low)) break;
        y_low--;
    }

    if(y_low <= right_down_y + 3) return 0U;

    return old_judge_slope_two_rows(
        y_low,
        right_down_y,
        s_old_right_pos,
        s_old_right_sign,
        OLD_RC_RIGHT_SLOPE_THR,
        OLD_RC_RIGHT_SLOPE_DELTA
    );
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证右环稳定侧左边界斜率。
// 使用方法     右环入口综合判定中调用。
// 接口属性     内部接口；参数形式：old_left_slope_for_right_circle(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_left_slope_for_right_circle(int start, int end)
{
    return old_judge_visible_slope_segment(
        start,
        end,
        s_old_left_pos,
        s_old_left_sign,
        OLD_RC_LEFT_SLOPE_THR,
        OLD_RC_LEFT_SLOPE_DELTA,
        OLD_RC_LEFT_VISIBLE_DY_MIN,
        OLD_RC_LEFT_VISIBLE_DY_MAX,
        OLD_RC_LEFT_VISIBLE_HIT_MIN
    );
}

// 右环入口预筛：不依赖 RD，只判断右侧当前可见边界是否存在右下斜出趋势。
// 右环入环早期右侧是丢边侧，固定 5 行斜率判据太苛刻，容易因为某几个采样行丢线而失败。
// 因此这里改为“可见线段斜率”判据：只要可见右边界里有若干段满足斜率范围，就允许继续找 RU/RD。
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证右环早期可见右边界局部斜率。
// 使用方法     右环入口预筛时调用，用于右侧丢线较多的情况。
// 接口属性     内部接口；参数形式：old_right_slope_for_right_circle_pre(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_slope_for_right_circle_pre(int start, int end)
{
    return old_judge_visible_slope_segment(
        start,
        end,
        s_old_right_pos,
        s_old_right_sign,
        OLD_RC_RIGHT_PRE_SLOPE_THR,
        OLD_RC_RIGHT_PRE_SLOPE_DELTA,
        OLD_RC_RIGHT_PRE_DY_MIN,
        OLD_RC_RIGHT_PRE_DY_MAX,
        OLD_RC_RIGHT_PRE_HIT_MIN
    );
}

// 左环入口预筛：不依赖 LD，先判断左侧当前可见边界整体斜率。
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证左环早期可见左边界局部斜率。
// 使用方法     左环入口预筛时调用，逻辑镜像右环预筛。
// 接口属性     内部接口；参数形式：old_left_slope_for_left_circle_pre(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_left_slope_for_left_circle_pre(int start, int end)
{
    return old_judge_visible_slope_segment(
        start,
        end,
        s_old_left_pos,
        s_old_left_sign,
        OLD_LC_LEFT_SLOPE_THR,
        OLD_LC_LEFT_SLOPE_DELTA,
        OLD_RC_RIGHT_PRE_DY_MIN,
        OLD_RC_RIGHT_PRE_DY_MAX,
        OLD_RC_RIGHT_PRE_HIT_MIN
    );
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证左环稳定侧右边界斜率。
// 使用方法     左环入口综合判定中调用。
// 接口属性     内部接口；参数形式：old_right_slope_for_left_circle(int start, int end)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_right_slope_for_left_circle(int start, int end)
{
    return old_judge_slope(
        start,
        end,
        s_old_right_pos,
        s_old_right_sign,
        OLD_LC_RIGHT_SLOPE_THR,
        OLD_LC_RIGHT_SLOPE_DELTA
    ) ? 1U : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     验证左环左下段边界斜率是否满足入环特征。
// 使用方法     左环入口综合判定中调用。
// 接口属性     内部接口；参数形式：old_left_down_slope_for_left_circle(int start, int left_down_y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 old_left_down_slope_for_left_circle(int start, int left_down_y)
{
    if(left_down_y <= 0) return 0U;

    int y_low = start - 2;
    if(y_low > old_row_begin()) y_low = old_row_begin();

    // 往上找一个左边界有效的下方点
    while(y_low > left_down_y + 3)
    {
        if(old_left_sign_at(y_low)) break;
        y_low--;
    }

    if(y_low <= left_down_y + 3) return 0U;

    return old_judge_slope_two_rows(
        y_low,
        left_down_y,
        s_old_left_pos,
        s_old_left_sign,
        OLD_LC_LEFT_SLOPE_THR,
        OLD_LC_LEFT_SLOPE_DELTA
    );
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     运行旧版环岛入口判定并决定是否进入左/右环状态。
// 使用方法     每帧元素状态更新前调用，满足角点、斜率和丢线条件后切换环岛状态。
// 接口属性     内部接口；参数形式：road_is_roundabout_old(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static uint8 road_is_roundabout_old(void)
{
    s_circle_debug_code = 0U;
    s_left_circle_point = 0U;
    s_right_circle_point = 0U;
    s_left_state = 0U;
    s_right_state = 0U;
    s_l_k_state = 0U;
    s_r_k_state = 0U;

    s_dbg_right_down_y_current = 0;
    s_dbg_right_up_y_current = 0;
    s_dbg_right_two_corner_ok_current = 0U;
    s_dbg_rc_left_slope_ok_current = 0U;
    s_dbg_rc_right_slope_ok_current = 0U;
    s_dbg_rc_left_straight_ok_current = 0U;
    s_dbg_rc_right_straight_ok_current = 0U;
    s_dbg_left_down_y_current = 0;
    s_dbg_left_up_y_current = 0;
    s_dbg_left_two_corner_ok_current = 0U;
    s_dbg_lc_right_slope_ok_current = 0U;
    s_dbg_lc_left_slope_ok_current = 0U;

    // 右环新判据变量
    int right_down_y = 0;
    int right_up_y = 0;
    uint8 right_up_ok = 0U;
    uint8 right_two_corner_ok = 0U;
    uint8 left_slope_ok = 0U;
    uint8 left_straight_ok = 0U;       // 右环稳定侧左边界整体直线性
    uint8 right_slope_ok = 0U;       // 右环入口右侧可见斜率，控制是否允许找 RD
    uint8 right_down_slope_ok = 0U;  // 找到 RD 后的下段斜率复核，只用于最终确认

    // 左环新判据变量
    int left_down_y = 0;
    int left_up_y = 0;
    uint8 left_up_ok = 0U;
    uint8 left_two_corner_ok = 0U;
    uint8 right_straight_ok = 0U;
    uint8 lc_right_slope_ok = 0U;
    uint8 lc_left_down_slope_ok = 0U;

    int rowEnd = (old_search_end() <= 10) ? 10 : old_search_end();
    int start = old_row_begin() - 10;
    if(start < rowEnd + 5) start = rowEnd + 5;
    int left_slope_start = old_row_begin();

// 左环判定分支：先判断左右斜率，两个斜率都过了才允许寻找左下角点。
if((s_old_right_lost_count < OLD_CIRCLE_OTHER_LOST_MAX) &&
   (s_old_left_lost_count > OLD_CIRCLE_LEFT_LOST_MIN))
{
    // 1. 镜像右环：先判断右边稳定边斜率、直线性，以及左侧入口斜率。
    lc_right_slope_ok = old_right_slope_for_left_circle(start, rowEnd);
    right_straight_ok = old_is_line_straight_and_continuous(
        s_old_right_pos,
        s_old_right_sign,
        left_slope_start,
        rowEnd
    );
    lc_left_down_slope_ok = old_left_slope_for_left_circle_pre(start, rowEnd);

    s_dbg_lc_right_slope_ok_current = lc_right_slope_ok;
    s_dbg_lc_left_slope_ok_current = lc_left_down_slope_ok;

    if((lc_right_slope_ok != 0U) &&
       (right_straight_ok != 0U) &&
       (lc_left_down_slope_ok != 0U))
    {
        // 2. 两侧预筛满足后，先找 LU，再优先用 LU 约束 LD。
        left_up_y = old_find_left_up_corner_in_recover_segment(
            start,
            rowEnd,
            0
        );

        left_down_y = 0;
        if((left_up_y > 0) && old_left_sign_at(left_up_y))
        {
            left_down_y = old_find_left_down_corner_from_up(
                start,
                rowEnd,
                left_up_y,
                old_left_pos_at(left_up_y)
            );
        }

        if(left_down_y <= 0)
        {
            left_down_y = old_find_left_down_corner_stable(start, rowEnd, left_up_y);
        }

        // 3. 判断两角点是否合法。
        left_two_corner_ok = old_left_circle_two_corner_ok(
            left_up_y,
            left_down_y
        );

        // 4. 找到 LD 后，再用 LD 到下方有效边界做一次更精确的左下段斜率确认。
        lc_left_down_slope_ok = old_left_down_slope_for_left_circle(
            start,
            left_down_y
        );
        s_dbg_lc_left_slope_ok_current = lc_left_down_slope_ok;

        // 5. 只有合法 LD 才缓存；LU 只有在与 LD 合法配对后才缓存给 PREP 补线。
        if((left_down_y > 0) && (left_up_y > 0) &&
           (left_up_y < left_down_y) &&
           old_left_down_corner_valid_now(left_down_y))
        {
            s_left_round_point_row = left_down_y;
            s_left_round_point = old_left_pos_at(left_down_y);

            int lu_x = old_left_pos_at(left_up_y);
            int ld_x = old_left_pos_at(left_down_y);
            int dy = left_down_y - left_up_y;
            int dx = lu_x - ld_x;

            bool lu_cache_ok =
                (left_up_y <= rowEnd + OLD_RU_PREP_MAX_OFFSET_FROM_SEARCH_END) &&
                (dy >= OLD_RU_PREP_DY_MIN) &&
                (dy <= OLD_RU_PREP_DY_MAX) &&
                (dx >= OLD_RU_PREP_X_GAP_MIN);

            if(lu_cache_ok)
            {
                s_left_round_up_point = left_up_y;
                s_left_round_up_point_x = lu_x;
                s_left_round_up_hold_cnt = 0U;
            }
        }
        else
        {
            if((left_down_y > 0) &&
               old_left_down_corner_valid_now(left_down_y))
            {
                s_left_round_point_row = left_down_y;
                s_left_round_point = old_left_pos_at(left_down_y);
            }
        }
    }

    s_dbg_left_down_y_current = left_down_y;
    s_dbg_left_up_y_current = left_up_y;
    s_dbg_left_two_corner_ok_current = left_two_corner_ok;

    // 8. 调试码：21~26 对应左环入口各阶段。
    //    注意：斜率不过时，LD 保持 0，屏幕/上位机不会标假下角点。
    if(!lc_right_slope_ok)              s_circle_debug_code = 24U;
    else if(!right_straight_ok || !lc_left_down_slope_ok)
                                        s_circle_debug_code = 25U;
    else if(!left_down_y)               s_circle_debug_code = 21U;
    else if(!left_up_y)                 s_circle_debug_code = 22U;
    else if(!left_two_corner_ok)        s_circle_debug_code = 23U;
    else if(s_old_search_stop_line > OLD_CIRCLE_SEARCH_STOP_MIN)
                                        s_circle_debug_code = 26U;
}

// 右环判定分支：先判断左右斜率，两个斜率都过了才允许寻找右下角点。
if((s_old_left_lost_count < OLD_CIRCLE_OTHER_LOST_MAX) &&
   (s_old_right_lost_count > OLD_CIRCLE_RIGHT_LOST_MIN))
{
    // 1. 先判断左边稳定边斜率和右侧入口斜率。
    //    这里不依赖 RD；只有左右斜率满足环岛入口特征，才继续找 RU/RD。
    // 左边界斜率和直线性检查覆盖图像最底部到最长白列到达的行。
    left_slope_ok = old_left_slope_for_right_circle(left_slope_start, rowEnd);
    left_straight_ok = old_is_line_straight_and_continuous(
        s_old_left_pos,
        s_old_left_sign,
        left_slope_start,
        rowEnd
    );
    s_dbg_rc_left_straight_ok_current = left_straight_ok;
    s_dbg_rc_right_straight_ok_current = old_is_line_straight_and_continuous(
        s_old_right_pos,
        s_old_right_sign,
        left_slope_start,
        rowEnd
    );
    right_slope_ok = old_right_slope_for_right_circle_pre(start, rowEnd);

    s_dbg_rc_left_slope_ok_current = left_slope_ok;
    s_dbg_rc_right_slope_ok_current = right_slope_ok;

    if((left_slope_ok != 0U) &&
       (left_straight_ok != 0U) &&
       (right_slope_ok != 0U))
    {
        // 2. 两侧斜率满足后，才开始找右上角点 RU。
        right_up_y = old_find_right_up_corner_in_recover_segment(
            start,
            rowEnd,
            0
        );

        // 3. 如果 RU 当前帧有效，则优先用 RU 约束 RD：RD 必须在 RU 的右下方。
        right_down_y = 0;
        if((right_up_y > 0) && old_right_sign_at(right_up_y))
        {
            right_down_y = old_find_right_down_corner_from_up(
                start,
                rowEnd,
                right_up_y,
                old_right_pos_at(right_up_y)
            );
        }

        // 4. RU 没找到或由 RU 没能约束出 RD 时，再退回独立 RD 判据。
        if(right_down_y <= 0)
        {
            right_down_y = old_find_right_down_corner_stable(start, rowEnd, right_up_y);
        }

        // 5. two_corner 是当前帧 RU/RD 配对结果，只用于调试显示和 RU 缓存。
        right_two_corner_ok = old_right_circle_two_corner_ok(
            right_up_y,
            right_down_y
        );

        // 6. 找到 RD 后，再用 RD 到下方有效边界做一次更精确的右下段斜率确认。
        // 注意：这里不再覆盖 right_slope_ok，也不再覆盖屏幕上的 Rrs。
        // Rrs 表示“是否允许开始找 RD 的右侧入口斜率”；right_down_slope_ok 只参与最终确认。
        right_down_slope_ok = old_right_down_slope_for_right_circle(
            start,
            right_down_y
        );

        // 7. 只有合法 RD 才缓存。RU 只有在与 RD 合法配对后才缓存给 PREP 补线。
        if((right_down_y > 0) &&
           old_right_down_corner_valid_now(right_down_y))
        {
            s_right_round_point_row = right_down_y;
            s_right_round_point = old_right_pos_at(right_down_y);
            s_right_round_down_hold_cnt = 0U;

            if((right_up_y > 0) &&
               (right_up_y < right_down_y) &&
               (right_two_corner_ok != 0U) &&
               old_row_valid(right_up_y) &&
               old_right_sign_at(right_up_y))
            {
                int ru_x = old_right_pos_at(right_up_y);
                int rd_x = old_right_pos_at(right_down_y);
                int dy = right_down_y - right_up_y;
                int dx = rd_x - ru_x;

                bool ru_cache_ok =
                    (right_up_y <= rowEnd + OLD_RU_PREP_MAX_OFFSET_FROM_SEARCH_END) &&
                    (dy >= OLD_RU_PREP_DY_MIN) &&
                    (dy <= OLD_RU_PREP_DY_MAX) &&
                    (dx >= OLD_RU_PREP_X_GAP_MIN);

                if(ru_cache_ok)
                {
                    s_right_round_up_point = right_up_y;
                    s_right_round_up_point_x = ru_x;
                    s_right_round_up_hold_cnt = 0U;
                }
            }
        }
    }

    s_dbg_right_down_y_current = right_down_y;
    s_dbg_right_up_y_current = right_up_y;
    s_dbg_right_two_corner_ok_current = right_two_corner_ok;

    // 8. 调试码：右环入口各阶段。
    //    注意：斜率不过时，RD 保持 0，屏幕/上位机不会标假下角点。
    if(!left_slope_ok)                 s_circle_debug_code = 14U;
    else if(!right_slope_ok)           s_circle_debug_code = 15U;
    else if(!right_down_y)             s_circle_debug_code = 11U;
    else if(!old_right_down_corner_valid_now(right_down_y))
                                      s_circle_debug_code = 13U;
    else if(s_old_search_stop_line > OLD_CIRCLE_SEARCH_STOP_MIN)
                                      s_circle_debug_code = 16U;
}
    s_circle_candidate = 0U;

    // 当前帧 / 缓存 RU 有效性：右上角点必须在右下角点上方
    right_up_ok = 0U;
    if((right_up_y > 0) &&
       old_row_valid(right_up_y) &&
       old_right_sign_at(right_up_y) &&
       (right_down_y > 0) &&
       (right_up_y < right_down_y))
    {
        right_up_ok = 1U;
    }
    else if((s_right_round_up_point > 0) &&
            old_row_valid(s_right_round_up_point) &&
            (right_down_y > 0) &&
            (s_right_round_up_point < right_down_y))
    {
        right_up_ok = 1U;
    }

    // 当前帧 / 缓存 LU 有效性：左上角点必须在左下角点上方
    left_up_ok = 0U;
    if((left_up_y > 0) &&
       old_row_valid(left_up_y) &&
       old_left_sign_at(left_up_y) &&
       (left_down_y > 0) &&
       (left_up_y < left_down_y))
    {
        left_up_ok = 1U;
    }
    else if((s_left_round_up_point > 0) &&
            old_row_valid(s_left_round_up_point) &&
            (left_down_y > 0) &&
            (s_left_round_up_point < left_down_y))
    {
        left_up_ok = 1U;
    }

    // 右环提前入态：RU + RD + 左边线斜率 + 右侧丢线
    uint8 right_round_early_ok = 0U;
    if((right_up_ok != 0U) &&
       (right_down_y > 0) &&
       old_right_down_corner_valid_now(right_down_y) &&
       (left_slope_ok != 0U) &&
       (left_straight_ok != 0U) &&
       (s_old_right_lost_count >= OLD_RC_EARLY_RIGHT_LOST_MIN) &&
       (s_old_left_lost_count <= OLD_CIRCLE_OTHER_LOST_MAX) &&
       (s_old_both_lost_count <= 4))
    {
        right_round_early_ok = 1U;
    }

    bool right_round_strict =
        (right_up_ok != 0U) &&
        (right_down_y > 0) &&
        old_right_down_corner_valid_now(right_down_y) &&
        (left_slope_ok != 0U) &&
        (left_straight_ok != 0U) &&
        ((right_down_slope_ok != 0U) || (right_round_early_ok != 0U)) &&
        (s_old_both_lost_count <= 4);

    // 左环提前入态：LU + LD + 右边线斜率 + 左侧丢线
    uint8 left_round_early_ok = 0U;
    if((left_up_ok != 0U) &&
       (left_down_y > 0) &&
       old_left_down_corner_valid_now(left_down_y) &&
       (lc_right_slope_ok != 0U) &&
       (right_straight_ok != 0U) &&
       (s_old_left_lost_count >= OLD_LC_EARLY_LEFT_LOST_MIN) &&
       (s_old_right_lost_count <= OLD_CIRCLE_OTHER_LOST_MAX) &&
       (s_old_both_lost_count <= 4))
    {
        left_round_early_ok = 1U;
    }

    bool left_round_strict =
        (left_up_ok != 0U) &&
        (left_down_y > 0) &&
        old_left_down_corner_valid_now(left_down_y) &&
        (lc_right_slope_ok != 0U) &&
        (right_straight_ok != 0U) &&
        ((lc_left_down_slope_ok != 0U) || (left_round_early_ok != 0U)) &&
        (s_old_both_lost_count <= 4);

    if(right_round_strict)
    {
        int right_stop_min = (right_down_slope_ok != 0U) ? OLD_CIRCLE_SEARCH_STOP_MIN
                                                         : OLD_CIRCLE_SEARCH_STOP_EARLY_MIN;
        if(s_old_search_stop_line > right_stop_min)
        {
            // 缓存 RD
            s_right_round_point_row = right_down_y;
            s_right_round_point = old_right_pos_at(right_down_y);
            s_right_round_down_hold_cnt = 0U;

            // 缓存 RU，供 PRE 阶段作为补线上端点
            if((right_up_y > 0) &&
               old_row_valid(right_up_y) &&
               old_right_sign_at(right_up_y) &&
               (right_up_y < right_down_y))
            {
                s_right_round_up_point = right_up_y;
                s_right_round_up_point_x = old_right_pos_at(right_up_y);
                s_right_round_up_hold_cnt = 0U;
            }

            s_circle_debug_code = 8U;
            s_circle_candidate = 1U;
            s_circle_lost_side = 2U;
            return 2U;
        }
    }
    else if(left_round_strict)
    {
        int left_stop_min = (lc_left_down_slope_ok != 0U) ? OLD_CIRCLE_SEARCH_STOP_MIN
                                                          : OLD_CIRCLE_SEARCH_STOP_EARLY_MIN;
        if(s_old_search_stop_line > left_stop_min)
        {
            // 缓存 LD
            s_left_round_point_row = left_down_y;
            s_left_round_point = old_left_pos_at(left_down_y);
            s_left_round_down_hold_cnt = 0U;

            // 缓存 LU，供 PRE 阶段作为补线上端点
            if((left_up_y > 0) &&
               old_row_valid(left_up_y) &&
               old_left_sign_at(left_up_y) &&
               (left_up_y < left_down_y))
            {
                int lu_x = old_left_pos_at(left_up_y);
                int ld_x = old_left_pos_at(left_down_y);
                int dy = left_down_y - left_up_y;
                int dx = lu_x - ld_x;

                bool lu_cache_ok =
                    (left_up_y <= rowEnd + OLD_RU_PREP_MAX_OFFSET_FROM_SEARCH_END) &&
                    (dy >= OLD_RU_PREP_DY_MIN) &&
                    (dy <= OLD_RU_PREP_DY_MAX) &&
                    (dx >= OLD_RU_PREP_X_GAP_MIN);

                if(lu_cache_ok)
                {
                    s_left_round_up_point = left_up_y;
                    s_left_round_up_point_x = lu_x;
                    s_left_round_up_hold_cnt = 0U;
                }
            }

            s_circle_debug_code = 9U;
            s_circle_candidate = 1U;
            s_circle_lost_side = 1U;
            return 1U;
        }
    }

    return 0U;
}
// ================================================================================================
// 模板线调试导出
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     清空环岛模板线调试导出数据。
// 使用方法     每帧或环岛重置时调用，避免上帧补线显示残留。
// 接口属性     内部接口；参数形式：circle_template_debug_clear(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_template_debug_clear(void)
{
    s_circle_tpl_valid = 0U;
    s_circle_tpl_side = 0U;
    s_circle_tpl_x1 = s_circle_tpl_y1 = s_circle_tpl_x2 = s_circle_tpl_y2 = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     记录当前环岛补线模板线用于上位机/屏幕调试显示。
// 使用方法     补线线段确定后调用，只保存调试坐标，不直接修改边界。
// 接口属性     内部接口；参数形式：circle_template_debug_set(uint8 side, int x1, int y1, int x2, int y2)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_template_debug_set(uint8 side, int x1, int y1, int x2, int y2)
{
    s_circle_tpl_valid = 1U;
    s_circle_tpl_side  = side;
    s_circle_tpl_x1 = clamp_col(x1);
    s_circle_tpl_y1 = CLAMP(y1, 0, UVC_HEIGHT - 1);
    s_circle_tpl_x2 = clamp_col(x2);
    s_circle_tpl_y2 = CLAMP(y2, 0, UVC_HEIGHT - 1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对外导出当前环岛补线调试信息。
// 使用方法     图像显示或上位机调试模块调用，用于画出环岛补线模板。
// 接口属性     对外接口；参数形式：element_get_circle_draw_debug(element_circle_draw_debug_struct *dbg)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
void element_get_circle_draw_debug(element_circle_draw_debug_struct *dbg)
{
    if(dbg == NULL) return;
    memset(dbg, 0, sizeof(*dbg));

    dbg->valid = s_circle_tpl_valid;
    dbg->state = (uint8)s_circle_state;
    dbg->template_side = s_circle_tpl_side;
    dbg->left_round_point_row  = (int16)s_left_round_point_row;
    dbg->right_round_point_row = (int16)s_right_round_point_row;
    dbg->left_round_up_point   = (int16)s_left_round_up_point;
    dbg->right_round_up_point  = (int16)s_right_round_up_point;
    dbg->template_x1 = (int16)s_circle_tpl_x1;
    dbg->template_y1 = (int16)s_circle_tpl_y1;
    dbg->template_x2 = (int16)s_circle_tpl_x2;
    dbg->template_y2 = (int16)s_circle_tpl_y2;
    for(int i = 0; i < 6; ++i)
    {
        dbg->last_point_x[i] = (int16)s_last_point[i][0];
        dbg->last_point_y[i] = (int16)s_last_point[i][1];
    }
}

// ================================================================================================
// 环岛复位 / 入态 / 状态更新
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     重置环岛状态机、角点缓存和补线缓存。
// 使用方法     环岛结束、误判取消或系统复位时调用。
// 接口属性     内部接口；参数形式：circle_reset_internal(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_reset_internal(void)
{
    s_circle_state = CIRCLE_IDLE;
    s_circle_candidate = 0U;
    s_circle_confirm_frames = 0U;
    s_circle_flag = false;
    s_in_circle = false;
    s_old_in_circle_state = 0U;
    s_circle_yaw_start = 0.0f;
    s_circle_yaw_active = 0U;
    s_old_circle_state_flag = 0U;
    s_circle_lost_side = 0U;
    capture_stage_distance_ref();
    capture_circle_enter_distance_ref();
    s_left_round_point = 0;
    s_left_round_point_row = old_row_begin();
    s_left_round_up_point = 0;
    s_left_round_up_point_x = 0;
    s_right_round_point = USE_WIDTH_R - 1;
    s_right_round_point_row = old_row_begin();
    s_right_round_up_point = 0;
    s_right_round_up_point_x = 0;

s_right_round_down_hold_cnt = 0U;
s_right_round_up_hold_cnt = 0U;
s_right_inside_down_lost_frames = 0U;
s_left_round_down_hold_cnt = 0U;
s_left_round_up_hold_cnt = 0U;

s_right_exit_prep_line_valid = 0U;
s_right_exit_prep_line_x = 0;
s_right_exit_prep_line_y = 0;
s_left_exit_prep_line_valid = 0U;
s_left_exit_prep_line_x = 0;
s_left_exit_prep_line_y = 0;

memset(s_last_point, 0, sizeof(s_last_point));
circle_repair_smooth_clear();
circle_template_debug_clear();


    circle_template_debug_clear();

    if(s_element_mode == ELEMENT_ROUNDABOUT)
    {
        s_element_mode = ELEMENT_NONE;
        s_element_busy = false;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     按旧版环岛状态进入方式切换状态并初始化参考量。
// 使用方法     检测到左环或右环入口后调用。
// 接口属性     内部接口；参数形式：circle_enter_old(uint8 state)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_enter_old(uint8 state)
{
    // 记录进入前的状态，判断这次是不是“第一次进入整个环岛”
    uint8 was_in_circle = s_circle_flag ? 1U : 0U;

    s_circle_state = (circle_state_enum)state;
    s_circle_flag = true;
    s_element_mode = ELEMENT_ROUNDABOUT;
    s_element_busy = true;

    // 只有第一次进入整个环岛时，抓一次统一的环岛角度参考
    if(!was_in_circle)
    {
        s_circle_yaw_start = yaw;
        s_circle_yaw_active = 1U;

        // 进入整个环岛时抓总里程参考。
        capture_circle_enter_distance_ref();
    }

    // 当前子状态自己的阶段参考值，继续保留
    capture_stage_distance_ref();

    // 子状态局部参考也继续保留，供调试/观察
    capture_yaw_ref();
}


//-------------------------------------------------------------------------------------------------------------------
// 函数作用     更新环岛状态机并生成对应阶段的补线模板。
// 使用方法     每帧元素处理过程中调用，负责 PREP/ENTER/INSIDE/EXIT 等阶段推进。
// 接口属性     内部接口；参数形式：circle_state_update(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_state_update(void)
{
    circle_import_old_view();

    if(!s_circle_flag)
    {
        uint8 flag = road_is_roundabout_old();
        if(flag != 0U)
        {
            // 环岛不触发蜂鸣器；蜂鸣器用于坡道和十字正式确认。
            circle_enter_old(flag == 1U ? CIRCLE_LEFT_PREP : CIRCLE_RIGHT_PREP);
            return;
        }
        return;
    }

if((s_circle_state != CIRCLE_EXIT_PREP) &&
   (((circle_yaw_value() < -350.0f) || (circle_yaw_value() > 340.0f))))
{
    circle_enter_old(CIRCLE_EXIT_PREP);
    printf("444444444444\n");
    return;
}
else if((s_circle_state != CIRCLE_EXIT_PREP) &&
        (circle_yaw_value() > 330.0f) &&
        (s_old_right_lost_count < 7))
{
    circle_enter_old(CIRCLE_EXIT_PREP);
    return;
}
else if((s_circle_state != CIRCLE_EXIT_PREP) &&
        (circle_yaw_value() < -340.0f) &&
        (s_old_left_lost_count < 7))
{
    printf("33333333333\n");
    circle_enter_old(CIRCLE_EXIT_PREP);
    return;
}

    if(s_circle_state != CIRCLE_IDLE) s_last_valid_midline = clamp_col(s_last_valid_midline);

    switch(s_circle_state)
    {
case CIRCLE_LEFT_PREP:
{
    // 镜像右环 PREP：左环使用负向累计角保护。
    if(circle_yaw_value() < -60.0f)
    {
        circle_reset_internal();
        break;
    }

    int leftCount = 0;
    for(int y = old_row_begin() - 1; y > old_row_begin() - 30 && y >= 0; --y)
    {
        if(old_left_sign_at(y)) leftCount++;
    }
    if(leftCount > 5) s_old_in_circle_state = 1U;
    s_in_circle = (s_old_in_circle_state != 0U);

    // PREP 阶段实时寻找 LU / LD。
    // LU：找到新 LU 就更新；找不到就沿用上一帧缓存。
    // LD：优先用 LU 约束；约束失败再退回普通稳定搜索。
    {
        int rowEnd = (old_search_end() <= 10) ? 10 : old_search_end();
        int start = old_row_begin() - 10;
        if(start < rowEnd + 5) start = rowEnd + 5;

        int new_left_up_y = old_find_left_up_corner_in_recover_segment(
            start,
            rowEnd,
            s_left_round_point_row
        );

        if((new_left_up_y > 0) &&
           old_row_valid(new_left_up_y) &&
           old_left_sign_at(new_left_up_y))
        {
            s_left_round_up_point = new_left_up_y;
            s_left_round_up_point_x = old_left_pos_at(new_left_up_y);
            s_left_round_up_hold_cnt = 0U;
        }
        else
        {
            if(s_left_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
            {
                s_left_round_up_hold_cnt++;
            }
        }

        s_dbg_left_up_y_current = s_left_round_up_point;

        int new_left_down_y = 0;
        if((s_left_round_up_point > 0) &&
           old_row_valid(s_left_round_up_point) &&
           old_left_sign_at(s_left_round_up_point))
        {
            new_left_down_y = old_find_left_down_corner_from_up(
                start,
                rowEnd,
                s_left_round_up_point,
                old_left_pos_at(s_left_round_up_point)
            );
        }

        if(new_left_down_y <= 0)
        {
            new_left_down_y = old_find_left_down_corner_stable(
                start,
                rowEnd,
                s_left_round_up_point
            );
        }

        if((new_left_down_y > 0) &&
           old_left_down_corner_valid_now(new_left_down_y) &&
           old_left_sign_at(new_left_down_y))
        {
            s_left_round_point_row = new_left_down_y;
            s_left_round_point = old_left_pos_at(new_left_down_y);
            s_left_round_down_hold_cnt = 0U;
        }
        else
        {
            if(s_left_round_down_hold_cnt < OLD_CIRCLE_DOWN_HOLD_MAX)
            {
                s_left_round_down_hold_cnt++;
            }
        }

        s_dbg_left_down_y_current = s_left_round_point_row;
    }

    if((s_left_round_up_point > 0) &&
       old_row_valid(s_left_round_up_point) &&
       old_left_sign_at(s_left_round_up_point))
    {
        int cur_up_x = old_left_pos_at(s_left_round_up_point);

        if((s_left_round_up_point_x > 0) &&
           (iabs_local(cur_up_x - s_left_round_up_point_x) <= OLD_CORNER_X_REFRESH_MAX))
        {
            s_left_round_up_point_x = cur_up_x;
            s_left_round_up_hold_cnt = 0U;
        }
        else if(s_left_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
        {
            s_left_round_up_hold_cnt++;
        }
    }
    else if(s_left_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
    {
        s_left_round_up_hold_cnt++;
    }

    if(stage_distance_sum() >= OLD_CIRCLE_DIST1)
    {
        circle_enter_old(CIRCLE_LEFT_ENTER);
        return;
    }
} break;

case CIRCLE_LEFT_ENTER:
{
    int rightCount = 0;
    for(int y = old_row_begin() - 1; y > old_row_begin() - 20 && y >= 0; --y)
    {
        if(old_right_sign_at(y)) rightCount++;
    }

    // 镜像右环 ENTER：左环以正向累计角进入 INSIDE。
    if(circle_yaw_value() > 90.0f)
    {
        circle_reset_internal();
        break;
    }

    if(circle_yaw_value() > 70.0f /**&& rightCount > 0**/)
    {
        s_circle_state = CIRCLE_LEFT_INSIDE;
        printf("2222222222\n");
    }

    int new_left_enter_up_x = 0;
    int new_left_enter_up_y = old_find_left_enter_up_corner_current(&new_left_enter_up_x);

    if((new_left_enter_up_y > 0) &&
       (new_left_enter_up_x > 0) &&
       old_row_valid(new_left_enter_up_y))
    {
        s_left_round_up_point = new_left_enter_up_y;
        s_left_round_up_point_x = new_left_enter_up_x;
        s_left_round_up_hold_cnt = 0U;
    }
    else if(s_left_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
    {
        s_left_round_up_hold_cnt++;
    }
} break;

case CIRCLE_LEFT_INSIDE:
{
    if(circle_yaw_value() < -60.0f)
    {
        circle_reset_internal();
        break;
    }

    if(circle_yaw_value() > 200.0f)
    {
        int rowEnd = CLAMP(old_search_end(), old_search_end() + 5, 90);
        s_left_round_up_point = old_monotonicity_change_right(old_row_begin(), rowEnd, 4U, 15U, 3U);
        if(s_left_round_up_point) s_old_circle_state_flag = 1U;
    }
} break;

case CIRCLE_RIGHT_PREP:
{
    // PREP 阶段仍然允许用整圈累计角做保护。
    // 因为第一次进入环岛时，circle_enter_old() 已经抓过整圈 yaw 起点。
    if(circle_yaw_value() > 60.0f)
    {
        circle_reset_internal();
        break;
    }

    int rightCount = 0;
    for(int y = old_row_begin() - 1; y > old_row_begin() - 30 && y >= 0; --y)
    {
        if(old_right_sign_at(y)) rightCount++;
    }
    if(rightCount > 5) s_old_in_circle_state = 1U;
    s_in_circle = (s_old_in_circle_state != 0U);

// PREP 阶段实时寻找 RU / RD。
// RU：找到新 RU 就更新；找不到就沿用上一帧缓存。
// RD：找到新 RD 就更新；找不到就沿用上一帧缓存。
{
    int rowEnd = (old_search_end() <= 10) ? 10 : old_search_end();
    int start = old_row_begin() - 10;
    if(start < rowEnd + 5) start = rowEnd + 5;

    // ---------- 1. 实时寻找 RU ----------
    int new_right_up_y = old_find_right_up_corner_in_recover_segment(
        start,
        rowEnd,
        s_right_round_point_row
    );

    if((new_right_up_y > 0) &&
       old_row_valid(new_right_up_y) &&
       old_right_sign_at(new_right_up_y))
    {
        s_right_round_up_point = new_right_up_y;
        s_right_round_up_point_x = old_right_pos_at(new_right_up_y);
        s_right_round_up_hold_cnt = 0U;
    }
    else
    {
        // 当前帧没找到新 RU，不清空缓存，继续用上一帧 RU
        if(s_right_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
        {
            s_right_round_up_hold_cnt++;
        }
    }

    s_dbg_right_up_y_current = s_right_round_up_point;

    // ---------- 2. 实时寻找 RD ----------
    int new_right_down_y = 0;

    // 优先用当前 RU 约束寻找 RD
    if((s_right_round_up_point > 0) &&
       old_row_valid(s_right_round_up_point) &&
       old_right_sign_at(s_right_round_up_point))
    {
        new_right_down_y = old_find_right_down_corner_from_up(
            start,
            rowEnd,
            s_right_round_up_point,
            old_right_pos_at(s_right_round_up_point)
        );
    }

    // RU 约束找不到，再退回普通稳定搜索
    if(new_right_down_y <= 0)
    {
        new_right_down_y = old_find_right_down_corner_stable(
            start,
            rowEnd,
            s_right_round_up_point
        );
    }

    // 找到合法 RD，才刷新缓存
    if((new_right_down_y > 0) &&
       old_row_valid(new_right_down_y) &&
       old_right_sign_at(new_right_down_y) &&
       old_right_down_corner_valid_now(new_right_down_y))
    {
        s_right_round_point_row = new_right_down_y;
        s_right_round_point = old_right_pos_at(new_right_down_y);
        s_right_round_down_hold_cnt = 0U;
    }
    else
    {
        // 当前帧没找到新 RD，不清空缓存，继续用上一帧 RD
        if(s_right_round_down_hold_cnt < OLD_CIRCLE_DOWN_HOLD_MAX)
        {
            s_right_round_down_hold_cnt++;
        }
    }

    s_dbg_right_down_y_current = s_right_round_point_row;
}


if(stage_distance_sum() >= OLD_CIRCLE_DIST1)
    {
        // 关键改动：
        // 不再手动 s_circle_state = CIRCLE_RIGHT_ENTER;
        // 而是统一走 circle_enter_old()，这样会自动刷新：
        // 1. 阶段距离参考
        // 2. 局部 yaw 参考 s_yaw_ref
        // 同时又不会重抓整圈 s_circle_yaw_start
        circle_enter_old(CIRCLE_RIGHT_ENTER);
        return;
    }
} break;

case CIRCLE_RIGHT_ENTER:
{
    int leftCount = 0;
    for(int y = old_row_begin() - 1; y > old_row_begin() - 20 && y >= 0; --y)
    {
        if(old_left_sign_at(y)) leftCount++;
    }

    // 恢复成原来的整圈累计角判断
    if(circle_yaw_value() < -90.0f)
    {
        circle_reset_internal();
        break;
    }

    if(circle_yaw_value() < -70.0f /**&& leftCount > 0**/)
    {
        s_circle_state = CIRCLE_RIGHT_INSIDE;
        printf("1111111111\n");
    }

    int new_right_enter_up_x = 0;
    int new_right_enter_up_y = old_find_right_enter_up_corner_current(&new_right_enter_up_x);

    if((new_right_enter_up_y > 0) &&
       (new_right_enter_up_x > 0) &&
       old_row_valid(new_right_enter_up_y))
    {
        s_right_round_up_point = new_right_enter_up_y;
        s_right_round_up_point_x = new_right_enter_up_x;
        s_right_round_up_hold_cnt = 0U;
    }
    else if(s_right_round_up_hold_cnt < OLD_CIRCLE_UP_HOLD_MAX)
    {
        s_right_round_up_hold_cnt++;
    }
} break;

case CIRCLE_RIGHT_INSIDE:
{
    // 恢复成原来的整圈累计角判断
    if(circle_yaw_value() > 60.0f)
    {
        circle_reset_internal();
        break;
    }

    if(circle_yaw_value() < -200.0f)
    {
        printf("2\n");
        int rowEnd = CLAMP(old_search_end(), old_search_end() + 5, 90);
        s_right_round_up_point = old_monotonicity_change_left(old_row_begin(), rowEnd, 4U, 15U, 3U);
        if(s_right_round_up_point) s_old_circle_state_flag = 1U;
    }
} break;
case CIRCLE_EXIT_PREP:
{
    /*
     * EXIT_PREP 阶段按入环方向镜像寻找真实黑线的上端点。
     * 右环：找右边界，导出右侧模板线；
     * 左环：找左边界，导出左侧模板线。
     *
     * 找到：更新对应侧缓存；
     * 找不到：沿用上一帧缓存；
     * 从来没找到过：用图像中间行对应侧边界点兜底。
     */
    uint8 exit_side = s_circle_lost_side;
    if((exit_side != 1U) && (exit_side != 2U))
    {
        // 理论上不会走到这里。兜底：正向 yaw 认为左环，负向 yaw 认为右环。
        exit_side = (circle_yaw_value() > 0.0f) ? 1U : 2U;
    }

    uint8 *line_valid = (exit_side == 1U) ?
        &s_left_exit_prep_line_valid : &s_right_exit_prep_line_valid;
    int *line_x = (exit_side == 1U) ?
        &s_left_exit_prep_line_x : &s_right_exit_prep_line_x;
    int *line_y = (exit_side == 1U) ?
        &s_left_exit_prep_line_y : &s_right_exit_prep_line_y;

    int found_x = 0;
    int found_y = 0;

    int scan_top = old_search_end() + 2;
    if(scan_top < 2) scan_top = 2;

    int scan_bottom = old_row_begin() - 2;
    if(scan_bottom < scan_top) scan_bottom = scan_top;

    for(int y = scan_top; y <= scan_bottom; ++y)
    {
        if(!old_row_valid(y - 2)) continue;
        if(!old_row_valid(y - 1)) continue;
        if(!old_row_valid(y))     continue;
        if(!old_row_valid(y + 1)) continue;
        if(!old_row_valid(y + 2)) continue;

        // 候选行必须有当前出环侧边界；周围 4 行允许少量丢点。
        uint8 sign_y = (exit_side == 1U) ? old_left_sign_at(y) : old_right_sign_at(y);
        if(!sign_y) continue;

        int x0 = (exit_side == 1U) ? old_left_pos_at(y) : old_right_pos_at(y);
        int near_hit = 0;
        int upper_hit = 0;
        int lower_hit = 0;

        uint8 sign_y_m1 = (exit_side == 1U) ? old_left_sign_at(y - 1) : old_right_sign_at(y - 1);
        int   pos_y_m1  = (exit_side == 1U) ? old_left_pos_at(y - 1)  : old_right_pos_at(y - 1);
        if(sign_y_m1 && (iabs_local(pos_y_m1 - x0) <= OLD_EXIT_PREP_EDGE_NEAR1_MAX))
        {
            near_hit++;
            upper_hit++;
        }

        uint8 sign_y_m2 = (exit_side == 1U) ? old_left_sign_at(y - 2) : old_right_sign_at(y - 2);
        int   pos_y_m2  = (exit_side == 1U) ? old_left_pos_at(y - 2)  : old_right_pos_at(y - 2);
        if(sign_y_m2 && (iabs_local(pos_y_m2 - x0) <= OLD_EXIT_PREP_EDGE_NEAR2_MAX))
        {
            near_hit++;
            upper_hit++;
        }

        uint8 sign_y_p1 = (exit_side == 1U) ? old_left_sign_at(y + 1) : old_right_sign_at(y + 1);
        int   pos_y_p1  = (exit_side == 1U) ? old_left_pos_at(y + 1)  : old_right_pos_at(y + 1);
        if(sign_y_p1 && (iabs_local(pos_y_p1 - x0) <= OLD_EXIT_PREP_EDGE_NEAR1_MAX))
        {
            near_hit++;
            lower_hit++;
        }

        uint8 sign_y_p2 = (exit_side == 1U) ? old_left_sign_at(y + 2) : old_right_sign_at(y + 2);
        int   pos_y_p2  = (exit_side == 1U) ? old_left_pos_at(y + 2)  : old_right_pos_at(y + 2);
        if(sign_y_p2 && (iabs_local(pos_y_p2 - x0) <= OLD_EXIT_PREP_EDGE_NEAR2_MAX))
        {
            near_hit++;
            lower_hit++;
        }

        if((upper_hit >= 1) &&
           (lower_hit >= 1) &&
           (near_hit >= OLD_EXIT_PREP_EDGE_NEAR_HIT_MIN))
        {
            // 防止帧间补线 x 大幅跳变。左右环各自使用自己的缓存。
            if((*line_valid) &&
               (iabs_local(x0 - (*line_x)) > OLD_EXIT_PREP_EDGE_REFRESH_X_MAX))
            {
                continue;
            }

            found_x = x0;
            found_y = y;
            break;
        }
    }

    if((found_y > 0) && old_row_valid(found_y))
    {
        *line_valid = 1U;
        *line_x = found_x;
        *line_y = found_y;
    }
    else if(!(*line_valid))
    {
        // 从来没找到过上端点时，用图像中间这一行的对应侧边界点兜底。
        int mid_y = UVC_HEIGHT / 2;
        uint8 mid_sign = 0U;
        int mid_x = 0;

        if(exit_side == 1U)
        {
            mid_sign = old_left_sign_at(mid_y);
            mid_x = old_left_pos_at(mid_y);
        }
        else
        {
            mid_sign = old_right_sign_at(mid_y);
            mid_x = old_right_pos_at(mid_y);
        }

        if(old_row_valid(mid_y) && mid_sign)
        {
            *line_valid = 1U;
            *line_x = mid_x;
            *line_y = mid_y;
        }
    }

    if(stage_distance_sum() >= OLD_CIRCLE_DIST2 /**|| count_bottom_recover_rows() >= 4**/)
    {
        circle_reset_internal();
        printf("555555555555555555555555555\n");            
    }
} break;



        case CIRCLE_IDLE:
        default:
            break;
    }
}

// ================================================================================================
// 模板线补线工具
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     根据两点直线计算指定 y 行上的 x 坐标。
// 使用方法     补线函数内部调用，用于把模板线投影到每一行边界。
// 接口属性     内部接口；参数形式：line_x_at_y(int x1, int y1, int x2, int y2, int y)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static int line_x_at_y(int x1, int y1, int x2, int y2, int y)
{
    if(y1 == y2) return clamp_col(x1);
    float t = (float)(y - y1) / (float)(y2 - y1);
    int x = (int)(x1 + t * (x2 - x1));
    return clamp_col(x);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     把一条直线写入左边界数组。
// 使用方法     十字或环岛补线需要重建左边界时调用。
// 接口属性     内部接口；参数形式：apply_line_as_left_boundary(int x1, int y1, int x2, int y2)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void apply_line_as_left_boundary(int x1, int y1, int x2, int y2)
{
    int y_start = (y1 < y2) ? y1 : y2;
    int y_end   = (y1 > y2) ? y1 : y2;
    for(int y = y_start; y <= y_end; ++y)
    {
        if(!old_row_valid(y)) continue;
        int idx = old_to_idx(y);
        if(!row_valid(idx)) continue;
        int x = line_x_at_y(x1, y1, x2, y2, y);
        // 黑色调试线和最终左右边界共用同一个 y，避免 set_left_edge() 再次反推行号造成错位。
        set_left_edge_at_y(idx, x, y, 1U);
        s_left_repair_mask[idx] = 1U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     把一条直线写入右边界数组。
// 使用方法     十字或环岛补线需要重建右边界时调用。
// 接口属性     内部接口；参数形式：apply_line_as_right_boundary(int x1, int y1, int x2, int y2)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void apply_line_as_right_boundary(int x1, int y1, int x2, int y2)
{
    int y_start = (y1 < y2) ? y1 : y2;
    int y_end   = (y1 > y2) ? y1 : y2;
    for(int y = y_start; y <= y_end; ++y)
    {
        if(!old_row_valid(y)) continue;
        int idx = old_to_idx(y);
        if(!row_valid(idx)) continue;
        int x = line_x_at_y(x1, y1, x2, y2, y);
        // 黑色调试线和最终左右边界共用同一个 y，避免 set_right_edge() 再次反推行号造成错位。
        set_right_edge_at_y(idx, x, y, 1U);
        s_right_repair_mask[idx] = 1U;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     将指定高度以下缺失的右边界强制补到图像右边缘。
// 使用方法     十字或特殊阶段底部边界丢失时调用。
// 接口属性     内部接口；参数形式：force_missing_right_edge_to_image_border_until_y(int y_top)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void force_missing_right_edge_to_image_border_until_y(int y_top)
{
    int idx_top = old_to_idx(y_top);
    if(idx_top < 0) idx_top = 0;
    if(idx_top >= (int)element_valid_rows()) idx_top = (int)element_valid_rows() - 1;

    // 只处理图像下部到 y_top 这一段
    for(int i = 0; i <= idx_top; ++i)
    {
        if(!right_sign(i))
        {
            // 先不要顶到最边，留一点余量更稳
            set_right_edge(i, USE_WIDTH_R - 3, 1U);
            s_right_repair_mask[i] = 1U;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     将指定高度以下缺失的左边界强制补到图像左边缘。
// 使用方法     十字或特殊阶段底部边界丢失时调用。
// 接口属性     内部接口；参数形式：force_missing_left_edge_to_image_border_until_y(int y_top)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void force_missing_left_edge_to_image_border_until_y(int y_top)
{
    int idx_top = old_to_idx(y_top);
    if(idx_top < 0) idx_top = 0;
    if(idx_top >= (int)element_valid_rows()) idx_top = (int)element_valid_rows() - 1;

    // 只处理图像下部到 y_top 这一段
    for(int i = 0; i <= idx_top; ++i)
    {
        if(!left_sign(i))
        {
            // 先不要顶到最边，留一点余量更稳
            set_left_edge(i, USE_WIDTH_L + 2, 1U);
            s_left_repair_mask[i] = 1U;
        }
    }
}

// ================================================================================================
// 十字处理：原始丢线三帧初检 -> 当前帧角点确认 -> 直接修改左右边界补线
// ================================================================================================
static inline uint8 cross_left_valid_idx(int idx)
{
    return (row_valid(idx) && left_sign(idx)) ? 1U : 0U;
}

static inline uint8 cross_right_valid_idx(int idx)
{
    return (row_valid(idx) && right_sign(idx)) ? 1U : 0U;
}

// 检查指定边界窗口是否全部有效，并且能拟合成连续直/斜线。
static uint8 cross_line_window_valid(uint8 left_side, int first_idx, int point_count)
{
    const int rows = (int)element_valid_rows();
    if(point_count < 2) return 0U;
    const int last_idx = first_idx + point_count - 1;
    if(first_idx < 0 || last_idx >= rows) return 0U;

    for(int i = first_idx; i <= last_idx; ++i)
    {
        const uint8 valid = left_side ? cross_left_valid_idx(i) : cross_right_valid_idx(i);
        if(!valid) return 0U;
    }

    const int first_x = left_side ? left_pos(first_idx) : right_pos(first_idx);
    const int last_x = left_side ? left_pos(last_idx) : right_pos(last_idx);
    const int span = last_idx - first_idx;
    const int dx = last_x - first_x;

    // 使用端点直线的整数形式检查中间点，避免浮点误差。
    for(int i = first_idx; i <= last_idx; ++i)
    {
        const int x = left_side ? left_pos(i) : right_pos(i);
        const int predicted_num = first_x * span + dx * (i - first_idx);
        const int actual_num = x * span;
        if(iabs_local(actual_num - predicted_num) >
           CROSS_UP_ABOVE_LINE_ERR_MAX * span)
        {
            return 0U;
        }
    }

    return 1U;
}

// 找上角点时，从候选上角点自身开始向上取 7 个点：i 到 i+6。
// 这 7 个点必须全部有效，并且构成连续直线或斜线。
static uint8 cross_upper_above_line_valid(uint8 left_side, int corner_idx)
{
    return cross_line_window_valid(left_side, corner_idx, CROSS_UP_ABOVE_LINE_POINTS);
}

static cross_corner_detect_struct cross_find_left_upper(uint8 loose,
                                                        int search_start_idx,
                                                        int search_end_limit)
{
    cross_corner_detect_struct c = {0U, 0, 0, 0};
    int rows = (int)element_valid_rows();
    if(rows <= CROSS_UP_SEARCH_BOTTOM_IDX + 2) return c;

    int diff_high = loose ? CROSS_LOOSE_UP_JUMP_MIN : CROSS_STRICT_UP_JUMP_MIN;
    int start_idx = search_start_idx;
    if(start_idx < CROSS_UP_SEARCH_BOTTOM_IDX) start_idx = CROSS_UP_SEARCH_BOTTOM_IDX;
    int end_idx = rows - CROSS_UP_ABOVE_LINE_POINTS;
    if(search_end_limit >= 0 && end_idx > search_end_limit) end_idx = search_end_limit;
    if(start_idx > end_idx) return c;

    for(int i = start_idx; i <= end_idx; ++i)
    {
        // 右上角点旧算法的左右镜像：上方连续三点有效，
        // 候选点相对下方局部点向右突变。
        if(!cross_left_valid_idx(i) ||
           !cross_left_valid_idx(i + 1) ||
           !cross_left_valid_idx(i + 2)) continue;

        if(!cross_upper_above_line_valid(1U, i)) continue;

        if(left_pos(i) <= left_pos(i - 1) ||
           left_pos(i) <= left_pos(i - 2)) continue;

        if((left_pos(i + 1) - left_pos(i - 3)) <= diff_high ||
           (left_pos(i) - left_pos(i - 4)) <= diff_high) continue;

        c.found = 1U;
        c.idx = i;
        c.x = left_pos(i);
        c.y = edge_row_y_from_existing(i);
        return c;
    }
    return c;
}

static cross_corner_detect_struct cross_find_right_upper(uint8 loose,
                                                         int search_start_idx,
                                                         int search_end_limit)
{
    cross_corner_detect_struct c = {0U, 0, 0, 0};
    int rows = (int)element_valid_rows();
    if(rows <= CROSS_UP_SEARCH_BOTTOM_IDX + 2) return c;

    int diff_high = loose ? CROSS_LOOSE_UP_JUMP_MIN : CROSS_STRICT_UP_JUMP_MIN;
    int start_idx = search_start_idx;
    if(start_idx < CROSS_UP_SEARCH_BOTTOM_IDX) start_idx = CROSS_UP_SEARCH_BOTTOM_IDX;
    int end_idx = rows - CROSS_UP_ABOVE_LINE_POINTS;
    if(search_end_limit >= 0 && end_idx > search_end_limit) end_idx = search_end_limit;
    if(start_idx > end_idx) return c;

    for(int i = start_idx; i <= end_idx; ++i)
    {
        // 按旧算法原式判断右上角点。
        if(!cross_right_valid_idx(i) ||
           !cross_right_valid_idx(i + 1) ||
           !cross_right_valid_idx(i + 2)) continue;

        if(!cross_upper_above_line_valid(0U, i)) continue;

        if(right_pos(i) >= right_pos(i - 1) ||
           right_pos(i) >= right_pos(i - 2)) continue;

        if((right_pos(i - 3) - right_pos(i + 1)) <= diff_high ||
           (right_pos(i - 4) - right_pos(i)) <= diff_high) continue;

        c.found = 1U;
        c.idx = i;
        c.x = right_pos(i);
        c.y = edge_row_y_from_existing(i);
        return c;
    }
    return c;
}

static cross_corner_detect_struct cross_find_left_lower(uint8 loose)
{
    cross_corner_detect_struct c = {0U, 0, 0, 0};
    int rows = (int)element_valid_rows();
    if(rows < 30) return c;

    (void)loose;
    const int diff_small = 4;
    int start_idx = 0;
    int end_idx = rows - 4;
    if(end_idx < start_idx) return c;

    // 只在图像最下方～y=50 内从下往上找：当前点有效，且比上方三个位置明显向右跳变。
    for(int i = start_idx; i <= end_idx; ++i)
    {
        if(i < 0 || i + 3 >= rows) continue;
        const int candidate_y = edge_row_y_from_existing(i);
        if(candidate_y > CROSS_DOWN_SEARCH_Y_MAX ||
           candidate_y < CROSS_DOWN_SEARCH_Y_MIN) continue;
        if(!cross_left_valid_idx(i)) continue;

        int x_i = left_pos(i);
        if((x_i - left_pos(i + 1)) < diff_small) continue;
        if((x_i - left_pos(i + 2)) < diff_small) continue;
        if((x_i - left_pos(i + 3)) < diff_small) continue;

        c.found = 1U;
        c.idx = i;
        c.x = x_i;
        c.y = candidate_y;
        return c;
    }
    return c;
}

static cross_corner_detect_struct cross_find_right_lower(uint8 loose)
{
    cross_corner_detect_struct c = {0U, 0, 0, 0};
    int rows = (int)element_valid_rows();
    if(rows < 30) return c;

    (void)loose;
    const int diff_small = 4;
    int start_idx = 0;
    int end_idx = rows - 4;
    if(end_idx < start_idx) return c;

    // 只在图像最下方～y=50 内从下往上找：当前点有效，且比上方三个位置明显向左跳变。
    for(int i = start_idx; i <= end_idx; ++i)
    {
        if(i < 0 || i + 3 >= rows) continue;
        const int candidate_y = edge_row_y_from_existing(i);
        if(candidate_y > CROSS_DOWN_SEARCH_Y_MAX ||
           candidate_y < CROSS_DOWN_SEARCH_Y_MIN) continue;
        if(!cross_right_valid_idx(i)) continue;

        if((right_pos(i + 1) - right_pos(i)) < diff_small) continue;
        if((right_pos(i + 2) - right_pos(i)) < diff_small) continue;
        if((right_pos(i + 3) - right_pos(i)) < diff_small) continue;

        c.found = 1U;
        c.idx = i;
        c.x = right_pos(i);
        c.y = candidate_y;
        return c;
    }
    return c;
}

static void cross_clear_current_corners(void)
{
    s_cross_lu = {0U, 0, 0, 0};
    s_cross_ld = {0U, 0, 0, 0};
    s_cross_ru = {0U, 0, 0, 0};
    s_cross_rd = {0U, 0, 0, 0};
}

static void cross_set_current_corner(cross_corner_current_struct *dst,
                                     const cross_corner_detect_struct *src)
{
    if(dst == NULL || src == NULL)
    {
        return;
    }

    if(!src->found)
    {
        *dst = {0U, 0, 0, 0};
        return;
    }

    dst->valid = 1U;
    dst->x = clamp_col(src->x);
    dst->y = (int)CLAMP(src->y, 0, UVC_HEIGHT - 1);
    dst->idx = src->idx;
}

// 四个角点都只采用当前帧结果；本帧漏检时立即清除，不沿用上一帧。
// 每帧必须先找下角点，再按本帧下角点位置确定上角点搜索起点。
static uint8 cross_detect_current_corners(uint8 loose)
{
    cross_corner_detect_struct ld = cross_find_left_lower(loose);
    cross_corner_detect_struct rd = cross_find_right_lower(loose);

    cross_set_current_corner(&s_cross_ld, &ld);
    cross_set_current_corner(&s_cross_rd, &rd);

    // 搜索起点只看本帧是否真正找到下角点。
    const int left_up_start = ld.found
        ? (ld.idx + CROSS_UP_FROM_LOWER_GAP)
        : CROSS_UP_SEARCH_BOTTOM_IDX;
    const int right_up_start = rd.found
        ? (rd.idx + CROSS_UP_FROM_LOWER_GAP)
        : CROSS_UP_SEARCH_BOTTOM_IDX;
    const int left_up_end = ld.found
        ? -1
        : CROSS_UP_NO_LOWER_SEARCH_END_IDX;
    const int right_up_end = rd.found
        ? -1
        : CROSS_UP_NO_LOWER_SEARCH_END_IDX;

    cross_corner_detect_struct lu = cross_find_left_upper(loose, left_up_start, left_up_end);
    cross_corner_detect_struct ru = cross_find_right_upper(loose, right_up_start, right_up_end);

    cross_set_current_corner(&s_cross_lu, &lu);
    cross_set_current_corner(&s_cross_ru, &ru);

    return (uint8)(s_cross_lu.valid + s_cross_ld.valid +
                   s_cross_ru.valid + s_cross_rd.valid);
}

static uint8 cross_corner_pattern_ready(void)
{
    // 至少三个角点，并且必须是两侧下角点都有、至少一侧上下角点完整。
    const uint8 count = (uint8)(s_cross_lu.valid + s_cross_ld.valid +
                                s_cross_ru.valid + s_cross_rd.valid);
    return (uint8)((count >= 3U && s_cross_ld.valid && s_cross_rd.valid &&
                    (s_cross_lu.valid || s_cross_ru.valid)) ? 1U : 0U);
}

static uint8 cross_entry_condition(void)
{
    const uint8 both_lost =
        (s_cross_raw_both_lost_count > CROSS_ENTRY_BOTH_LOST_MIN) ? 1U : 0U;
    const uint8 both_sides_lost =
        (s_cross_raw_left_lost_count > CROSS_ENTRY_SIDE_LOST_MIN &&
         s_cross_raw_right_lost_count > CROSS_ENTRY_SIDE_LOST_MIN) ? 1U : 0U;
    return (uint8)(both_lost || both_sides_lost);
}

static void cross_count_raw_lost_excluding_bottom(void)
{
    const int rows = (int)element_valid_rows();
    const int first_counted_y = IMG_PROC_HEIGHT - CROSS_ENTRY_SKIP_BOTTOM_ROWS;
    int left_lost = 0;
    int right_lost = 0;
    int both_lost = 0;

    for(int i = 0; i < rows; ++i)
    {
        const int y = edge_row_y_from_existing(i);
        // 图像最下方 30 行 y=90～119 不参与十字粗检测丢线统计。
        if(y >= first_counted_y)
        {
            continue;
        }

        const uint8 l_ok = cross_left_valid_idx(i);
        const uint8 r_ok = cross_right_valid_idx(i);
        if(!l_ok) left_lost++;
        if(!r_ok) right_lost++;
        if(!l_ok && !r_ok) both_lost++;
    }

    s_cross_raw_left_lost_count = (int16)left_lost;
    s_cross_raw_right_lost_count = (int16)right_lost;
    s_cross_raw_both_lost_count = (int16)both_lost;
}

static uint8 cross_bottom_edge_recovered(void)
{
    const int rows = (int)element_valid_rows();
    int checked = 0;
    int left_valid = 0;
    int right_valid = 0;

    // filter_left/right 尚未被十字补线改写；这里统计的是本帧原始搜线有效性。
    for(int i = 0; i < rows; ++i)
    {
        const int y = edge_row_y_from_existing(i);
        if(y < CROSS_EXIT_BOTTOM_Y_MIN || y > CROSS_EXIT_BOTTOM_Y_MAX)
        {
            continue;
        }

        checked++;
        if(cross_left_valid_idx(i)) left_valid++;
        if(cross_right_valid_idx(i)) right_valid++;
    }

    s_cross_bottom_left_valid = (int16)left_valid;
    s_cross_bottom_right_valid = (int16)right_valid;
    s_cross_bottom_checked_rows = (int16)checked;

    if(checked < CROSS_EXIT_BOTTOM_MIN_CHECKED_ROWS)
    {
        return 0U;
    }

    const uint8 left_recovered =
        (left_valid * 100 >= checked * CROSS_EXIT_BOTTOM_VALID_PERCENT) ? 1U : 0U;
    const uint8 right_recovered =
        (right_valid * 100 >= checked * CROSS_EXIT_BOTTOM_VALID_PERCENT) ? 1U : 0U;
    return (uint8)(left_recovered || right_recovered);
}

static void cross_state_update(void)
{
    if(s_circle_state != CIRCLE_IDLE || s_circle_flag)
    {
        cross_force_disable_internal();
        return;
    }

    s_cross_candidate = cross_entry_condition();

    if(s_cross_state == CROSS_IDLE)
    {
        // 空闲时用宽松条件探测角点，仅供第一页显示，不参与初检测。
        s_cross_line_index = cross_detect_current_corners(1U);
        s_cross_lost_frames = 0U;
        s_cross_bottom_recover_frames = 0U;
        s_cross_bottom_left_valid = 0;
        s_cross_bottom_right_valid = 0;
        s_cross_bottom_checked_rows = 0;

        if(s_cross_candidate)
        {
            if(s_cross_confirm_frames < 255U) s_cross_confirm_frames++;
        }
        else
        {
            s_cross_confirm_frames = 0U;
        }

        if(s_cross_confirm_frames >= CROSS_ENTRY_CONFIRM_FRAMES)
        {
            s_cross_state = CROSS_DETECTED;
            s_element_mode = ELEMENT_CROSSROAD;
            s_element_busy = true;
            s_cross_confirm_frames = 0U;
            s_cross_lost_frames = 0U;
            // 下一帧开始严格角点判断，不能沿用空闲阶段的宽松角点。
            cross_clear_current_corners();
        }
        return;
    }

    if(s_cross_state == CROSS_DETECTED)
    {
        const uint8 corner_count = cross_detect_current_corners(0U);
        s_cross_line_index = corner_count;

        if(cross_corner_pattern_ready())
        {
            s_cross_state = CROSS_INSIDE;
            s_cross_lost_frames = 0U;
#if CROSS_BEEP_ENABLE
            // 三角点组合确认、正式开始十字补线时响一次；粗检测阶段不响。
            beep_request();
#endif
            return;
        }

        if(corner_count < CROSS_CORNER_MIN_FOR_KEEP)
        {
            if(s_cross_lost_frames < 255U) s_cross_lost_frames++;
        }
        else
        {
            s_cross_lost_frames = 0U;
        }

        if(s_cross_lost_frames >= CROSS_CORNER_FAIL_FRAMES)
        {
#if CROSS_CORNER_FAIL_EXIT_ENABLE
            // 角点阶段连续三帧不足两个角点，本帧输出一次退出脉冲。
            s_cross_exit_frames = 1U;
            cross_force_disable_internal();
#endif
        }
        return;
    }

    // 补线阶段先用本帧原始边界更新角点。
    s_cross_line_index = cross_detect_current_corners(1U);

    // 退出条件：当前帧两侧下角点都消失，同时 y=80～100 中至少一侧
    // 原始边线大部分恢复。上角点不参与退出判断。
    const uint8 bottom_recovered = cross_bottom_edge_recovered();
    if(!s_cross_ld.valid && !s_cross_rd.valid && bottom_recovered)
    {
        s_cross_exit_frames = 1U;
        // 同时清空当前角点、左右补线模型和十字状态。
        cross_force_disable_internal();
        return;
    }
}

static int cross_round_to_int(float value)
{
    return (int)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void cross_make_patch_line_from_points(int upper_x, int upper_y,
                                              int lower_x, int lower_y,
                                              int *x1, int *y1, int *x2, int *y2)
{
    if(x1 == NULL || y1 == NULL || x2 == NULL || y2 == NULL) return;
    if(lower_y < upper_y)
    {
        int tx = upper_x; upper_x = lower_x; lower_x = tx;
        int ty = upper_y; upper_y = lower_y; lower_y = ty;
    }
    *x1 = clamp_col(upper_x);
    *y1 = (int)CLAMP(upper_y, 0, UVC_HEIGHT - 1);
    *x2 = clamp_col(lower_x);
    *y2 = (int)CLAMP(lower_y, 0, UVC_HEIGHT - 1);
}

static void cross_store_patch_model(cross_patch_model_struct *model,
                                    int x1, int y1, int x2, int y2)
{
    if(model == NULL || y1 == y2) return;

    const float slope = (float)(x2 - x1) / (float)(y2 - y1);

    model->valid = 1U;
    model->slope = slope;
    model->x1 = clamp_col(x1);
    model->y1 = (int)CLAMP(y1, 0, UVC_HEIGHT - 1);
    model->x2 = clamp_col(x2);
    model->y2 = (int)CLAMP(y2, 0, UVC_HEIGHT - 1);
}

// 用上角点及其上方 5 行中的有效边界点拟合 x = k*y+b。
static uint8 cross_measure_upper_side_slope(uint8 left_side,
                                            const cross_corner_current_struct *upper,
                                            float *slope_out)
{
    if(upper == NULL || !upper->valid || slope_out == NULL) return 0U;

    const int rows = (int)element_valid_rows();
    const int first_idx = upper->idx;
    int last_idx = upper->idx + CROSS_UPPER_SLOPE_ABOVE_ROWS;
    if(first_idx < 0 || first_idx >= rows) return 0U;
    if(last_idx >= rows) last_idx = rows - 1;
    if(last_idx - first_idx != CROSS_UPPER_SLOPE_ABOVE_ROWS) return 0U;

    const uint8 corner_valid = left_side
        ? cross_left_valid_idx(first_idx)
        : cross_right_valid_idx(first_idx);
    if(!corner_valid ||
       !cross_line_window_valid(left_side,
                                first_idx,
                                CROSS_UPPER_SLOPE_ABOVE_ROWS + 1)) return 0U;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;
    int count = 0;

    for(int i = first_idx; i <= last_idx; ++i)
    {
        const uint8 valid = left_side ? cross_left_valid_idx(i) : cross_right_valid_idx(i);
        if(!valid) return 0U;

        const float x = (float)(left_side ? left_pos(i) : right_pos(i));
        const float y = (float)edge_row_y_from_existing(i);
        sum_x += x;
        sum_y += y;
        sum_yy += y * y;
        sum_yx += y * x;
        count++;
    }

    if(count != CROSS_UPPER_SLOPE_ABOVE_ROWS + 1) return 0U;
    const float denominator = (float)count * sum_yy - sum_y * sum_y;
    if(fabsf(denominator) < 1e-4f) return 0U;

    float slope = ((float)count * sum_yx - sum_y * sum_x) / denominator;
    if(slope > CROSS_LOWER_SLOPE_K_ABS_MAX) slope = CROSS_LOWER_SLOPE_K_ABS_MAX;
    if(slope < -CROSS_LOWER_SLOPE_K_ABS_MAX) slope = -CROSS_LOWER_SLOPE_K_ABS_MAX;
    *slope_out = slope;
    return 1U;
}

// 下角点消失时，以当前上角点为锚，沿其上方 5 行实时斜率补到图像底部。
static uint8 cross_extend_upper_with_current_slope(uint8 left_side,
                                                   const cross_corner_current_struct *upper,
                                                   cross_patch_model_struct *model,
                                                   int *x1, int *y1, int *x2, int *y2)
{
    if(upper == NULL || !upper->valid || model == NULL) return 0U;

    float slope = 0.0f;
    if(!cross_measure_upper_side_slope(left_side, upper, &slope)) return 0U;

    const int bottom_y = edge_row_y_from_existing(0);
    if(bottom_y <= upper->y) return 0U;

    const int bottom_x = cross_round_to_int(
        (float)upper->x + slope * (float)(bottom_y - upper->y));
    cross_make_patch_line_from_points(upper->x, upper->y,
                                      bottom_x, bottom_y,
                                      x1, y1, x2, y2);

    cross_store_patch_model(model, *x1, *y1, *x2, *y2);
    return 1U;
}

// 用下角点下方连续 10 个有效边界点拟合 x = k*y+b。
static uint8 cross_measure_lower_side_slope(uint8 left_side,
                                            const cross_corner_current_struct *lower,
                                            float *slope_out)
{
    if(lower == NULL || !lower->valid || slope_out == NULL) return 0U;

    const int rows = (int)element_valid_rows();
    int first_idx = lower->idx - 1;
    int last_idx = lower->idx - CROSS_LOWER_SLOPE_FIT_ROWS;
    if(first_idx >= rows) first_idx = rows - 1;
    if(last_idx < 0) return 0U;
    if(first_idx < last_idx) return 0U;

    float sum_x = 0.0f;
    float sum_y = 0.0f;
    float sum_yy = 0.0f;
    float sum_yx = 0.0f;
    int count = 0;

    for(int i = first_idx; i >= last_idx; --i)
    {
        const uint8 valid = left_side ? cross_left_valid_idx(i) : cross_right_valid_idx(i);
        if(!valid) return 0U;

        const float x = (float)(left_side ? left_pos(i) : right_pos(i));
        const float y = (float)edge_row_y_from_existing(i);
        if((int)y <= lower->y) return 0U;

        sum_x += x;
        sum_y += y;
        sum_yy += y * y;
        sum_yx += y * x;
        count++;
    }

    if(count != CROSS_LOWER_SLOPE_FIT_ROWS) return 0U;
    const float denominator = (float)count * sum_yy - sum_y * sum_y;
    if(fabsf(denominator) < 1e-4f) return 0U;

    float slope = ((float)count * sum_yx - sum_y * sum_x) / denominator;
    if(slope > CROSS_LOWER_SLOPE_K_ABS_MAX) slope = CROSS_LOWER_SLOPE_K_ABS_MAX;
    if(slope < -CROSS_LOWER_SLOPE_K_ABS_MAX) slope = -CROSS_LOWER_SLOPE_K_ABS_MAX;
    *slope_out = slope;
    return 1U;
}

static uint8 cross_fit_lower_side_line(uint8 left_side,
                                       const cross_corner_current_struct *lower,
                                       int target_y,
                                       int *x1, int *y1, int *x2, int *y2)
{
    if(lower == NULL || !lower->valid || target_y >= lower->y) return 0U;

    float slope = 0.0f;
    if(!cross_measure_lower_side_slope(left_side, lower, &slope)) return 0U;

    const int upper_x = cross_round_to_int(
        (float)lower->x + slope * (float)(target_y - lower->y));
    cross_make_patch_line_from_points(upper_x, target_y,
                                      lower->x, lower->y,
                                      x1, y1, x2, y2);
    return (*y1 != *y2) ? 1U : 0U;
}

static void cross_collect_patch_lines(uint8 *left_ready,
                                      int *lx1, int *ly1, int *lx2, int *ly2,
                                      uint8 *right_ready,
                                      int *rx1, int *ry1, int *rx2, int *ry2)
{
    if(left_ready != NULL) *left_ready = 0U;
    if(right_ready != NULL) *right_ready = 0U;
    if(s_cross_state != CROSS_INSIDE) return;

    // 补线模型不跨帧沿用：每次只保留当前角点能实际生成的线。
    s_cross_left_patch_model  = {0U, 0.0f, 0, 0, 0, 0};
    s_cross_right_patch_model = {0U, 0.0f, 0, 0, 0, 0};

    const uint8 left_full = (uint8)(s_cross_lu.valid && s_cross_ld.valid);
    const uint8 right_full = (uint8)(s_cross_ru.valid && s_cross_rd.valid);
    uint8 left_ok = 0U;
    uint8 right_ok = 0U;

    if(left_full)
    {
        cross_make_patch_line_from_points(s_cross_lu.x, s_cross_lu.y,
                                          s_cross_ld.x, s_cross_ld.y,
                                          lx1, ly1, lx2, ly2);
        left_ok = 1U;
    }
    else if(right_full && s_cross_ld.valid)
    {
        left_ok = cross_fit_lower_side_line(1U, &s_cross_ld, s_cross_ru.y,
                                            lx1, ly1, lx2, ly2);
    }

    if(left_ok)
    {
        cross_store_patch_model(&s_cross_left_patch_model,
                                *lx1, *ly1, *lx2, *ly2);
    }
    else if(s_cross_lu.valid)
    {
        left_ok = cross_extend_upper_with_current_slope(1U,
                                                        &s_cross_lu,
                                                        &s_cross_left_patch_model,
                                                        lx1, ly1, lx2, ly2);
    }

    if(right_full)
    {
        cross_make_patch_line_from_points(s_cross_ru.x, s_cross_ru.y,
                                          s_cross_rd.x, s_cross_rd.y,
                                          rx1, ry1, rx2, ry2);
        right_ok = 1U;
    }
    else if(left_full && s_cross_rd.valid)
    {
        right_ok = cross_fit_lower_side_line(0U, &s_cross_rd, s_cross_lu.y,
                                             rx1, ry1, rx2, ry2);
    }

    if(right_ok)
    {
        cross_store_patch_model(&s_cross_right_patch_model,
                                *rx1, *ry1, *rx2, *ry2);
    }
    else if(s_cross_ru.valid)
    {
        right_ok = cross_extend_upper_with_current_slope(0U,
                                                         &s_cross_ru,
                                                         &s_cross_right_patch_model,
                                                         rx1, ry1, rx2, ry2);
    }

    if(left_ready != NULL) *left_ready = left_ok;
    if(right_ready != NULL) *right_ready = right_ok;
}

static void cross_build_final_boundary(void)
{
    if(s_cross_state != CROSS_INSIDE)
    {
        s_cross_left_patch_model  = {0U, 0.0f, 0, 0, 0, 0};
        s_cross_right_patch_model = {0U, 0.0f, 0, 0, 0, 0};
        return;
    }

    uint8 left_ready = 0U;
    uint8 right_ready = 0U;
    int lx1 = 0, ly1 = 0, lx2 = 0, ly2 = 0;
    int rx1 = 0, ry1 = 0, rx2 = 0, ry2 = 0;
    cross_collect_patch_lines(&left_ready, &lx1, &ly1, &lx2, &ly2,
                              &right_ready, &rx1, &ry1, &rx2, &ry2);

    // 十字补线不写入左右边界数组，也不修改 sign；上面的模型只供中线重构使用。
    (void)left_ready;
    (void)right_ready;
}

// ================================================================================================
// 环岛补线：用老 CircleDeal 的“角点 + 模板线”思想，直接改数组，不改 raw_img
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     将环岛补线模板写入最终边界。
// 使用方法     环岛状态有效时调用，按模板线 side 写入左或右边界并重建中线。
// 接口属性     内部接口；参数形式：circle_build_final_boundary(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void circle_build_final_boundary(void)
{
    uint8 rows = element_valid_rows();
    if(rows == 0U || s_circle_state == CIRCLE_IDLE) return;

    circle_template_debug_clear();

    switch(s_circle_state)
    {
case CIRCLE_LEFT_PREP:
{
    /*
     * 左环 PREP 阶段补线：镜像右环 PREP。
     * 端点输出统一经过防跳变缓存：
     * 1) 当前帧找到端点，用候选点限速更新；
     * 2) 当前帧没找到端点，优先沿用上一帧平滑端点；
     * 3) 从来没缓存过，才使用固定端点兜底。
     */
    int x_up = OLD_LC_PREP_FIXED_UP_X;
    int y_up = OLD_LC_PREP_FIXED_UP_Y;
    uint8 have_up = 0U;

    // PRE 阶段优先使用 LU 作为补线上端点；没有 LU 时才用固定点兜底。
    if((s_left_round_up_point > 0) &&
       old_row_valid(s_left_round_up_point))
    {
        y_up = s_left_round_up_point;

        if(s_left_round_up_point_x > 0)
        {
            x_up = s_left_round_up_point_x;
            have_up = 1U;
        }
        else if(old_left_sign_at(y_up))
        {
            x_up = old_left_pos_at(y_up);
            have_up = 1U;
        }
    }

    int x_down = USE_WIDTH_L + 1;
    int y_down = old_row_begin();
    uint8 have_down = 1U;

    if(x_up < USE_WIDTH_L + 1) x_up = USE_WIDTH_L + 1;
    if(x_up > USE_WIDTH_R - 1) x_up = USE_WIDTH_R - 1;
    if(x_down < USE_WIDTH_L + 1) x_down = USE_WIDTH_L + 1;
    if(x_down > USE_WIDTH_R - 1) x_down = USE_WIDTH_R - 1;

    if(y_up < old_search_end()) y_up = old_search_end();
    if(y_up > old_row_begin())  y_up = old_row_begin();
    if(y_down < old_search_end()) y_down = old_search_end();
    if(y_down > old_row_begin())  y_down = old_row_begin();

    int sx_up = x_up;
    int sy_up = y_up;
    int sx_down = x_down;
    int sy_down = y_down;

    if(circle_repair_line_smooth_or_fixed(CIRCLE_REPAIR_SLOT_LEFT_PREP,
                                          have_up,
                                          x_up,
                                          y_up,
                                          OLD_LC_PREP_FIXED_UP_X,
                                          OLD_LC_PREP_FIXED_UP_Y,
                                          have_down,
                                          x_down,
                                          y_down,
                                          OLD_LC_PREP_FIXED_DOWN_X,
                                          OLD_LC_PREP_FIXED_DOWN_Y,
                                          &sx_up,
                                          &sy_up,
                                          &sx_down,
                                          &sy_down))
    {
        apply_line_as_left_boundary(sx_up, sy_up, sx_down, sy_down);
        circle_template_debug_set(1U, sx_up, sy_up, sx_down, sy_down);
    }
} break;


case CIRCLE_LEFT_ENTER:
{
    int x1 = 0;
    int y1 = old_find_left_enter_up_corner_current(&x1);
    uint8 have_line = 0U;
    int x2 = 0;
    int y2 = 0;

    if((y1 > 0) && (x1 > 0))
    {
        if(y1 > old_row_begin()) y1 = old_row_begin();
        if(y1 < old_search_end()) y1 = old_search_end();

        y2 = y1 + 55;
        if(y2 > old_row_begin()) y2 = old_row_begin();
        x2 = old_right_pos_at(y2);

        have_line = 1U;
        s_last_point[2][0] = x1;
        s_last_point[2][1] = y1;
    }
    else if(s_last_point[2][1] > 0)
    {
        // 当前帧没找到新点时，先用旧缓存作为候选；平滑层还会继续限速。
        y1 = s_last_point[2][1];
        x1 = s_last_point[2][0];

        y2 = y1 + 55;
        if(y2 > old_row_begin()) y2 = old_row_begin();
        x2 = old_right_pos_at(y2);

        have_line = 1U;
    }

    int sx2 = x2;
    int sy2 = y2;
    int sx1 = x1;
    int sy1 = y1;

    if(circle_repair_line_smooth_update(CIRCLE_REPAIR_SLOT_LEFT_ENTER,
                                        have_line,
                                        x2,
                                        y2,
                                        have_line,
                                        x1,
                                        y1,
                                        &sx2,
                                        &sy2,
                                        &sx1,
                                        &sy1))
    {
        // 先按老逻辑补“右边界模板线”
        apply_line_as_right_boundary(sx2, sy2, sx1, sy1);

        // 再把下部缺失的左边界，直接补到图像左边缘附近
        force_missing_left_edge_to_image_border_until_y(sy1);

        circle_template_debug_set(2U, sx2, sy2, sx1, sy1);
    }
} break;
case CIRCLE_LEFT_INSIDE:
{
    // 镜像右环 INSIDE：只导出右边界模板线，不直接强改数组。
    if(circle_yaw_value() > 185.0f)
    {
        const int x1 = USE_WIDTH_R - 1;
        const int y1 = 100;
        const int x2 = USE_WIDTH_L + 1;
        const int y2 = 15;

        s_last_point[4][0] = x1;
        s_last_point[4][1] = y1;
        // 该黑线是固定模板，直接输出端点，不经过平滑或缓存。
        circle_template_debug_set(2U, x1, y1, x2, y2);
    }
    else
    {
        circle_template_debug_set(0U, 0, 0, 0, 0);
    }
} break;
case CIRCLE_RIGHT_PREP:
{
    /*
     * 右环 PREP 阶段补线：
     * 上端点优先使用 RU，下端点优先使用 RD；
     * 两个端点最终统一经过防跳变缓存。
     */
    int x_up = OLD_RC_PREP_FIXED_UP_X;
    int y_up = OLD_RC_PREP_FIXED_UP_Y;
    uint8 have_up = 0U;

    // PRE 阶段优先使用 RU 作为补线上端点；没有 RU 时才用固定点兜底。
    if((s_right_round_up_point > 0) &&
       old_row_valid(s_right_round_up_point))
    {
        y_up = s_right_round_up_point;

        if(s_right_round_up_point_x > 0)
        {
            x_up = s_right_round_up_point_x;
            have_up = 1U;
        }
        else if(old_right_sign_at(y_up))
        {
            x_up = old_right_pos_at(y_up);
            have_up = 1U;
        }
    }

    int x_down = USE_WIDTH_R - 1;
    int y_down = old_row_begin();
    uint8 have_down = 1U;

    if(x_up < USE_WIDTH_L + 1) x_up = USE_WIDTH_L + 1;
    if(x_up > USE_WIDTH_R - 1) x_up = USE_WIDTH_R - 1;

    if(x_down < USE_WIDTH_L + 1) x_down = USE_WIDTH_L + 1;
    if(x_down > USE_WIDTH_R - 1) x_down = USE_WIDTH_R - 1;

    if(y_up < old_search_end()) y_up = old_search_end();
    if(y_up > old_row_begin())  y_up = old_row_begin();
    if(y_down < old_search_end()) y_down = old_search_end();
    if(y_down > old_row_begin())  y_down = old_row_begin();

    int sx_up = x_up;
    int sy_up = y_up;
    int sx_down = x_down;
    int sy_down = y_down;

    if(circle_repair_line_smooth_or_fixed(CIRCLE_REPAIR_SLOT_RIGHT_PREP,
                                          have_up,
                                          x_up,
                                          y_up,
                                          OLD_RC_PREP_FIXED_UP_X,
                                          OLD_RC_PREP_FIXED_UP_Y,
                                          have_down,
                                          x_down,
                                          y_down,
                                          OLD_RC_PREP_FIXED_DOWN_X,
                                          OLD_RC_PREP_FIXED_DOWN_Y,
                                          &sx_up,
                                          &sy_up,
                                          &sx_down,
                                          &sy_down))
    {
        apply_line_as_right_boundary(sx_up, sy_up, sx_down, sy_down);
        circle_template_debug_set(2U, sx_up, sy_up, sx_down, sy_down);
    }
} break;


case CIRCLE_RIGHT_ENTER:
{
    /*
     * 右环 ENTER 阶段补线：
     * 当前帧找到上端点则刷新；找不到时由平滑缓存沿用上一帧整条补线。
     */
    int x1 = 0;
    int y1 = old_find_right_enter_up_corner_current(&x1);
    uint8 have_line = 0U;
    int x2 = 0;
    int y2 = 0;

    if((y1 > 0) && (x1 > 0))
    {
        if(y1 > old_row_begin()) y1 = old_row_begin();
        if(y1 < old_search_end()) y1 = old_search_end();

        y2 = y1 + 55;
        if(y2 > old_row_begin()) y2 = old_row_begin();

        x2 = old_left_pos_at(y2);
        have_line = 1U;

        // 这里只用于调试显示，不参与下一帧补线上端点选择
        s_last_point[3][0] = x1;
        s_last_point[3][1] = y1;
    }

    int sx2 = x2;
    int sy2 = y2;
    int sx1 = x1;
    int sy1 = y1;

    if(circle_repair_line_smooth_update(CIRCLE_REPAIR_SLOT_RIGHT_ENTER,
                                        have_line,
                                        x2,
                                        y2,
                                        have_line,
                                        x1,
                                        y1,
                                        &sx2,
                                        &sy2,
                                        &sx1,
                                        &sy1))
    {
        // 先按老逻辑补“左边界模板线”
        apply_line_as_left_boundary(sx2, sy2, sx1, sy1);

        // 再把下部缺失的右边界，直接补到图像右边缘附近
        force_missing_right_edge_to_image_border_until_y(sy1);

        circle_template_debug_set(1U, sx2, sy2, sx1, sy1);
    }
} break;

case CIRCLE_RIGHT_INSIDE:
{
    // 右环 INSIDE 使用左环固定模板线的水平镜像：
    // 左环：上端在图像右边界、下端在图像左边界；
    // 右环：上端在图像左边界、下端在图像右边界。
    if(circle_yaw_value() < -185.0f)
    {
        const int x1 = USE_WIDTH_L + 1;
        const int y1 = 100;
        const int x2 = USE_WIDTH_R - 1;
        const int y2 = 15;

        s_last_point[5][0] = x1;
        s_last_point[5][1] = y1;
        // 该黑线是固定模板，直接输出端点，不经过平滑或缓存。
        circle_template_debug_set(1U, x1, y1, x2, y2);
    }
    else
    {
        circle_template_debug_set(0U, 0, 0, 0, 0);
    }
} break;
case CIRCLE_EXIT_PREP:
{
    /*
     * EXIT_PREP 阶段按入环方向导出真实黑线端点。
     * 右环：导出右侧模板线，image.cpp 应涂黑模板线右侧；
     * 左环：导出左侧模板线，image.cpp 应涂黑模板线左侧。
     */
    uint8 exit_side = s_circle_lost_side;
    if((exit_side != 1U) && (exit_side != 2U))
    {
        exit_side = (circle_yaw_value() > 0.0f) ? 1U : 2U;
    }

    uint8 line_valid = (exit_side == 1U) ?
        s_left_exit_prep_line_valid : s_right_exit_prep_line_valid;
    int line_x = (exit_side == 1U) ?
        s_left_exit_prep_line_x : s_right_exit_prep_line_x;
    int line_y = (exit_side == 1U) ?
        s_left_exit_prep_line_y : s_right_exit_prep_line_y;

    uint8 have_line = 0U;
    int x_up = 0;
    int y_up = 0;
    int x_down = 0;
    int y_down = 0;

    if(line_valid)
    {
        x_up = line_x;
        y_up = line_y;

        if(x_up < USE_WIDTH_L + 1) x_up = USE_WIDTH_L + 1;
        if(x_up > USE_WIDTH_R - 1) x_up = USE_WIDTH_R - 1;

        if(y_up < old_search_end()) y_up = old_search_end();
        if(y_up > old_row_begin())  y_up = old_row_begin();

        /*
         * 右环：x_down > x_up，越往下黑区边缘越靠右；
         * 左环：x_down < x_up，越往下黑区边缘越靠左。
         */
        if(exit_side == 1U)
        {
            x_down = x_up - OLD_EXIT_PREP_BLACK_EDGE_DX_TO_BOTTOM;
        }
        else
        {
            x_down = x_up + OLD_EXIT_PREP_BLACK_EDGE_DX_TO_BOTTOM;
        }
        y_down = IMG_PROC_HEIGHT - 1;

        if(x_down < USE_WIDTH_L + 1) x_down = USE_WIDTH_L + 1;
        if(x_down > USE_WIDTH_R - 1) x_down = USE_WIDTH_R - 1;

        have_line = 1U;
    }

    int sx_up = x_up;
    int sy_up = y_up;
    int sx_down = x_down;
    int sy_down = y_down;

    if(circle_repair_line_smooth_update(CIRCLE_REPAIR_SLOT_EXIT_PREP,
                                        have_line,
                                        x_up,
                                        y_up,
                                        have_line,
                                        x_down,
                                        y_down,
                                        &sx_up,
                                        &sy_up,
                                        &sx_down,
                                        &sy_down))
    {
        circle_template_debug_set(exit_side, sx_up, sy_up, sx_down, sy_down);
    }
    else
    {
        circle_template_debug_set(0U, 0, 0, 0, 0);
    }
} break;

default:
    break;

}

    
}

// ================================================================================================
// 数组后处理
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     限制单侧边界相邻行之间的异常跳变。
// 使用方法     边界后处理阶段调用，降低单侧扫线毛刺。
// 接口属性     内部接口；参数形式：limit_boundary_jump_single(BoundaryInfo edge[], uint8 valid_rows)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void limit_boundary_jump_single(BoundaryInfo edge[], uint8 valid_rows)
{
    if(valid_rows <= 2U) return;
    for(uint8 i = 1U; i < valid_rows; ++i)
    {
        int prev = (int)edge[i - 1U].pos[0];
        int cur  = (int)edge[i].pos[0];
        int diff = cur - prev;
        if(diff > ELEMENT_JUMP_LIMIT) cur = prev + ELEMENT_JUMP_LIMIT;
        else if(diff < -ELEMENT_JUMP_LIMIT) cur = prev - ELEMENT_JUMP_LIMIT;
        edge[i].pos[0] = (uint8)clamp_col(cur);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对左右边界执行跳变限制。
// 使用方法     元素处理后处理阶段调用，补线标记会参与保护。
// 接口属性     内部接口；参数形式：element_limit_boundary_jump(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void element_limit_boundary_jump(void)
{
    uint8 rows = element_valid_rows();
    if(rows == 0U) return;
    limit_boundary_jump_single(g_image_info.filter_left, rows);
    limit_boundary_jump_single(g_image_info.filter_right, rows);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对单侧边界进行局部平滑。
// 使用方法     边界后处理阶段调用，跳过或保护本帧补线点。
// 接口属性     内部接口；参数形式：smooth_boundary_single(BoundaryInfo edge[], const uint8 *repair_mask, uint8 valid_rows)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void smooth_boundary_single(BoundaryInfo edge[], const uint8 *repair_mask, uint8 valid_rows)
{
    if(valid_rows <= 2U) return;
    for(uint8 i = 1U; i + 1U < valid_rows; ++i)
    {
        if(repair_mask[i] == 0U) continue;
        int x0 = (int)edge[i - 1U].pos[0];
        int x1 = (int)edge[i].pos[0];
        int x2 = (int)edge[i + 1U].pos[0];
        int val = (x0 + 2 * x1 + x2) / 4;
        edge[i].pos[0] = (uint8)clamp_col(val);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对左右边界进行整体平滑处理。
// 使用方法     元素处理后处理阶段调用。
// 接口属性     内部接口；参数形式：element_smooth_boundary(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void element_smooth_boundary(void)
{
    uint8 rows = element_valid_rows();
    if(rows == 0U) return;
    smooth_boundary_single(g_image_info.filter_left,  s_left_repair_mask, rows);
    smooth_boundary_single(g_image_info.filter_right, s_right_repair_mask, rows);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     对中线进行局部平滑处理。
// 使用方法     最终控制使用中线前调用，降低中线抖动。
// 接口属性     内部接口；参数形式：element_smooth_midline(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
static void element_smooth_midline(void)
{
    uint8 rows = element_valid_rows();
    if(rows <= 2U) return;
    for(uint8 i = 1U; i + 1U < rows; ++i)
    {
        int x0 = (int)g_image_info.filter_midline[i - 1U][0];
        int x1 = (int)g_image_info.filter_midline[i][0];
        int x2 = (int)g_image_info.filter_midline[i + 1U][0];
        int val = (x0 + 2 * x1 + x2) / 4;
        int prev = (int)g_image_info.filter_midline[i - 1U][0];
        if(val - prev > ELEMENT_MID_JUMP_LIMIT) val = prev + ELEMENT_MID_JUMP_LIMIT;
        if(val - prev < -ELEMENT_MID_JUMP_LIMIT) val = prev - ELEMENT_MID_JUMP_LIMIT;
        g_image_info.filter_midline[i][0] = (uint8)clamp_col(val);
    }
}

// ================================================================================================
// 对外接口
// ================================================================================================
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     初始化元素识别模块的全局状态和查表数据。
// 使用方法     系统启动时调用一次，完成半宽表、状态机和缓存初始化。
// 接口属性     对外接口；参数形式：element_init(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
void element_init(void)
{
    s_element_mode          = ELEMENT_NONE;
    s_cross_state           = CROSS_IDLE;
    s_circle_state          = CIRCLE_IDLE;

    s_element_busy          = false;
    s_circle_flag           = false;
    s_in_circle             = false;
    s_old_in_circle_state   = 0U;
    s_old_circle_state_flag = 0U;

    s_cross_candidate       = 0U;
    s_circle_candidate      = 0U;
    s_cross_confirm_frames  = 0U;
    s_circle_confirm_frames = 0U;
    s_cross_line_index      = 0U;
    s_cross_lost_frames     = 0U;
    s_cross_exit_frames     = 0U;
    s_cross_bottom_recover_frames = 0U;
    s_cross_raw_left_lost_count = 0;
    s_cross_raw_right_lost_count = 0;
    s_cross_raw_both_lost_count = 0;
    s_cross_bottom_left_valid = 0;
    s_cross_bottom_right_valid = 0;
    s_cross_bottom_checked_rows = 0;
    s_circle_lost_side      = 0U;
    s_right_inside_down_lost_frames = 0U;
    s_left_circle_point     = 0U;
    s_right_circle_point    = 0U;
    s_left_state            = 0U;
    s_right_state           = 0U;
    s_l_k_state             = 0U;
    s_r_k_state             = 0U;
    s_circle_debug_code     = 0U;

    s_dbg_right_arc_rightCount1 = 0;
    s_dbg_right_arc_arcMinCount = 0;
    s_dbg_right_arc_arcMaxCount = 0;
    s_dbg_right_arc_t_line      = 0;


    s_lane_width_ref        = 80;
    s_last_valid_midline    = USE_MIDDLE_VALUE;
    s_yaw_ref               = 0.0f;
    s_left_round_point      = 0;
    s_left_round_point_row  = start_line;
    s_left_round_up_point   = 0;
    s_left_round_up_point_x = 0;
    s_right_round_point     = USE_WIDTH_R - 1;
    s_right_round_point_row = start_line;
    s_right_round_up_point  = 0;
    s_right_round_up_point_x = 0;
    s_right_round_down_hold_cnt = 0U;
    s_right_round_up_hold_cnt = 0U;
    s_left_round_down_hold_cnt = 0U;
    s_left_round_up_hold_cnt = 0U;

    s_right_exit_prep_line_valid = 0U;
    s_right_exit_prep_line_x = 0;
    s_right_exit_prep_line_y = 0;
    s_left_exit_prep_line_valid = 0U;
    s_left_exit_prep_line_x = 0;
    s_left_exit_prep_line_y = 0;

    memset(s_last_point, 0, sizeof(s_last_point));
    circle_repair_smooth_clear();
    memset(s_half_road_width, 0, sizeof(s_half_road_width));
    memset(s_old_left_pos, 0, sizeof(s_old_left_pos));
    memset(s_old_right_pos, 0, sizeof(s_old_right_pos));
    memset(s_old_midline, 0, sizeof(s_old_midline));
    memset(s_old_left_sign, 0, sizeof(s_old_left_sign));
    memset(s_old_right_sign, 0, sizeof(s_old_right_sign));
    clear_repair_mask();
    s_cross_lu = {0U, 0, 0, 0};
    s_cross_ld = {0U, 0, 0, 0};
    s_cross_ru = {0U, 0, 0, 0};
    s_cross_rd = {0U, 0, 0, 0};
    s_cross_left_patch_model  = {0U, 0.0f, 0, 0, 0, 0};
    s_cross_right_patch_model = {0U, 0.0f, 0, 0, 0, 0};
    circle_template_debug_clear();
    beep_reset();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     复位元素识别模块运行状态。
// 使用方法     需要重新开始识别或调试强制重置时调用。
// 接口属性     对外接口；参数形式：element_reset(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
void element_reset(void)
{
    element_init();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     获取当前元素模式。
// 使用方法     外部控制或调试界面调用，用于判断当前处于普通、环岛或十字等模式。
// 接口属性     对外接口；参数形式：element_get_mode(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
uint8 element_get_mode(void)
{
    return (uint8)s_element_mode;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     获取当前元素状态值。
// 使用方法     外部控制或调试界面调用，用于显示当前状态机阶段。
// 接口属性     对外接口；参数形式：element_get_state(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
uint8 element_get_state(void)
{
    if(s_element_mode == ELEMENT_CROSSROAD) return (uint8)s_cross_state;
    if(s_element_mode == ELEMENT_ROUNDABOUT) return (uint8)s_circle_state;
    return 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     判断元素模块是否处于特殊元素处理状态。
// 使用方法     外部控制逻辑调用，用于决定是否按普通巡线还是特殊元素策略控制。
// 接口属性     对外接口；参数形式：element_is_busy(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
bool element_is_busy(void)
{
    return s_element_busy;
}




//-------------------------------------------------------------------------------------------------------------------
// 函数作用     执行一帧完整元素识别、补线和后处理流程。
// 使用方法     图像扫线完成后每帧调用一次，是本文件的主处理入口。
// 接口属性     对外接口；参数形式：element_process(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
void element_process(void)
{
    // 每帧推进元素确认时触发的蜂鸣器脉冲序列。
    beep_update();

    // 十字退出标志是一帧脉冲；本帧若发生退出，状态机随后会重新置 1。
    s_cross_exit_frames = 0U;

    uint8 rows = element_valid_rows();

    if(rows < ELEMENT_MIN_VALID_ROWS)
    {
        if(s_element_busy) element_reset();
        return;
    }

    clear_repair_mask();
    build_half_road_width_table();
    circle_import_old_view();

    // 从这里开始本帧尚未执行任何十字补线，按实际图像坐标统计原始丢线；
    // 最下方 30 行不参与十字粗检测。环岛继续使用自己的旧视图统计。
    cross_count_raw_lost_excluding_bottom();

    // 环岛优先。只有环岛为空闲时，才允许十字逻辑接管，避免十字误抢环岛入口。
#if ELEMENT_ENABLE_ROUNDABOUT
    circle_state_update();

if(s_circle_state != CIRCLE_IDLE)
{
    cross_force_disable_internal();

    if(s_circle_state == CIRCLE_LEFT_PREP ||
       s_circle_state == CIRCLE_RIGHT_PREP ||
       s_circle_state == CIRCLE_LEFT_ENTER ||
       s_circle_state == CIRCLE_RIGHT_ENTER ||
       s_circle_state == CIRCLE_LEFT_INSIDE ||
       s_circle_state == CIRCLE_RIGHT_INSIDE ||
       s_circle_state == CIRCLE_EXIT_PREP)
    {
      circle_build_final_boundary();

      element_limit_boundary_jump();
      element_smooth_boundary();

      build_half_road_width_table();
      rebuild_midline_from_current_edges();

      element_smooth_midline();
    }

    update_last_midline();
    return;
}
#else
    if(s_circle_state != CIRCLE_IDLE || s_circle_flag || s_element_mode == ELEMENT_ROUNDABOUT)
    {
        circle_reset_internal();
    }
#endif

#if ELEMENT_ENABLE_CROSSROAD
    const bool cross_was_active = (s_cross_state != CROSS_IDLE);
    cross_state_update();

    // 十字刚刚退出时，当前 filter_left/right 已经是本帧真实扫线结果。
    // 立即清掉十字锁定中线，避免上一帧的补线参考继续影响普通巡线。
    if(cross_was_active && s_cross_state == CROSS_IDLE)
    {
        s_last_valid_midline = clamp_col(get_bottom_valid_midline());
        rebuild_midline_from_current_edges();
    }

    if(s_cross_state != CROSS_IDLE)
    {
        // CROSS_DETECTED 阶段不产生补线；CROSS_INSIDE 的补线只作为虚拟边界参与中线重构。
        cross_build_final_boundary();
        build_half_road_width_table();
        rebuild_midline_from_current_edges();
        element_smooth_midline();
        update_last_midline();
        return;
    }
#else
    cross_force_disable_internal();
#endif

    update_last_midline();
}

#if (1 == ELEMENT_ENABLE_DEBUG_API)
//-------------------------------------------------------------------------------------------------------------------
// 函数作用     导出元素调试快照。
// 使用方法     屏幕或上位机调试模块调用，用于显示模式、状态、角点和关键计数。
// 接口属性     对外接口；参数形式：element_debug_get_snapshot(element_debug_snapshot_struct *snapshot)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
void element_debug_get_snapshot(element_debug_snapshot_struct *snapshot)
{
    if(snapshot == NULL) return;
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->mode               = (uint8)s_element_mode;
    snapshot->cross_state        = (uint8)s_cross_state;
    snapshot->circle_state       = (uint8)s_circle_state;
    snapshot->busy               = s_element_busy ? 1U : 0U;
    snapshot->cross_candidate    = s_cross_candidate;
    snapshot->circle_candidate   = s_circle_candidate;
    snapshot->cross_line_index   = s_cross_line_index;
    snapshot->circle_lost_side   = s_circle_lost_side;
    snapshot->valid_rows         = element_valid_rows();

    snapshot->left_lost_rows     = s_cross_raw_left_lost_count;
    snapshot->right_lost_rows    = s_cross_raw_right_lost_count;
    snapshot->both_lost_rows     = s_cross_raw_both_lost_count;
    snapshot->lane_width_ref     = (int16)s_lane_width_ref;
    snapshot->last_midline       = (int16)s_last_valid_midline;

    snapshot->search_stop_line   = (int16)s_old_search_stop_line;
    snapshot->circle_flag        = s_circle_flag ? 1U : 0U;
    snapshot->in_circle_state    = s_old_in_circle_state;
    snapshot->left_state         = s_left_state;
    snapshot->right_state        = s_right_state;
    snapshot->l_k_state          = s_l_k_state;
    snapshot->r_k_state          = s_r_k_state;
    snapshot->left_circle_point  = s_left_circle_point;
    snapshot->right_circle_point = s_right_circle_point;

    snapshot->dist_left  = stage_distance_left();
    snapshot->dist_right = stage_distance_right();
    snapshot->circle_total_left  = circle_enter_distance_left();
    snapshot->circle_total_right = circle_enter_distance_right();
snapshot->yaw_ref            = s_yaw_ref;
snapshot->current_yaw        = yaw;
snapshot->delta_yaw          = circle_yaw_value();
snapshot->circle_yaw         = circle_yaw_value();
// 复刻 RIGHT_ENTER -> INSIDE 跳转时真正使用的 leftCount
{
    int leftCount = 0;
    for(int y = old_row_begin() - 1; y > old_row_begin() - 20 && y >= 0; --y)
    {
        if(old_left_sign_at(y)) leftCount++;
    }
    snapshot->right_enter_leftCount = (int16)leftCount;
}

snapshot->circle_debug_code  = s_circle_debug_code;    snapshot->circle_debug_code  = s_circle_debug_code;

    snapshot->right_arc_rightCount1 = (int16)s_dbg_right_arc_rightCount1;
snapshot->right_arc_arcMinCount = (int16)s_dbg_right_arc_arcMinCount;
snapshot->right_arc_arcMaxCount = (int16)s_dbg_right_arc_arcMaxCount;
snapshot->right_arc_t_line      = (int16)s_dbg_right_arc_t_line;


    // 双角点环岛入口调试导出
    // 右环 debug：11/12/13/14/15/16/8
    // 左环 debug：21/22/23/24/25/26/9
    const uint8 dbg = s_circle_debug_code;

    const uint8 dbg_right_active = (((dbg >= 11U) && (dbg <= 16U)) || (dbg == 8U)) ? 1U : 0U;
    const uint8 dbg_left_active  = (((dbg >= 21U) && (dbg <= 26U)) || (dbg == 9U)) ? 1U : 0U;

    // 角点显示使用当前帧调试量，不再用缓存点或 debug_code 推导。
    snapshot->right_down_y = (dbg_right_active && (s_dbg_right_down_y_current > 0)) ? (int16)s_dbg_right_down_y_current : 0;
    snapshot->right_up_y   = (dbg_right_active && (s_dbg_right_up_y_current > 0)) ? (int16)s_dbg_right_up_y_current : 0;

    snapshot->left_down_y  = (dbg_left_active && (s_dbg_left_down_y_current > 0)) ? (int16)s_dbg_left_down_y_current : 0;
    snapshot->left_up_y    = (dbg_left_active && (s_dbg_left_up_y_current > 0)) ? (int16)s_dbg_left_up_y_current : 0;

    snapshot->right_two_corner_ok = dbg_right_active ? s_dbg_right_two_corner_ok_current : 0U;
    snapshot->left_two_corner_ok  = dbg_left_active ? s_dbg_left_two_corner_ok_current : 0U;

    snapshot->rc_left_slope_ok        = dbg_right_active ? s_dbg_rc_left_slope_ok_current : 0U;
    snapshot->rc_right_down_slope_ok  = dbg_right_active ? s_dbg_rc_right_slope_ok_current : 0U;
    snapshot->rc_left_straight_ok     = dbg_right_active ? s_dbg_rc_left_straight_ok_current : 0U;
    snapshot->rc_right_straight_ok    = dbg_right_active ? s_dbg_rc_right_straight_ok_current : 0U;

    snapshot->lc_right_slope_ok       = dbg_left_active ? s_dbg_lc_right_slope_ok_current : 0U;
    snapshot->lc_left_down_slope_ok   = dbg_left_active ? s_dbg_lc_left_slope_ok_current : 0U;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     导出新十字状态机当前的左右补线端点。
//-------------------------------------------------------------------------------------------------------------------
void element_debug_get_cross_patch_lines_simple(
    uint8 *left_valid,  int16 *left_x1,  int16 *left_y1,  int16 *left_x2,  int16 *left_y2,
    uint8 *right_valid, int16 *right_x1, int16 *right_y1, int16 *right_x2, int16 *right_y2)
{
    uint8 left_ready = 0U;
    uint8 right_ready = 0U;
    int lx1 = 0, ly1 = 0, lx2 = 0, ly2 = 0;
    int rx1 = 0, ry1 = 0, rx2 = 0, ry2 = 0;
    cross_collect_patch_lines(&left_ready, &lx1, &ly1, &lx2, &ly2,
                              &right_ready, &rx1, &ry1, &rx2, &ry2);

    if(left_valid != NULL) *left_valid = left_ready;
    if(left_x1 != NULL) *left_x1 = (int16)lx1;
    if(left_y1 != NULL) *left_y1 = (int16)ly1;
    if(left_x2 != NULL) *left_x2 = (int16)lx2;
    if(left_y2 != NULL) *left_y2 = (int16)ly2;

    if(right_valid != NULL) *right_valid = right_ready;
    if(right_x1 != NULL) *right_x1 = (int16)rx1;
    if(right_y1 != NULL) *right_y1 = (int16)ry1;
    if(right_x2 != NULL) *right_x2 = (int16)rx2;
    if(right_y2 != NULL) *right_y2 = (int16)ry2;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     导出新十字状态、补线前丢线统计和当前帧四角点。
//-------------------------------------------------------------------------------------------------------------------
void element_debug_get_cross_snapshot_simple(
    uint8 *mode,
    uint8 *cross_state,
    uint8 *cross_candidate,
    uint8 *cross_confirm_frames,
    uint8 *cross_line_index,
    uint8 *cross_lost_frames,
    uint8 *cross_exit_frames,
    uint8 *valid_rows,
    int16 *left_lost_rows,
    int16 *right_lost_rows,
    int16 *both_lost_rows,
    int16 *bottom_left_valid,
    int16 *bottom_right_valid,
    int16 *bottom_checked_rows,
    uint8 *lu_valid, int16 *lu_x, int16 *lu_y,
    uint8 *ld_valid, int16 *ld_x, int16 *ld_y,
    uint8 *ru_valid, int16 *ru_x, int16 *ru_y,
    uint8 *rd_valid, int16 *rd_x, int16 *rd_y)
{
    if(mode != NULL) *mode = (uint8)s_element_mode;
    if(cross_state != NULL) *cross_state = (uint8)s_cross_state;
    if(cross_candidate != NULL) *cross_candidate = s_cross_candidate;
    if(cross_confirm_frames != NULL) *cross_confirm_frames = s_cross_confirm_frames;
    if(cross_line_index != NULL) *cross_line_index = s_cross_line_index;
    if(cross_lost_frames != NULL) *cross_lost_frames = s_cross_lost_frames;
    if(cross_exit_frames != NULL) *cross_exit_frames = s_cross_exit_frames;
    if(valid_rows != NULL) *valid_rows = element_valid_rows();

    if(left_lost_rows != NULL) *left_lost_rows = s_cross_raw_left_lost_count;
    if(right_lost_rows != NULL) *right_lost_rows = s_cross_raw_right_lost_count;
    if(both_lost_rows != NULL) *both_lost_rows = s_cross_raw_both_lost_count;
    if(bottom_left_valid != NULL) *bottom_left_valid = s_cross_bottom_left_valid;
    if(bottom_right_valid != NULL) *bottom_right_valid = s_cross_bottom_right_valid;
    if(bottom_checked_rows != NULL) *bottom_checked_rows = s_cross_bottom_checked_rows;

    if(lu_valid != NULL) *lu_valid = s_cross_lu.valid;
    if(lu_x != NULL) *lu_x = s_cross_lu.valid ? (int16)s_cross_lu.x : 0;
    if(lu_y != NULL) *lu_y = s_cross_lu.valid ? (int16)s_cross_lu.y : 0;
    if(ld_valid != NULL) *ld_valid = s_cross_ld.valid;
    if(ld_x != NULL) *ld_x = s_cross_ld.valid ? (int16)s_cross_ld.x : 0;
    if(ld_y != NULL) *ld_y = s_cross_ld.valid ? (int16)s_cross_ld.y : 0;
    if(ru_valid != NULL) *ru_valid = s_cross_ru.valid;
    if(ru_x != NULL) *ru_x = s_cross_ru.valid ? (int16)s_cross_ru.x : 0;
    if(ru_y != NULL) *ru_y = s_cross_ru.valid ? (int16)s_cross_ru.y : 0;
    if(rd_valid != NULL) *rd_valid = s_cross_rd.valid;
    if(rd_x != NULL) *rd_x = s_cross_rd.valid ? (int16)s_cross_rd.x : 0;
    if(rd_y != NULL) *rd_y = s_cross_rd.valid ? (int16)s_cross_rd.y : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     导出补线前检测得到的四个当前帧角点坐标。
//-------------------------------------------------------------------------------------------------------------------
void element_debug_get_cross_corner_probe_points_simple(
    uint8 *lu_found, int16 *lu_x, int16 *lu_y, int16 *lu_idx,
    uint8 *ld_found, int16 *ld_x, int16 *ld_y, int16 *ld_idx,
    uint8 *ru_found, int16 *ru_x, int16 *ru_y, int16 *ru_idx,
    uint8 *rd_found, int16 *rd_x, int16 *rd_y, int16 *rd_idx)
{
    if(lu_found != NULL) *lu_found = s_cross_lu.valid;
    if(lu_x != NULL) *lu_x = s_cross_lu.valid ? (int16)s_cross_lu.x : 0;
    if(lu_y != NULL) *lu_y = s_cross_lu.valid ? (int16)s_cross_lu.y : 0;
    if(lu_idx != NULL) *lu_idx = s_cross_lu.valid ? (int16)s_cross_lu.idx : 0;

    if(ld_found != NULL) *ld_found = s_cross_ld.valid;
    if(ld_x != NULL) *ld_x = s_cross_ld.valid ? (int16)s_cross_ld.x : 0;
    if(ld_y != NULL) *ld_y = s_cross_ld.valid ? (int16)s_cross_ld.y : 0;
    if(ld_idx != NULL) *ld_idx = s_cross_ld.valid ? (int16)s_cross_ld.idx : 0;

    if(ru_found != NULL) *ru_found = s_cross_ru.valid;
    if(ru_x != NULL) *ru_x = s_cross_ru.valid ? (int16)s_cross_ru.x : 0;
    if(ru_y != NULL) *ru_y = s_cross_ru.valid ? (int16)s_cross_ru.y : 0;
    if(ru_idx != NULL) *ru_idx = s_cross_ru.valid ? (int16)s_cross_ru.idx : 0;

    if(rd_found != NULL) *rd_found = s_cross_rd.valid;
    if(rd_x != NULL) *rd_x = s_cross_rd.valid ? (int16)s_cross_rd.x : 0;
    if(rd_y != NULL) *rd_y = s_cross_rd.valid ? (int16)s_cross_rd.y : 0;
    if(rd_idx != NULL) *rd_idx = s_cross_rd.valid ? (int16)s_cross_rd.idx : 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数作用     外部调试用强制复位元素状态。
// 使用方法     调试按键或上位机命令调用，用于清空环岛/十字状态机。
// 接口属性     对外接口；参数形式：element_debug_force_reset(void)。
// 备注说明     本注释只说明函数用途，不改变任何执行逻辑。
//-------------------------------------------------------------------------------------------------------------------
void element_debug_force_reset(void)
{
    element_reset();
}
#endif
