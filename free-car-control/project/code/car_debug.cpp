#include "car_init.hpp"
#include "car_debug.hpp"
#include "imu.hpp"
#include "image.hpp"
#include "motor.hpp"
#include "encoder.hpp"
#include "menu.hpp"
#include "element.hpp"
#include "red_roi.hpp"
#include "lq_ncnn.hpp"
#include "fan.hpp"
#include "ramp_detect.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// 调试与可视化模块：
// - 向上位机助手发送图像与边线数据
// - 在 IPS200 上绘制运行信息
// - 支持在线参数调试

uint8 tcp_left_boundary_x[IMG_PROC_HEIGHT], tcp_right_boundary_x[IMG_PROC_HEIGHT], tcp_middle_line_x[IMG_PROC_HEIGHT],tcp_max_white_x[IMG_PROC_HEIGHT];
uint8 tcp_left_boundary_y[IMG_PROC_HEIGHT], tcp_right_boundary_y[IMG_PROC_HEIGHT], tcp_middle_line_y[IMG_PROC_HEIGHT],tcp_max_white_y[IMG_PROC_HEIGHT];
TcpDebugImageMode g_tcp_debug_image_mode = TCP_DEBUG_IMAGE_GRAY;//TCP_DEBUG_IMAGE_COLOR;//TCP_DEBUG_IMAGE_RED_MASK;
TcpRedMaskRectMode g_tcp_red_mask_rect_mode = TCP_RED_MASK_RECT_ROI;

extern float yaw;
extern float gyro_z;
extern float f_gyro_z;
extern int16 raw_gyro_z;

extern float f_encoder_left;
extern float f_encoder_right;

void ips200_show_cross_debug(void);

#if (1 == ELEMENT_ENABLE_DEBUG_API)
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
    uint8 *rd_valid, int16 *rd_x, int16 *rd_y);

void element_debug_get_cross_corner_probe_points_simple(
    uint8 *lu_found, int16 *lu_x, int16 *lu_y, int16 *lu_idx,
    uint8 *ld_found, int16 *ld_x, int16 *ld_y, int16 *ld_idx,
    uint8 *ru_found, int16 *ru_x, int16 *ru_y, int16 *ru_idx,
    uint8 *rd_found, int16 *rd_x, int16 *rd_y, int16 *rd_idx);

void element_debug_get_cross_patch_lines_simple(
    uint8 *left_valid,  int16 *left_x1,  int16 *left_y1,  int16 *left_x2,  int16 *left_y2,
    uint8 *right_valid, int16 *right_x1, int16 *right_y1, int16 *right_x2, int16 *right_y2);
#endif

typedef enum
{
    // 避障页作为第一页，方便优先调试候选、入态和退出条件。
    DEBUG_VIEW_OBSTACLE = 0,
    DEBUG_VIEW_RAMP = 1,
    DEBUG_VIEW_CROSS = 2,
    DEBUG_VIEW_CIRCLE = 3,
    DEBUG_VIEW_RUN = 4,
    DEBUG_VIEW_IMU = 5,
    DEBUG_VIEW_COUNT
} DebugViewMode;

static DebugViewMode g_debug_view_mode = DEBUG_VIEW_OBSTACLE;

//roi_debug
namespace
{
TcpDebugImageMode s_tcp_configured_image_mode = static_cast<TcpDebugImageMode>(-1);
bool s_tcp_boundary_configured = false;

static void ensure_tcp_debug_stream_config()
{
    if (g_tcp_debug_image_mode != s_tcp_configured_image_mode)
    {
        if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_GRAY)
        {
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                         send_gray_image[0],
                                                         IMG_PROC_WIDTH,
                                                         IMG_PROC_HEIGHT);
        }
        else
        {
            seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_SCC8660,
                                                         send_color_image_120[0],
                                                         IMG_PROC_WIDTH,
                                                         IMG_PROC_HEIGHT);
        }
        s_tcp_configured_image_mode = g_tcp_debug_image_mode;
    }

    if (!s_tcp_boundary_configured)
    {
        seekfree_assistant_camera_boundary_config(XY_BOUNDARY,
                                                  IMG_PROC_HEIGHT,
                                                  tcp_left_boundary_x,
                                                  tcp_right_boundary_x,
                                                  tcp_middle_line_x,
                                                  tcp_left_boundary_y,
                                                  tcp_right_boundary_y,
                                                  tcp_middle_line_y);
        s_tcp_boundary_configured = true;
    }
}

static const char* debug_view_name(DebugViewMode mode)
{
    switch(mode)
    {
    case DEBUG_VIEW_CIRCLE: return "CIRCLE";
    case DEBUG_VIEW_CROSS: return "CROSS";
    case DEBUG_VIEW_IMU: return "IMU";
    case DEBUG_VIEW_RAMP: return "RAMP";
    case DEBUG_VIEW_OBSTACLE: return "OBSTACLE";
    default: return "RUN";
    }
}

static void debug_view_update_by_key(void)
{
    static uint8 key_latch = 0U;
    const uint8 key_long = (key_get_state(1) == KEY_LONG_PRESS) ? 1U : 0U;

    // 只在长按事件刚出现时切换一次，避免 tcp_main_use() 和 ips200_main_use() 同一轮都调用时连跳两页。
    if(key_long && !key_latch)
    {
        g_debug_view_mode = static_cast<DebugViewMode>((static_cast<int>(g_debug_view_mode) + 1) % static_cast<int>(DEBUG_VIEW_COUNT));
        printf("debug view -> %s\r\n", debug_view_name(g_debug_view_mode));
    }

    key_latch = key_long;
}

static void draw_debug_cross_point(int x, int y, const uint16 color)
{
    if(x <= 1 || x >= (IMG_PROC_WIDTH - 2) || y <= 1 || y >= (IMG_PROC_HEIGHT - 2))
    {
        return;
    }

    ips200.draw_point((uint16)x,     (uint16)y,     color);
    ips200.draw_point((uint16)(x-1), (uint16)y,     color);
    ips200.draw_point((uint16)(x+1), (uint16)y,     color);
    ips200.draw_point((uint16)x,     (uint16)(y-1), color);
    ips200.draw_point((uint16)x,     (uint16)(y+1), color);
}

static bool find_debug_edge_x_by_y(int y, bool right_side, int* out_x, int* out_y)
{
    if(out_x == nullptr || out_y == nullptr)
    {
        return false;
    }
    if(y <= 1 || y >= (IMG_PROC_HEIGHT - 2))
    {
        return false;
    }

    for(int i = 0; i < IMG_PROC_HEIGHT; ++i)
    {
        if(right_side)
        {
            if((g_image_info.filter_right[i].sign != 0U) &&
               ((int)g_image_info.filter_right[i].pos[1] == y))
            {
                int x = (int)g_image_info.filter_right[i].pos[0];
                if(x > 1 && x < (IMG_PROC_WIDTH - 2))
                {
                    *out_x = x;
                    *out_y = y;
                    return true;
                }
            }
        }
        else
        {
            if((g_image_info.filter_left[i].sign != 0U) &&
               ((int)g_image_info.filter_left[i].pos[1] == y))
            {
                int x = (int)g_image_info.filter_left[i].pos[0];
                if(x > 1 && x < (IMG_PROC_WIDTH - 2))
                {
                    *out_x = x;
                    *out_y = y;
                    return true;
                }
            }
        }
    }

    return false;
}

static bool get_circle_down_corner_debug_point(int* out_x, int* out_y, uint8* out_side)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    if(out_x == nullptr || out_y == nullptr || out_side == nullptr)
    {
        return false;
    }

    element_debug_snapshot_struct snap = {0};
    element_debug_get_snapshot(&snap);

    // 优先显示当前正在判的环岛方向：2-右环下角点，1-左环下角点。
    if((snap.circle_lost_side == 2U) &&
       (snap.right_down_y > 0) &&
       find_debug_edge_x_by_y((int)snap.right_down_y, true, out_x, out_y))
    {
        *out_side = 2U;
        return true;
    }

    if((snap.circle_lost_side == 1U) &&
       (snap.left_down_y > 0) &&
       find_debug_edge_x_by_y((int)snap.left_down_y, false, out_x, out_y))
    {
        *out_side = 1U;
        return true;
    }

    // 兜底：还没正式入态但某一侧已经找到下角点时，也显示出来。
    if((snap.right_down_y > 0) &&
       find_debug_edge_x_by_y((int)snap.right_down_y, true, out_x, out_y))
    {
        *out_side = 2U;
        return true;
    }

    if((snap.left_down_y > 0) &&
       find_debug_edge_x_by_y((int)snap.left_down_y, false, out_x, out_y))
    {
        *out_side = 1U;
        return true;
    }
#else
    (void)out_x;
    (void)out_y;
    (void)out_side;
#endif
    return false;
}


typedef struct
{
    uint8 mode;
    uint8 cross_state;
    uint8 cross_candidate;
    uint8 cross_confirm_frames;
    uint8 cross_line_index;
    uint8 cross_lost_frames;
    uint8 cross_exit_frames;
    uint8 valid_rows;
    int16 left_lost_rows;
    int16 right_lost_rows;
    int16 both_lost_rows;
    int16 bottom_left_valid;
    int16 bottom_right_valid;
    int16 bottom_checked_rows;
    uint8 lu_valid; int16 lu_x; int16 lu_y;
    uint8 ld_valid; int16 ld_x; int16 ld_y;
    uint8 ru_valid; int16 ru_x; int16 ru_y;
    uint8 rd_valid; int16 rd_x; int16 rd_y;
    uint8 patch_valid;
    uint8 patch_side;      // 兼容字段：v3 中表示有效黑线数量，0/1/2。
    uint8 left_patch_valid;
    uint8 right_patch_valid;
    int16 left_patch_x1;  int16 left_patch_y1;
    int16 left_patch_x2;  int16 left_patch_y2;
    int16 right_patch_x1; int16 right_patch_y1;
    int16 right_patch_x2; int16 right_patch_y2;
    uint8 probe_lu_found; int16 probe_lu_x; int16 probe_lu_y; int16 probe_lu_idx;
    uint8 probe_ld_found; int16 probe_ld_x; int16 probe_ld_y; int16 probe_ld_idx;
    uint8 probe_ru_found; int16 probe_ru_x; int16 probe_ru_y; int16 probe_ru_idx;
    uint8 probe_rd_found; int16 probe_rd_x; int16 probe_rd_y; int16 probe_rd_idx;
} cross_screen_debug_struct;

static void get_cross_screen_debug(cross_screen_debug_struct *dbg)
{
    if(dbg == nullptr)
    {
        return;
    }
    std::memset(dbg, 0, sizeof(*dbg));
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    element_debug_get_cross_snapshot_simple(
        &dbg->mode,
        &dbg->cross_state,
        &dbg->cross_candidate,
        &dbg->cross_confirm_frames,
        &dbg->cross_line_index,
        &dbg->cross_lost_frames,
        &dbg->cross_exit_frames,
        &dbg->valid_rows,
        &dbg->left_lost_rows,
        &dbg->right_lost_rows,
        &dbg->both_lost_rows,
        &dbg->bottom_left_valid,
        &dbg->bottom_right_valid,
        &dbg->bottom_checked_rows,
        &dbg->lu_valid, &dbg->lu_x, &dbg->lu_y,
        &dbg->ld_valid, &dbg->ld_x, &dbg->ld_y,
        &dbg->ru_valid, &dbg->ru_x, &dbg->ru_y,
        &dbg->rd_valid, &dbg->rd_x, &dbg->rd_y);

    element_debug_get_cross_corner_probe_points_simple(
        &dbg->probe_lu_found, &dbg->probe_lu_x, &dbg->probe_lu_y, &dbg->probe_lu_idx,
        &dbg->probe_ld_found, &dbg->probe_ld_x, &dbg->probe_ld_y, &dbg->probe_ld_idx,
        &dbg->probe_ru_found, &dbg->probe_ru_x, &dbg->probe_ru_y, &dbg->probe_ru_idx,
        &dbg->probe_rd_found, &dbg->probe_rd_x, &dbg->probe_rd_y, &dbg->probe_rd_idx);

    // v3：显示层使用 element.cpp 导出的虚拟走廊补线，不再自己用 LU-LD / RU-RD 拼旧角点线。
    uint8 left_valid = 0U;
    uint8 right_valid = 0U;
    int16 lx1 = 0, ly1 = 0, lx2 = 0, ly2 = 0;
    int16 rx1 = 0, ry1 = 0, rx2 = 0, ry2 = 0;

    element_debug_get_cross_patch_lines_simple(
        &left_valid,  &lx1, &ly1, &lx2, &ly2,
        &right_valid, &rx1, &ry1, &rx2, &ry2);

    if(left_valid)
    {
        dbg->left_patch_valid = 1U;
        dbg->left_patch_x1 = lx1;
        dbg->left_patch_y1 = ly1;
        dbg->left_patch_x2 = lx2;
        dbg->left_patch_y2 = ly2;
    }

    if(right_valid)
    {
        dbg->right_patch_valid = 1U;
        dbg->right_patch_x1 = rx1;
        dbg->right_patch_y1 = ry1;
        dbg->right_patch_x2 = rx2;
        dbg->right_patch_y2 = ry2;
    }

    dbg->patch_side = (uint8)((dbg->left_patch_valid ? 1U : 0U) + (dbg->right_patch_valid ? 1U : 0U));
    dbg->patch_valid = (dbg->patch_side > 0U) ? 1U : 0U;
    #endif
}

static void DrawCrossUpperCornerProbeDebug(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    cross_screen_debug_struct dbg = {0};
    get_cross_screen_debug(&dbg);

    // 四角点测试页：显示当前帧重新探测到的 LU/LD/RU/RD。
    // LU=青色，LD=绿色，RU=黄色，RD=品红。
    if(dbg.probe_lu_found)
    {
        draw_debug_cross_point((int)dbg.probe_lu_x, (int)dbg.probe_lu_y, RGB565_CYAN);
    }
    if(dbg.probe_ld_found)
    {
        draw_debug_cross_point((int)dbg.probe_ld_x, (int)dbg.probe_ld_y, RGB565_GREEN);
    }
    if(dbg.probe_ru_found)
    {
        draw_debug_cross_point((int)dbg.probe_ru_x, (int)dbg.probe_ru_y, RGB565_YELLOW);
    }
    if(dbg.probe_rd_found)
    {
        draw_debug_cross_point((int)dbg.probe_rd_x, (int)dbg.probe_rd_y, RGB565_MAGENTA);
    }
#endif
}

static void DrawCrossPatchLinesDebug(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    cross_screen_debug_struct dbg = {0};
    get_cross_screen_debug(&dbg);

    const bool left_coords_valid =
        dbg.left_patch_x1 >= 0 && dbg.left_patch_x1 < IMG_PROC_WIDTH &&
        dbg.left_patch_x2 >= 0 && dbg.left_patch_x2 < IMG_PROC_WIDTH &&
        dbg.left_patch_y1 >= 0 && dbg.left_patch_y1 < IMG_PROC_HEIGHT &&
        dbg.left_patch_y2 >= 0 && dbg.left_patch_y2 < IMG_PROC_HEIGHT;
    const bool right_coords_valid =
        dbg.right_patch_x1 >= 0 && dbg.right_patch_x1 < IMG_PROC_WIDTH &&
        dbg.right_patch_x2 >= 0 && dbg.right_patch_x2 < IMG_PROC_WIDTH &&
        dbg.right_patch_y1 >= 0 && dbg.right_patch_y1 < IMG_PROC_HEIGHT &&
        dbg.right_patch_y2 >= 0 && dbg.right_patch_y2 < IMG_PROC_HEIGHT;

    // 十字补线只参与中线重构，不写回原始边界；这里单独用异色显示虚拟线。
    // 左虚拟线=蓝色，右虚拟线=粉色。
    if(dbg.left_patch_valid && left_coords_valid)
    {
        ips200.draw_line((uint16)dbg.left_patch_x1,
                         (uint16)dbg.left_patch_y1,
                         (uint16)dbg.left_patch_x2,
                         (uint16)dbg.left_patch_y2,
                         RGB565_BLUE);
        draw_debug_cross_point((int)dbg.left_patch_x1, (int)dbg.left_patch_y1, RGB565_BLUE);
        draw_debug_cross_point((int)dbg.left_patch_x2, (int)dbg.left_patch_y2, RGB565_BLUE);
    }

    if(dbg.right_patch_valid && right_coords_valid)
    {
        ips200.draw_line((uint16)dbg.right_patch_x1,
                         (uint16)dbg.right_patch_y1,
                         (uint16)dbg.right_patch_x2,
                         (uint16)dbg.right_patch_y2,
                         RGB565_PINK);
        draw_debug_cross_point((int)dbg.right_patch_x1, (int)dbg.right_patch_y1, RGB565_PINK);
        draw_debug_cross_point((int)dbg.right_patch_x2, (int)dbg.right_patch_y2, RGB565_PINK);
    }
#endif
}

static void draw_tcp_gray_patch_pixel(int x, int y, bool left_side)
{
    if(x < 1 || x >= (IMG_PROC_WIDTH - 1) || y < 0 || y >= IMG_PROC_HEIGHT)
    {
        return;
    }

    // 灰度上位机中使用互为反色的三像素线，保证补线落在黑区或白区都可见。
    const uint8 center = left_side ? 0U : 255U;
    const uint8 edge = left_side ? 255U : 0U;
    // send_gray_image 的物理声明仍是 UVC_WIDTH 步长，但逐飞助手当前发送的是
    // 紧凑的 IMG_PROC_WIDTH x IMG_PROC_HEIGHT 数据，必须按 160 列的发送步长寻址。
    uint8 *send_ptr = &send_gray_image[0][0];
    const int offset = y * IMG_PROC_WIDTH + x;
    send_ptr[offset - 1] = edge;
    send_ptr[offset] = center;
    send_ptr[offset + 1] = edge;
}

static void draw_tcp_gray_patch_line(int x0, int y0, int x1, int y1, bool left_side)
{
    int dx = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    int sx = (x0 < x1) ? 1 : -1;
    int dy_abs = (y1 >= y0) ? (y1 - y0) : (y0 - y1);
    int dy = -dy_abs;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for(;;)
    {
        draw_tcp_gray_patch_pixel(x0, y0, left_side);
        if(x0 == x1 && y0 == y1)
        {
            break;
        }

        const int e2 = 2 * err;
        if(e2 >= dy)
        {
            err += dy;
            x0 += sx;
        }
        if(e2 <= dx)
        {
            err += dx;
            y0 += sy;
        }
    }
}

static void OverlayCrossPatchLinesToTcpGrayImage(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    cross_screen_debug_struct dbg = {0};
    get_cross_screen_debug(&dbg);

    if(dbg.left_patch_valid)
    {
        draw_tcp_gray_patch_line((int)dbg.left_patch_x1, (int)dbg.left_patch_y1,
                                 (int)dbg.left_patch_x2, (int)dbg.left_patch_y2,
                                 true);
    }
    if(dbg.right_patch_valid)
    {
        draw_tcp_gray_patch_line((int)dbg.right_patch_x1, (int)dbg.right_patch_y1,
                                 (int)dbg.right_patch_x2, (int)dbg.right_patch_y2,
                                 false);
    }
#endif
}

static void DrawCircleDownCornerDebug(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    int x = 0;
    int y = 0;
    uint8 side = 0U;
    if(get_circle_down_corner_debug_point(&x, &y, &side))
    {
        // 下角点专用颜色：品红。和左右边界/中线/补线颜色区分开。
        draw_debug_cross_point(x, y, RGB565_MAGENTA);

        // 右下角点再额外加一圈粉色，调右环时更醒目；左下角点只保留品红十字。
        if(side == 2U)
        {
            draw_debug_cross_point(x, y, RGB565_PINK);
        }
    }
#endif
}

static uint8 debug_clamp_u8_local(int value, int low, int high)
{
    if(value < low) value = low;
    if(value > high) value = high;
    return static_cast<uint8>(value);
}

static void overlay_circle_down_corner_to_tcp_boundary(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    int x = 0;
    int y = 0;
    uint8 side = 0U;
    if(!get_circle_down_corner_debug_point(&x, &y, &side))
    {
        return;
    }

    (void)side;

    // 上位机第 4 路点列作为“下角点标记通道”。
    // 先全部压到中心点，避免旧的 max_white 轨迹干扰观察。
    for(int i = 0; i < IMG_PROC_HEIGHT; ++i)
    {
        tcp_max_white_x[i] = debug_clamp_u8_local(x, 0, IMG_PROC_WIDTH - 1);
        tcp_max_white_y[i] = debug_clamp_u8_local(y, 0, IMG_PROC_HEIGHT - 1);
    }

    // 前 9 个点做一个小十字，逐飞助手上能看到一个单独标记。
    const int dx[9] = { 0,  0,  0,  0,  0, -2, -1,  1,  2};
    const int dy[9] = {-2, -1,  0,  1,  2,  0,  0,  0,  0};
    for(int i = 0; i < 9 && i < IMG_PROC_HEIGHT; ++i)
    {
        tcp_max_white_x[i] = debug_clamp_u8_local(x + dx[i], 0, IMG_PROC_WIDTH - 1);
        tcp_max_white_y[i] = debug_clamp_u8_local(y + dy[i], 0, IMG_PROC_HEIGHT - 1);
    }
#else
    // 调试 API 关闭时不改上位机通道。
#endif
}

static void DrawCircleTemplateDebug(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    element_circle_draw_debug_struct dbg = {0};
    element_get_circle_draw_debug(&dbg);

    if(!dbg.valid)
    {
        return;
    }

    ips200.draw_line((uint16)dbg.template_x1,
                     (uint16)dbg.template_y1,
                     (uint16)dbg.template_x2,
                     (uint16)dbg.template_y2,
                     RGB565_YELLOW);

    draw_debug_cross_point(dbg.template_x1, dbg.template_y1, RGB565_BLUE);
    draw_debug_cross_point(dbg.template_x2, dbg.template_y2, RGB565_GREEN);

    for(int i = 0; i < 6; ++i)
    {
        if(dbg.last_point_x[i] > 0 && dbg.last_point_y[i] > 0)
        {
            draw_debug_cross_point(dbg.last_point_x[i], dbg.last_point_y[i], RGB565_CYAN);
        }
    }
#endif
}

//限制
inline uint8 clamp_u8_with_bounds(int value, int low, int high)
{
    if (value < low)
    {
        value = low;
    }
    if (value > high)
    {
        value = high;
    }
    return static_cast<uint8>(value);
}
//方便逐飞助手显示,把 OpenCV 的 BGR888 像素，转换成 RGB565 格式
inline uint16 bgr_to_rgb565(uint8 b, uint8 g, uint8 r)
{
    return static_cast<uint16>(((r & 0xF8U) << 8) | ((g & 0xFCU) << 3) | (b >> 3));
}
//交换 16 位数据的高低字节
inline uint16 swap_u16(uint16 value)
{
    return static_cast<uint16>((value << 8) | (value >> 8));
}

static void draw_tcp_color_cross_point(int x, int y, const uint16 color)
{
    if(x <= 1 || x >= (IMG_PROC_WIDTH - 2) || y <= 1 || y >= (IMG_PROC_HEIGHT - 2))
    {
        return;
    }

    const uint16 send_color = swap_u16(color);
    for(int d = -2; d <= 2; ++d)
    {
        send_color_image_120[y][x + d] = send_color;
        send_color_image_120[y + d][x] = send_color;
    }
}

static void overlay_circle_down_corner_to_tcp_color_image(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    int x = 0;
    int y = 0;
    uint8 side = 0U;
    if(get_circle_down_corner_debug_point(&x, &y, &side))
    {
        (void)side;
        draw_tcp_color_cross_point(x, y, RGB565_MAGENTA);
    }
#endif
}
//坐标之间的距离
float point_distance(const cv::Point2f& a, const cv::Point2f& b)
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * @brief tcp传输旋转矩形
 * @param rect_points 旋转矩形四个角点
 * @param out_x 输入的数组x
 * @param out_y 输入的数组y
 * @param sample_count 点的数目
 */
//沿着四条边均匀采样成一串点
bool fill_boundary_with_rotated_rect(const cv::Point2f* rect_points, uint8* out_x, uint8* out_y, int sample_count)
{
    if (rect_points == nullptr || out_x == nullptr || out_y == nullptr || sample_count <= 1)
    {
        return false;
    }

    //四个点
    float edge_len[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float perimeter = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        //四个边的距离
        edge_len[i] = point_distance(rect_points[i], rect_points[(i + 1) % 4]);
        perimeter += edge_len[i];
        //得到四条边的总长
    }
    if (perimeter <= 1e-3f)
    {
        return false;
    }

    //遍历所有的点
    for (int i = 0; i < sample_count; ++i)
    {
        //从起点到终点，把一整圈周长均匀分成 sample_count-1 段，采样距离
        const float walk_dist = (sample_count > 1)
            ? perimeter * static_cast<float>(i) / static_cast<float>(sample_count - 1)
            : 0.0f;
        //判断当前 walk_dist 落在哪一条边上
        float accum = 0.0f; //前面几条边已经累计走了多长
        int edge_idx = 0;   //当前点落在哪一条边上
        for (; edge_idx < 4; ++edge_idx)
        {
            
            if (walk_dist <= accum + edge_len[edge_idx] || edge_idx == 3)
            {
                break;
            }
            //accum 表示已经走过前面几条边的总长度
            accum += edge_len[edge_idx];
        }

        //在线段上做线性插值 p=p0​+t(p1​−p0​)
        const cv::Point2f p0 = rect_points[edge_idx];
        const cv::Point2f p1 = rect_points[(edge_idx + 1) % 4];
        //计算采样点在当前边上的局部比例 local_t
        const float seg_len = std::max(edge_len[edge_idx], 1e-3f);
        const float local_t = (walk_dist - accum) / seg_len;
        const cv::Point2f p = p0 + (p1 - p0) * local_t;

        //把浮点坐标变成图像坐标
        out_x[i] = clamp_u8_with_bounds(static_cast<int>(std::lround(p.x)), 0, IMG_PROC_WIDTH - 1);
        out_y[i] = clamp_u8_with_bounds(static_cast<int>(std::lround(p.y)), 0, IMG_PROC_HEIGHT - 1);
    }
    return true;
}

/**
 * @brief tcp逐飞助手传输红色掩码图，转换成彩图，最终显示红色框
 */
// bool tcp_send_red_mask_image()
// {
//     RedRoiDebugInfo debug_info = {};
//     //得到参数
//     if (!red_roi_get_debug_info(&debug_info) || !debug_info.mask_valid || debug_info.red_mask_full.empty())
//     {
//         return false;
//     }

//     cv::Mat vis_bgr;
//     //灰度掩码转三通道 BGR 图
//     cv::cvtColor(debug_info.red_mask_full, vis_bgr, cv::COLOR_GRAY2BGR);
//     //如果尺寸不对，就缩放到处理分辨率
//     if (vis_bgr.cols != IMG_PROC_WIDTH || vis_bgr.rows != IMG_PROC_HEIGHT)
//     {
//         cv::resize(vis_bgr, vis_bgr, cv::Size(IMG_PROC_WIDTH, IMG_PROC_HEIGHT), 0.0, 0.0, cv::INTER_NEAREST);
//     }
//     //如果矩形有效，就把 4 条边画出来
//     if (debug_info.rect_valid)
//     {
//         for (int i = 0; i < 4; ++i)
//         {
//             const cv::Point p0(static_cast<int>(std::lround(debug_info.rect_points[i].x)),
//                                static_cast<int>(std::lround(debug_info.rect_points[i].y)));
//             const cv::Point p1(static_cast<int>(std::lround(debug_info.rect_points[(i + 1) % 4].x)),
//                                static_cast<int>(std::lround(debug_info.rect_points[(i + 1) % 4].y)));
//             cv::line(vis_bgr, p0, p1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
//         }
//     }
//     // 画出旋转ROI矩形
//     // if (debug_info.roi_valid)
//     // {
//     //     // ROI 点顺序原始为 tl,tr,bl,br，这里改为 tl->tr->br->bl->tl 画线
//     //     const int ring_idx[4] = {0, 1, 3, 2};
//     //     for (int i = 0; i < 4; ++i)
//     //     {
//     //         const int idx0 = ring_idx[i];
//     //         const int idx1 = ring_idx[(i + 1) % 4];
//     //         const cv::Point p0(static_cast<int>(std::lround(debug_info.roi_points[idx0].x)),
//     //                            static_cast<int>(std::lround(debug_info.roi_points[idx0].y)));
//     //         const cv::Point p1(static_cast<int>(std::lround(debug_info.roi_points[idx1].x)),
//     //                            static_cast<int>(std::lround(debug_info.roi_points[idx1].y)));
//     //         // 绿色框显示 build_rotated_roi_above_red 预测ROI旋转矩形
//     //         cv::line(vis_bgr, p0, p1, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
//     //     }
//     // }
//     //把 BGR888 转成 RGB565
//     for (int y = 0; y < IMG_PROC_HEIGHT; ++y)
//     {
//         const cv::Vec3b* row = vis_bgr.ptr<cv::Vec3b>(y);
//         for (int x = 0; x < IMG_PROC_WIDTH; ++x)
//         {
//             const cv::Vec3b bgr = row[x];
//             //转换成RGB565格式
//             const uint16 rgb565 = bgr_to_rgb565(bgr[0], bgr[1], bgr[2]);
//             send_color_image_120[y][x] = swap_u16(rgb565);
//         }
//     }
//     seekfree_assistant_camera_send();
//     return true;
// }

/**
 * @brief debug，把当前保存的调试 ROI 小图，缩放成和摄像头分辨率一样大，再转成 RGB565 格式，放进发送缓冲区 send_color_image，最后通过 TCP/上位机发送出去
 */
bool tcp_send_roi_patch_image()
{
    cv::Mat roi_img;
    //得到roi_img图片
    if (!lq_ncnn_get_debug_roi_patch(&roi_img) || roi_img.empty())
    {
        return false;
    }

    //转换为BGR
    cv::Mat roi_bgr;
    if (roi_img.channels() == 1)
    {
        //如果 ROI 是单通道灰度图，就转成 BGR
        cv::cvtColor(roi_img, roi_bgr, cv::COLOR_GRAY2BGR);
    }
    else
    {
        roi_bgr = roi_img;
    }

    cv::Mat vis_bgr;
    //把 ROI 图放大到发送分辨率 120*160
    cv::resize(roi_bgr, vis_bgr, cv::Size(IMG_PROC_WIDTH, IMG_PROC_HEIGHT), 0.0, 0.0, cv::INTER_NEAREST);
    for (int y = 0; y < IMG_PROC_HEIGHT; ++y)
    {
        //逐像素把 BGR888 转成 RGB565
        const cv::Vec3b* row = vis_bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < IMG_PROC_WIDTH; ++x)
        {
            const cv::Vec3b bgr = row[x];
            const uint16 rgb565 = bgr_to_rgb565(bgr[0], bgr[1], bgr[2]);
            send_color_image_120[y][x] = swap_u16(rgb565);
        }
    }
    seekfree_assistant_camera_send();
    return true;
}

/**
 * @brief 把红块检测关键参数发成波形
 */
void send_red_roi_debug_oscilloscope()
{
    RedRoiDebugInfo debug_info = {};
    const bool has_rect = red_roi_get_debug_info(&debug_info) && debug_info.rect_valid;

    //如果没有矩形
    if (!has_rect)
    {
        for (int i = 0; i < SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT; ++i)
        {
            seekfree_assistant_oscilloscope_data.data[i] = 0.0f;
        }
        seekfree_assistant_oscilloscope_data.channel_num = SEEKFREE_ASSISTANT_SET_OSCILLOSCOPE_COUNT;
        seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
        return;
    }
    //如果有矩形
    seekfree_assistant_oscilloscope_data.data[0] = debug_info.rect_long_side_px;
    seekfree_assistant_oscilloscope_data.data[1] = debug_info.rect_short_side_px;
    seekfree_assistant_oscilloscope_data.data[2] = debug_info.rect_center.x;
    seekfree_assistant_oscilloscope_data.data[3] = debug_info.rect_center.y;
    seekfree_assistant_oscilloscope_data.channel_num = 4;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}
}
//roi_debug

/**
 * @brief 主循环调用逐飞助手函数的接口
 * @note 发送图像和边界信息到助手端，供远程监控和调试使用。
 */
void tcp_main_use()
{
    debug_view_update_by_key();
    ensure_tcp_debug_stream_config();

    if (g_debug_view_mode == DEBUG_VIEW_IMU)
    {
        tcp_send_gray_image();
        return;
    }

    if (g_debug_view_mode == DEBUG_VIEW_CIRCLE ||
        g_debug_view_mode == DEBUG_VIEW_CROSS ||
        g_debug_view_mode == DEBUG_VIEW_RAMP)
    {
        g_tcp_debug_image_mode = TCP_DEBUG_IMAGE_GRAY;
        tcp_send_gray_image();
        tcp_send_boundary_info_update();
        return;
    }

//roi_debug
    if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_RED_MASK)
    {
        // if (!tcp_send_red_mask_image())
        // {
            tcp_send_color_image();
        // }
    }
    else if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_ROI_PATCH)
    {
        if (!tcp_send_roi_patch_image())
        {
            tcp_send_color_image();
        }
    }
    else if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_GRAY)
    {
        tcp_send_gray_image();
    }
    else
    {
        tcp_send_color_image();
    }
//roi_debug
     tcp_send_boundary_info_update();//tcp传图像边界
}

/**
 * @brief 主循环调用ips200函数的接口
 * @note 显示图像和边线信息，显示参数
 */
void ips200_main_use()
{
    debug_view_update_by_key();

    if (g_debug_view_mode == DEBUG_VIEW_IMU)
    {
        ips200_show_yaw_only();
        return;
    }

    const uint8* show_img = get_gray_image_proc_ptr();
    if(show_img == nullptr)
    {
        return;
    }

    // 显示算法实际使用的 160x120 处理图
    ips200.show_gray_image(0, 0,
                           show_img,
                           IMG_PROC_WIDTH,
                           IMG_PROC_HEIGHT,
                           IMG_PROC_WIDTH,
                           IMG_PROC_HEIGHT,
                           0);

    uint8 draw_num = max_white.nums;
    if(draw_num > IMG_PROC_HEIGHT)
    {
        draw_num = IMG_PROC_HEIGHT;
    }

    DrawFilterLines_color(draw_num, 'L', RGB565_WHITE);
    DrawFilterLines_color(draw_num, 'R', RGB565_BROWN);
    DrawFilterLines_color(draw_num, 'M', RGB565_RED);
    DrawUseLines_color(draw_num, 'W', RGB565_GREEN);

    Draw_image_mid();
    Draw_prospect();

    if (g_debug_view_mode == DEBUG_VIEW_CIRCLE)
    {
        // 环岛页：显示环岛角点、模板补线和环岛判据参数。
        DrawCircleDownCornerDebug();
        DrawCircleTemplateDebug();
        ips200_show_element_debug();
        ips200_show_string(0, 0, "VIEW:CIRCLE");
        return;
    }

    if (g_debug_view_mode == DEBUG_VIEW_CROSS)
    {
        DrawCrossPatchLinesDebug();
        DrawCrossUpperCornerProbeDebug();
        ips200_show_cross_debug();
        ips200_show_string(0, 0, "VIEW:CROSS");
        return;
    }

    if (g_debug_view_mode == DEBUG_VIEW_RAMP)
    {
        ips200_show_ramp_debug();
        return;
    }

    if (g_debug_view_mode == DEBUG_VIEW_OBSTACLE)
    {
        ips200_show_obstacle_debug();
        return;
    }

    ips200_show_param();
    ips200_show_string(0, 0, "VIEW:RUN");
}

// #if (1 == ELEMENT_ENABLE_DEBUG_API)
// {
//     static int circle_dbg_cnt = 0;
//     circle_dbg_cnt++;

//     if(circle_dbg_cnt >= 10)
//     {
//         circle_dbg_cnt = 0;

//         element_debug_snapshot_struct snap = {0};
//         element_debug_get_snapshot(&snap);

//         printf("[CIRCLE] cf=%d cs=%d dbg=%d ls=%d rs=%d lk=%d rk=%d lcp=%d rcp=%d L=%d R=%d B=%d rw=%d mw=%d col=%d pf=%d\r\n",
//                (int)snap.circle_flag,
//                (int)snap.circle_state,
//                (int)snap.circle_debug_code,
//                (int)snap.left_state,
//                (int)snap.right_state,
//                (int)snap.l_k_state,
//                (int)snap.r_k_state,
//                (int)snap.left_circle_point,
//                (int)snap.right_circle_point,
//                (int)snap.left_lost_rows,
//                (int)snap.right_lost_rows,
//                (int)snap.both_lost_rows,
//                (int)snap.valid_rows,
//                (int)max_white.nums,
//                (int)max_white.col,
//                (int)imgpro_var.protect_flag);
//     }
// }

// #endif

/**
 * @brief 在IPS200上显示关键参数
 * @note 显示前瞻行误差、PID参数、保护状态等,根据实际情况自行更改显示的参数和格式
 */
void ips200_show_param()
{
    ips200_show_string(0,8,"err:");
    ips200_show_float(40,8,imgpro_var.midline_error,2,2);

    ips200_show_string(0,9,"white:");
    ips200_show_int(56,9,(int32)max_white.nums,3);

    ips200_show_string(0,10,"kp:");
    ips200_show_float(40,10,speed_dir_info.dir_kp,3,1);

    ips200_show_string(80,10,"kd:");
    ips200_show_float(120,10,speed_dir_info.dir_kd,3,1);

    ips200_show_string(0,11,"protect_flag:");
    ips200_show_int(104,11,(int32)imgpro_var.protect_flag,1);

    // 左右编码器当前滤波值
    ips200_show_string(0,12,"encL:");
    ips200_show_int(48,12,(int32)f_encoder_left,5);

    ips200_show_string(112,12,"encR:");
    ips200_show_int(160,12,(int32)f_encoder_right,5);
}

void ips200_show_ramp_debug()
{
    element_debug_snapshot_struct elem_dbg = {};
    element_debug_get_snapshot(&elem_dbg);

    // 处理图像占用屏幕上方约 120 像素（前 7～8 行），
    // 坡道参数统一从第 8 行开始显示，避免覆盖图像。
    ips200_show_string(0, 8, "VIEW:RAMP");

    ips200_show_string(0, 9, "state:");
    ips200_show_int(48, 9, (int32)tof_ramp_state, 1);
    ips200_show_string(88, 9, "det:");
    ips200_show_int(120, 9, tof_ramp_detected ? 1 : 0, 1);
    ips200_show_string(152, 9, "cnt:");
    ips200_show_int(184, 9, (int32)tof_ramp_suspect_count, 1);

    ips200_show_string(0, 10, "raw:");
    ips200_show_int(40, 10, (int32)dl1x_distance_raw, 4);
    ips200_show_string(136, 10, "med:");
    ips200_show_float(176, 10, tof_distance_median, 4, 1);

    ips200_show_string(0, 11, "filt:");
    ips200_show_float(48, 11, tof_distance_filtered, 4, 1);
    ips200_show_string(120, 11, "fth:");
    ips200_show_float(160, 11, tof_filtered_enter_threshold, 4, 1);

    ips200_show_string(0, 12, "tok:");
    ips200_show_int(40, 12, tof_filtered_enter_ok ? 1 : 0, 1);
    ips200_show_string(72, 12, "vis:");
    ips200_show_int(112, 12, visual_ramp_allow ? 1 : 0, 1);
    ips200_show_string(144, 12, "match:");
    ips200_show_int(200, 12, tof_distance_match_ok ? 1 : 0, 1);

    ips200_show_string(0, 14, "arm:");
    ips200_show_float(40, 14, ramp_arm_encoder_distance, 4, 0);
    ips200_show_string(96, 14, "ath:");
    ips200_show_float(136, 14, RAMP_ARM_ENCODER_DISTANCE, 4, 0);
    ips200_show_string(192, 14, "white:");
    ips200_show_int(248, 14, (int32)max_white.nums, 3);

    ips200_show_string(0, 15, "Wok:");
    ips200_show_int(40, 15, ramp_enough_long_white_columns ? 1 : 0, 1);
    ips200_show_string(72, 15, "valid:");
    ips200_show_int(128, 15, tof_baseline_ready ? 1 : 0, 1);
    ips200_show_string(160, 15, "tof:");
    ips200_show_int(200, 15, tof_distance_valid ? 1 : 0, 1);

    ips200_show_string(0, 16, "em:");
    ips200_show_int(32, 16, (int32)elem_dbg.mode, 1);
    ips200_show_string(56, 16, "cs:");
    ips200_show_int(88, 16, (int32)elem_dbg.cross_state, 1);
    ips200_show_string(112, 16, "rs:");
    ips200_show_int(144, 16, (int32)elem_dbg.circle_state, 1);
    ips200_show_string(168, 16, "pit:");
    ips200_show_float(208, 16, ramp_pitch_relative, 4, 1);
}

void ips200_show_obstacle_debug()
{
    obstacle_debug_snapshot_struct dbg = {};
    obstacle_detect_get_snapshot(&dbg);

    ips200_show_string(0, 8, "VIEW:OBS");
    ips200_show_string(88, 8, "st:");
    ips200_show_int(112, 8, (int32)dbg.state, 1);
    ips200_show_string(144, 8, "busy:");
    ips200_show_int(192, 8, element_is_busy() ? 1 : 0, 1);

    ips200_show_string(0, 9, "L:");
    ips200_show_int(16, 9, (int32)dbg.left_candidate, 1);
    ips200_show_string(32, 9, "j:");
    ips200_show_int(48, 9, (int32)dbg.left_jump, 3);
    ips200_show_string(104, 9, "R:");
    ips200_show_int(120, 9, (int32)dbg.right_candidate, 1);
    ips200_show_string(136, 9, "j:");
    ips200_show_int(152, 9, (int32)dbg.right_jump, 3);

    ips200_show_string(0, 10, "jth:");
    ips200_show_int(40, 10, (int32)dbg.jump_threshold, 2);
    ips200_show_string(88, 10, "cnt:");
    ips200_show_int(128, 10, (int32)dbg.left_enter_count, 1);
    ips200_show_string(136, 10, "/");
    ips200_show_int(144, 10, (int32)dbg.right_enter_count, 1);
    ips200_show_string(168, 10, "eth:");
    ips200_show_int(208, 10, (int32)dbg.enter_confirm_frames, 1);

    ips200_show_string(0, 11, "sh:");
    ips200_show_int(24, 11, (int32)dbg.width_shrink_found, 1);
    ips200_show_string(48, 11, "val:");
    ips200_show_int(88, 11, (int32)dbg.width_shrink_value, 3);
    ips200_show_string(136, 11, "sth:");
    ips200_show_int(176, 11, (int32)dbg.width_shrink_threshold, 2);

    ips200_show_string(0, 12, "fit:");
    ips200_show_int(40, 12, (int32)dbg.opposite_fit_ok, 1);
    ips200_show_string(72, 12, "vr:");
    ips200_show_float(104, 12, dbg.opposite_valid_ratio, 1, 2);
    ips200_show_string(152, 12, "rm:");
    ips200_show_float(184, 12, dbg.opposite_rmse, 2, 1);

    ips200_show_string(0, 13, "dist:");
    ips200_show_float(48, 13, dbg.encoder_distance, 5, 0);
    ips200_show_string(104, 13, "dth:");
    ips200_show_float(144, 13, dbg.exit_encoder_distance_threshold, 4, 0);

    ips200_show_string(0, 14, "exit:");
    ips200_show_int(48, 14, (int32)dbg.exit_count, 1);
    ips200_show_string(64, 14, "/");
    ips200_show_int(72, 14, (int32)dbg.exit_confirm_frames, 1);
    ips200_show_string(104, 14, "gate:");
    ips200_show_int(144, 14, (int32)dbg.exit_distance_ready, 1);
    ips200_show_string(168, 14, "up:");
    ips200_show_int(200, 14, (int32)dbg.exit_upper_jump_found, 1);
}

void ips200_show_element_debug(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    element_debug_snapshot_struct snap = {0};
    element_debug_get_snapshot(&snap);

    // 新环岛入口判据调试显示：
    // R：右环，L：左环
    // D：down corner，下角点行号
    // U：up corner，上角点行号
    // C：two corner ok，两角点合法
    // S1/S2：斜率条件；Lt/Rt：左右边界直线性条件

    ips200_show_string(0, 13, "cf:");
    ips200_show_int(24, 13, (int32)snap.circle_flag, 1);

    ips200_show_string(48, 13, "cs:");
    ips200_show_int(72, 13, (int32)snap.circle_state, 2);

    ips200_show_string(104, 13, "dbg:");
    ips200_show_int(136, 13, (int32)snap.circle_debug_code, 2);

    ips200_show_string(176, 13, "sd:");
    ips200_show_int(200, 13, (int32)snap.circle_lost_side, 1);

    // 丢线统计：L/R/B/st
    ips200_show_string(0, 14, "L:");
    ips200_show_int(16, 14, (int32)snap.left_lost_rows, 3);

    ips200_show_string(56, 14, "R:");
    ips200_show_int(72, 14, (int32)snap.right_lost_rows, 3);

    ips200_show_string(112, 14, "B:");
    ips200_show_int(128, 14, (int32)snap.both_lost_rows, 3);

    ips200_show_string(176, 14, "st:");
    ips200_show_int(200, 14, (int32)snap.search_stop_line, 3);

    // 右环两角点：右下角点 / 右上角点 / 两角点合法
    ips200_show_string(0, 15, "RD:");
    ips200_show_int(24, 15, (int32)snap.right_down_y, 3);

    ips200_show_string(64, 15, "RU:");
    ips200_show_int(88, 15, (int32)snap.right_up_y, 3);

    ips200_show_string(128, 15, "RC:");
    ips200_show_int(152, 15, (int32)snap.right_two_corner_ok, 1);

    ips200_show_string(176, 15, "rw:");
    ips200_show_int(200, 15, (int32)snap.valid_rows, 3);

    // 右环条件：左边线斜率 / 左边界直线 / 右下段斜率 / 右边界直线
    ips200_show_string(0, 16, "Rls:");
    ips200_show_int(32, 16, (int32)snap.rc_left_slope_ok, 1);

    ips200_show_string(48, 16, "Rlt:");
    ips200_show_int(80, 16, (int32)snap.rc_left_straight_ok, 1);

    ips200_show_string(96, 16, "Rrs:");
    ips200_show_int(128, 16, (int32)snap.rc_right_down_slope_ok, 1);

    ips200_show_string(144, 16, "Rrt:");
    ips200_show_int(176, 16, (int32)snap.rc_right_straight_ok, 1);

    // 左环两角点：左下角点 / 左上角点 / 两角点合法
    ips200_show_string(0, 17, "LD:");
    ips200_show_int(24, 17, (int32)snap.left_down_y, 3);

    ips200_show_string(64, 17, "LU:");
    ips200_show_int(88, 17, (int32)snap.left_up_y, 3);

    ips200_show_string(128, 17, "LC:");
    ips200_show_int(152, 17, (int32)snap.left_two_corner_ok, 1);

    ips200_show_string(176, 17, "Ry:");
    ips200_show_int(200, 17, (int32)(snap.right_down_y - snap.right_up_y), 3);

    // 左环斜率条件：右边线斜率 / 左下段斜率
    ips200_show_string(0, 18, "Lrs:");
    ips200_show_int(32, 18, (int32)snap.lc_right_slope_ok, 1);

    ips200_show_string(64, 18, "Lls:");
    ips200_show_int(96, 18, (int32)snap.lc_left_down_slope_ok, 1);

    ips200_show_string(128, 18, "Ly:");
    ips200_show_int(152, 18, (int32)(snap.left_down_y - snap.left_up_y), 3);

    // 姿态与环岛累计角：保留，方便 ENTER / INSIDE 阶段调试
    ips200_show_string(0, 19, "yaw:");
    ips200_show_float(32, 19, snap.current_yaw, 4, 1);

    ips200_show_string(112, 19, "cy:");
    ips200_show_float(136, 19, snap.circle_yaw, 4, 1);
#endif
}


void ips200_show_cross_debug(void)
{
#if (1 == ELEMENT_ENABLE_DEBUG_API)
    cross_screen_debug_struct dbg = {0};
    get_cross_screen_debug(&dbg);

    ips200_show_string(0, 8, "                              ");
    ips200_show_string(0, 8, "LU:");
    ips200_show_int(24, 8, (int32)dbg.probe_lu_found, 1);
    ips200_show_string(40, 8, "x:");
    ips200_show_int(56, 8, (int32)dbg.probe_lu_x, 3);
    ips200_show_string(96, 8, "y:");
    ips200_show_int(112, 8, (int32)dbg.probe_lu_y, 3);
    ips200_show_string(152, 8, "i:");
    ips200_show_int(168, 8, (int32)dbg.probe_lu_idx, 3);

    ips200_show_string(0, 9, "                              ");
    ips200_show_string(0, 9, "LD:");
    ips200_show_int(24, 9, (int32)dbg.probe_ld_found, 1);
    ips200_show_string(40, 9, "x:");
    ips200_show_int(56, 9, (int32)dbg.probe_ld_x, 3);
    ips200_show_string(96, 9, "y:");
    ips200_show_int(112, 9, (int32)dbg.probe_ld_y, 3);
    ips200_show_string(152, 9, "i:");
    ips200_show_int(168, 9, (int32)dbg.probe_ld_idx, 3);

    ips200_show_string(0, 10, "                              ");
    ips200_show_string(0, 10, "RU:");
    ips200_show_int(24, 10, (int32)dbg.probe_ru_found, 1);
    ips200_show_string(40, 10, "x:");
    ips200_show_int(56, 10, (int32)dbg.probe_ru_x, 3);
    ips200_show_string(96, 10, "y:");
    ips200_show_int(112, 10, (int32)dbg.probe_ru_y, 3);
    ips200_show_string(152, 10, "i:");
    ips200_show_int(168, 10, (int32)dbg.probe_ru_idx, 3);

    ips200_show_string(0, 11, "                              ");
    ips200_show_string(0, 11, "RD:");
    ips200_show_int(24, 11, (int32)dbg.probe_rd_found, 1);
    ips200_show_string(40, 11, "x:");
    ips200_show_int(56, 11, (int32)dbg.probe_rd_x, 3);
    ips200_show_string(96, 11, "y:");
    ips200_show_int(112, 11, (int32)dbg.probe_rd_y, 3);
    ips200_show_string(152, 11, "i:");
    ips200_show_int(168, 11, (int32)dbg.probe_rd_idx, 3);

    ips200_show_string(0, 12, "                              ");
    ips200_show_string(0, 12, "Lost L:");
    ips200_show_int(56, 12, (int32)dbg.left_lost_rows, 3);
    ips200_show_string(88, 12, "R:");
    ips200_show_int(104, 12, (int32)dbg.right_lost_rows, 3);
    ips200_show_string(136, 12, "Both:");
    ips200_show_int(176, 12, (int32)dbg.both_lost_rows, 3);

    ips200_show_string(0, 13, "                              ");
    ips200_show_string(0, 13, "Exit:");
    ips200_show_int(40, 13, (int32)dbg.cross_exit_frames, 1);
    ips200_show_string(64, 13, "XS:");
    ips200_show_int(88, 13, (int32)dbg.cross_state, 1);
    ips200_show_string(112, 13, "Cand:");
    ips200_show_int(152, 13, (int32)dbg.cross_candidate, 1);

    ips200_show_string(0, 14, "                              ");
    ips200_show_string(0, 14, "Entry:");
    ips200_show_int(48, 14, (int32)dbg.cross_confirm_frames, 1);
    ips200_show_string(64, 14, "/3");
    ips200_show_string(96, 14, "Corners:");
    ips200_show_int(160, 14, (int32)dbg.cross_line_index, 1);

    ips200_show_string(0, 15, "                              ");
    ips200_show_string(0, 15, "CornerAbort:OFF");

    ips200_show_string(0, 16, "                              ");
    ips200_show_string(0, 16, "Rule B>30 or L,R>20");

    ips200_show_string(0, 17, "                              ");
    ips200_show_string(0, 17, "Bottom L:");
    ips200_show_int(72, 17, (int32)dbg.bottom_left_valid, 2);
    ips200_show_string(96, 17, "R:");
    ips200_show_int(112, 17, (int32)dbg.bottom_right_valid, 2);
    ips200_show_string(144, 17, "/");
    ips200_show_int(152, 17, (int32)dbg.bottom_checked_rows, 2);

    ips200_show_string(0, 18, "                              ");
    ips200_show_string(0, 18, "LP");
    ips200_show_int(16, 18, (int32)dbg.left_patch_valid, 1);
    ips200_show_int(32, 18, (int32)dbg.left_patch_x1, 3);
    ips200_show_string(56, 18, ",");
    ips200_show_int(64, 18, (int32)dbg.left_patch_y1, 3);
    ips200_show_string(88, 18, ">");
    ips200_show_int(96, 18, (int32)dbg.left_patch_x2, 3);
    ips200_show_string(120, 18, ",");
    ips200_show_int(128, 18, (int32)dbg.left_patch_y2, 3);

    ips200_show_string(0, 19, "                              ");
    ips200_show_string(0, 19, "RP");
    ips200_show_int(16, 19, (int32)dbg.right_patch_valid, 1);
    ips200_show_int(32, 19, (int32)dbg.right_patch_x1, 3);
    ips200_show_string(56, 19, ",");
    ips200_show_int(64, 19, (int32)dbg.right_patch_y1, 3);
    ips200_show_string(88, 19, ">");
    ips200_show_int(96, 19, (int32)dbg.right_patch_x2, 3);
    ips200_show_string(120, 19, ",");
    ips200_show_int(128, 19, (int32)dbg.right_patch_y2, 3);
#endif
}


/**
 * @brief 向逐飞助手发送变线信息，边界信息一直更新
 * @note 将最新边线数组打包到 TCP 缓冲区供助手显示；
 *       第3路优先发送 ROI 矩形轮廓，若 ROI 不可用则回退为中线。
 */
void tcp_send_boundary_info_update()
{
    int valid_rows = (int)max_white.nums;
    if(valid_rows < 0) valid_rows = 0;
    if(valid_rows > IMG_PROC_HEIGHT) valid_rows = IMG_PROC_HEIGHT;

    // 左右边界在本帧搜线范围内全部原样发送，包括 sign==0 的无效边界点。
    // 有效搜线范围之外仍沿用最后一个已发送点，避免固定长度的上位机数组
    // 读取到未参与本帧搜线的清零区域。
    int left_hold_x = USE_WIDTH_L;
    int left_hold_y = start_line;
    int right_hold_x = USE_WIDTH_R - 1;
    int right_hold_y = start_line;

    int mid_hold_x = USE_MIDDLE_VALUE;
    int mid_hold_y = start_line;

    for (int i = 0; i < IMG_PROC_HEIGHT; ++i)
    {
        if(i < valid_rows)
        {
            left_hold_x = (int)g_image_info.filter_left[i].pos[0];
            left_hold_y = (int)g_image_info.filter_left[i].pos[1];
        }
        tcp_left_boundary_x[i] = (uint8)left_hold_x;
        tcp_left_boundary_y[i] = (uint8)left_hold_y;

        if(i < valid_rows)
        {
            right_hold_x = (int)g_image_info.filter_right[i].pos[0];
            right_hold_y = (int)g_image_info.filter_right[i].pos[1];
        }
        tcp_right_boundary_x[i] = (uint8)right_hold_x;
        tcp_right_boundary_y[i] = (uint8)right_hold_y;

        // 中线没有 sign，按有效行发送；有效行之外全部压到最后一个中线点。
        if(i < valid_rows)
        {
            mid_hold_x = (int)g_image_info.filter_midline[i][0];
            mid_hold_y = (int)g_image_info.filter_midline[i][1];
        }
        tcp_middle_line_x[i] = (uint8)mid_hold_x;
        tcp_middle_line_y[i] = (uint8)mid_hold_y;

        tcp_max_white_x[i] = max_white.col;
        tcp_max_white_y[i] = (uint8)mid_hold_y;
    }

    //roi_debug
    if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_RED_MASK)
    {
        RedRoiDebugInfo debug_info = {};
        std::memset(tcp_middle_line_x, 0, sizeof(tcp_middle_line_x));
        std::memset(tcp_middle_line_y, 0, sizeof(tcp_middle_line_y));
        if (red_roi_get_debug_info(&debug_info))
        {
            if (g_tcp_red_mask_rect_mode == TCP_RED_MASK_RECT_ROI)
            {
                if (debug_info.roi_valid)
                {
                    const int ring_idx[4] = {0, 1, 3, 2};
                    cv::Point2f roi_points_proc[4];
                    for (int i = 0; i < 4; ++i)
                    {
                        const int src_idx = ring_idx[i];
                        roi_points_proc[i].x = debug_info.roi_points[src_idx].x / static_cast<float>(IMG_PROC_DOWNSAMPLE);
                        roi_points_proc[i].y = debug_info.roi_points[src_idx].y / static_cast<float>(IMG_PROC_DOWNSAMPLE);
                    }
                    fill_boundary_with_rotated_rect(roi_points_proc,
                                                    tcp_middle_line_x,
                                                    tcp_middle_line_y,
                                                    IMG_PROC_HEIGHT);
                }
            }
            else
            {
                if (debug_info.rect_valid)
                {
                    cv::Point2f rect_points_proc[4];
                    for (int i = 0; i < 4; ++i)
                    {
                        rect_points_proc[i].x = debug_info.rect_points[i].x / static_cast<float>(IMG_PROC_DOWNSAMPLE);
                        rect_points_proc[i].y = debug_info.rect_points[i].y / static_cast<float>(IMG_PROC_DOWNSAMPLE);
                    }
                    fill_boundary_with_rotated_rect(rect_points_proc,
                                                    tcp_middle_line_x,
                                                    tcp_middle_line_y,
                                                    IMG_PROC_HEIGHT);
                }
            }
        }
    }
    //roi_debug

    // 找到下角点时，把第 4 路上位机点列改成下角点十字标记。
    overlay_circle_down_corner_to_tcp_boundary();
}

/**
 * @brief 向逐飞助手发送灰度图像
 * @note 将当前灰度图像数据拷贝到 TCP 发送缓冲区并触发发送。
 */
void tcp_send_gray_image()
{
    // 发送 160x120 搜线调试图，和搜线边界坐标系保持一致。
    // 普通帧显示 g_gray_image_proc；环岛黑线重扫帧显示画完补线后真正送入 run_lane_scan_once() 的图。
    const uint8* gray_proc = get_gray_image_debug_show_ptr();
    if (gray_proc == nullptr)
    {
        return;
    }
    memcpy(send_gray_image[0], gray_proc, IMG_PROC_WIDTH * IMG_PROC_HEIGHT);

    if(g_debug_view_mode == DEBUG_VIEW_CROSS)
    {
        OverlayCrossPatchLinesToTcpGrayImage();
    }

    seekfree_assistant_camera_send();
}

/**
 * @brief 向逐飞助手发送彩色图像 120*160
 * @note 将当前彩色图像数据（RGB565格式）进行字节序转换后拷贝到 TCP 发送缓冲区并触发发送。
 */
void tcp_send_color_image()
{
    get_color_image();
    if (color_image == nullptr)
    {
        return;
    }

    // 将 320x240 的 RGB565 彩图按 2x2 下采样为 160x120，并发送给逐飞助手。
    for (int y = 0; y < IMG_PROC_HEIGHT; ++y)
    {
        const int src_y = y * IMG_PROC_DOWNSAMPLE;
        for (int x = 0; x < IMG_PROC_WIDTH; ++x)
        {
            const int src_x = x * IMG_PROC_DOWNSAMPLE;
            const uint16 src_rgb565 = color_image[src_y * UVC_WIDTH + src_x];
            send_color_image_120[y][x] = swap_u16(src_rgb565);
        }
    }

    // 彩色图模式下直接在发送图像上叠加品红十字。
    overlay_circle_down_corner_to_tcp_color_image();

    seekfree_assistant_camera_send();
}

/**
 * @brief 示波器看数据，向上位机发送参数，上位机用波形显示
 * @note 放在定时器里面，发送关键变量到助手端示波器显示，便于观察变量变化趋势和调试。
 */
void Oscill()
{
//roi_debug
    if (g_tcp_debug_image_mode == TCP_DEBUG_IMAGE_RED_MASK)
    {
        send_red_roi_debug_oscilloscope();
        return;
    }
//roi_debug

    seekfree_assistant_oscilloscope_data.data[0] = pitch;
    seekfree_assistant_oscilloscope_data.data[1] = f_gyro_y;
    seekfree_assistant_oscilloscope_data.data[2] = gyro_y;
    // seekfree_assistant_oscilloscope_data.data[2]=speed_dir_info.l_motor_speed;
    // seekfree_assistant_oscilloscope_data.data[3]=speed_dir_info.r_motor_speed;
    // seekfree_assistant_oscilloscope_data.data[2]=model_yaw;//speed_dir_info.l_motor_pwm;
    // seekfree_assistant_oscilloscope_data.data[3]=speed_dir_info.r_motor_pwm;
    
    seekfree_assistant_oscilloscope_data.channel_num = 3;
    seekfree_assistant_oscilloscope_send(&seekfree_assistant_oscilloscope_data);
}

/**
 * @brief 用于左右轮速度环pid调差，接收上位机传过来的参数，上位机调参
 * @note 放在中断里面,最多可以使用8个通道
 */
void Oscill_param_spid()
{
    seekfree_assistant_data_analysis();
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        if(seekfree_assistant_parameter_update_flag[i])
        {
            switch(i)
            {
                case 0:
                {                                    
                    //通过DEBBUG串口发送信息        
                    l_motor.kp=seekfree_assistant_parameter[i];
                    printf("l_motor.kp : %f ", l_motor.kp);
                    printf("\r\n");
                }
                break;
                case 1:
                {
                    l_motor.ki =seekfree_assistant_parameter[i];
                    printf("l_motor.ki : %f ", l_motor.ki );
                    printf("\r\n");
                }
                break;
                case 2:
                {
                    l_motor.kd=seekfree_assistant_parameter[i];
                    printf("l_motor.kd : %f ",l_motor.kd);
                    printf("\r\n");
                }
                break;
                case 3:
                {
                    r_motor.kp=seekfree_assistant_parameter[i];
                    printf("r_motor.kp: %f ",r_motor.kp);
                    printf("\r\n");
                }
                break;
                case 4:
                {
                    r_motor.ki=seekfree_assistant_parameter[i];
                    printf("r_motor.ki : %f ", r_motor.ki);
                    printf("\r\n");
                }
                break;
                case 5:
                {
                    r_motor.kd=seekfree_assistant_parameter[i];
                    printf("r_motor.kd: %f ",r_motor.kd);
                    printf("\r\n");
                }
                break;
                case 6:
                {
                    speed_dir_info.expect_Speed=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.expect_Speed : %f ", speed_dir_info.expect_Speed);
                    printf("\r\n");
                }
                break;
                // case 7:
                // {
                //     speed_dir_info.r_motor_speed=seekfree_assistant_parameter[i];
                //     printf("speed_dir_info.r_motor_speed : %f ",speed_dir_info.r_motor_speed);
                //     printf("\r\n");
                // }
            }
            seekfree_assistant_parameter_update_flag[i] = 0;
        }
    }
}

void Oscill_param_img()
{
    seekfree_assistant_data_analysis();
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        if(seekfree_assistant_parameter_update_flag[i])
        {
            switch(i)
            {
                case 0:
                {                                    
                    //通过DEBBUG串口发送信息        
                    imgpro_var.diff_sum_mov=(uint8)seekfree_assistant_parameter[i];
                    printf("imgpro_var.diff_sum_mov : %d ", imgpro_var.diff_sum_mov);
                    printf("\r\n");
                }
                break;
                case 1:
                {
                    imgpro_var.diff_thershold=(int16)seekfree_assistant_parameter[i];
                    printf("imgpro_var.diff_thershold : %d ", imgpro_var.diff_thershold );
                    printf("\r\n");
                }
                break;
                case 2:
                {
                    imgpro_var.mw_diff_thershold=(int16)seekfree_assistant_parameter[i];
                    printf("imgpro_var.mw_diff_thershold : %d ", imgpro_var.mw_diff_thershold );
                    printf("\r\n");
                }
                break;
            }
            seekfree_assistant_parameter_update_flag[i] = 0;
        }
    }
}
/**
 * @brief 用于方向环和角速度环的参数调试，接收上位机传过来的参数，上位机调参
 * @note 放在中断里面,最多可以使用8个通道
 */
void Oscill_param_dir()
{
    seekfree_assistant_data_analysis();
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        if(seekfree_assistant_parameter_update_flag[i])
        {
            switch(i)
            {
                case 0:
                {                                    
                    //通过DEBBUG串口发送信息        
                    speed_dir_info.dir_kp=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.dir_kp : %f ", speed_dir_info.dir_kp);
                    printf("\r\n");
                }
                break;
                case 1:
                {
                    speed_dir_info.dir_kz=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.dir_kz : %f ", speed_dir_info.dir_kz);
                    printf("\r\n");
                }
                break;
                case 2:
                {
                    imgpro_var.lookahead=(uint8)seekfree_assistant_parameter[i];
                    printf("imgpro_var.lookahead: %u ", imgpro_var.lookahead);
                    printf("\r\n");
                }
                break;
                case 3:
                {
                    speed_limit.speed_step_up=seekfree_assistant_parameter[i];
                    printf("speed_limit.speed_step_up : %f ",speed_limit.speed_step_up);
                    printf("\r\n");
                    // imu_pid.kd=seekfree_assistant_parameter[i];
                    // printf("imu_pid.kd: %f ", imu_pid.kd);
                    // printf("\r\n");
                }
                break;
                case 4:
                {
                    speed_dir_info.diff_sub_gain=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.diff_sub_gain: %f ", speed_dir_info.diff_sub_gain);
                    printf("\r\n");
                }
                break;
                case 5:
                {
                    // speed_limit.speed_min=seekfree_assistant_parameter[i];
                    // printf("speed_limit.speed_min : %f ", speed_limit.speed_min);
                    // printf("\r\n");
                    speed_dir_info.expect_Speed=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.expect_Speed : %f ", speed_dir_info.expect_Speed);
                    printf("\r\n");

                }
                break;
                case 6:
                {

                    speed_limit.speed_max =seekfree_assistant_parameter[i];
                    printf("speed_limit.speed_max : %f ", speed_limit.speed_max );
                    printf("\r\n");

                }
                break;
                case 7:
                {
#if FAN_ENABLE
                    const bool fan_enable = (seekfree_assistant_parameter[i] > 0.5f);
                    fan_set_remote_enable(fan_enable);
                    printf("fan_remote_enable : %u ", fan_enable ? 1U : 0U);
                    printf("\r\n");
#endif
                }
                break;
            }

            seekfree_assistant_parameter_update_flag[i] = 0;
        }
    }
}

void Oscill_param_speed()
{
    seekfree_assistant_data_analysis();
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        if(seekfree_assistant_parameter_update_flag[i])
        {
            switch(i)
            {
                case 0:
                {
                    speed_dir_info.expect_Speed=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.expect_Speed : %f ", speed_dir_info.expect_Speed);
                    printf("\r\n");
                    // speed_limit.speed_min=seekfree_assistant_parameter[i];
                    // printf("speed_limit.speed_min : %f ", speed_limit.speed_min);
                    // printf("\r\n");
                }
                break;
                case 1:
                {
                    speed_limit.speed_max =seekfree_assistant_parameter[i];
                    printf("speed_limit.speed_max : %f ", speed_limit.speed_max );
                    printf("\r\n");
                }
                break;
                case 2:
                {
                    speed_limit.startup_step_up=seekfree_assistant_parameter[i];
                    printf("speed_limit.startup_step_up: %f ", speed_limit.startup_step_up);
                    printf("\r\n");
                }
                break;
                case 3:
                {
                    speed_limit.speed_step_up=seekfree_assistant_parameter[i];
                    printf("speed_limit.speed_step_up : %f ",speed_limit.speed_step_up);
                    printf("\r\n");
                }
                break;
                case 4:
                {
                    imgpro_var.protect_flag=seekfree_assistant_parameter[i];
                    printf("imgpro_var.protect_flag : %u ",imgpro_var.protect_flag);
                    printf("\r\n");
                }
                break;
                case 5:
                {

                    speed_dir_info.manual_speed_target = seekfree_assistant_parameter[i];
                    printf("speed_dir_info.manual_speed_target : %f ", speed_dir_info.manual_speed_target);
                    printf("\r\n");
                }
                break;
                case 6:
                {
                    const bool enable = (seekfree_assistant_parameter[i] > 0.5f);
                    motor_speed_decision_set_enable(enable);
                }
                break;
                case 7:
                {
#if FAN_ENABLE
                    const bool fan_enable = (seekfree_assistant_parameter[i] > 0.5f);
                    fan_set_remote_enable(fan_enable);
                    printf("fan_remote_enable : %u ", fan_enable ? 1U : 0U);
                    printf("\r\n");
#endif
                }
                break;
            }
            seekfree_assistant_parameter_update_flag[i] = 0;
        }
    }
}


void Oscill_param_turn_test()
{
    seekfree_assistant_data_analysis();
    for(uint8_t i = 0; i < SEEKFREE_ASSISTANT_SET_PARAMETR_COUNT; i++)
    {
        if(seekfree_assistant_parameter_update_flag[i])
        {
            switch(i)
            {
                case 0:
                {                                    
                    //通过DEBBUG串口发送信息        
                    speed_dir_info.dir_kp=seekfree_assistant_parameter[i];
                    printf("speed_dir_info.dir_kp : %f ", speed_dir_info.dir_kp);
                    printf("\r\n");
                }
                break;
                case 1:
                {
                    speed_dir_info.dir_kd =seekfree_assistant_parameter[i];
                    printf("speed_dir_info.dir_kd : %f ", speed_dir_info.dir_kd );
                    printf("\r\n");
                }
                break;
                case 2:
                {
                }
                break;
                case 3:
                {
                    imgpro_var.midline_error=seekfree_assistant_parameter[i];
                    printf("imgpro_var.midline_error: %f ", imgpro_var.midline_error);
                    printf("\r\n");
                }
                break;
                case 4:
                {
                    test_use_angle=seekfree_assistant_parameter[i];
                    printf("test_use_angle : %f ", test_use_angle);
                    printf("\r\n");
                }
                break;
            }
            seekfree_assistant_parameter_update_flag[i] = 0;
        }
    }    
}
//封装这些函数方便后面写菜单用
/**
 * @brief ips200显示字符串
 * @param x 行的起始显示地点
 * @param y 列行号
 * @param dat 需要显示的字符串首地址 数据类型 const char[]
 */
void ips200_show_string(uint16 x, uint16 y, const char dat[])
{
    ips200.show_string(x,y*LINE_HEIGHT,dat);
}

/**
 * @brief ips200显示有符号整型变量
 * @param x 行的起始显示地点
 * @param y 列行号
 * @param dat 需要显示的变量 数据类型 int32
 * @param num 整数显示长度 最高支持10位
 * @note 无符号的也用这个函数显示强制转换一下就行了
 */
void ips200_show_int(uint16 x, uint16 y, const int32 dat, uint8 num)
{
    ips200.show_int(x,y*LINE_HEIGHT,dat,num);
}

/**
 * @brief 显示浮点数
 * @param x 行的起始显示地点
 * @param y 列行号
 * @param dat 需要显示的变量 数据类型 double
 * @param num 整数位显示长度 最高8位
 * @param pointnum 小数位显示长度   最高6位
 */
void ips200_show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum)
{
    ips200.show_float(x,y*LINE_HEIGHT,dat,num,pointnum);
}


static inline bool ips_coord_valid(int x, int y)
{
    return (x >= 0 && x < IMG_PROC_WIDTH && y >= 0 && y < IMG_PROC_HEIGHT);
}

static inline void ips_draw_proc_point(int x, int y, const uint16 color)
{
    if(!ips_coord_valid(x, y))
    {
        return;
    }

    ips200.draw_point((uint16)x, (uint16)y, color);
}

static inline int clamp_int_local(int value, int low, int high)
{
    if(value < low)  return low;
    if(value > high) return high;
    return value;
}
/**
 * @brief 在某一列画一条线，画一个竖线
 */
void Draw_one_line_col(uint16 col, const uint16 color)
{
    if(col >= IMG_PROC_WIDTH)
    {
        return;
    }

    for(int i = 0; i < IMG_PROC_HEIGHT; i++)
    {
        ips_draw_proc_point((int)col, i, color);
    }
}

/**
 * @brief 在某一行画一条线，画一个横线
 */
void Draw_one_line_row(uint16 row, const uint16 color)
{
    if(row >= IMG_PROC_HEIGHT)
    {
        return;
    }

    for(int i = 0; i < IMG_PROC_WIDTH; i++)
    {
        ips_draw_proc_point(i, (int)row, color);
    }
}
/**
 * @brief ips显示滤波后的左右边线，中线
 * @param num 显示边线的点数
 * @param s 选择显示的左右边线
 * @note 边线显示两个点(边线加粗)，方便查看，最内边的是实际的边线，默认左边界是白色，右边界是黑色，中线默认是红色，最长白列默认是黄色
 */
void DrawFilterLines_color(uint8 num, char s, const uint16 color)
{
    uint8 draw_num = num;
    if(draw_num > IMG_PROC_HEIGHT)
    {
        draw_num = IMG_PROC_HEIGHT;
    }

    for(uint8 i = 0; i < draw_num; i++)
    {
        switch(s)
        {
            case 'L':
            {
                if(g_image_info.filter_left[i].sign == 0U)
                {
                    break;
                }
                int x = (int)g_image_info.filter_left[i].pos[0];
                int y = (int)g_image_info.filter_left[i].pos[1];

                ips_draw_proc_point(x - 1, y, color);
                ips_draw_proc_point(x,     y, color);
            }break;

            case 'R':
            {
                if(g_image_info.filter_right[i].sign == 0U)
                {
                    break;
                }
                int x = (int)g_image_info.filter_right[i].pos[0];
                int y = (int)g_image_info.filter_right[i].pos[1];

                ips_draw_proc_point(x,     y, color);
                ips_draw_proc_point(x + 1, y, color);
            }break;

            case 'M':
            {
                int x = (int)g_image_info.filter_midline[i][0];
                int y = (int)g_image_info.filter_midline[i][1];

                ips_draw_proc_point(x, y, color);
            }break;
        }
    }
}
/**
 * @brief ips显示左右边线，中线以及最长白列
 * @param num 显示边线的点数
 * @param s 选择显示的左右边线
 * @note 边线显示两个点(边线加粗)，方便查看，最内边的是实际的边线，默认左边界是白色，右边界是黑色，中线默认是红色，最长白列默认是黄色
 */
void DrawUseLines_color(uint8 num, char s, const uint16 color)
{
    uint8 draw_num = num;
    if(draw_num > IMG_PROC_HEIGHT)
    {
        draw_num = IMG_PROC_HEIGHT;
    }

    for(uint8 i = 0; i < draw_num; i++)
    {
        switch(s)
        {
            case 'L':
            {
                int x = (int)g_image_info.left[i].pos[0];
                int y = (int)g_image_info.left[i].pos[1];

                ips_draw_proc_point(x - 1, y, color);
                ips_draw_proc_point(x,     y, color);
            }break;

            case 'R':
            {
                int x = (int)g_image_info.right[i].pos[0];
                int y = (int)g_image_info.right[i].pos[1];

                ips_draw_proc_point(x,     y, color);
                ips_draw_proc_point(x + 1, y, color);
            }break;

            case 'M':
            {
                int x = (int)g_image_info.midline[i][0];
                int y = (int)g_image_info.midline[i][1];

                ips_draw_proc_point(x, y, color);
            }break;

            case 'W':
            {
                int x = (int)max_white.col;
                int y = (int)g_image_info.midline[i][1];

                ips_draw_proc_point(x, y, color);
            }break;
        }
    }
}

/**
 * @brief 画出图像实际使用中值
 * @note 默认颜色是粉色
 */
void Draw_image_mid()
{
    Draw_one_line_col(USE_MIDDLE_VALUE,RGB565_PINK);
}

/**
 * @brief 画出动态前瞻,以及起始行和终止行(最长的前瞻距离)
 * @note 默认蓝色的是最长前瞻范围，绿色的是动态前瞻范围
 */
void Draw_prospect()
{
    int lookahead_row = clamp_int_local((int)imgpro_var.lookahead, 0, IMG_PROC_HEIGHT - 1);
    int white_top_row = clamp_int_local((int)start_line - (int)max_white.nums, 0, IMG_PROC_HEIGHT - 1);
    int start_row     = clamp_int_local((int)start_line, 0, IMG_PROC_HEIGHT - 1);
    int end_row       = clamp_int_local((int)end_line, 0, IMG_PROC_HEIGHT - 1);

    Draw_one_line_row((uint16)lookahead_row, RGB565_GREEN);
    Draw_one_line_row((uint16)white_top_row, RGB565_GREEN);
    Draw_one_line_row((uint16)start_row, RGB565_BLUE);
    Draw_one_line_row((uint16)end_row, RGB565_BLUE);
}
void ips200_show_yaw_only(void)
{
    ips200.clear();

    ips200_show_string(0, 0, "VIEW:IMU");
    ips200_show_string(0, 1, "IMU YAW TEST");

    ips200_show_string(0, 2, "yaw:");
    ips200_show_float(48, 2, yaw, 4, 2);

    ips200_show_string(0, 4, "gyro_z:");
    ips200_show_float(64, 4, gyro_z, 4, 2);

    ips200_show_string(0, 6, "f_gyro_z:");
    ips200_show_float(80, 6, f_gyro_z, 4, 2);

    ips200_show_string(0, 8, "raw_z:");
    ips200_show_int(56, 8, (int32)raw_gyro_z, 6);

    ips200_show_string(0, 10, "NO IMG / NO TCP");
}
