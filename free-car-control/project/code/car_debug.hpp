#ifndef _CAR_DEBUG_HPP_
#define _CAR_DEBUG_HPP_

#include "zf_common_typedef.hpp"
#include <stdio.h>
#include "image.hpp"
#include "d1lb_tof.hpp"
#include "ramp_detect.hpp"
#include "obstacle_detect.hpp"

// 屏幕显示一个字符大小：8x16
// ips200 屏幕是 240x320：一行 30 字符，一列 20 字符
// 图像显示需要 120x160 大小
#define LINE_HEIGHT     16
#define LINE_WIDTH      8

extern uint8 tcp_left_boundary_x[IMG_PROC_HEIGHT], tcp_right_boundary_x[IMG_PROC_HEIGHT], tcp_middle_line_x[IMG_PROC_HEIGHT], tcp_max_white_x[IMG_PROC_HEIGHT];
extern uint8 tcp_left_boundary_y[IMG_PROC_HEIGHT], tcp_right_boundary_y[IMG_PROC_HEIGHT], tcp_middle_line_y[IMG_PROC_HEIGHT], tcp_max_white_y[IMG_PROC_HEIGHT];

typedef enum
{
    TCP_DEBUG_IMAGE_COLOR = 0,   // 原始彩色图
    TCP_DEBUG_IMAGE_GRAY = 1,    // 灰度图
    TCP_DEBUG_IMAGE_RED_MASK = 2, // 红色掩码图（叠加旋转矩形）
    TCP_DEBUG_IMAGE_ROI_PATCH = 3 // extract_rotated_roi_patch 后的 roi_img
} TcpDebugImageMode;
extern TcpDebugImageMode g_tcp_debug_image_mode;

// RED_MASK 模式下第3路边界显示哪一种旋转框
typedef enum
{
    TCP_RED_MASK_RECT_RED = 0, // 显示 red_rect（最优红块旋转矩形）
    TCP_RED_MASK_RECT_ROI = 1  // 显示 roi_rect（build_rotated_roi_above_red 结果）
} TcpRedMaskRectMode;
extern TcpRedMaskRectMode g_tcp_red_mask_rect_mode;

// 调试主接口
void tcp_main_use(void);
void ips200_main_use(void);
void ips200_show_yaw_only(void);

void Oscill(void);
void Oscill_param_spid(void);
void Oscill_param_dir(void);
void Oscill_param_img(void);
void Oscill_param_turn_test(void);
void Oscill_param_speed(void);

void tcp_send_boundary_info_update(void);
void tcp_send_gray_image(void);
void tcp_send_color_image(void);

void ips200_show_param(void);
void ips200_show_ramp_debug(void);
void ips200_show_obstacle_debug(void);
void ips200_show_string(uint16 x, uint16 y, const char dat[]);
void ips200_show_int(uint16 x, uint16 y, const int32 dat, uint8 num);
void ips200_show_float(uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum);
void Draw_one_line_col(uint16 col, const uint16 color);
void Draw_one_line_row(uint16 row, const uint16 color);
void Draw_prospect(void);
void DrawUseLines_color(uint8 num, char s, const uint16 color);
void DrawFilterLines_color(uint8 num, char s, const uint16 color);
void Draw_image_mid(void);
void ips200_show_element_debug(void);

#endif
