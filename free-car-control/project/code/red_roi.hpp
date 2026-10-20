#ifndef _RED_ROI_HPP_
#define _RED_ROI_HPP_

#include "zf_common_headfile.hpp"

/**
 * @brief 红色色块检测与ROI定位结果
 * @note  found=false 时，其余字段无效。
 */
typedef struct
{
    bool red_found;            // 是否找到红色色块
    bool found;                // 是否找到满足条件的ROI
    cv::Rect roi_rect;         // 旋转ROI的外接矩形（原图坐标系）
    float roi_angle_deg;       // 旋转ROI角度（度，x轴逆时针为正）
    cv::Point2f roi_points[4]; // 旋转ROI四角点（tl,tr,bl,br）
    int roi_size_px;           // 兼容字段：ROI宽度（像素）
    int roi_width_px;          // ROI宽度（像素）
    int roi_height_px;         // ROI高度（像素）
} RedRoiResult;

typedef struct
{
    bool mask_valid;
    bool rect_valid;
    bool roi_valid;
    cv::Mat red_mask_full;
    cv::Point2f rect_points[4];
    cv::Point2f roi_points[4];
    cv::Point2f rect_center;
    float rect_long_side_px;
    float rect_short_side_px;
} RedRoiDebugInfo;

/**
 * @brief 检测红色引导块并输出ROI结果（不做模型推理）
 */
int red_roi_find(const cv::Mat& src_bgr, RedRoiResult* roi_result);

// ===== 调试接口开始：给 car_debug/TCP 读取 red_roi 调试 patch =====
bool red_roi_get_debug_min_rect_patch(cv::Mat* rect_img);
bool red_roi_get_debug_min_rect_mask_patch(cv::Mat* mask_img);
bool red_roi_get_debug_info(RedRoiDebugInfo* debug_info);
bool red_roi_get_debug_rect_points(cv::Point2f* rect_points);
bool red_roi_get_debug_selected_edge_points(cv::Point2f* edge_points);
bool red_roi_get_debug_bottom_axis_points(cv::Point2f* axis_points);
// ===== 调试接口结束：给 car_debug/TCP 读取 red_roi 调试 patch =====

/**
 * @brief 根据旋转ROI四点提取并拉正 patch
 * @param src_bgr 原图（BGR）
 * @param roi_points 旋转ROI四角点（tl,tr,bl,br）
 * @param roi_width 输出宽
 * @param roi_height 输出高
 * @param roi_img 输出ROI图
 * @return true=成功，false=失败
 */
bool extract_rotated_roi_patch(const cv::Mat& src_bgr,
                               const cv::Point2f* roi_points,
                               int roi_width,
                               int roi_height,
                               cv::Mat* roi_img);

bool extract_rotated_roi_patch(const cv::Mat& src_bgr,
                               const cv::Point2f* roi_points,
                               int roi_size,
                               cv::Mat* roi_img);

void red_roi_debug_test_pic(const cv::Mat& src_bgr);
void red_roi_debug_test_video(const cv::Mat& src_bgr);

#endif
