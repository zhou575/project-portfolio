#ifndef _IMAGE_HPP_
#define _IMAGE_HPP_
//很干净的头文件

#include "zf_common_typedef.hpp"
#include <stdio.h>


/*
    对于图像
    ----------->x轴(行)
    |
    |
    |
    ^
    y轴(列)
 */
//方便图像处理调用
#define IMG_PROC_DOWNSAMPLE 2
#define IMG_PROC_WIDTH      (UVC_WIDTH / IMG_PROC_DOWNSAMPLE)
#define IMG_PROC_HEIGHT     (UVC_HEIGHT / IMG_PROC_DOWNSAMPLE)

#define AT(img, row, cols)  ((img)[(row) * (IMG_PROC_WIDTH) + (cols)])

/**
 * @brief 差比和,传入int16,输出也是int类型,放大200倍
 * @note 需要left>right,并避免除0问题
 */
#define GRAY_DIFF_SUM(left, right) \
    (((left) + (right) == 0) ? 0 : \
    (((left) - (right))*200 / ((left) + (right))))

/**
 * @brief 搜线优化
 * @note 优化方案类似二值化，只不过给了一个范围，在这个范围内进行差比和
 */
//根据自己的图像曝光调
#define BLACK_THRESHOLD 50 // 黑色像素的灰度阈值,小于这个直接判断为黑点
#define WHITEMAXMUL     13 // 白色像素最大值基于参考点的放大倍数，10为不放大
#define WHITEMINMUL     7  // 白色像素最小值基于参考点的放大倍数，10为不放大
#define REFPCNTROW 5 //参考点统计行数
#define SEARCHWIDTH 8 //边线跟踪搜索宽度，左右各10列
#define MAXWOFFSET 2 //最长白列搜索偏移，每隔n行搜索最长白列，加快速度


//前瞻 最大范围从100-20
#define start_line 100 
#define end_line   20
#define error_maxw_start (IMG_PROC_HEIGHT-start_line)
#define error_maxw_end (IMG_PROC_HEIGHT-end_line)
#define EDGE_MAX_LEN (start_line-end_line)
#define Min_white_nums 20

//让图像对称
//目前图像很正不用更改
#define USE_WIDTH_L 0
#define USE_WIDTH_R IMG_PROC_WIDTH
#define UVC_USE_WIDTH (USE_WIDTH_R-USE_WIDTH_L)//图像使用宽度
#define USE_MIDDLE_VALUE ((USE_WIDTH_L+USE_WIDTH_R)/2) //图像实际使用中值

//最边界的不进行搜索，觉得10太小也可以改大一点，速度决策可能有问题
#define Max_white_L_limit (USE_WIDTH_L+20)
#define Max_white_R_limit (USE_WIDTH_R-20)

//限幅函数
#define CLAMP(x, low, high) ((x) < (low) ? (low) : ((x) > (high) ? (high) : (x)))

/*
绕行状态机总流程（高层）：
1) BYPASS_IDLE
   - 正常巡线，无模型绕行接管。
2) BYPASS_STAGE1_TURN_OUT
   - 模型识别到左/右绕行后进入。
   - 先执行“绕出”动作，目标是绕开障碍区域。
3) BYPASS_STAGE2_TURN_BACK
   - Stage1 满足阶段切换条件后进入。
   - 反向回摆，等待重新搜到稳定边线。
4) BYPASS_STAGE3_TURN_BACK
   - 仅三阶段策略使用。
   - 用边线作为临时中线跟踪，准备切回正常搜线。
5) BYPASS_RECOVER
   - 切回正常中线，并满足连续安全帧后退出。
6) 回到 BYPASS_IDLE
   - 释放绕行状态，解锁下一次模型识别。
*/
//绕行状态机
typedef enum
{
    BYPASS_IDLE = 0,               //没有模型绕行，输出正常搜线误差
    BYPASS_STAGE1_TURN_OUT = 1,    //阶段1：按固定误差快速绕出（转向障碍外侧）
    BYPASS_STAGE2_TURN_BACK = 2,   //阶段2：反向回摆（目标是重新找到有效边线）
    BYPASS_STAGE3_TURN_BACK = 3,   //阶段3：边线跟踪回赛道，准备切回正常中线
    BYPASS_RECOVER = 4,            //恢复态：完成收尾复位并释放模型锁存
} bypass_state_enum;

//绕行状态
typedef enum
{
    BYPASS_PROFILE_UNIFIED = 0, //统一绕行：±50 -> ∓10 -> ±60 -> 边线回赛道 -> 正常中线恢复退出
} bypass_profile_enum;

//绕行结束原因
typedef enum
{
    BYPASS_FINISH_NONE = 0,    //未结束（运行中）
    BYPASS_FINISH_NORMAL = 1,  //满足恢复条件正常结束
    BYPASS_FINISH_PROTECT = 2, //触发保护结束
    BYPASS_FINISH_MANUAL = 3,  //按键手动复位结束
} bypass_finish_reason_enum;

//绕行时的中线来源
typedef enum
{
    BYPASS_CENTER_NORMAL = 0,    // 正常中线：左右边线共同求中线
    BYPASS_CENTER_LEFT_EDGE = 1, // 左边线中线：直接把左边线当作“临时中线”
    BYPASS_CENTER_RIGHT_EDGE = 2,// 右边线中线：直接把右边线当作“临时中线”
} bypass_center_mode_enum;

//用于存储图像处理需要用到的变量信息
typedef struct{
    uint8 diff_sum_mov;//每n个点进行差比和，中间n-1个点
    int16 mw_diff_thershold;
    int16 diff_thershold;//差比和阈值
    uint8 lookahead;
    uint8 ref_point; //参考点的值
    uint8 white_max_point;//最大的白点
    uint8 white_min_point;//最小的白点
    float midline_error;
    bool protect_flag;
}ImagePro_var;

//边界信息
typedef struct {
    uint8 pos[2];      // [0]:x轴, [1]:y轴
    uint8 sign;        // 边界类型 0丢边，1有边
} BoundaryInfo;

//图像信息
typedef struct {
    BoundaryInfo left[UVC_HEIGHT];
    BoundaryInfo right[UVC_HEIGHT];
    BoundaryInfo filter_left[UVC_HEIGHT];
    BoundaryInfo filter_right[UVC_HEIGHT];
    uint8 midline[UVC_HEIGHT][2];  // 中线信息
    uint8 filter_midline[UVC_HEIGHT][2];
    uint8 road_width[UVC_HEIGHT];   // 道路宽度
} ImageInfo;

//最长白列
typedef struct{
    uint8 col;//列的位置
    uint8 nums;//列的数量，最长白列的长度
    uint8 center_col_nums;//图像对称轴列的白列长度
    uint8 round_nums;//圆环最长白列会重新搜索
    uint8 round_cols;//圆环最长白列的列位置
}Max_white_cols;

extern ImageInfo g_image_info;//搜线信息
extern Max_white_cols max_white;//最长白列信息
extern ImagePro_var imgpro_var;//图像变量信息
extern uint8* gray_image;//灰度图像信息
extern uint16* color_image;
const uint8* get_gray_image_proc_ptr(void);
const uint8* get_gray_image_debug_show_ptr(void);

void image_pro_init(void);
int get_gray_image(void);
int get_color_image(void);
void clear_line_info(void);
void out_protect(void);
void image_protect_reset_state(void);
void image_bypass_reset_state(void);

void get_reference_point(void);
void search_max_white(void);
int search_edge(void);

void find_middline(uint8 Midline[][2],BoundaryInfo edge1[],BoundaryInfo edge2[]);
void filter_one_edge_median3(BoundaryInfo filter_edge[],BoundaryInfo edge[],uint8 valid_len);
void smooth_midline_121(uint8 filter_line[][2],uint8 line[][2],uint8 valid_len);
float calc_midline_error(uint8 Midline[][2],uint8 nums,int lookaheadline);

void image_pro(void);
void update_image_error(void);

void image_pro_bypass(void);
void update_image_error_bypass(void);

//test
void image_max_white_test(void);

#endif
