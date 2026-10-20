/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K0300 开源库的一部分
*
* LS2K0300 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
********************************************************************************************************************/

#include "zf_device_ips200_fb.hpp"
#include "zf_common_font.hpp"
#include "zf_common_function.hpp"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无参构造函数
// 参数说明     void
// 返回参数     void
// 使用示例     zf_device_ips200 ips200;
// 备注信息     初始化默认画笔和背景颜色，显存地址置空
//-------------------------------------------------------------------------------------------------------------------
zf_device_ips200::zf_device_ips200(void)
{
    pen_color = DEFAULT_PENCOLOR;
    bg_color = DEFAULT_BGCOLOR;
    width = 0;
    height = 0;
    screen_base = nullptr;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清屏函数
// 参数说明     void
// 返回参数     void
// 使用示例     ips200.clear();
// 备注信息     将屏幕清空成背景颜色
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::clear(void)
{
    full(DEFAULT_BGCOLOR);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     屏幕填充函数
// 参数说明     color           颜色格式 RGB565 或者可以使用 zf_common_font.h 内 rgb565_color_enum 枚举值或者自行写入
// 返回参数     void
// 使用示例     ips200.full(RGB565_BLACK);
// 备注信息     将屏幕填充成指定颜色
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::full(const uint16 color)
{
    uint16 i,j;
    for(i=0;i<240; i++)
    {
        for(j=0; j<320; j++)
        {
            draw_point(i, j, color);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画点函数
// 参数说明     x               坐标x方向的点位置 [0, width-1]
// 参数说明     y               坐标y方向的点位置 [0, height-1]
// 参数说明     color           颜色格式 RGB565 或者可以使用 zf_common_font.h 内 rgb565_color_enum 枚举值或者自行写入
// 返回参数     void
// 使用示例     ips200.draw_point(10, 20, RGB565_BLUE);
// 备注信息     在指定坐标绘制一个像素点
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::draw_point(uint16 x, uint16 y, const uint16 color)
{
    if(nullptr != screen_base)
    {
        screen_base[y * width + x] = color;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     画线函数
// 参数说明     x_start         坐标x方向的起点 [0, width-1]
// 参数说明     y_start         坐标y方向的起点 [0, height-1]
// 参数说明     x_end           坐标x方向的终点 [0, width-1]
// 参数说明     y_end           坐标y方向的终点 [0, height-1]
// 参数说明     color           颜色格式 RGB565 或者可以使用 zf_common_font.h 内 rgb565_color_enum 枚举值或者自行写入
// 返回参数     void
// 使用示例     ips200.draw_line(0, 0, 10, 10, RGB565_RED);
// 备注信息     在指定两点之间绘制一条直线
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::draw_line (uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color)
{
    int16 x_dir = (x_start < x_end ? 1 : -1);
    int16 y_dir = (y_start < y_end ? 1 : -1);
    float temp_rate = 0;
    float temp_b = 0;

    do
    {
        if(x_start != x_end)
        {
            temp_rate = (float)(y_start - y_end) / (float)(x_start - x_end);
            temp_b = (float)y_start - (float)x_start * temp_rate;
        }
        else
        {
            while(y_start != y_end)
            {
                draw_point(x_start, y_start, color);
                y_start += y_dir;
            }
            draw_point(x_start, y_start, color);
            break;
        }
        if(func_abs(y_start - y_end) > func_abs(x_start - x_end))
        {
            while(y_start != y_end)
            {
                draw_point(x_start, y_start, color);
                y_start += y_dir;
                x_start = (int16)(((float)y_start - temp_b) / temp_rate);
            }
            draw_point(x_start, y_start, color);
        }
        else
        {
            while(x_start != x_end)
            {
                draw_point(x_start, y_start, color);
                x_start += x_dir;
                y_start = (int16)((float)x_start * temp_rate + temp_b);
            }
            draw_point(x_start, y_start, color);
        }
    }while(0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS200 显示波形
// 参数说明     x               坐标x方向的起点 参数范围 [0, width-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, height-1]
// 参数说明     *wave           波形数组指针
// 参数说明     width           波形实际宽度
// 参数说明     value_max       波形实际最大值
// 参数说明     dis_width       波形显示宽度 参数范围 [0, width]
// 参数说明     dis_value_max   波形显示最大值 参数范围 [0, height]
// 返回参数     void
// 使用示例     ips200.show_wave(0, 0, data, 128, 64, 64, 32);
// 备注信息     
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_wave (uint16 x, uint16 y, const uint16 *wave, uint16 width, uint16 value_max, uint16 dis_width, uint16 dis_value_max)
{
    uint32 i = 0, j = 0;
    uint32 width_index = 0, value_max_index = 0;

    for(j = 0; j < dis_value_max; j ++)
    {
        for(i = 0; i < dis_width; i ++)
        {
            draw_point(i, y + j, bg_color);
        }
    }

    for(i = 0; i < dis_width; i ++)
    {
        width_index = i * width / dis_width;
        value_max_index = *(wave + width_index) * (dis_value_max - 1) / value_max;
        draw_point(i + x, (dis_value_max - 1) - value_max_index + y, pen_color);
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示单个字符
// 参数说明     x               坐标x方向的起点 [0, width-1]
// 参数说明     y               坐标y方向的起点 [0, height-1]
// 参数说明     dat             需要显示的单个字符 数据类型 char
// 返回参数     void
// 使用示例     ips200.show_char(10, 20, 'A');
// 备注信息     显示8x16的ASCII字符集，字符范围ASCII码32~127
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_char(uint16 x, uint16 y, const char dat)
{
    uint8 i = 0, j = 0;
    for(i = 0; 8 > i; i ++)
    {
        uint8 temp_top = ascii_font_8x16[dat - 32][i];
        uint8 temp_bottom = ascii_font_8x16[dat - 32][i + 8];
        for(j = 0; 8 > j; j ++)
        {
            if(temp_top & 0x01)
            {
                draw_point(x + i, y + j, pen_color);
            }
            else
            {
                draw_point(x + i, y + j, bg_color);
            }
            temp_top >>= 1;
        }

        for(j = 0; 8 > j; j ++)
        {
            if(temp_bottom & 0x01)
            {
                draw_point(x + i, y + j + 8, pen_color);
            }
            else
            {
                draw_point(x + i, y + j + 8, bg_color);
            }
            temp_bottom >>= 1;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示字符串
// 参数说明     x               坐标x方向的起点 [0, width-1]
// 参数说明     y               坐标y方向的起点 [0, height-1]
// 参数说明     dat             需要显示的字符串首地址 数据类型 const char[]
// 返回参数     void
// 使用示例     ips200.show_string(10, 20, "SEEKFREE");
// 备注信息     连续显示8x16的ASCII字符，字符串以'\0'结尾
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_string(uint16 x, uint16 y, const char dat[])
{
    uint16 j = 0;

    while('\0' != dat[j])
    {
        show_char(x + 8 * j,  y, dat[j]);
        j ++;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示有符号整型数
// 参数说明     x               坐标x方向的起点 [0, width-1]
// 参数说明     y               坐标y方向的起点 [0, height-1]
// 参数说明     dat             需要显示的变量 数据类型 int32
// 参数说明     num             整数显示长度 最高支持10位
// 返回参数     void
// 使用示例     ips200.show_int(10, 20, -123, 3);
// 备注信息     超出长度会显示低位数值，不足长度补空格，支持负数显示负号
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_int(uint16 x, uint16 y, const int32 dat, uint8 num)
{
    int32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12] = {0};

    std::memset(data_buffer, 0, 12);
    std::memset(data_buffer, ' ', num+1);

    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_int_to_str(data_buffer, dat_temp);
    show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示无符号整型数
// 参数说明     x               坐标x方向的起点 [0, width-1]
// 参数说明     y               坐标y方向的起点 [0, height-1]
// 参数说明     dat             需要显示的变量 数据类型 uint32
// 参数说明     num             整数显示长度 最高支持10位
// 返回参数     void
// 使用示例     ips200.show_uint(10, 20, 123, 3);
// 备注信息     超出长度会显示低位数值，不足长度补空格，无符号不显示负号
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_uint(uint16 x, uint16 y, const uint32 dat, uint8 num)
{
    uint32 dat_temp = dat;
    int32 offset = 1;
    char data_buffer[12] = {0};
    std::memset(data_buffer, 0, 12);
    std::memset(data_buffer, ' ', num);

    if(10 > num)
    {
        for(; 0 < num; num --)
        {
            offset *= 10;
        }
        dat_temp %= offset;
    }
    func_uint_to_str(data_buffer, dat_temp);
    show_string(x, y, (const char *)&data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示浮点数(去除整数部分无效的0)
// 参数说明     x               坐标x方向的起点 参数范围 [0, width-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, height-1]
// 参数说明     dat             需要显示的变量 数据类型 double
// 参数说明     num             整数位显示长度   最高8位
// 参数说明     pointnum        小数位显示长度   最高6位
// 返回参数     void
// 使用示例     ips200.show_float(0, 0, x, 2, 3);
// 备注信息     特别注意当发现小数部分显示的值与你写入的值不一样的时候，可能是由于浮点数精度丢失问题导致的，这并不是显示函数的问题，有关问题的详情，请自行百度学习   浮点数精度丢失问题。负数会显示一个 '-'号
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_float (uint16 x, uint16 y, const double dat, uint8 num, uint8 pointnum)
{
    double dat_temp = dat;
    double offset = 1.0;
    char data_buffer[17] = {0};
    std::memset(data_buffer, 0, 17);
    std::memset(data_buffer, ' ', num+pointnum+2);

    for(; 0 < num; num --)
    {
        offset *= 10;
    }
    dat_temp = dat_temp - ((int)dat_temp / (int)offset) * offset;
    func_double_to_str(data_buffer, dat_temp, pointnum);
    show_string(x, y, data_buffer);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS200 显示 8bit 灰度图像 带二值化阈值
// 参数说明     x               坐标x方向的起点 参数范围 [0, ips200_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, ips200_y_max-1]
// 参数说明     *image          图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 参数说明     dis_width       图像显示宽度 参数范围 [0, ips200_x_max]
// 参数说明     dis_height      图像显示高度 参数范围 [0, ips200_y_max]
// 参数说明     threshold       二值化显示阈值 0-不开启二值化
// 返回参数     void
// 使用示例     show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
// 备注信息     用于显示总钻风的图像
//              如果要显示二值化图像 直接修改最后一个参数为需要的二值化阈值即可
//              如果要显示二值化图像 直接修改最后一个参数为需要的二值化阈值即可
//              如果要显示二值化图像 直接修改最后一个参数为需要的二值化阈值即可
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_gray_image (uint16 x, uint16 y, const uint8 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 threshold)
{
    uint32 x_start = 0, y_start = 0;
    uint16 color = 0;

    for(y_start = y; y_start < (y + height); y_start++)
    {
        for(x_start = x; x_start < (x + width); x_start++)
        {
            uint8 gray_value = image[(x_start - x) + (y_start- y) * width];
            if(threshold == 0)
            {
                uint16 r = (gray_value >> 3) & 0b11111;
                uint16 g = (gray_value >> 2) & 0b111111;
                uint16 b = (gray_value >> 3) & 0b11111;
                color = (r << 11) | (g << 5) | (b << 0);
            }
            else if(gray_value < threshold)
            {
                color = (RGB565_BLACK);
            }
            else
            {
                color = (RGB565_WHITE);
            }
            draw_point(x_start, y_start, color);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     IPS200 显示 RGB565 彩色图像
// 参数说明     x               坐标x方向的起点 参数范围 [0, ips200_x_max-1]
// 参数说明     y               坐标y方向的起点 参数范围 [0, ips200_y_max-1]
// 参数说明     *image          图像数组指针
// 参数说明     width           图像实际宽度
// 参数说明     height          图像实际高度
// 参数说明     dis_width       图像显示宽度 参数范围 [0, ips200_x_max]
// 参数说明     dis_height      图像显示高度 参数范围 [0, ips200_y_max]
// 参数说明     color_mode      色彩模式 0-低位在前 1-高位在前
// 返回参数     void
// 使用示例     show_rgb565_image(0, 0, scc8660_image[0], SCC8660_W, SCC8660_H, SCC8660_W, SCC8660_H, 1);
// 备注信息     用于显示凌瞳的 RGB565 的图像
//              如果要显示低位在前的其他 RGB565 图像 修改最后一个参数即可
//              如果要显示低位在前的其他 RGB565 图像 修改最后一个参数即可
//              如果要显示低位在前的其他 RGB565 图像 修改最后一个参数即可
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::show_rgb565_image (uint16 x, uint16 y, const uint16 *image, uint16 width, uint16 height, uint16 dis_width, uint16 dis_height, uint8 color_mode)
{
    uint32 x_start = 0, y_start = 0;
    for(y_start = y; y_start < (y + height); y_start++)
    {
        for(x_start = x; x_start < (x + width); x_start++)
        {
            uint16 color = image[(x_start - x) + (y_start- y) * width];
            if(1 == color_mode)
            {
                color = ((color & 0x00FF) << 8) | ((color & 0xFF00) >> 8);
            }
            draw_point(x_start, y_start, color);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     显示屏初始化函数
// 参数说明     path            framebuffer设备节点路径 如 "/dev/fb0"
// 返回参数     void
// 使用示例     ips200.init("/dev/fb0");
// 备注信息     打开fb设备、获取屏幕参数、映射显存、初始化清屏，程序初始化阶段只调用一次
//-------------------------------------------------------------------------------------------------------------------
void zf_device_ips200::init(const char *path, uint8 is_reload_driver)
{
    struct fb_fix_screeninfo fb_fix;
    struct fb_var_screeninfo fb_var;
    unsigned int screen_size;
    int fd;

    if (is_reload_driver)
    {
        printf("ips200: rmmod fb_st7789v driver ...\n");
        // 卸载fb_st7789v驱动模块，忽略卸载失败(比如驱动本就没加载的情况)
        system("rmmod fb_st7789v > /dev/null 2>&1");
        usleep(200*1000); // 延时200ms，保证驱动卸载干净，避免加载失败
        printf("ips200: insmod fb_st7789v driver ...\n");
        // 重新加载fb_st7789v驱动模块，必须保证驱动文件路径正确
        if(system("insmod /lib/modules/4.19.190/fb_st7789v.ko") != 0)
        {
            perror("insmod fb_st7789v error");
            exit(EXIT_FAILURE);
        }
        usleep(200*1000); // 延时300ms，驱动加载完成+硬件初始化完成，关键延时
        /***********************************************************************************************/
    }
    
 
    if (0 > (fd = open(path, O_RDWR))) {
        perror("open error");
        exit(EXIT_FAILURE);
    }

    ioctl(fd, FBIOGET_VSCREENINFO, &fb_var);
    ioctl(fd, FBIOGET_FSCREENINFO, &fb_fix);

    screen_size = fb_fix.line_length * fb_var.yres;
    this->width = fb_var.xres;
    this->height = fb_var.yres;

    screen_base = (unsigned short *)mmap(nullptr, screen_size, PROT_WRITE, MAP_SHARED, fd, 0);
    if (MAP_FAILED == (void *)screen_base) {
        perror("mmap error");
        close(fd);
        exit(EXIT_FAILURE);
    }

    for(uint16 i=0;i<240;i++)
    {
        for(uint16 j=0;j<320;j++)
        {
            draw_point(i, j, DEFAULT_BGCOLOR);
        }
    }
}