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
* 文件名称          zf_driver_pit
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K0300
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
* 2025-12-30        修正           解决pthread_t无符号比较警告，零警告编译
********************************************************************************************************************/

#include "zf_driver_pit.hpp"

#define PIT_THREAD_PRIORITY      99
#define EPOLL_WAIT_TIMEOUT       -1

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  类的无参构造函数
// 参数说明  无
// 返回参数  无
// 使用示例  zf_driver_pit pit_timer;
// 备注信息  初始化成员变量，句柄置-1，标志位置0，回调置空指针
//-------------------------------------------------------------------------------------------------------------------
zf_driver_pit::zf_driver_pit(void)
{
    printf("zf_driver_pit\r\n");
    pit_exit_flag = 0;
    pit_timer_fd = -1;
    pit_epoll_fd = -1;
    pit_thread_id = 0;    // ✅ 修复1: pthread_t是无符号类型，初始化用0表示未创建，替代-1
    pit_period_ms = PIT_MIN_PERIOD_MS;
    pit_user_callback = NULL;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  类的析构函数
// 参数说明  无
// 返回参数  无
// 使用示例  对象销毁自动调用
// 备注信息  调用停止接口，确保所有资源正常释放，无内存泄漏
//-------------------------------------------------------------------------------------------------------------------
zf_driver_pit::~zf_driver_pit(void)
{
    printf("~zf_driver_pit\r\n");
    stop();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  设置线程为SCHED_FIFO实时调度+99级最高优先级
// 参数说明  无
// 返回参数  int 成功返回0 失败返回-1
// 使用示例  内部调用，无需外部调用
// 备注信息 
//-------------------------------------------------------------------------------------------------------------------
int zf_driver_pit::set_realtime_priority(void)
{
    pthread_attr_t attr;
    struct sched_param sched_param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    memset(&sched_param, 0, sizeof(sched_param));
    sched_param.sched_priority = PIT_THREAD_PRIORITY;
    pthread_attr_setschedparam(&attr, &sched_param);

    if (pthread_setschedparam(pit_thread_id, SCHED_FIFO, &sched_param) != 0)
    {
        perror("pthread_setschedparam error");
        pthread_attr_destroy(&attr);
        return -1;
    }

    pthread_attr_destroy(&attr);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化内核timerfd定时器句柄
// 参数说明  无
// 返回参数  int 成功返回定时器文件句柄 失败返回-1
// 使用示例  内部调用，无需外部调用
// 备注信息  基于CLOCK_MONOTONIC系统时钟，定时时间不受系统时间修改影响
//-------------------------------------------------------------------------------------------------------------------
static int timerfd_handle_init(void)
{
    int timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    return timer_fd;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  初始化内核epoll句柄
// 参数说明  无
// 返回参数  int 成功返回epoll文件句柄 失败返回-1
// 使用示例  内部调用，无需外部调用
// 备注信息  创建epoll实例，用于监听定时器fd的可读事件
//-------------------------------------------------------------------------------------------------------------------
static int epollfd_handle_init(void)
{
    int epoll_fd = epoll_create1(EPOLL_CLOEXEC);
    return epoll_fd;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  PIT定时器核心线程，阻塞监听定时器事件
// 参数说明  arg 类对象指针
// 返回参数  void* 线程返回值，固定返回NULL
// 使用示例  内部调用，无需外部调用
// 备注信息  epoll边缘触发无轮询，无事件时CPU占用率0%，触发后执行回调
//-------------------------------------------------------------------------------------------------------------------
void *zf_driver_pit::pit_timer_thread(void *arg)
{
    zf_driver_pit *pit_obj = (zf_driver_pit *)arg;
    struct epoll_event events[1];
    uint64_t timer_expire_cnt = 0;
    int ret = 0;

    prctl(PR_SET_NAME, "pit_timer_thread");

    while (!pit_obj->pit_exit_flag)
    {
        ret = epoll_wait(pit_obj->pit_epoll_fd, events, 1, EPOLL_WAIT_TIMEOUT);
        if (ret < 0)
        {
            if (errno == EINTR && pit_obj->pit_exit_flag) break;
            perror("epoll_wait error");
            break;
        }

        for (int i = 0; i < ret; i++)
        {
            if (events[i].data.fd == pit_obj->pit_timer_fd && (events[i].events & EPOLLIN))
            {
                read(pit_obj->pit_timer_fd, &timer_expire_cnt, sizeof(uint64_t));
                if(pit_obj->pit_user_callback != NULL)
                {
                    pit_obj->pit_user_callback();
                }
            }
        }
    }

    pthread_exit(NULL);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  PIT定时器初始化，配置周期并注册回调函数
// 参数说明  period_ms 定时器周期，单位ms
// 参数说明  callback 用户业务回调函数指针
// 返回参数  int 成功返回0 失败返回-1
// 使用示例  pit_timer.pit_ms_init(5, my_callback);
// 备注信息  整合所有初始化流程，内部调用timerfd和epoll初始化接口
//-------------------------------------------------------------------------------------------------------------------
int zf_driver_pit::init_ms(uint32_t period_ms, pit_callback_fun callback)
{
    struct itimerspec timer_spec;
    struct epoll_event epoll_ev;
    int ret = 0;

    pit_period_ms = period_ms;

    if(callback == NULL)
    {
        printf("pit_init error: callback function is null!\n");
        return -1;
    }
    pit_user_callback = callback;

    pit_timer_fd = timerfd_handle_init();
    if (pit_timer_fd < 0)
    {
        perror("timerfd_create error");
        return -1;
    }

    memset(&timer_spec, 0, sizeof(timer_spec));
    timer_spec.it_interval.tv_sec  = pit_period_ms / 1000;
    timer_spec.it_interval.tv_nsec = (pit_period_ms % 1000) * 1000000;
    timer_spec.it_value.tv_sec     = 0;
    timer_spec.it_value.tv_nsec    = 1;

    ret = timerfd_settime(pit_timer_fd, 0, &timer_spec, NULL);
    if (ret < 0)
    {
        perror("timerfd_settime error");
        close(pit_timer_fd);
        pit_timer_fd = -1;
        return -1;
    }

    pit_epoll_fd = epollfd_handle_init();
    if (pit_epoll_fd < 0)
    {
        perror("epoll_create1 error");
        close(pit_timer_fd);
        pit_timer_fd = -1;
        return -1;
    }

    memset(&epoll_ev, 0, sizeof(epoll_ev));
    epoll_ev.data.fd = pit_timer_fd;
    epoll_ev.events  = EPOLLIN | EPOLLET;
    ret = epoll_ctl(pit_epoll_fd, EPOLL_CTL_ADD, pit_timer_fd, &epoll_ev);
    if (ret < 0)
    {
        perror("epoll_ctl error");
        close(pit_timer_fd);
        close(pit_epoll_fd);
        pit_timer_fd = -1;
        pit_epoll_fd = -1;
        return -1;
    }

    ret = pthread_create(&pit_thread_id, NULL, pit_timer_thread, this);
    if (ret != 0)
    {
        perror("pthread_create error");
        close(pit_timer_fd);
        close(pit_epoll_fd);
        pit_timer_fd = -1;
        pit_epoll_fd = -1;
        return -1;
    }

    if (set_realtime_priority() != 0)
    {
        printf("pit warning: 请用sudo运行程序，否则无法设置实时优先级！\n");
    }

    printf("pit init success, period = %d ms\r\n", pit_period_ms);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  停止PIT定时器并释放所有资源
// 参数说明  无
// 返回参数  void 无返回值
// 使用示例  pit_timer.stop();
// 备注信息  先置退出标志，再等待线程结束，最后关闭所有句柄，安全无异常
//-------------------------------------------------------------------------------------------------------------------
void zf_driver_pit::stop(void)
{
    if(pit_exit_flag) return;

    pit_exit_flag = 1;

    if(pit_thread_id != 0)    // ✅ 修复2: 无符号类型判断，0表示未创建线程，非0表示有效线程ID
    {
        pthread_join(pit_thread_id, NULL);
        pit_thread_id = 0;    // ✅ 修复3: 重置为0，而非-1，无符号类型规范赋值
    }

    if(pit_timer_fd >= 0)
    {
        close(pit_timer_fd);
        pit_timer_fd = -1;
    }

    if(pit_epoll_fd >= 0)
    {
        close(pit_epoll_fd);
        pit_epoll_fd = -1;
    }

    pit_user_callback = NULL;
    printf("pit timer stop success, resource release ok\n");
}