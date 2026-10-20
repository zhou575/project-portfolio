# 当前工程相对 `6481903` 的代码差异说明

本文档只记录当前工程相对 `6481903 (origin/feat/final-local-update) 5_5绕行完成-速度决策已加complete,添加gitignore和README` 这一版本的**代码差异部分**。

注：现在的代码是无模型绕行纯控制的代码

## 当前工程中要调节的参数以及参数的作用

现在重要的参数有 控制方面的参数和图像方面的参数

1. 左右速度环PID参数
速度环PID依旧用的是增量式PID，这个PID很重要，影响我们车的反应速度，车能不能跑快就看这个PID了
```
    //左电机闭环参数
    l_motor.kp=220;
    l_motor.ki=20;
    l_motor.kd=104;
    //右电机闭环参数
    r_motor.kp=220；
    r_motor.ki=20;
    r_motor.kd=104;
```

2. 现在使用双串，外环的参数只有
```
speed_dir_info.dir_kp=  1.05f;   
    这个参数影响的是差速的范围对应着 float steer_output = base_speed * (normalized_error * dir_kp_use);
    影响差速跟随误差的变化速度，同样三轮转向误差和基础速度挂钩，要乘以基础速度base_speed
    
speed_dir_info.dir_kz=  2.0f;
    这个参数是一个抑制项，用来减少因为speed_dir_info.dir_kp，造成的车身抖动
```
但是要注意上面这两个参数都是调节转向的，最重要的是速度环PID参数，并且调节转向还要和前瞻imgpro_var.lookahead=65;一起调

```
imgpro_var.lookahead=65;
    这个参数调的远了会导致进弯道很内切，但是弯道中转不过来，调的近了会导致小车外切，并且反应比较慢
```

3. 差速加减比例项目
这个也可以和外环一起调
编码器速度越大变化越快，加减速相同速度快的话可能转弯转不过来
```
speed_dir_info.diff_add_gain= 1.0f;   //外侧轮加速系数
speed_dir_info.diff_sub_gain= 1.45f;  //内侧轮减速系数
    加速系数不变就是计算出的差速值，减速为计算出差速值乘以1.45，减的更多，过弯容易了，但是弯道的速度下降了
```

4. 速度决策参数
```
speed_limit.speed_min=0;//42;
speed_limit.speed_max=0;//65;
    当前测的speed_limit.speed_min可以是42，speed_limit.speed_max可以是65

speed_limit.startup_step_up = 0.25f;          // 起步阶段最大加速步进
speed_limit.speed_step_up=0.25f;    // 巡航阶段每周期最大加速步进
    参数作用是匀加速，防止翘头
    这两个参数是每一帧差速能够变化的速度，起步就是每一帧期望速度最多增加0.25，期望速度开始是30，然后实际的期望速度变化为0，0.25,0.5
```

## 2. 修改的文件列表

当前工程相对 `6481903` 发生差异的文件如下：

1. `libraries/zf_device/zf_device_ips200_fb.hpp`
2. `project/code/car_debug.cpp`
3. `project/code/car_debug.hpp`
4. `project/code/encoder.cpp`
5. `project/code/image.cpp`
6. `project/code/image.hpp`
7. `project/code/menu.cpp`
8. `project/code/motor.cpp`
9. `project/code/motor.hpp`
10. `project/user/build.sh`
11. `project/user/main.cpp`
12. `project/user/zhy_log.txt`

其中主要更改的代码的地方是`project/code/motor.cpp`，其他的地方只是添加了一些功能，修改了一些bug
---

## 3. 各文件修改内容

### 主要控制逻辑的修改

控制主方案已经从旧的多套并存，收敛到当前的“速度决策 + 图像差速 + 角速度补偿 + 双轮速度闭环”这一套。

1. 保护现在分了两套逻辑：
    图像保护 `imgpro_var.protect_flag`
- 把左右轮期望速度给 0
- 再走速度环，利用PID停下来
    堵转保护 `s_motor_stall_protect_flag`
-  直接设置PWM为0

2. 三串级主控逻辑重写
```
当前 `three_cascade_test()` 的核心思路是：

1. 对图像误差乘以基于最长白列的增益
2. 得到 `error_use`
3. 再归一化、限幅
4. 用 `base_speed * dir_kp_use * normalized_error` 算出差速主量
5. 再叠加角速度补偿：
   - `gyro_comp = dir_kz * (motor_control_gyro_z * deg2rad)`
6. 最后通过：
   - `diff_sub_gain`
   - `diff_add_gain`
   对左右轮做不对称分配
```

3. 新增最长白列对应误差增益函数
```
- 新增：
  - `calc_error_gain_from_white(uint8 white_num)`

逻辑：
- 最长白列大时，误差倍率接近 1
- 最长白列小时，误差倍率变大
- 中间采用 smoothstep 方式平滑过渡

- 大弯或复杂路段时放大图像误差，提高转向灵敏度。

```

4. 速度决策 `motor_speed_control()` 重写
```
当前速度决策逻辑的主要变化：

- `speed_min > speed_max` 时，不再自动交换，而是直接返回 0
- 当 `speed_min == 0` 或 `speed_max == 0` 时，认为速度决策关闭
- 起步退出阈值不再来自固定成员变量，而是动态计算：
  - `startup_exit_th = CLAMP(speed_min - 2.0f, 0.0f, 100.0f);`
- 最终增加调试量保存：
  - `speed_dir_info.speed_score`
  - `speed_dir_info.speed_target_debug`
```

5. 更改了速度决策起步翘头的bug
```
- `speed_uniform_acceleration()` 删除了 `speed_min` 参数。
- 下限统一按 0 处理，而不是外部再传一个最小值。
```

6. 删除旧控制方案冗余逻辑
```
相对 `6481903`，当前版本删除或不再保留声明的旧方案包括：
- `three_cascade_motor_control`
- `double_cascade_motor_control`
- `parallel_motor_control`
- 一部分旧的差速整形辅助逻辑
- 旧的部分角速度/自动电机 PID 配套逻辑
```

7. 相应的motor.hpp也有了变化

### 新增的内容和控制以外bug的修改

1. 新增赛道宽度表初始化函数
==新增的赛道宽度还没有用，之后再用==
```
- 新增：
  - `static void init_road_width_table(void);`

逻辑：
- 按行号对 `g_image_info.road_width[row]` 做线性赋值。
- 当前实现中：
  - 远处行宽较小
  - 近处行宽较大
- 形成一张随行号变化的赛道宽度表。
```

2. 图像bug修改
```
`search_edge()` 中右边界初始搜索部分的方向判断和循环方向做了修正

- `filter_one_edge_median3()` 中，滤波后的边线结构体补上了：
  - `filter_edge[i].sign = edge[i].sign;`

- `updata_image_error()` 改为 `update_image_error()`
- `updata_image_error_bypass()` 改为 `update_image_error_bypass()`
```

3. 其他bug都是小bug就说了