# TensorFlow Lite Micro — kernels 目录算子清单
说明：本文件基于目录中的源码文件名推断对应的算子实现。`*_common.cc` / `.h` 文件为共用实现或辅助函数，通常不单独列为算子。
若需要精确的“构建时启用”清单，请检查工程的构建脚本（如 CMakeLists.txt 或 BUILD 文件）。

---

## 一、神经网络层 / 线性变换
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| CONV_2D | `conv.cc`（共用：`conv_common.cc`） | 二维卷积核心算子，通过滑动卷积核提取特征图空间特征，支持步长、填充、分组等配置，是 CNN 网络核心特征提取层 |
| DEPTHWISE_CONV_2D | `depthwise_conv.cc`（共用：`depthwise_conv_common.cc`） | 深度可分离卷积，拆分标准卷积为「深度卷积（逐通道卷积）+逐点卷积（1x1卷积）」，大幅降低计算/参数量，是 MobileNet 等轻量化网络核心 |
| TRANSPOSE_CONV | `transpose_conv.cc`（转置卷积/反卷积） | 转置卷积核实现特征图上采样，恢复尺寸，用于语义分割、生成模型上采样环节 |
| FULLY_CONNECTED | `fully_connected.cc`（共用：`fully_connected_common.cc`） | 全连接层，将高维特征展平为一维向量，建立特征与输出类别的线性映射，多用于网络最后一层 |
| BATCH_MATMUL | `batch_matmul.cc`（共用：`batch_matmul_common.cc`） | 批量矩阵乘法，支持多批次矩阵相乘，适用于 Transformer 注意力计算、批量线性变换 |
| SVDF | `svdf.cc`（共用：`svdf_common.cc`） | 基于 SVD 的时序滤波器（Singular Value Decomposition Filter），用于低算力设备的时序特征提取（如语音识别） |
| UNIDIRECTIONAL_SEQUENCE_LSTM | `lstm_eval.cc`/`unidirectional_sequence_lstm.cc`（共用：`lstm_eval_common.cc`） | 单向序列 LSTM 算子，通过门控机制捕捉时序数据（语音/文本）的长短期依赖关系 |

## 二、池化 / 下采样 / 上采样
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| MAX_POOL_2D/AVG_POOL_2D | `pooling.cc`（共用：`pooling_common.cc`） | 最大/平均池化：最大池化保留特征图显著特征，平均池化平滑特征，均用于下采样、降低特征图维度 |
| L2_POOL_2D | `l2_pool_2d.cc` | L2 池化，对池化窗口内元素计算 L2 范数，相比平均池化更突出高值特征，用于高精度特征提取 |
| RESIZE_BILINEAR | `resize_bilinear.cc` | 双线性插值缩放，通过加权平均计算插值点像素，平滑调整图像/特征图尺寸，分类模型输入归一化核心算子 |

## 三、数学 / 算术 / 比较类
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| ADD | `add.cc` | 逐元素加法，支持两张量元素级相加，用于特征融合、ResNet 残差连接 |
| ADD_N | `add_n.cc` | 多张量逐元素求和（≥2个张量），适用于多分支特征融合场景 |
| SUB | `sub.cc` | 逐元素减法，计算张量元素差值，用于特征差分、梯度计算 |
| MUL | `mul.cc`（共用：`mul_common.cc`） | 逐元素乘法，实现特征加权、注意力权重应用 |
| DIV | `div.cc` | 逐元素除法，用于数值归一化、比例计算 |
| NEG | `neg.cc` | 逐元素取反，实现符号翻转、反向计算 |
| SQUARED_DIFFERENCE | `squared_difference.cc` | 逐元素平方差，计算两张量差值的平方，用于 MSE 损失计算、相似度评估 |
| MAXIMUM/MINIMUM | `maximum_minimum.cc` | 逐元素取最大/最小值：MAXIMUM 保留特征峰值，MINIMUM 过滤低值噪声，用于特征筛选 |
| LOGICAL_AND/OR/NOT | `logical.cc`（共用：`logical_common.cc`） | 按位逻辑运算，处理布尔型张量，用于条件判断、掩码生成 |
| ROUND | `round.cc` | 逐元素四舍五入，将浮点张量转为近似整数，用于数值量化、坐标取整 |
| FLOOR | `floor.cc` | 逐元素向下取整，保留不大于元素值的最大整数，用于整数运算、索引计算 |
| CEIL | `ceil.cc` | 逐元素向上取整，保留不小于元素值的最小整数，用于数值量化、尺寸向上对齐 |
| FLOOR_DIV | `floor_div.cc` | 向下取整除法，结果向下取整，适用于整数索引计算、维度拆分 |
| FLOOR_MOD | `floor_mod.cc` | 向下取整取模，计算除法余数（符号与除数一致），用于循环索引、周期计算 |
| EXP | `exp.cc` | 逐元素指数运算（e^x），用于 Softmax 中间计算、Exp 激活函数 |

## 四、激活函数
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| RELU / LeakyReLU | `leaky_relu.cc`（共用：`leaky_relu_common.cc`） | RELU（负值置0）；LeakyReLU 对负值保留小斜率以缓解“死亡 ReLU”问题 |
| SIGMOID/LOGISTIC | `logistic.cc`（共用：`logistic_common.cc`） | Sigmoid 激活，将输出映射到 [0,1]，用于二分类、门控机制（如 LSTM） |
| TANH | `tanh.cc` | Tanh 激活，将输出映射到 [-1,1]，常用于时序模型（LSTM/GRU）输出层 |
| ELU | `elu.cc` | ELU 激活，负值区域平滑衰减，提升梯度稳定性，减少过拟合 |
| HARD_SWISH | `hard_swish.cc`（共用：`hard_swish_common.cc`） | 硬 Swish 激活，MobileNetV3 核心激活函数，兼顾精度与计算效率 |
| RELU6 | `activations.cc`（共用：`activations_common.cc`） | RELU6 激活，将输出限制在 [0,6]，常用于量化友好的网络（如 MobileNet） |

## 五、形状与张量变换
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| RESHAPE | `reshape.cc`（共用：`reshape_common.cc`） | 调整张量形状（不改变元素顺序），用于特征展平、维度适配 |
| TRANSPOSE | `transpose.cc`（共用：`transpose_common.cc`） | 张量维度转置（如 HWC→CHW），适配不同卷积算子的输入格式 |
| SQUEEZE | `squeeze.cc` | 移除张量中尺寸为1的维度，简化张量结构 |
| EXPAND_DIMS | `expand_dims.cc` | 为张量增加尺寸为1的维度，用于维度对齐、广播运算 |
| PAD | `pad.cc`（共用：`pad_common.cc`） | 张量填充，在指定维度补充数值（默认0），用于卷积边界填充 |
| MIRROR_PAD | `mirror_pad.cc` | 镜像填充，按镜像方式填充张量边界，避免边缘特征丢失，提升卷积精度 |
| SPACE_TO_DEPTH | `space_to_depth.cc` | 空间转深度，将特征图空间维度（宽/高）转为通道维度，扩充通道数，MobileNet 核心算子 |
| DEPTH_TO_SPACE | `depth_to_space.cc` | 深度转空间，将通道维度转回空间维度，实现上采样，用于特征图尺寸恢复 |
| SLICE | `slice.cc` | 张量切片，提取指定区域的子张量，用于特征裁剪、局部特征提取 |
| STRIDED_SLICE | `strided_slice.cc`（共用：`strided_slice_common.cc`） | 带步长的张量切片，支持按步长提取子张量，用于维度降维、特征采样 |
| SPLIT/SPLIT_V | `split.cc`/`split_v.cc` | 张量拆分：SPLIT 按维度均分，SPLIT_V 按指定尺寸拆分，用于多分支网络（如 ResNet/Inception） |
| CONCATENATION | `concatenation.cc` | 张量拼接，将多个张量沿指定维度合并，用于多分支特征融合 |
| PACK | `pack.cc` | 张量打包，将多个低维张量合并为高维张量，用于维度提升 |
| UNPACK | `unpack.cc` | 张量解包，将高维张量拆分为多个低维张量，用于多标签分类输出 |
| BROADCAST_TO | `broadcast_to.cc` | 张量广播，将低维张量扩展为高维张量（维度对齐），用于不同维度张量运算 |
| SHAPE | `shape.cc` | 获取张量形状，返回张量各维度的尺寸，用于动态维度计算 |
| ZEROS_LIKE | `zeros_like.cc` | 生成与输入张量形状相同的全零张量，用于初始化、掩码生成 |

## 六、量化 / 反量化 / 类型转换
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| QUANTIZE | `quantize.cc`（共用：`quantize_common.cc`） | 张量量化，将浮点张量转为整型量化张量，降低存储/计算开销，适配嵌入式设备 |
| DEQUANTIZE | `dequantize.cc`（共用：`dequantize_common.cc`） | 张量反量化，将整型量化张量转回浮点张量，用于量化模型的输出还原 |
| CAST | `cast.cc` | 张量类型转换（如 float→int、int→bool），适配不同算子的输入类型要求 |

## 七、激活/归一化类（其他）
| 算子名称 | 对应文件 | 核心功能注释 |
|----------|----------|--------------|
| SOFTMAX | `softmax.cc`（共用：`softmax_common.cc`） | Softmax 激活，将输出映射为概率分布（和为1），用于多分类任务输出层 |
| REDUCE_SUM/MEAN/MAX/MIN | `reduce.cc`（共用：`reduce_common.cc`） | 张量汇约运算：沿指定维度求和/均值/最大/最小值，用于全局特征提取（如全局平均池化） |
| L2_NORMALIZE | `l2norm.cc` | L2 归一化，将张量沿指定维度做 L2 范数归一化，提升特征鲁棒性，用于相似度计算 |

## 八、其它工具 / 实用算子 / 工具文件
| 文件名称 | 核心功能注释 |
|----------|--------------|
| `kernel_util.cc`/`kernel_util.h` | 算子通用工具函数，包含张量维度检查、内存计算、数据拷贝等基础功能 |
| `micro_ops.h` | 算子注册声明文件，汇总所有 TFLM 算子的注册接口 |
| `decompress.h` | 解压相关头文件，定义压缩张量的解压接口（具体实现可能位于 `unused/` 子目录） |

## 九、位于 `unused/` 子目录（非顶层/可能未启用）
| 文件名 | 对应算子/功能 | 核心功能注释 |
|--------|---------------|--------------|
| `arg_min_max.cc` | ArgMin/ArgMax | 查找张量指定维度的最小/最大值索引，用于分类任务取预测类别 |
| `assign_variable.cc` | ASSIGN_VARIABLE | 变量赋值算子，用于模型内变量的更新与状态保持 |
| `batch_to_space_nd.cc` | BATCH_TO_SPACE_ND | 批量维度转空间维度，用于嵌入式设备的批次拆分与尺寸恢复 |
| `broadcast_args.cc` | BROADCAST_ARGS | 计算张量广播的目标形状，辅助 BROADCAST_TO 算子实现 |
| `call_once.cc` | CALL_ONCE | 单次执行算子，用于模型初始化、一次性配置（如权重加载） |
| `circular_buffer.cc`/`circular_buffer_common.cc` | CircularBuffer | 循环缓冲算子，用于时序数据（如语音）的缓存与滑动窗口处理 |
| `comparisons.cc` | Greater/Less/Equal 等 | 张量比较运算，返回布尔型结果，用于条件判断、阈值筛选 |
| `cumsum.cc` | CUMSUM | 累积求和（前缀和），用于时序数据累加、积分计算 |
| `detection_postprocess.cc` | DetectionPostProcess | 目标检测后处理算子，解析检测模型输出（边界框、类别、置信度） |
