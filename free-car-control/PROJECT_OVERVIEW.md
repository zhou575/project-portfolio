# Free Car Control 项目概览

## 项目简介

这是一个面向嵌入式智能车的视觉控制项目。项目从摄像头图像中提取赛道边界和误差信息，再结合速度决策、电机编码器反馈和陀螺仪角速度，计算左右轮目标速度，完成循迹、转向与保护控制。

## 核心内容

- 图像边线搜索、滤波和误差更新
- 基于赛道图像特征的转向误差增益
- 速度决策与匀加速限制
- 左右轮增量式 PID 速度闭环
- 图像差速、角速度补偿和不对称差速分配
- 图像保护与堵转保护
- NCNN、TensorFlow Lite Micro 及其他嵌入式支持库
- 轻量级图像分类模型的数据整理、训练、独立测试集评估，以及 ONNX/NCNN 导出

## 目录说明

| 目录 | 内容 |
| --- | --- |
| `project/code` | 图像处理、控制逻辑、电机与编码器代码 |
| `project/user` | 用户入口、构建脚本和调试记录 |
| `project/model` | 项目使用的模型文件 |
| `model-training` | PyTorch 训练与评估脚本、数据准备工具、模型权重及 ONNX/NCNN 导出 |
| `libraries` | 板级支持、视觉和推理相关依赖 |
| `README.md` | 参数调节、控制逻辑和代码变更说明 |
| `视觉避障方案_Codex实现说明.md` | 视觉避障方案交接说明 |
| `TOF_视觉联合坡道检测_Codex交接说明.md` | TOF 与视觉联合检测交接说明 |

## 版本整理说明

本项目已从原始工程中整理为个人作品集的独立目录，未包含 Git 历史目录和 `project/out` 等编译输出；源码、模型、依赖库和技术说明均保留。

模型训练相关的原始图片集和 Windows `pnnx.exe` 未纳入作品集目录。已保存的评估指标显示验证集准确率为 95.27%，独立测试集准确率为 60.17%；详见 [`model-training/README.md`](./model-training/README.md) 和 [`model-training/artifacts/metrics.json`](./model-training/artifacts/metrics.json)。

