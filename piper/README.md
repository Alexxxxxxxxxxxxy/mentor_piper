# Piper 机械臂模块

本模块用于将 Mentor 框架与 Piper 机械臂集成，支持在真实硬件或模拟环境中训练机械臂完成 Coffee Push 任务。

## 文件说明

- `__init__.py` - 模块初始化
- `env.py` - Piper 环境包装器，与 Mentor 框架兼容
- `robot.py` - Piper 机器人接口，连接 piper_sdk 和相机模块
- `cfgs/config.yaml` - 训练配置文件

## 功能特性

1. **机械臂控制** - 使用 piper_sdk 控制 Piper 机械臂
2. **Gripper 控制** - 支持夹爪开合（7 维动作空间）
3. **AprilTag 物体追踪** - 自动检测带 AprilTag 的物体并计算坐标
4. **手眼标定** - 支持将相机坐标系转换为机械臂坐标系
5. **可视化** - 实时显示摄像头画面和训练进度

## 快速开始

### 1. 环境准备

确保已安装以下依赖：

```bash
# Piper 机械臂 SDK（请从官方获取）
# pip install piper_sdk

# AprilTag 库
pip install pupil-apriltags

# 其他依赖
pip install opencv-python numpy gym dm_env hydra
```

### 2. 手眼标定（真实机械臂必须）

运行手眼标定工具：

```bash
python easy_hand_eye_calibration.py
```

标定完成后，会生成 `simple_hand_eye.json` 文件。

### 3. 配置文件

修改 `piper/cfgs/config.yaml`：

```yaml
# piper specific
use_sim: false          # false 使用真实机械臂，true 使用模拟
visualize: true         # true 显示摄像头画面
print_reward: true      # true 打印 reward 信息

# AprilTag 物体追踪设置
use_apriltag: true      # true 使用 AprilTag 检测物体
tag_size: 0.05          # AprilTag 物理大小（米）

# 目标位置设置（米）
goal_pos: [0.0, 0.75, 0.0]   # 目标位置
obj_pos: [0.0, 0.6, 0.0]      # 物体初始位置
```

### 4. 开始训练

在项目根目录运行：

```bash
python train_piper.py --config-path piper/cfgs --config-name config
```

或者更简单：

```bash
cd piper
python ../train_piper.py
```

## 运行时调试信息

运行时会显示以下调试信息：

```
正在使能机械臂...
✓ 机械臂连接成功并已使能
✓ 加载简单手眼标定: simple_hand_eye.json
  偏移量: [205.32268473 70.99607139 55.43848002]

[DEBUG] camera_pos (tag_pos): [0.074 0.0057 0.8201]
[DEBUG] offset_m: [0.2053 0.0710 0.0554]
[DEBUG] robot_pos: [0.2793 0.0767 0.8755]
[DEBUG] joint_0-5: 0, 0, 0, 0, 0, 0
[DEBUG] gripper_cmd: 0

[Step   1] Reward:  0.00 | Episode Reward:  0.00 | Obj->Target: 0.1500m | Success: ❌
```

## 修复的问题

1. **关节位置单位转换** - 修复了 `get_joint_pos` 方法没有除以 factor 的问题
2. **机械臂复位逻辑** - 修复了 `reset` 方法强制机械臂归零的问题
3. **目标位置随机化** - 修复了真实模式下目标位置被随机覆盖的问题
4. **Gripper 动作映射** - 修复了 gripper 输入值从 [-1, 1] 到 [0, 1] 的映射
5. **AprilTag 丢失惩罚** - 添加了 AprilTag 不可见时的持续惩罚
6. **调试打印** - 添加了详细的调试信息以便问题定位

## 坐标系说明

- **相机坐标系** - AprilTag 返回的原始坐标
- **机械臂坐标系** - 转换后的坐标，用于机械臂控制
- **手眼标定** - `robot_pos = tag_pos + offset`（offset 从 simple_hand_eye.json 读取）

## 详细文档

请查看 [TRAIN_PIPER_GUIDE.md](../TRAIN_PIPER_GUIDE.md) 获取完整的训练指南。
