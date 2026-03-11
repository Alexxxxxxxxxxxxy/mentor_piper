# Piper 机械臂真实世界训练完整指南

---

## 📋 目录
1. [快速开始（5步完成）](#快速开始5步完成)
2. [环境准备](#环境准备)
3. [硬件连接](#硬件连接)
4. [CAN 接口配置（关键！）](#can-接口配置关键)
5. [AprilTag 准备](#apriltag-准备)
6. [相机标定](#相机标定)
7. [手眼标定（关键！）](#手眼标定关键)
8. [目标位置标定](#目标位置标定)
9. [开始训练](#开始训练)
10. [调试模式](#调试模式)
11. [模型保存与恢复](#模型保存与恢复)
12. [常见问题](#常见问题)
13. [故障排除](#故障排除)

---

## 🚀 快速开始（5步完成）

### **如果你已经完成了所有标定：**

```bash
# 1. 配置 CAN 接口
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

# 2. 确认标定文件都在
ls camera_calibration.npz simple_hand_eye.json

# 3. 标定目标位置（可选，如果需要调整）
python calibrate_positions.py

# 4. 测试机械臂连接
python test_piper_connection.py

# 5. 开始训练！
python train_piper.py
```

---

## 🛠️ 环境准备

### 1️⃣ 安装依赖

```bash
# 基础依赖
pip install numpy opencv-python pyyaml

# AprilTag 依赖
pip install pupil-apriltags

# 其他依赖（根据需要）
pip install hydra-core dm-control
```

### 2️⃣ 安装 CAN 工具（Linux）

```bash
# Ubuntu/Debian
sudo apt-get update
sudo apt-get install can-utils

# 验证安装
cansend --version
candump --version
```

### 3️⃣ 检查配置文件

打开 `piper/cfgs/config.yaml`，确认已设置为真实世界模式：

```yaml
# piper specific
use_sim: false              # ✅ 关闭模拟，使用真实机械臂
visualize: true             # ✅ 显示摄像头画面
print_reward: true          # ✅ 打印 reward 信息
debug_mode: false           # ✅ false=训练模式，true=调试模式

# AprilTag 物体追踪设置
use_apriltag: true          # ✅ 启用 AprilTag
tag_size: 0.05              # AprilTag 大小（米），根据实际调整

# 目标位置设置（通过 calibrate_positions.py 标定）
goal_pos: [0.0, 0.75, 0.0]
obj_pos: [0.0, 0.6, 0.0]
```

---

## 🔌 硬件连接

### 1️⃣ 连接 Piper 机械臂

#### **硬件连接步骤：**

1. **电源连接**
   - 将机械臂电源适配器连接到电源插座
   - 确保机械臂电源指示灯亮起

2. **CAN 接口连接**
   - 使用 USB 转 CAN 模块连接机械臂到电脑
   - 确认驱动已正确安装
   - 检查设备是否被识别：`ls /dev/tty*` 或 `dmesg | grep -i usb`

3. **测试连接**

```bash
# 测试机械臂连接
python test_piper_connection.py
```

#### **预期输出：**
```
✓ 机械臂连接成功
机械臂模式：CAN_CTRL(0x1)
```

#### **如果连接失败：**
```bash
# 检查 USB 设备
lsusb

# 检查系统日志
dmesg | tail -20

# 检查权限
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0  # 如果需要
```

### 2️⃣ 连接 RealSense 摄像头

#### **硬件连接步骤：**

1. **USB 连接**
   - 使用 USB 3.0 线连接 RealSense 摄像头到电脑
   - 确保连接到 USB 3.0 端口（蓝色接口）

2. **固定摄像头位置**
   - 将摄像头固定在机械臂正前方
   - 建议高度：与机械臂工作台平行
   - 建议距离：距离工作台约 30-50cm
   - 确保摄像头视野覆盖整个工作区域

3. **测试摄像头**

```bash
# 测试摄像头连接
python test_piper_camera.py
```

#### **预期输出：**
```
✓ 摄像头初始化成功
✓ 相机标定文件已加载
✓ AprilTag 检测器初始化成功
```

#### **如果摄像头无法初始化：**
```bash
# 检查 RealSense 设备
rs-enumerate-devices

# 检查 USB 权限
ls -l /dev/video*

# 安装 RealSense 驱动（如果需要）
sudo apt-get install librealsense2-utils librealsense2-dev
```

---

## 🔧 CAN 接口配置（关键！）

### **为什么需要配置 CAN 接口？**

Piper 机械臂通过 CAN 总线与电脑通信，必须正确配置 CAN 接口才能控制机械臂。

### **CAN 接口配置步骤**

#### **步骤 1：加载 CAN 内核模块**

```bash
# 加载 CAN 模块
sudo modprobe can

# 加载 CAN raw 模块
sudo modprobe can_raw

# 验证模块已加载
lsmod | grep can
```

#### **预期输出：**
```
can                  65536  1 can_raw
can_raw               49152  0
```

#### **步骤 2：配置 CAN 接口**

```bash
# 设置 CAN 接口类型和比特率
sudo ip link set can0 type can bitrate 500000

# 启动 CAN 接口
sudo ip link set up can0

# 查看 CAN 接口状态
ip link show can0
```

#### **预期输出：**
```
3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UNKNOWN mode DEFAULT group default qlen 10
    link/can
```

#### **步骤 3：验证 CAN 接口**

```bash
# 监听 CAN 总线（在另一个终端）
candump can0
```

#### **预期输出：**
```
如果机械臂已连接并通电，应该能看到 CAN 消息：
can0  155#01020304050607
can0  151#01000000320000
...
```

#### **步骤 4：测试 CAN 通信**

```bash
# 发送测试消息
cansend can0 151#01000000320000

# 如果发送成功，没有错误输出
```

### **CAN 接口配置脚本**

为了方便，可以创建一个配置脚本：

```bash
#!/bin/bash
# 文件名: setup_can.sh

echo "配置 CAN 接口..."

# 加载 CAN 模块
sudo modprobe can
sudo modprobe can_raw

# 配置 CAN 接口
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 显示 CAN 接口状态
echo "CAN 接口状态："
ip link show can0

echo "CAN 接口配置完成！"
```

使用方法：
```bash
chmod +x setup_can.sh
./setup_can.sh
```

### **开机自动配置 CAN 接口**

#### **方法 1：使用 systemd 服务**

创建服务文件 `/etc/systemd/system/can0.service`：

```ini
[Unit]
Description=Setup CAN0 interface
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/ip link set can0 type can bitrate 500000
ExecStart=/usr/sbin/ip link set up can0
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

启用服务：
```bash
sudo systemctl daemon-reload
sudo systemctl enable can0.service
sudo systemctl start can0.service
```

#### **方法 2：使用 /etc/network/interfaces**

编辑 `/etc/network/interfaces`：

```bash
auto can0
iface can0 can static
    bitrate 500000
```

重启网络：
```bash
sudo systemctl restart networking
```

### **CAN 接口故障排除**

#### **问题 1：找不到 can0 接口**

```bash
# 检查 CAN 模块是否加载
lsmod | grep can

# 如果没有加载，手动加载
sudo modprobe can
sudo modprobe can_raw

# 检查硬件连接
dmesg | grep -i can
```

#### **问题 2：CAN 接口无法启动**

```bash
# 检查接口状态
ip link show can0

# 尝试重新配置
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 检查系统日志
dmesg | tail -20
```

#### **问题 3：CAN 消息无法接收**

```bash
# 检查机械臂是否通电
# 检查 USB 转 CAN 模块是否连接
lsusb

# 检查 CAN 总线比特率是否匹配
# 机械臂默认比特率：500000 bps

# 尝试不同的比特率
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
```

#### **问题 4：权限问题**

```bash
# 将用户添加到 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录生效
# 或临时修改权限
sudo chmod 666 /dev/ttyUSB0
```

### **CAN 接口调试工具**

#### **1. 监听 CAN 总线**

```bash
# 监听所有 CAN 消息
candump can0

# 监听特定 CAN ID
candump can0 --filter=151:

# 保存到文件
candump can0 -l can_log.txt
```

#### **2. 发送 CAN 消息**

```bash
# 发送单条消息
cansend can0 151#01000000320000

# 发送多条消息
for i in {1..10}; do
    cansend can0 151#01000000320000
    sleep 0.1
done
```

#### **3. 查看 CAN 统计信息**

```bash
# 查看接口统计
ip -s link show can0

# 查看错误计数
candump can0 -n 1
```

---

## 🏷️ AprilTag 准备

### 1️⃣ 打印 AprilTag

#### **下载 AprilTag**

1. 访问：https://april.eecs.umich.edu/software/apriltag
2. 下载 `tag36h11` 家族的标签（推荐 ID: 0）
3. 选择高分辨率版本（300 DPI 或更高）

#### **打印 AprilTag**

- 推荐大小：**5cm × 5cm**
- 纸张：普通 A4 纸即可
- 打印质量：高
- 打印后用尺子实际测量大小

#### **粘贴 AprilTag**

1. 将打印好的 AprilTag 剪下
2. 用双面胶或胶带贴在咖啡杯侧面
3. 确保标签平整，无褶皱
4. 确保标签在摄像头视野内

### 2️⃣ 调整 `tag_size`

用尺子测量打印出的 AprilTag 实际大小，修改 `config.yaml`：

```yaml
tag_size: 0.05  # 如果实际是 5cm，就是 0.05 米
tag_size: 0.04  # 如果实际是 4cm，就是 0.04 米
```

### 3️⃣ 测试 AprilTag 检测

```bash
python april_tag_tracker.py
```

#### **预期输出：**

你应该能看到：
- 摄像头实时画面
- AprilTag 被绿色框标记
- 红色中心点
- 终端显示检测到的位置信息

#### **操作说明：**
- 按 `q` 退出程序
- 按 `s` 保存当前截图

#### **如果检测不到 AprilTag：**

```bash
# 检查光照
# 确保环境光线充足，避免强光直射

# 检查标签
# 确保标签平整，无褶皱
# 确保标签在摄像头视野内

# 检查摄像头对焦
# 调整摄像头焦距，确保画面清晰

# 尝试更大的 AprilTag
# 打印更大的标签（如 10cm × 10cm）
```

---

## 📷 相机标定

### **为什么需要相机标定？**

相机标定用于获取相机的内参（焦距、主点等），这对于准确估计 AprilTag 的 3D 位置至关重要。

### 1️⃣ 打印棋盘格

#### **下载棋盘格**

搜索 "chessboard calibration printable"，或使用以下规格：

- 内角点：9×6（即 10×7 个格子）
- 每个格子大小：约 25mm × 25mm
- 打印质量：高（300 DPI 或更高）

#### **打印棋盘格**

1. 下载或生成棋盘格图像
2. 打印到 A4 纸上
3. 用尺子测量实际格子大小
4. 确保打印平整，无变形

### 2️⃣ 运行相机标定

```bash
python calibrate_camera.py
```

### 3️⃣ 采集图像

#### **选择采集方式：**

程序会显示菜单：
```
请选择操作：
1. 采集新图像
2. 使用已有图像
3. 退出
```

选择 `1` - 采集新图像

#### **采集步骤：**

1. 将棋盘格放在摄像头前
2. 调整棋盘格位置和角度
3. 按 `c` 键采集图像
4. 移动棋盘格到不同位置、不同角度
5. 重复采集 10-20 张图像
6. 按 `q` 键结束采集

#### **采集技巧：**

- 覆盖整个摄像头视野
- 包含不同距离（近、中、远）
- 包含不同角度（倾斜、旋转）
- 确保棋盘格完全在画面内
- 避免模糊和反光

### 4️⃣ 自动生成文件

标定完成后，会自动生成：

- ✅ `camera_calibration.npz` - 相机内参（**自动加载，不用管**）
- `camera_config.yaml` - 配置片段（供参考）

#### **文件内容：**

`camera_calibration.npz` 包含：
- `fx`: x 方向焦距
- `fy`: y 方向焦距
- `cx`: x 方向主点
- `cy`: y 方向主点

### 5️⃣ 验证标定结果

```bash
# 检查文件是否生成
ls -lh camera_calibration.npz

# 查看标定误差（在标定输出中）
# 重投影误差应该 < 1.0 像素
```

#### **如果误差过大：**

```bash
# 重新采集更多图像
# 确保图像质量高
# 确保覆盖不同位置和角度
```

---

## 👁️ 手眼标定（关键！）

### **为什么需要手眼标定？**

```
相机坐标系 → [手眼标定] → 机械臂坐标系
   (x,y,z)                    (x,y,z)
```

手眼标定用于建立相机坐标系和机械臂坐标系之间的转换关系，这对于准确控制机械臂至关重要。

### **使用推荐工具：easy_hand_eye_calibration.py**

#### **步骤：**

```bash
python easy_hand_eye_calibration.py
```

#### **操作流程（超简单！）：**

```
程序会引导你采集 5 个点，每个点：

1. 把贴有 AprilTag 的咖啡杯放在一个位置
   ↓
2. 按示教按钮（灯亮）
   ↓
3. 用手把机械臂末端移到 AprilTag 中心
   ↓
4. 按示教按钮退出
   ↓
5. 按回车确认
   ↓
6. 重复 5 次！

完成后自动生成：simple_hand_eye.json（自动加载，不用管！）
```

#### **推荐的 5 个采集点：**

```
                    点 4 (稍远)
                        ↑
                        │
    点 3 (左侧) ←─────┼─────→ 点 2 (右侧)
                        │
                        ↓
                    点 1 (正前)
                    
                    点 5 (稍近)
```

#### **采集点说明：**

| 点 | 位置 | 说明 |
|----|------|------|
| 点 1 | 正前方 | 距离机械臂约 30cm，正对机械臂 |
| 点 2 | 右侧 | 在机械臂右侧，距离约 30cm |
| 点 3 | 左侧 | 在机械臂左侧，距离约 30cm |
| 点 4 | 稍远 | 在正前方，距离约 40cm |
| 点 5 | 稍近 | 在正前方，距离约 20cm |

#### **注意事项：**

1. **确保 AprilTag 可见**
   - 每个点都要确保 AprilTag 在摄像头视野内
   - 确保光照充足

2. **准确对准**
   - 机械臂末端要准确对准 AprilTag 中心
   - 可以从侧面观察对齐情况

3. **保持稳定**
   - 移动机械臂时要缓慢、平稳
   - 避免碰撞

4. **记录数据**
   - 每个点都要确认记录成功
   - 如果出错，可以重新采集该点

#### **输出文件：**

`simple_hand_eye.json` 包含：
```json
{
  "offset": [x, y, z]
}
```

这个偏移量会自动加载，用于将相机坐标系转换到机械臂坐标系。

---

## 🎯 目标位置标定

### **标定物体位置和目标位置**

目标位置标定用于定义训练任务中的物体初始位置和目标位置。

### **使用工具：calibrate_positions.py**

```bash
python calibrate_positions.py
```

### **两种方式可选：**

| 方式 | 说明 | 适用场景 |
|------|------|---------|
| **1. 示教模式** | 用手移动机械臂到位置 | 推荐使用，更直观 |
| **2. AprilTag 模式** | 把物体放在位置，自动检测 | 快速，但需要 AprilTag |

### **操作流程（示教模式）：**

```
1. 运行脚本
   ↓
2. 选择示教模式
   ↓
3. 标定物体初始位置
   ├─ 把咖啡杯放在初始位置
   ├─ 按示教按钮（灯亮）
   ├─ 用手移动机械臂到物体位置
   ├─ 按示教按钮退出
   └─ 按回车确认
   ↓
4. 标定目标位置
   ├─ 把咖啡杯放在目标位置
   ├─ 按示教按钮（灯亮）
   ├─ 用手移动机械臂到目标位置
   ├─ 按示教按钮退出
   └─ 按回车确认
   ↓
5. 复制输出到 piper/cfgs/config.yaml
```

### **配置文件更新示例：**

```yaml
# 目标位置设置（单位：米）
# X: 左右方向（右为正）
# Y: 前后方向（前为正）
# Z: 上下方向（上为正）

goal_pos: [0.0, 0.75, 0.0]  # 替换为你的标定结果
obj_pos: [0.0, 0.6, 0.0]    # 替换为你的标定结果
```

### **位置说明：**

| 坐标 | 方向 | 正方向 | 说明 |
|------|------|--------|------|
| X | 左右 | 右为正 | 机械臂左右移动 |
| Y | 前后 | 前为正 | 机械臂前后移动 |
| Z | 上下 | 上为正 | 机械臂上下移动 |

### **推荐位置设置：**

```
工作台布局：

    目标位置 (0.0, 0.75, 0.0)
         ↑
         │
         │
         │
    物体初始位置 (0.0, 0.6, 0.0)
         ↑
         │
    机械臂基座 (0.0, 0.0, 0.0)
```

---

## 🧪 测试所有功能

在开始训练前，先测试一下所有功能：

```bash
python test_piper_visualize.py
```

### **预期输出：**

你应该看到：
- ✅ 机械臂连接成功
- ✅ 摄像头画面显示
- ✅ AprilTag 被检测到
- ✅ 物体位置正确显示（通过手眼标定转换）
- ✅ Reward 信息打印
- ✅ 可视化窗口显示

### **可视化窗口内容：**

- Step: 当前步数
- Reward: 当前步 reward
- Episode Reward: 本回合累计 reward
- Obj->Target: 物体到目标距离（绿色=成功范围内）
- Success: 是否成功

### **操作说明：**
- 按 `q` 退出程序
- 按 `s` 保存截图

### **如果测试失败：**

```bash
# 检查 CAN 接口
ip link show can0

# 检查机械臂连接
python test_piper_connection.py

# 检查摄像头连接
python test_piper_camera.py

# 检查 AprilTag 检测
python april_tag_tracker.py
```

---

## 🚀 开始训练！

### 1️⃣ 确认所有标定文件都在

```bash
# 检查这些文件是否存在
ls -lh camera_calibration.npz      # ✅ 相机标定（自动加载）
ls -lh simple_hand_eye.json        # ✅ 手眼标定（自动加载）
```

### 2️⃣ 确认 CAN 接口已配置

```bash
# 检查 CAN 接口状态
ip link show can0

# 如果没有配置，运行：
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

### 3️⃣ 确认配置文件

检查 `piper/cfgs/config.yaml`：

```yaml
# piper specific
use_sim: false          # 使用真实机械臂
visualize: true         # 显示画面
print_reward: true      # 打印 reward
debug_mode: false       # false=训练模式，true=调试模式

# AprilTag 物体追踪设置
use_apriltag: true      # 使用 AprilTag
tag_size: 0.05          # AprilTag 大小（米）

# 目标位置设置
goal_pos: [x, y, z]     # 你标定的目标位置
obj_pos: [x, y, z]      # 你标定的物体位置
```

### 4️⃣ 准备工作区

1. 确保咖啡杯在初始位置
2. 确保 AprilTag 在摄像头视野内
3. 确保周围环境安全
4. 清理工作区域，移除障碍物
5. 准备好紧急停止按钮（如有）

### 5️⃣ 启动训练

```bash
python train_piper.py
```

### 6️⃣ 观察训练

#### **终端输出：**

```
start training
[Step   1] Reward:   0.12 | Episode Reward:   0.12 | Obj->Target: 0.1485m | Success: ❌
[Step   2] Reward:   0.15 | Episode Reward:   0.27 | Obj->Target: 0.1452m | Success: ❌
[Step   3] Reward:   0.20 | Episode Reward:   0.47 | Obj->Target: 0.1389m | Success: ❌
...
[Step  42] Reward:  10.00 | Episode Reward:  45.23 | Obj->Target: 0.0450m | Success: ✅
```

#### **可视化窗口显示：**

- Step: 当前步数
- Reward: 当前步 reward
- Episode Reward: 本回合累计 reward
- Obj->Target: 物体到目标距离（绿色=成功范围内）
- Success: 是否成功

### 7️⃣ 停止训练

按 `Ctrl+C` 停止训练。

---

## 🐛 调试模式

### **什么是调试模式？**

调试模式会打印详细的调试信息，帮助你排查问题。

### **如何启用调试模式？**

编辑 `piper/cfgs/config.yaml`：

```yaml
debug_mode: true  # 启用调试模式
```

### **调试模式输出示例：**

```bash
# 正常模式（debug_mode: false）
[Step   1] Reward:   0.12 | Episode Reward:   0.12 | Obj->Target: 0.1485m | Success: ❌

# 调试模式（debug_mode: true）
[DEBUG] obj_pos: [0.0, 0.6, 0.0], target_pos: [0.0, 0.75, 0.0]
[DEBUG] aprilag_visible: True
[DEBUG] joint_0-5: 0, 0, 0, 0, 0, 0
[DEBUG] gripper_cmd: 0
[DEBUG] camera_pos (tag_pos): [0.1, 0.6, 0.0]
[DEBUG] offset_m: [0.05, 0.0, 0.0]
[DEBUG] robot_pos: [0.15, 0.6, 0.0]
[DEBUG] Arm Status: ArmStatus(...)
[Step   1] Reward:   0.12 | Episode Reward:   0.12 | Obj->Target: 0.1485m | Success: ❌
```

### **调试信息说明：**

| 调试信息 | 说明 |
|---------|------|
| obj_pos | 物体位置（相机坐标系） |
| target_pos | 目标位置（相机坐标系） |
| aprilag_visible | AprilTag 是否可见 |
| joint_0-5 | 6个关节的角度值 |
| gripper_cmd | 夹爪控制命令 |
| camera_pos | 相机检测到的 AprilTag 位置 |
| offset_m | 手眼标定偏移量（米） |
| robot_pos | 转换后的机械臂坐标系位置 |
| Arm Status | 机械臂完整状态信息 |

### **何时使用调试模式？**

- ✅ 机械臂不动时
- ✅ 物体位置不准确时
- ✅ AprilTag 检测不到时
- ✅ Reward 异常时
- ✅ 需要排查问题时

### **何时关闭调试模式？**

- ✅ 正常训练时
- ✅ 减少日志输出时
- ✅ 提高训练速度时

---

## 💾 模型保存与恢复

### **自动保存机制**

训练过程中，模型会自动保存到检查点文件。

### **配置保存参数**

编辑 `piper/cfgs/config.yaml`：

```yaml
# 模型保存设置
save_snapshot: true      # 是否保存模型检查点
save_interval: 10000     # 每隔多少步保存一次
load_from_id: false      # 是否从指定ID加载
load_id: 0               # 要加载的检查点ID
snapshot_path: ''        # 自定义检查点路径
```

### **保存内容**

每次保存会包含：
- **agent**: 训练好的智能体（包含 actor、critic 等网络）
- **timer**: 训练计时器
- **_global_step**: 全局步数
- **_global_episode**: 全局回合数

### **保存位置**

```
工作目录/
├── snapshot.pt              # 最新检查点
├── snapshots/              # 历史检查点目录
│   ├── snapshot_10000.pt   # 第 10000 步的检查点
│   ├── snapshot_20000.pt   # 第 20000 步的检查点
│   └── ...
└── ...
```

### **加载检查点**

#### **方法 1：自动加载最新检查点**

```bash
python train_piper.py
```

程序会自动加载 `snapshot.pt`（最新检查点）。

#### **方法 2：从指定 ID 加载**

编辑 `piper/cfgs/config.yaml`：

```yaml
load_from_id: true
load_id: 10000  # 加载 snapshot_10000.pt
```

然后运行：
```bash
python train_piper.py
```

#### **方法 3：从自定义路径加载**

编辑 `piper/cfgs/config.yaml`：

```yaml
snapshot_path: '/path/to/snapshot.pt'
```

然后运行：
```bash
python train_piper.py
```

### **仅评估模式**

如果你想评估已训练的模型而不继续训练：

编辑 `piper/cfgs/config.yaml`：

```yaml
eval_only: true
```

然后运行：
```bash
python train_piper.py
```

### **检查点管理**

```bash
# 查看所有检查点
ls -lh snapshot.pt snapshots/

# 删除旧检查点（节省空间）
rm snapshots/snapshot_10000.pt

# 备份重要检查点
cp snapshot.pt snapshot_backup.pt
```

---

## 📊 训练输出

### **终端输出示例**

```
start training
[Step   1] Reward:   0.12 | Episode Reward:   0.12 | Obj->Target: 0.1485m | Success: ❌
[Step   2] Reward:   0.15 | Episode Reward:   0.27 | Obj->Target: 0.1452m | Success: ❌
[Step   3] Reward:   0.20 | Episode Reward:   0.47 | Obj->Target: 0.1389m | Success: ❌
[Step   4] Reward:   0.25 | Episode Reward:   0.72 | Obj->Target: 0.1321m | Success: ❌
...
[Step  42] Reward:  10.00 | Episode Reward:  45.23 | Obj->Target: 0.0450m | Success: ✅
```

### **输出字段说明**

| 字段 | 说明 |
|------|------|
| Step | 当前步数 |
| Reward | 当前步的 reward |
| Episode Reward | 本回合累计 reward |
| Obj->Target | 物体到目标的距离（米） |
| Success | 是否成功（✅=成功，❌=失败） |

### **可视化窗口显示**

- Step: 当前步数
- Reward: 当前步 reward
- Episode Reward: 本回合累计 reward
- Obj->Target: 物体到目标距离（绿色=成功范围内）
- Success: 是否成功

---

## 📁 标定文件总结

| 文件 | 是否自动加载 | 需要手动设置吗？ | 说明 |
|------|-------------|----------------|------|
| `camera_calibration.npz` | ✅ 自动 | ❌ 不需要 | 相机内参 |
| `simple_hand_eye.json` | ✅ 自动 | ❌ 不需要 | 手眼标定偏移量 |
| `config.yaml` 中的 goal_pos | ❌ 手动 | ⚠️ 需要标定 | 目标位置 |
| `config.yaml` 中的 obj_pos | ❌ 手动 | ⚠️ 需要标定 | 物体初始位置 |

---

## ❓ 常见问题

### Q1: 标定好了之后，还用手动设置参数吗？

**A:** 大部分不用！
- ✅ `camera_calibration.npz` - 自动加载
- ✅ `simple_hand_eye.json` - 自动加载
- ⚠️ `goal_pos` 和 `obj_pos` - 需要用 `calibrate_positions.py` 标定

### Q2: AprilTag 检测不到怎么办？

**A:** 检查以下几点：
1. 光照是否充足
2. AprilTag 是否平整
3. AprilTag 是否在摄像头视野内
4. 摄像头对焦是否清晰
5. 尝试更大的 AprilTag

### Q3: 物体位置不准确怎么办？

**A:** 
1. 重新运行相机标定
2. 重新运行手眼标定，采集更多点
3. 检查 `tag_size` 设置是否正确
4. 确认 AprilTag 粘贴位置正确

### Q4: 如何判断训练是否成功？

**A:** 观察：
1. Reward 是否逐渐增加
2. Obj->Target 距离是否逐渐减小
3. Success 标记是否频繁出现 ✅
4. 机械臂是否能稳定推动物体到目标

### Q5: 训练太慢怎么办？

**A:** 
1. 可以关闭 `visualize` 来加速
2. 可以关闭 `print_reward` 来加速
3. 调整 `action_repeat` 参数

### Q6: CAN 接口配置失败怎么办？

**A:** 
1. 检查 CAN 模块是否加载：`lsmod | grep can`
2. 检查 USB 转 CAN 模块是否连接：`lsusb`
3. 检查 CAN 接口状态：`ip link show can0`
4. 尝试重新配置 CAN 接口
5. 检查机械臂是否通电

### Q7: 机械臂连接失败怎么办？

**A:** 
1. 检查 CAN 接口是否正确配置
2. 检查 USB 线是否插好
3. 检查机械臂是否上电
4. 运行 `test_piper_connection.py` 测试
5. 检查系统日志：`dmesg | tail -20`

### Q8: 摄像头无法初始化怎么办？

**A:** 
1. 检查 USB 线是否插好
2. 检查 RealSense 驱动是否安装
3. 运行 `test_piper_camera.py` 测试
4. 检查设备列表：`rs-enumerate-devices`
5. 尝试重新插拔摄像头

---

## 🔧 故障排除

### **问题：CAN 接口配置失败**

#### **症状：**
```
Error: cannot find device "can0"
```

#### **检查步骤：**

```bash
# 1. 检查 CAN 模块是否加载
lsmod | grep can

# 2. 如果没有加载，手动加载
sudo modprobe can
sudo modprobe can_raw

# 3. 检查硬件连接
lsusb
dmesg | grep -i can

# 4. 重新配置 CAN 接口
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 5. 验证 CAN 接口
ip link show can0
```

#### **解决方案：**

1. **加载 CAN 模块**
```bash
sudo modprobe can
sudo modprobe can_raw
```

2. **配置 CAN 接口**
```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

3. **开机自动加载**
```bash
# 编辑 /etc/modules
echo "can" | sudo tee -a /etc/modules
echo "can_raw" | sudo tee -a /etc/modules
```

---

### **问题：机械臂连接失败**

#### **症状：**
```
警告：无法连接机械臂：...，将使用模拟模式
```

#### **检查步骤：**

```bash
# 1. 检查 CAN 接口
ip link show can0

# 2. 检查 USB 设备
lsusb

# 3. 检查机械臂是否通电
# 确认机械臂电源指示灯亮起

# 4. 运行测试脚本
python test_piper_connection.py

# 5. 检查系统日志
dmesg | tail -20
```

#### **解决方案：**

1. **配置 CAN 接口**
```bash
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

2. **检查 USB 权限**
```bash
# 将用户添加到 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录生效
```

3. **重新插拔 USB**
```bash
# 拔掉 USB 线，等待 5 秒，重新插入
```

---

### **问题：摄像头无法初始化**

#### **症状：**
```
警告：Camera_Module 未找到，相机功能不可用
```

#### **检查步骤：**

```bash
# 1. 检查 USB 设备
lsusb

# 2. 检查 RealSense 设备
rs-enumerate-devices

# 3. 运行测试脚本
python test_piper_camera.py

# 4. 检查 USB 权限
ls -l /dev/video*
```

#### **解决方案：**

1. **安装 RealSense 驱动**
```bash
# Ubuntu/Debian
sudo apt-get install librealsense2-utils librealsense2-dev

# 验证安装
realsense-viewer
```

2. **检查 USB 连接**
```bash
# 确保连接到 USB 3.0 端口（蓝色接口）
# 重新插拔摄像头
```

3. **检查权限**
```bash
# 添加用户到 video 组
sudo usermod -a -G video $USER

# 重新登录生效
```

---

### **问题：AprilTag 初始化失败**

#### **症状：**
```
警告：pupil-apriltags 未安装，AprilTag 功能不可用
```

#### **检查步骤：**

```bash
# 1. 检查是否安装
pip list | grep apriltag

# 2. 检查相机标定文件
ls -lh camera_calibration.npz

# 3. 检查手眼标定文件
ls -lh simple_hand_eye.json
```

#### **解决方案：**

1. **安装 AprilTag**
```bash
pip install pupil-apriltags
```

2. **运行相机标定**
```bash
python calibrate_camera.py
```

3. **运行手眼标定**
```bash
python easy_hand_eye_calibration.py
```

---

### **问题：手眼标定后物体位置还是不对**

#### **症状：**
```
物体到目标距离显示不准确
```

#### **检查步骤：**

```bash
# 1. 检查 tag_size 设置
grep tag_size piper/cfgs/config.yaml

# 2. 检查手眼标定文件
cat simple_hand_eye.json

# 3. 测试 AprilTag 检测
python april_tag_tracker.py
```

#### **解决方案：**

1. **重新运行手眼标定**
```bash
python easy_hand_eye_calibration.py
```

2. **采集更多点**
```
采集 5-10 个点，而不是 5 个
```

3. **确保准确对准**
```
机械臂末端要准确对准 AprilTag 中心
可以从侧面观察对齐情况
```

4. **检查 tag_size**
```bash
# 用尺子测量 AprilTag 实际大小
# 更新 config.yaml
tag_size: 0.05  # 如果实际是 5cm
```

---

### **问题：机械臂不动**

#### **症状：**
```
机械臂状态显示正常，但实际不动
```

#### **检查步骤：**

```bash
# 1. 检查 CAN 接口
ip link show can0

# 2. 检查机械臂状态
python test_piper_connection.py

# 3. 启用调试模式
# 编辑 config.yaml
debug_mode: true
```

#### **解决方案：**

1. **检查控制模式**
```
确保机械臂处于 CAN 命令控制模式
```

2. **检查电机使能**
```
确保所有电机已使能
```

3. **检查关节角度限制**
```
确保关节角度在安全范围内
```

4. **清除错误**
```bash
# 重新运行训练程序
# 程序会自动清除错误
```

---

## 📚 所有工具文件

| 文件 | 用途 | 是否必须 |
|------|------|---------|
| `easy_hand_eye_calibration.py` | ✅ 手眼标定（推荐） | ⭐⭐⭐⭐⭐ |
| `calibrate_positions.py` | ✅ 目标位置标定 | ⭐⭐⭐⭐⭐ |
| `calibrate_camera.py` | 相机标定 | ⭐⭐⭐⭐⭐ |
| `april_tag_tracker.py` | AprilTag 检测测试 | ⭐⭐⭐ |
| `test_piper_visualize.py` | 完整功能测试 | ⭐⭐⭐⭐ |
| `test_piper_connection.py` | 机械臂连接测试 | ⭐⭐⭐⭐ |
| `test_piper_camera.py` | 摄像头测试 | ⭐⭐⭐⭐ |

---

## 🎉 总结

### **完整流程回顾：**

```
1. 环境准备
   ├─ 安装依赖
   ├─ 安装 CAN 工具
   └─ 检查配置文件
   ↓
2. 硬件连接
   ├─ 连接机械臂
   ├─ 连接摄像头
   └─ 测试连接
   ↓
3. CAN 接口配置（关键！）
   ├─ 加载 CAN 模块
   ├─ 配置 CAN 接口
   └─ 验证 CAN 通信
   ↓
4. AprilTag 准备
   ├─ 打印 AprilTag
   ├─ 调整 tag_size
   └─ 测试检测
   ↓
5. 相机标定
   ├─ 打印棋盘格
   ├─ 采集图像
   └─ 生成 camera_calibration.npz
   ↓
6. 手眼标定
   ├─ 采集 5 个点
   └─ 生成 simple_hand_eye.json
   ↓
7. 目标位置标定
   ├─ 标定物体位置
   ├─ 标定目标位置
   └─ 更新 config.yaml
   ↓
8. 测试所有功能
   └─ 运行 test_piper_visualize.py
   ↓
9. 开始训练！
   └─ 运行 train_piper.py
```

### **你需要手动操作的：**
- ✅ 配置 CAN 接口（每次开机）
- ✅ 标定目标位置和物体位置（`calibrate_positions.py`）
- ✅ 把结果复制到 `config.yaml`

### **自动加载，不用管的：**
- ✅ `camera_calibration.npz` - 相机标定
- ✅ `simple_hand_eye.json` - 手眼标定

### **快速检查清单：**

```bash
# 1. CAN 接口配置
ip link show can0

# 2. 标定文件检查
ls -lh camera_calibration.npz simple_hand_eye.json

# 3. 配置文件检查
cat piper/cfgs/config.yaml | grep -E "use_sim|use_apriltag|goal_pos|obj_pos"

# 4. 机械臂测试
python test_piper_connection.py

# 5. 摄像头测试
python test_piper_camera.py

# 6. 完整测试
python test_piper_visualize.py
```

---

## 🎯 祝你训练顺利！

### **遇到问题时，按顺序检查：**

1. ✅ 硬件连接
2. ✅ CAN 接口配置
3. ✅ 标定文件是否存在
4. ✅ 配置文件是否正确
5. ✅ 运行测试脚本验证
6. ✅ 启用调试模式排查

### **有用的命令：**

```bash
# CAN 接口配置
sudo modprobe can
sudo modprobe can_raw
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0

# 检查 CAN 接口
ip link show can0

# 监听 CAN 总线
candump can0

# 测试脚本
python test_piper_connection.py
python test_piper_camera.py
python test_piper_visualize.py

# 标定脚本
python calibrate_camera.py
python easy_hand_eye_calibration.py
python calibrate_positions.py

# 训练
python train_piper.py
```

---

**最后更新：2026-03-11**
**版本：v2.0**
