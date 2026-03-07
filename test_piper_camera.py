"""
实时监测版相机和机械臂连接测试
功能：
1. 连接相机和机械臂
2. 实时循环打印机械臂位姿 (按 Ctrl+C 退出)
3. 退出前自动保存最终状态到 YAML
"""

import numpy as np
import pyrealsense2 as rs
import yaml
import sys
import os
import time
import cv2
import signal

# --- 1. 智能查找 SDK 路径 ---
def find_sdk_path():
    possible_paths = [
        os.path.join(os.getcwd(), 'piper_sdk'),
        os.path.join(os.getcwd(), '../piper_sdk'),
        '/home/isee604/piper_mentor/piper_sdk',
        '/home/isee604/mentor-main/piper_sdk',
    ]
    for path in possible_paths:
        if os.path.exists(path) and os.path.isdir(path):
            return path
    try:
        import piper_sdk
        return os.path.dirname(piper_sdk.__file__)
    except ImportError:
        pass
    return None

sdk_path = find_sdk_path()
if sdk_path:
    sys.path.insert(0, sdk_path)
    print(f"✓ 找到 SDK 路径：{sdk_path}")
else:
    print("✗ 错误：未找到 piper_sdk 路径。")
    sys.exit(1)

try:
    from piper_sdk import *
except ImportError as e:
    print(f"✗ 无法导入 piper_sdk: {e}")
    sys.exit(1)

# 配置参数
OUTPUT_DIR = "exp_local/camera_output"
CAN_DEVICE_NAME = "can0"
UPDATE_INTERVAL = 0.2  # 刷新频率 (秒)

def ensure_dir(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)

def test_camera():
    """测试 RealSense 相机 (只初始化一次，不循环)"""
    print("=" * 50)
    print("测试 RealSense 相机...")
    print("=" * 50)
    
    pipeline = None
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
        
        ctx = rs.context()
        devices = ctx.query_devices()
        if not devices:
            print("✗ 未检测到 RealSense 设备")
            return None
            
        device = devices[0]
        serial_number = device.get_info(rs.camera_info.serial_number)
        print(f"✓ 发现设备：{device.get_info(rs.camera_info.name)} (SN: {serial_number})")
        
        config.enable_device(serial_number)
        profile = pipeline.start(config)
        time.sleep(1.0)
        
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        intrinsics = depth_profile.get_intrinsics()
        
        print(f"✓ 相机内参: fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}, cx={intrinsics.ppx:.2f}, cy={intrinsics.ppy:.2f}")
        
        # 获取一帧验证
        frames = pipeline.wait_for_frames(timeout_ms=3000)
        if frames.get_color_frame() and frames.get_depth_frame():
            print("✓ 相机图像流正常")
            # 保存一张测试图
            ensure_dir(OUTPUT_DIR)
            color_image = np.asanyarray(frames.get_color_frame().get_data())
            cv2.imwrite(os.path.join(OUTPUT_DIR, "test_color.jpg"), color_image)
        else:
            print("⚠️ 相机图像流获取失败，但将继续运行")
            
        return pipeline, intrinsics
        
    except Exception as e:
        print(f"✗ 相机测试失败：{e}")
        if pipeline:
            pipeline.stop()
        return None

def run_realtime_monitor(robot, intrinsics, pipeline):
    """实时循环打印位姿，直到 Ctrl+C"""
    print("\n" + "=" * 50)
    print("🔄 开始实时监测机械臂位姿...")
    print("👉 请手动移动机械臂，观察数据变化")
    print("👉 按 Ctrl+C 停止并保存结果")
    print("=" * 50)
    
    last_pose = None
    
    try:
        while True:
            # 获取最新位姿
            pose = robot.GetArmEndPoseMsgs()
            last_pose = pose
            
            # 格式化输出 (使用 \r 实现单行刷新)
            log_msg = (
                f"\r[实时位姿] X: {pose.end_pose.X_axis:7.2f} mm | "
                f"Y: {pose.end_pose.Y_axis:7.2f} mm | "
                f"Z: {pose.end_pose.Z_axis:7.2f} mm | "
                f"RX: {pose.end_pose.RX_axis:6.2f}° | "
                f"RY: {pose.end_pose.RY_axis:6.2f}° | "
                f"RZ: {pose.end_pose.RZ_axis:6.2f}°"
            )
            print(log_msg, end='', flush=True)
            
            time.sleep(UPDATE_INTERVAL)
            
    except KeyboardInterrupt:
        print("\n\n⏹️  用户中断 (Ctrl+C)，正在停止监测...")
        
        if last_pose:
            save_final_results(last_pose, intrinsics)
        return True # 表示正常退出
    
    return False

def save_final_results(pose, intrinsics):
    """保存最终结果"""
    print("\n💾 正在保存最终数据...")
    ensure_dir(OUTPUT_DIR)
    save_path = os.path.join(OUTPUT_DIR, "final_result.yaml")
    
    result = {
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        "robot_end_effector": {
            "position_mm": {
                "x": float(pose.end_pose.X_axis),
                "y": float(pose.end_pose.Y_axis),
                "z": float(pose.end_pose.Z_axis)
            },
            "rotation_deg": {
                "rx": float(pose.end_pose.RX_axis),
                "ry": float(pose.end_pose.RY_axis),
                "rz": float(pose.end_pose.RZ_axis)
            }
        },
        "camera_intrinsics": {
            "fx": float(intrinsics.fx),
            "fy": float(intrinsics.fy),
            "cx": float(intrinsics.ppx),
            "cy": float(intrinsics.ppy),
            "width": int(intrinsics.width),
            "height": int(intrinsics.height)
        }
    }
    
    with open(save_path, 'w', encoding='utf-8') as f:
        yaml.dump(result, f, default_flow_style=False, allow_unicode=True)
    
    print(f"✓ 最终数据已保存至：{save_path}")
    print(f"   最终位置: X={pose.end_pose.X_axis:.2f}, Y={pose.end_pose.Y_axis:.2f}, Z={pose.end_pose.Z_axis:.2f}")

def main():
    print("\n🚀 启动实时监测系统")
    
    camera_data = None
    robot = None
    
    try:
        # 1. 初始化相机
        camera_data = test_camera()
        if not camera_data:
            print("⚠️ 相机初始化失败，将仅测试机械臂。")
        
        # 2. 初始化机械臂
        print("\n" + "=" * 50)
        print("连接 Piper 机械臂...")
        print("=" * 50)
        
        robot = C_PiperInterface(
            can_name=CAN_DEVICE_NAME,
            judge_flag=True,
            can_auto_init=False,
            dh_is_offset=1,
            start_sdk_joint_limit=False,
            start_sdk_gripper_limit=False,
            logger_level=LogLevel.WARNING,
            log_to_file=False
        )
        
        # 关键步骤：手动创建 CAN 总线
        robot.CreateCanBus(can_name=CAN_DEVICE_NAME)
        robot.ConnectPort()
        robot.MasterSlaveConfig(0xFC, 0, 0, 0)
        time.sleep(0.5)
        
        # 验证连接
        test_pose = robot.GetArmEndPoseMsgs()
        print(f"✓ 机械臂连接成功！初始位姿: X={test_pose.end_pose.X_axis:.1f}, Y={test_pose.end_pose.Y_axis:.1f}, Z={test_pose.end_pose.Z_axis:.1f}")
        
        # 3. 进入实时循环
        intrinsics = camera_data[1] if camera_data else None
        run_realtime_monitor(robot, intrinsics, camera_data[0] if camera_data else None)
        
    except Exception as e:
        print(f"\n❌ 发生严重错误：{e}")
        
    finally:
        # 4. 清理资源
        print("\n🧹 正在清理资源...")
        if robot:
            try:
                robot.DisconnectPort()
                print("✓ 机械臂端口已关闭")
            except:
                pass
        
        if camera_data and camera_data[0]:
            try:
                camera_data[0].stop()
                print("✓ 相机流水线已停止")
            except:
                pass
        
        print("✅ 程序完全退出。")

if __name__ == "__main__":
    main()