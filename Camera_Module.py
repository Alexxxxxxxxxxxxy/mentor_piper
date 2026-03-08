import pyrealsense2 as rs
import numpy as np
import cv2 
from scipy.spatial.transform import Rotation as R
import open3d as o3d
import matplotlib.colors as mcolors




class DepthCameraModule:
    def __init__(self, color_width=1280, color_height=720, depth_width=640, depth_height=480, fps=30, is_decimate=False):
        self.pipeline = rs.pipeline()
        self.config = rs.config()

    
        try:
            self.config.enable_stream(rs.stream.depth, depth_width, depth_height, rs.format.z16, fps)
            self.config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)
        except RuntimeError as e:
            print(f"配置流时出错: {e}")
            print("请检查你的相机是否支持该分辨率/帧率组合。")
            print("你可以使用 list_supported_modes.py (见之前回复) 脚本查看支持的模式。")
            exit()

        connect_device_serial = None
        # 发现并选择设备
        # 这部分逻辑假设只有一个非 'Platform Camera' 的 RealSense 设备连接
        # 如果你有多个 RealSense 设备，你可能需要更复杂的逻辑来选择正确的设备
        context = rs.context()
        devices = context.query_devices()
        if not devices:
            print("没有检测到 RealSense 设备！")
            exit()

        found_devices_info = []
        for d in devices:
            device_name = d.get_info(rs.camera_info.name)
            serial_number = d.get_info(rs.camera_info.serial_number)
            found_devices_info.append(f"设备: {device_name}, 序列号: {serial_number}")
            if 'platform camera' not in device_name.lower(): # 忽略集成摄像头等
                if connect_device_serial is None:
                    connect_device_serial = serial_number
                else:
                    print('检测到多个 RealSense 设备。此代码目前仅明确支持一个。')
                    # 可以选择退出，或者让用户选择，或者默认选择第一个
                    # exit() # 取消注释则在多于一个设备时退出

        if connect_device_serial is None:
            print('未找到符合条件的 RealSense 设备 (非 Platform Camera)。')
            if found_devices_info:
                print("检测到的所有设备：")
                for info in found_devices_info:
                    print(info)
            exit()
        
        print(f"选择设备序列号: {connect_device_serial}")
        self.config.enable_device(connect_device_serial)

        try:
            self.profile = self.pipeline.start(self.config)
        except RuntimeError as e:
            print(f"启动管线时出错: {e}")
            print("这通常意味着请求的流配置不被选定设备支持，或者设备已被其他程序占用。")
            exit()
        
        self.device = self.profile.get_device()
        self._check_for_rgb_camera(self.device) # 检查RGB相机是否存在
        
        # 获取深度传感器的深度缩放因子
        depth_sensor = self.device.first_depth_sensor()
        if depth_sensor is None:
            print("错误：未找到深度传感器。")
            exit()
        self.depth_scale = depth_sensor.get_depth_scale()
        print(f"设备深度缩放因子 (Depth Scale): {self.depth_scale}")

        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)
        
        self.is_decimate = is_decimate
        if self.is_decimate:
            self.decimation_filter = rs.decimation_filter()
            # 你可以设置抽取滤波器的参数，例如：
            # self.decimation_filter.set_option(rs.option.filter_magnitude, 2) # 2 表示分辨率减半

        # 彩色流的宽度和高度，因为深度图将对齐到彩色图
        self.w = color_width # 对齐后的深度图将匹配彩色图的宽度
        self.h = color_height # 对齐后的深度图将匹配彩色图的高度
        
        # 获取彩色流的内参 (因为深度是对齐到彩色流的)
        color_stream_profile = rs.video_stream_profile(self.profile.get_stream(rs.stream.color))
        self.intrinsics = color_stream_profile.get_intrinsics()

    def _check_for_rgb_camera(self, device):
        found_rgb = False
        for s in device.sensors:
            # D400系列通常有一个名为 "RGB Camera" 的传感器
            # L500系列可能不同，其彩色来源于深度传感器本身，名称可能类似 "L500 Depth Sensor" 但支持彩色流
            if s.supports(rs.camera_info.name): # 确保传感器对象有效
                 sensor_name = s.get_info(rs.camera_info.name)
                 if 'rgb camera' in sensor_name.lower(): # 更通用的检查
                    found_rgb = True
                    print(f"找到RGB相机传感器: {sensor_name}")
                    break
                 # 对于某些相机（如L515），彩色数据可能来自深度传感器
                 # 也可以检查传感器是否支持彩色流格式
                 for profile in s.get_stream_profiles():
                     if profile.stream_type() == rs.stream.color and profile.format() == rs.format.bgr8:
                         found_rgb = True
                         print(f"找到支持彩色流的传感器: {sensor_name}")
                         break
                 if found_rgb:
                     break
        if not found_rgb:
            print("警告：未明确找到名为 'RGB Camera' 的传感器或支持期望彩色流的传感器。")
            print("如果相机型号特殊 (如 L515)，这可能是正常的，只要彩色流能成功获取。")
            # exit(0) # 你可能不想在这里强制退出，除非确定这是一个错误

    def get_image(self):
        try:
            frames = self.pipeline.wait_for_frames(timeout_ms=5000) # 增加超时以防卡住
        except RuntimeError as e:
            print(f"等待帧时出错: {e}")
            return None, None, None

        aligned_frames = self.align.process(frames)
        
        depth_frame_aligned = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame_aligned or not color_frame:
            print("未能获取对齐后的深度帧或彩色帧。")
            return None, None, None
            
        # 如果启用了抽取滤波器，应用它 (通常在对齐之后对深度帧进行)
        if self.is_decimate:
            depth_frame_aligned = self.decimation_filter.process(depth_frame_aligned)

        # 转换为 numpy 数组
        # 原始深度数据 (Z16)
        depth_array_raw = np.asanyarray(depth_frame_aligned.get_data())
        color_array = np.asanyarray(color_frame.get_data())
        
        # 将原始深度数据转换为米
        # 注意：对齐后的深度帧 (depth_frame_aligned) 的尺寸会匹配彩色帧
        depth_in_meters = depth_array_raw * self.depth_scale
        
        # 调试：打印以米为单位的深度值范围
        valid_depths = depth_in_meters[depth_in_meters > 0] # 排除0和无效深度
        # if valid_depths.size > 0:
        #     print(f"对齐后深度范围: {np.min(valid_depths):.3f} 到 {np.max(valid_depths):.3f} 米")
        # else:
        #     print("无有效深度数据！")
  
        frames_cat = np.concatenate([color_array / 255.0, depth_array_raw[..., np.newaxis]], axis=-1)
        
        # 返回：frames_cat, 以米为单位的深度图, 彩色图
        return frames_cat, depth_in_meters, color_array
        
    def get_frame(self):
  
       

        # 1. 获取图像数据
        # 假设 camera.get_image() 返回 (frames, depth_array, color_array)
        frames_result, depth_array, color_array_bgr = self.get_image()
        
        # 检查是否成功获取图像数据
        if frames_result is None or depth_array is None or color_array_bgr is None:
            print("错误：未能从相机获取有效的图像数据")
            print(f"frames_result: {frames_result is not None}")
            print(f"depth_array: {depth_array is not None}")
            print(f"color_array_bgr: {color_array_bgr is not None}")
            # self.stop()
            return None, None
        
        # 检查color_array_bgr是否为空
        if color_array_bgr.size == 0:
            print("错误：color_array_bgr为空数组")
            # self.stop()
            return None, None
        
        # print(f"成功获取图像数据:")
        # print(f"  深度数组形状: {depth_array.shape}")
        # print(f"  彩色数组形状: {color_array_bgr.shape}")
        # print(f"  彩色数组类型: {color_array_bgr.dtype}")
        
        # 将 BGR (uint8) 转换为 RGB (uint8)
        try:
            color_array_rgb = cv2.cvtColor(color_array_bgr, cv2.COLOR_BGR2RGB)
        except cv2.error as e:
            print(f"OpenCV颜色转换错误: {e}")
            print(f"color_array_bgr形状: {color_array_bgr.shape}")
            print(f"color_array_bgr是否为空: {color_array_bgr.size == 0}")
            # self.stop()
            return None, None

        # # # color
        # intrinsic = INTRINSIC_CV
        # fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        # cx, cy = intrinsic[0, 2], intrinsic[1, 2]
        # print(f"使用的内参: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

        # height, width = depth_array.shape
        # u = np.arange(width)
        # v = np.arange(height)
        # u, v = np.meshgrid(u, v)

        # # 反投影计算相机坐标系下的点云
        # z = depth_array.astype(np.float32)  # 确保深度是米单位的浮点数
        # z[z <= 0] = 0.000001 # 无效深度值设为NaN，避免计算错误
        # x = (u - cx) * z / fx
        # y = (v - cy) * z / fy
        
        # # 组合成 (H, W, 3) 的有组织点云
        # points_camera = np.dstack((x, y, z))

        # points_camera_flat = points_camera.reshape(-1, 3)
        # rot = R.from_quat(quaternion)
        # rotation_matrix = rot.as_matrix()
        # points_world_flat = points_camera_flat @ rotation_matrix.T + translation
        # dx = 0
        # dy = 0
        # dz = 0
        # points_world_flat[:,0]-=dx
        # points_world_flat[:,1]-=dy
        # points_world_flat[:,2]-=dz

        # # Reshape and normalize colors for Open3D visualization
        # colors_flat = color_array_rgb.reshape(-1, 3) / 255.0

        # print('points_world_flat.shape:', points_world_flat.shape)
        # print('type of points_world_flat:', type(points_world_flat))


        
        # self.visualize_point_cloud_(points_world_flat, colors_flat)


        # 停止相机以释放资源
        # self.stop()
        
        # 返回RGB图像、有组织的世界坐标系点云、深度数组和旋转矩阵
        return color_array_rgb, depth_array
    
    def stop(self):
        """停止相机并释放资源"""
        try:
            if hasattr(self, 'pipeline') and self.pipeline is not None:
                self.pipeline.stop()
                print("✓ 相机已停止")
        except Exception as e:
            print(f"⚠️  停止相机时出错: {e}")

    def visualize_point_cloud_(self,points, colors, frame_to_visualize=None):
        """
        使用Open3D可视化点云并允许选择点
        """
        
        # --- 修正 1: 重塑 (Reshape) 和归一化 (Normalize) 颜色 ---
        # 'colors' 传入时是 'rgb'，形状 (H, W, 3) 且类型为 uint8 (0-255)
        # 'points' 传入时是 'point_cloud'，形状 (H*W, 3)
        
        # 1. 将 'colors' 展平 (reshape)，使其形状与 'points' 匹配
        colors_flat = colors.reshape(-1, 3)

        # 2. 将 'colors' (uint8, 0-255) 归一化到 (float, 0-1)
        #    Open3D pcd.colors 期望 float 类型 [0, 1]
        colors_flat_normalized = colors_flat.astype(np.float64) / 255.0
        # --- 修正 1 结束 ---


        # --- 修正 2: 过滤点和颜色时保持一致 ---
        # 过滤掉距离原点太远的点 (这一步最好在创建 pcd 对象之前完成)
        in_box = (points[:, 0] > -3) & (points[:, 0] < 3) & \
                 (points[:, 1] > -3) & (points[:, 1] < 3) & \
                 (points[:, 2] > -3) & (points[:, 2] < 3)
        
        # 应用掩码
        points_filtered = points[in_box]
        colors_filtered = colors_flat_normalized[in_box]
        # --- 修正 2 结束 ---


        # 创建Open3D点云对象
        pcd = o3d.geometry.PointCloud()

        # 使用过滤和格式化后的数据
        pcd.points = o3d.utility.Vector3dVector(points_filtered)
        pcd.colors = o3d.utility.Vector3dVector(colors_filtered)

        # 打印点云的边界框
        print("点云边界框:", pcd.get_axis_aligned_bounding_box())

        # 创建可视化窗口
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        vis.add_geometry(pcd)
        
        # 添加世界坐标系 (位于 0,0,0)
        world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        vis.add_geometry(world_frame)

        if frame_to_visualize is not None:
            # 创建一个新的坐标系模型
            new_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
            # 应用变换
            new_frame.transform(frame_to_visualize)
            vis.add_geometry(new_frame)
       
        # # 运行可视化（shift+左键选点，Q退出）
        # print("可视化窗口已打开。按 'shift + 鼠标左键' 选择点，完成后按 'Q' 关闭窗口。")
        vis.run()
        vis.destroy_window()

        # vis = o3d.visualization.VisualizerWithEditing()
        # vis.create_window()
        # vis.add_geometry(pcd)
        # # 添加世界坐标系 (位于 0,0,0)
        # world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
        # vis.add_geometry(world_frame)

        # if frame_to_visualize is not None:
        #     # 创建一个新的坐标系模型
        #     new_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        #     # 应用变换
        #     new_frame.transform(frame_to_visualize)
        #     vis.add_geometry(new_frame)

        # vis.run()

        # # 获取选中的点索引
        # selected_indices = vis.get_picked_points()
        
        # vis.destroy_window()

        # if selected_indices:
        #     selected_points = np.asarray(pcd.points)[selected_indices]
        #     # 打印选择的点
        #     print("选择的点索引:", selected_indices)
        #     print("选择的点坐标:", selected_points)
        # else:
        #     print("没有选择任何点。")
   

    def downsample_point_cloud_(self, point_cloud, colors=None, num_points=4096, voxel_size=0.005):
        """
        对点云进行下采样，支持边界框过滤和最远点采样 (FPS)。

        Args:
            point_cloud (np.ndarray): 形状为 (N, 3) 的原始点云。
            colors (np.ndarray, optional): 形状为 (N, 3) 的点云颜色，将与点云同步下采样。
                                            如果为 None，则只返回下采样后的点云。
            num_points (int): 目标下采样点数。
            voxel_size (float): 用于Voxel下采样的体素大小（米）。
                                  在进行FPS之前，通常先用Voxel下采样减少点数。

        Returns:
            tuple: (下采样后的点云, 下采样后的颜色 (如果提供))
                   如果点云数量不足 num_points，则返回原始点云（经过边界框过滤）
                   或者填充零以达到 num_points。
        """
        # --- 1. 边界框过滤 ---
        print(f"原始点云点数: {point_cloud.shape[0]}")
        xmin = 0.10
        xmax = 0.70
        ymin = -0.5
        ymax = 0.5
        # zmin = -0.15
        zmin = -0.11
        zmax = 0.30
        in_box = (point_cloud[:, 0] > xmin) & (point_cloud[:, 0] < xmax) & \
                 (point_cloud[:, 1] > ymin) & (point_cloud[:, 1] < ymax) & \
                 (point_cloud[:, 2] > zmin) & (point_cloud[:, 2] < zmax)
        
        point_cloud_filtered = point_cloud[in_box]
        if colors is not None:
            colors_filtered = colors[in_box]
        else:
            colors_filtered = None

        print(f"边界框过滤后点数: {point_cloud_filtered.shape[0]}")

        # 如果过滤后点云为空，或者点数太少，直接返回处理
        if point_cloud_filtered.shape[0] == 0:
            print("警告: 边界框过滤后点云为空。")
            if colors is not None:
                return np.zeros((num_points, 3)), np.zeros((num_points, 3)) # 返回填充的空点云
            return np.zeros((num_points, 3))

        if point_cloud_filtered.shape[0] <= num_points:
            print(f"边界框过滤后点数 ({point_cloud_filtered.shape[0]}) 小于或等于目标点数 ({num_points})，不进行进一步下采样。")
            
            # 如果需要填充以达到num_points
            if point_cloud_filtered.shape[0] < num_points:
                pad_count = num_points - point_cloud_filtered.shape[0]
                point_cloud_filtered = np.vstack([point_cloud_filtered, np.zeros((pad_count, 3), dtype=point_cloud_filtered.dtype)])
                if colors_filtered is not None:
                    colors_filtered = np.vstack([colors_filtered, np.zeros((pad_count, 3), dtype=colors_filtered.dtype)])
                print(f"已将点云填充至 {num_points} 点。")

            if colors is not None:
                return point_cloud_filtered, colors_filtered
            return point_cloud_filtered

        # --- 2. Open3D Voxel Downsampling (可选，用于初步减少点数) ---
        # 这一步可以大大加速后续的FPS，特别是在点云非常密集时
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud_filtered)
        if colors_filtered is not None:
            # Open3D 颜色期望 [0, 1] 范围的浮点数
            if colors_filtered.dtype == np.uint8:
                pcd.colors = o3d.utility.Vector3dVector(colors_filtered.astype(np.float64) / 255.0)
            else:
                pcd.colors = o3d.utility.Vector3dVector(colors_filtered)

        # 尝试进行Voxel下采样
        pcd_voxel_downsampled = pcd.voxel_down_sample(voxel_size=voxel_size)
        points_voxel_downsampled = np.asarray(pcd_voxel_downsampled.points)
        if colors_filtered is not None:
            colors_voxel_downsampled = np.asarray(pcd_voxel_downsampled.colors)
            # 如果原始颜色是uint8，这里转回来以保持一致性，或者根据需求决定
            # colors_voxel_downsampled = (colors_voxel_downsampled * 255).astype(np.uint8)
        else:
            colors_voxel_downsampled = None
            
        print(f"Voxel下采样后点数: {points_voxel_downsampled.shape[0]}")

        # 如果Voxel下采样后点数已足够少，或接近目标，直接返回或填充
        if points_voxel_downsampled.shape[0] <= num_points:
            print(f"Voxel下采样后点数 ({points_voxel_downsampled.shape[0]}) 小于或等于目标点数 ({num_points})，不进行FPS。")
            if points_voxel_downsampled.shape[0] < num_points:
                pad_count = num_points - points_voxel_downsampled.shape[0]
                points_voxel_downsampled = np.vstack([points_voxel_downsampled, np.zeros((pad_count, 3), dtype=points_voxel_downsampled.dtype)])
                if colors_voxel_downsampled is not None:
                    colors_voxel_downsampled = np.vstack([colors_voxel_downsampled, np.zeros((pad_count, 3), dtype=colors_voxel_downsampled.dtype)])
                print(f"已将点云填充至 {num_points} 点。")

            if colors is not None:
                return points_voxel_downsampled, colors_voxel_downsampled
            return points_voxel_downsampled


        # --- 3. 最远点采样 (Farthest Point Sampling) ---
        # 实现FPS需要迭代地选择点。这里提供一个简单的基于欧氏距离的实现。
        # 注意：Open3D 本身没有直接的 FPS 函数，但可以通过构建 KDTree 后手动实现，
        # 或者使用其他库如 `torch-points3d` 或 `tensorflow` 实现。
        # 为了避免增加新的依赖，这里我们提供一个纯 numpy 的基本实现。
        
        # 原始点云
        current_points = points_voxel_downsampled
        if colors_voxel_downsampled is not None:
            current_colors = colors_voxel_downsampled
        
        n_points = current_points.shape[0]
        if n_points < num_points:
            print(f"警告: Voxel下采样后点数 {n_points} 小于目标点数 {num_points}。FPS将无法达到目标点数。")
            # 填充到 num_points
            pad_count = num_points - n_points
            current_points = np.vstack([current_points, np.zeros((pad_count, 3), dtype=current_points.dtype)])
            if current_colors is not None:
                current_colors = np.vstack([current_colors, np.zeros((pad_count, 3), dtype=current_colors.dtype)])
            if colors is not None:
                return current_points, current_colors
            return current_points


        # 初始化采样点和距离数组
        farthest_pts = np.zeros((num_points, 3), dtype=current_points.dtype)
        if colors is not None:
            farthest_colors = np.zeros((num_points, 3), dtype=current_colors.dtype)
        
        # 随机选择第一个点
        farthest_pts[0] = current_points[np.random.randint(n_points)]
        if colors is not None:
            farthest_colors[0] = current_colors[np.random.randint(n_points)] # 确保颜色和点对应
        
        # 计算所有点到第一个采样点的距离平方
        distances = np.sum((current_points - farthest_pts[0])**2, axis=1)

        # 迭代选择剩余的点
        for i in range(1, num_points):
            # 选择距离当前所有采样点最远的点
            # 这里的 argmax 找到距离数组中最大值的索引
            # distances 数组存储的是每个点到 "最近的已选择点" 的距离
            # 每选择一个新点，就更新所有点的 "最近距离"
            farthest_idx = np.argmax(distances)
            farthest_pts[i] = current_points[farthest_idx]
            if colors is not None:
                farthest_colors[i] = current_colors[farthest_idx]
            
            # 更新所有点到新采样点的距离，并取最小值 (更新距离数组)
            new_distances = np.sum((current_points - farthest_pts[i])**2, axis=1)
            distances = np.minimum(distances, new_distances)
            
        print(f"FPS下采样后点数: {farthest_pts.shape[0]}")

        if colors is not None:
            return farthest_pts, farthest_colors
        return farthest_pts
        # if point_cloud.shape[0] > num_points:
        #     pass



    def create_point_cloud_with_fps(self, rgb_img, depth_img, intrinsic, extrinsic, num_points=4096, min_depth_m=0.01, max_depth_m=30.0, debug=True):
        """
        从 RGB, Depth, 内参和外参生成世界坐标系下的采样点云。
        流程: 深度过滤 -> AABB 过滤 -> SOR -> [新增ROR] -> FPS 采样
        """
        # rgb, depth, point_cloud = camera.get_frame()
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]
        H, W = depth_img.shape

        # --- 1. 深度过滤 ---
        z_cam_m = depth_img.astype(np.float32)
        valid_depth_mask = (z_cam_m > min_depth_m) & (z_cam_m < max_depth_m)

        v, u = np.indices((H, W))
        
        u_valid = u[valid_depth_mask]
        v_valid = v[valid_depth_mask]
        z_valid_m = z_cam_m[valid_depth_mask]
        rgb_valid = rgb_img[valid_depth_mask] # (N_valid, 3)

        x_cam_valid = (u_valid - cx) * z_valid_m / fx
        y_cam_valid = (v_valid - cy) * z_valid_m / fy
        points_cam_xyz_valid = np.stack([x_cam_valid, y_cam_valid, z_valid_m], axis=-1)
        
        R = extrinsic[:3, :3]
        t = extrinsic[:3, 3]
        
        points_world_xyz_filtered = points_cam_xyz_valid @ R.T + t # (N_valid, 3)
        rgb_intermediate_pre_aabb = rgb_valid # (N_valid, 3) 
        if debug:
            print("before_aabb")
            self.visualize_point_cloud_(points_world_xyz_filtered, rgb_intermediate_pre_aabb)
        # --- 2. 工作空间 (AABB) 过滤 ---
        xmin = 0.25
        xmax = 0.80
        ymin = -0.4
        ymax = 0.4
        zmin = -0.10
        zmax = 0.30
        
        mask = (points_world_xyz_filtered[:, 0] > xmin) & \
            (points_world_xyz_filtered[:, 0] < xmax) & \
            (points_world_xyz_filtered[:, 1] > ymin) & \
            (points_world_xyz_filtered[:, 1] < ymax) & \
            (points_world_xyz_filtered[:, 2] > zmin) & \
            (points_world_xyz_filtered[:, 2] < zmax)
        
        points_aabb_filtered = points_world_xyz_filtered[mask] # (N_aabb, 3)
        rgb_aabb_filtered = rgb_intermediate_pre_aabb[mask]    # (N_aabb, 3)

        num_aabb_points = len(points_aabb_filtered)
        
        if num_aabb_points == 0:
            # print("  [Warning] 应用深度和AABB过滤后，没有有效点。")
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        if debug:
            print("after_aabb")
            self.visualize_point_cloud_(points_aabb_filtered, rgb_aabb_filtered)

        # --- 2.5. (保留) 统计离群点移除 (SOR) ---
        
        pcd_sor_input = o3d.geometry.PointCloud()
        pcd_sor_input.points = o3d.utility.Vector3dVector(points_aabb_filtered)
        pcd_sor_input.colors = o3d.utility.Vector3dVector(rgb_aabb_filtered / 255.0) # 归一化

        # (参数为您之前设定的值)
        cl_sor, ind_sor = pcd_sor_input.remove_statistical_outlier(nb_neighbors=20,
                                                                   std_ratio=0.5)
        
        pcd_sor_filtered = pcd_sor_input.select_by_index(ind_sor)

        points_sor_filtered = np.asarray(pcd_sor_filtered.points)
        rgb_sor_filtered = np.asarray(pcd_sor_filtered.colors) * 255.0

        num_sor_filtered_points = len(points_sor_filtered)

        if num_sor_filtered_points == 0:
            # print("  [Warning] SOR 过滤后没有有效点。")
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)


        # print("after_sor")
        if debug:
            print("after_sor")
            self.visualize_point_cloud_(points_sor_filtered, rgb_sor_filtered)
        # --- 2.6. (新增) 半径离群点移除 (ROR) ---
        
        # 1. 将 SOR 过滤后的点云作为 ROR 的输入
        pcd_ror_input = o3d.geometry.PointCloud()
        pcd_ror_input.points = o3d.utility.Vector3dVector(points_sor_filtered)
        pcd_ror_input.colors = o3d.utility.Vector3dVector(rgb_sor_filtered / 255.0) # 归一化

        # 2. 执行 ROR 滤波
        # (请调整这两个参数)
        MIN_NEIGHBORS_IN_RADIUS = 20  # 在半径内至少需要的邻居点数
        SEARCH_RADIUS_M = 0.01      # 搜索半径 (米)
    
        cl_ror, ind_ror = pcd_ror_input.remove_radius_outlier(nb_points=MIN_NEIGHBORS_IN_RADIUS,
                                                              radius=SEARCH_RADIUS_M)
        
        pcd_ror_filtered = pcd_ror_input.select_by_index(ind_ror)

        # 3. 将 ROR 过滤后的点云转换回 NumPy 数组
        points_ror_filtered = np.asarray(pcd_ror_filtered.points)
        rgb_ror_filtered = np.asarray(pcd_ror_filtered.colors) * 255.0

        # 更新点数，用于后续 FPS 步骤
        num_ror_filtered_points = len(points_ror_filtered)
        
        if num_ror_filtered_points == 0:
            # print("  [Warning] ROR 过滤后没有有效点。")
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

         # --- 3. [修改] 带桌子比例控制的两阶段采样 ---
    


        # --- 3b. 第二阶段：拆分为 "桌子" 和 "其他" 点云并分别FPS ---

       # 1. 定义采样数量
        num_table_samples_target = num_points // 4
        
        # --- 2. 定义掩码 (Masks) ---
        
        # 重新定义 zmin/zmax（用于桌子高度过滤）
        zmin_table = -0.15
        zmax_table = -0.1

        # 掩码 A: 桌子平面高度 (Z 轴)
        table_height_mask = (points_ror_filtered[:, 2] >= zmin_table) & (points_ror_filtered[:, 2] <= zmax_table)
        
        # 掩码 B: 在桌子平面 *之上* 的点 (物体)
        objects_above_table_mask = (points_ror_filtered[:, 2] > -0.108)

        # 掩码 C: "青色" (Cyan) 颜色掩码 (这才是你的桌子)
        # 我们将使用 HSV 空间来精确过滤，排除黄色(H~30)和灰色(S<100)
        
        # 1. 将 (N, 3) RGB 数组转换为 HSV
        # 确保RGB值在0-255范围内并转换为uint8
        rgb_for_hsv = np.clip(rgb_ror_filtered, 0, 255).astype(np.uint8)
        
        # 检查是否有有效的RGB数据
        if len(rgb_for_hsv) == 0:
            cyan_mask = np.zeros(len(points_ror_filtered), dtype=bool)
        else:
            # 重塑为OpenCV期望的格式 (1, N, 3)
            rgb_for_hsv = rgb_for_hsv.reshape(1, -1, 3)
            hsv_image = cv2.cvtColor(rgb_for_hsv, cv2.COLOR_RGB2HSV)
            hsv_flat = hsv_image.reshape(-1, 3)  # (N, 3) H, S, V
            
            # 确保hsv_flat是uint8类型
            hsv_flat = hsv_flat.astype(np.uint8)

            # 2. 定义青色 (Cyan) 的范围 (H: 0-179, S/V: 0-255)
            # 你的桌子平均 H=87, S=166, V=75
            lower_cyan = np.array([80, 100, 50], dtype=np.uint8)  
            upper_cyan = np.array([100, 255, 200], dtype=np.uint8)
            
            # 3. 创建掩码
            # 对于2D数组，我们需要逐个通道进行比较
            h_mask = (hsv_flat[:, 0] >= lower_cyan[0]) & (hsv_flat[:, 0] <= upper_cyan[0])
            s_mask = (hsv_flat[:, 1] >= lower_cyan[1]) & (hsv_flat[:, 1] <= upper_cyan[1])
            v_mask = (hsv_flat[:, 2] >= lower_cyan[2]) & (hsv_flat[:, 2] <= upper_cyan[2])
            cyan_mask = h_mask & s_mask & v_mask
            SHADOW_V_THRESHOLD = 50  # 使用与 lower_cyan[2] 相同的值
            shadow_mask = (hsv_flat[:, 2] < SHADOW_V_THRESHOLD)
        # --- 3. [修正] 组合掩码 ---
        
        # 最终的 "桌子" 掩码：在桌子高度 且 是青色
        # table_mask = table_height_mask & cyan_mask
        table_mask = table_height_mask
        # 最终的 "其他物体" 掩码：
        # (在桌子高度 且 *不是* 青色)  <-- 这是桌上的扁平物体
        #           或者
        # (在桌子高度 *之上*)           <-- 这是桌上的凸起物体
        other_mask = (table_height_mask & (~cyan_mask) & (~shadow_mask)) | objects_above_table_mask
        # other_mask = ~table_height_mask
        # print('[HSV 过滤] cyan_mask.sum():', cyan_mask.sum())
        # print('[最终逻辑] table_mask.sum():', table_mask.sum())
        # print('[最终逻辑] other_mask.sum():', other_mask.sum())

        # 4. 分离点云
        table_points = points_ror_filtered[table_mask]
        table_rgb = rgb_ror_filtered[table_mask]


        
        # table_rgb = rgb_ror_filtered[table_mask]
        color_average = np.mean(table_rgb, axis=0)
        # print('color_average:', color_average)
        if debug:
            print("table_points")
            self.visualize_point_cloud_(table_points, table_rgb)
        other_points = points_ror_filtered[other_mask]
        other_rgb = rgb_ror_filtered[other_mask]
        if debug:
            print("other_points")
            self.visualize_point_cloud_(other_points, other_rgb)
        # 4. 分别进行采样 (并处理点数不足的边界情况)
        
        final_points_list = []
        final_rgb_list = []
        actual_table_samples_count = 0

        # # --- 4a. 采样桌子点 ---
        # if len(table_points) > 0:
        #     if len(table_points) <= num_table_samples_target:
        #         # 桌子点不够目标数，全部保留
        #         sampled_table_points = table_points
        #         sampled_table_rgb = table_rgb
        #     else:
        #         # 桌子点充足，执行FPS
        #         table_pcd = o3d.geometry.PointCloud()
        #         table_pcd.points = o3d.utility.Vector3dVector(table_points)
        #         table_pcd.colors = o3d.utility.Vector3dVector(table_rgb / 255.0)
        #         sampled_table_pcd = table_pcd.farthest_point_down_sample(num_table_samples_target)
                
        #         sampled_table_points = np.asarray(sampled_table_pcd.points)
        #         sampled_table_rgb = np.asarray(sampled_table_pcd.colors) * 255.0
                
        #     final_points_list.append(sampled_table_points)
        #     final_rgb_list.append(sampled_table_rgb)
        #     actual_table_samples_count = len(sampled_table_points)

        # --- 4b. 采样其他点 ---
        # 我们需要补充的点数 = 总目标 - 实际采到的桌子点
        # 这会自动补偿桌子点不足的情况
        num_other_samples_needed = num_points - actual_table_samples_count

        if len(other_points) > 0 and num_other_samples_needed > 0:
            if len(other_points) <= num_other_samples_needed:
                # 其他点不够，全部保留
                sampled_other_points = other_points
                sampled_other_rgb = other_rgb
            else:
                # 其他点充足，执行FPS
                other_pcd = o3d.geometry.PointCloud()
                other_pcd.points = o3d.utility.Vector3dVector(other_points)
                other_pcd.colors = o3d.utility.Vector3dVector(other_rgb / 255.0)
                sampled_other_pcd = other_pcd.farthest_point_down_sample(num_other_samples_needed)
                
                sampled_other_points = np.asarray(sampled_other_pcd.points)
                sampled_other_rgb = np.asarray(sampled_other_pcd.colors) * 255.0

            final_points_list.append(sampled_other_points)
            final_rgb_list.append(sampled_other_rgb)

        # 5. 合并结果
        if not final_points_list:
            # 极端情况：如果两个列表都为空（例如过滤后点云为空）
            points_final_sampled = np.empty((0, 3), dtype=np.float32)
            rgb_final_sampled = np.empty((0, 3), dtype=np.float32)
        else:
            points_final_sampled = np.concatenate(final_points_list, axis=0)
            rgb_final_sampled = np.concatenate(final_rgb_list, axis=0)

        return points_final_sampled.astype(np.float32), rgb_final_sampled.astype(np.float32)  
    
    def create_point_cloud_with_dbscan(self, rgb_img, depth_img, intrinsic, extrinsic, num_points=4096, min_depth_m=0.01, max_depth_m=30.0, debug=True):
        """
        从 RGB, Depth, 内参和外参生成世界坐标系下的采样点云。
        流程: AABB 过滤 -> 随机采样 (至 10000 点) -> DBSCAN (取最大簇) 
              -> 桌面/物体分离 (HSV+高度) -> 分别 FPS 采样
        """
        TARGET_RANDOM_POINTS = 10000 # 随机采样的目标点数

        # --- 1. 深度过滤 & 2. 投影到世界坐标系 ---
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]
        H, W = depth_img.shape

        z_cam_m = depth_img.astype(np.float32)
        valid_depth_mask = (z_cam_m > min_depth_m) & (z_cam_m < max_depth_m)

        v, u = np.indices((H, W))
        
        u_valid = u[valid_depth_mask]
        v_valid = v[valid_depth_mask]
        z_valid_m = z_cam_m[valid_depth_mask]
        rgb_valid = rgb_img[valid_depth_mask] # (N_valid, 3)

        x_cam_valid = (u_valid - cx) * z_valid_m / fx
        y_cam_valid = (v_valid - cy) * z_valid_m / fy
        points_cam_xyz_valid = np.stack([x_cam_valid, y_cam_valid, z_valid_m], axis=-1)
        
        R = extrinsic[:3, :3]
        t = extrinsic[:3, 3]
        
        points_world_xyz_filtered = points_cam_xyz_valid @ R.T + t # (N_valid, 3)
        rgb_intermediate_pre_aabb = rgb_valid # (N_valid, 3) 
        
        # --- 3. 工作空间 (AABB) 过滤 ---
        xmin = 0.15
        xmax = 0.80
        ymin = -0.5
        ymax = 0.5
        zmin = -0.15
        zmax = 0.30
        
        mask = (points_world_xyz_filtered[:, 0] > xmin) & \
            (points_world_xyz_filtered[:, 0] < xmax) & \
            (points_world_xyz_filtered[:, 1] > ymin) & \
            (points_world_xyz_filtered[:, 1] < ymax) & \
            (points_world_xyz_filtered[:, 2] > zmin) & \
            (points_world_xyz_filtered[:, 2] < zmax)
        
        points_aabb_filtered = points_world_xyz_filtered[mask] # (N_aabb, 3)
        rgb_aabb_filtered = rgb_intermediate_pre_aabb[mask]    # (N_aabb, 3)

        num_aabb_points = len(points_aabb_filtered)
        
        if num_aabb_points == 0:
            print("  [Warning] 应用深度和AABB过滤后，没有有效点。")
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        if debug:
            print("[Hybrid] after_aabb")
            self.visualize_point_cloud_(points_aabb_filtered, rgb_aabb_filtered)

        # --- 4. (替换 SOR) 随机下采样 ---
        # 随机采样到 TARGET_RANDOM_POINTS (10000) 点，为 DBSCAN 做准备
        
        if num_aabb_points <= TARGET_RANDOM_POINTS:
            # 点数已经足够少，无需采样
            points_random_sampled = points_aabb_filtered
            rgb_random_sampled = rgb_aabb_filtered
            print(f"  [Info] AABB 过滤后点数 ({num_aabb_points}) <= 目标 ({TARGET_RANDOM_POINTS})，跳过随机采样。")
        else:
            # 点数过多，执行随机采样
            print(f"  [Info] AABB 过滤后点数 ({num_aabb_points})，随机采样至 {TARGET_RANDOM_POINTS} 点。")
            indices = np.random.choice(num_aabb_points, TARGET_RANDOM_POINTS, replace=False)
            points_random_sampled = points_aabb_filtered[indices]
            rgb_random_sampled = rgb_aabb_filtered[indices]

        if debug:
            print("[Hybrid] after_random_sampling (10k)")
            self.visualize_point_cloud_(points_random_sampled, rgb_random_sampled)

        # --- 5. DBSCAN 聚类过滤 ---
        # 在随机采样后的点云上执行 DBSCAN
        
        pcd_dbscan_input = o3d.geometry.PointCloud()
        pcd_dbscan_input.points = o3d.utility.Vector3dVector(points_random_sampled)

        DBSCAN_EPS = 0.015       # 邻域距离 (米)
        DBSCAN_MIN_POINTS = 30 # 形成簇的最小点数

        labels = np.array(pcd_dbscan_input.cluster_dbscan(eps=DBSCAN_EPS, min_points=DBSCAN_MIN_POINTS, print_progress=False))
        
        unique_labels, counts = np.unique(labels, return_counts=True)
        non_noise_mask = (unique_labels != -1)
        
        if not np.any(non_noise_mask):
            print("  [Warning] DBSCAN 过滤后没有找到非噪声簇。")
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        largest_cluster_label = unique_labels[non_noise_mask][np.argmax(counts[non_noise_mask])]
        dbscan_mask = (labels == largest_cluster_label)
        
        points_dbscan_filtered = points_random_sampled[dbscan_mask]
        rgb_dbscan_filtered = rgb_random_sampled[dbscan_mask] # 保持颜色同步

        num_dbscan_filtered_points = len(points_dbscan_filtered)

        if num_dbscan_filtered_points == 0:
            print("  [Warning] DBSCAN 过滤后点云为空。")
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        if debug:
            print(f"[Hybrid] after_dbscan (保留最大簇 {largest_cluster_label}，点数 {num_dbscan_filtered_points})")
            self.visualize_point_cloud_(points_dbscan_filtered, rgb_dbscan_filtered)

        # --- 6. (新增) 带桌子比例控制的两阶段采样 ---
        # (此逻辑完全复制自 create_point_cloud_with_fps，但输入改为 dbscan 过滤后的点)

        # 1. 定义采样数量
        num_table_samples_target = num_points // 4
        
        # 2. 定义掩码 (Masks)
        # 输入: points_dbscan_filtered, rgb_dbscan_filtered
        
        # 重新定义 zmin/zmax（用于桌子高度过滤）
        zmin_table = -0.15
        zmax_table = -0.11

        # 掩码 A: 桌子平面高度 (Z 轴)
        table_height_mask = (points_dbscan_filtered[:, 2] >= zmin_table) & (points_dbscan_filtered[:, 2] <= zmax_table)
        
        # 掩码 B: 在桌子平面 *之上* 的点 (物体)
        objects_above_table_mask = (points_dbscan_filtered[:, 2] > -0.108)

        # 掩码 C: "青色" (Cyan) 颜色掩码
        rgb_for_hsv = np.clip(rgb_dbscan_filtered, 0, 255).astype(np.uint8)
        
        if len(rgb_for_hsv) == 0:
            cyan_mask = np.zeros(len(points_dbscan_filtered), dtype=bool)
            shadow_mask = np.zeros(len(points_dbscan_filtered), dtype=bool)
        else:
            rgb_for_hsv = rgb_for_hsv.reshape(1, -1, 3)
            hsv_image = cv2.cvtColor(rgb_for_hsv, cv2.COLOR_RGB2HSV)
            hsv_flat = hsv_image.reshape(-1, 3)  # (N, 3) H, S, V
            hsv_flat = hsv_flat.astype(np.uint8)

            lower_cyan = np.array([80, 100, 50], dtype=np.uint8)  
            upper_cyan = np.array([100, 255, 200], dtype=np.uint8)
            
            h_mask = (hsv_flat[:, 0] >= lower_cyan[0]) & (hsv_flat[:, 0] <= upper_cyan[0])
            s_mask = (hsv_flat[:, 1] >= lower_cyan[1]) & (hsv_flat[:, 1] <= upper_cyan[1])
            v_mask = (hsv_flat[:, 2] >= lower_cyan[2]) & (hsv_flat[:, 2] <= upper_cyan[2])
            cyan_mask = h_mask & s_mask & v_mask
            
            SHADOW_V_THRESHOLD = 50 
            shadow_mask = (hsv_flat[:, 2] < SHADOW_V_THRESHOLD)

        # 3. 组合掩码
        table_mask = table_height_mask & cyan_mask
        other_mask = (table_height_mask & (~cyan_mask) & (~shadow_mask)) | objects_above_table_mask

        # 4. 分离点云
        table_points = points_dbscan_filtered[table_mask]
        table_rgb = rgb_dbscan_filtered[table_mask]

        other_points = points_dbscan_filtered[other_mask]
        other_rgb = rgb_dbscan_filtered[other_mask]

        if debug:
            print(f"[Hybrid] 分离后: 桌子点数 {len(table_points)}, 物体点数 {len(other_points)}")
            print("--- [Debug] 显示分离的桌子点 ---")
            self.visualize_point_cloud_(table_points, table_rgb)
            print("--- [Debug] 显示分离的物体点 ---")
            self.visualize_point_cloud_(other_points, other_rgb)

        # 5. 分别进行采样
        final_points_list = []
        final_rgb_list = []
        actual_table_samples_count = 0

        # --- 5a. 采样桌子点 ---
        if len(table_points) > 0:
            if len(table_points) <= num_table_samples_target:
                sampled_table_points = table_points
                sampled_table_rgb = table_rgb
            else:
                table_pcd = o3d.geometry.PointCloud()
                table_pcd.points = o3d.utility.Vector3dVector(table_points)
                table_pcd.colors = o3d.utility.Vector3dVector(table_rgb / 255.0)
                sampled_table_pcd = table_pcd.farthest_point_down_sample(num_table_samples_target)
                sampled_table_points = np.asarray(sampled_table_pcd.points)
                sampled_table_rgb = np.asarray(sampled_table_pcd.colors) * 255.0
                
            final_points_list.append(sampled_table_points)
            final_rgb_list.append(sampled_table_rgb)
            actual_table_samples_count = len(sampled_table_points)
            print(f"  [Info] 采样到 {actual_table_samples_count} 个桌子点。")

        # --- 5b. 采样其他点 ---
        num_other_samples_needed = num_points - actual_table_samples_count

        if len(other_points) > 0 and num_other_samples_needed > 0:
            if len(other_points) <= num_other_samples_needed:
                sampled_other_points = other_points
                sampled_other_rgb = other_rgb
            else:
                other_pcd = o3d.geometry.PointCloud()
                other_pcd.points = o3d.utility.Vector3dVector(other_points)
                other_pcd.colors = o3d.utility.Vector3dVector(other_rgb / 255.0)
                sampled_other_pcd = other_pcd.farthest_point_down_sample(num_other_samples_needed)
                sampled_other_points = np.asarray(sampled_other_pcd.points)
                sampled_other_rgb = np.asarray(sampled_other_pcd.colors) * 255.0

            final_points_list.append(sampled_other_points)
            final_rgb_list.append(sampled_other_rgb)
            print(f"  [Info] 采样到 {len(sampled_other_points)} 个物体点。")

        # 6. 合并结果
        if not final_points_list:
            points_final_sampled = np.empty((0, 3), dtype=np.float32)
            rgb_final_sampled = np.empty((0, 3), dtype=np.float32)
        else:
            points_final_sampled = np.concatenate(final_points_list, axis=0)
            rgb_final_sampled = np.concatenate(final_rgb_list, axis=0)
        
        print(f"  [Info] 最终总点数: {len(points_final_sampled)}")
        return points_final_sampled.astype(np.float32), rgb_final_sampled.astype(np.float32)

    def create_point_cloud_with_fps_ori(self,rgb_img, depth_img, intrinsic, extrinsic, num_points=4096, min_depth_m=0.01, max_depth_m=30.0,debug = False):
  
    
        fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        cx, cy = intrinsic[0, 2], intrinsic[1, 2]
        H, W = depth_img.shape

        # --- 1. 深度过滤 ---
        z_cam_m = depth_img.astype(np.float32)
        z_cam_m = z_cam_m
        valid_depth_mask = (z_cam_m > min_depth_m) & (z_cam_m < max_depth_m)

        v, u = np.indices((H, W))
        
        u_valid = u[valid_depth_mask]
        v_valid = v[valid_depth_mask]
        z_valid_m = z_cam_m[valid_depth_mask]
        rgb_valid = rgb_img[valid_depth_mask]

        x_cam_valid = (u_valid - cx) * z_valid_m / fx
        y_cam_valid = (v_valid - cy) * z_valid_m / fy
        points_cam_xyz_valid = np.stack([x_cam_valid, y_cam_valid, z_valid_m], axis=-1)
        
        R = extrinsic[:3, :3]
        t = extrinsic[:3, 3]
        
        points_world_xyz_filtered = points_cam_xyz_valid @ R.T + t 
        rgb_intermediate = rgb_valid 

        # --- 2. 工作空间 (AABB) 过滤 ---
        xmin = 0.2
        xmax = 0.7
        ymin = -0.35
        ymax = 0.35
        
        # 这里保持宽泛的下限，比如 -0.15，以便包含放在桌上的薄物体
        zmin = -0.115
        zmax = 0.35 
        
        mask = (points_world_xyz_filtered[:, 0] > xmin) & \
            (points_world_xyz_filtered[:, 0] < xmax) & \
            (points_world_xyz_filtered[:, 1] > ymin) & \
            (points_world_xyz_filtered[:, 1] < ymax) & \
            (points_world_xyz_filtered[:, 2] > zmin) & \
            (points_world_xyz_filtered[:, 2] < zmax)
        
        points_aabb_filtered = points_world_xyz_filtered[mask]
        rgb_aabb_filtered = rgb_intermediate[mask]



        points_filtered = points_aabb_filtered
        rgb_filtered = rgb_aabb_filtered
        ###############################################################################################
        rgb_filtered = self.adjust_saturation_brightness(rgb_filtered, 1.0, 1.0)

        num_points_left = len(points_filtered)
        
        if num_points_left == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros((0, 3), dtype=np.float32)

        # --- 3. FPS 采样 ---
        
        if num_points_left <= num_points:
            return points_filtered.astype(np.float32), rgb_filtered.astype(np.float32)
        if len(points_filtered) > 0:
            r, g, b = rgb_filtered[:, 0], rgb_filtered[:, 1], rgb_filtered[:, 2]
            is_green = (g > r) & (g > b) & (g > 40)
            is_low = points_filtered[:, 2] < -0.105
            keep_mask = ~(is_green & is_low)
            points_filtered = points_filtered[keep_mask]
            rgb_filtered = rgb_filtered[keep_mask]
            num_points_left = len(points_filtered)
        # 随机降采样 (加速)
        FPS_INPUT_LIMIT = 20000 
        if num_points_left > FPS_INPUT_LIMIT:
            indices = np.random.choice(num_points_left, FPS_INPUT_LIMIT, replace=False)
            points_intermediate = points_filtered[indices]
            rgb_intermediate = rgb_filtered[indices]
        else:
            points_intermediate = points_filtered
            rgb_intermediate = rgb_filtered

        # 运行 FPS
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_intermediate)
        pcd.colors = o3d.utility.Vector3dVector(rgb_intermediate / 255.0)
        
        sampled_pcd = pcd.farthest_point_down_sample(num_points)
        
        points_final_sampled = np.asarray(sampled_pcd.points)
        rgb_final_sampled = np.asarray(sampled_pcd.colors) * 255.0

        if debug:
            print("after_fps")
            self.visualize_point_cloud_(points_final_sampled, rgb_final_sampled)

        # dbscan filter
        # points_final_sampled, rgb_final_sampled = self.filter_dbscan(points_final_sampled, rgb_final_sampled)
        
        return points_final_sampled.astype(np.float32), rgb_final_sampled.astype(np.float32)
  
    def create_point_cloud_with_mask_hybrid(self, 
        rgb_img, ori_depth_img, repaired_depth_img, mask, 
        extrinsic, num_points=4096, min_depth_m=0.01,
        max_depth_m=30.0, debug=False):
        """
        混合点云生成策略：
        1. Mask 区域（物体）：使用 repaired_depth_img 的深度。
        2. 非 Mask 区域（背景）：使用 ori_depth_img 的深度。
        3. 过滤（仅对背景点进行）：AABB -> SOR -> DBSCAN 过滤。
        4. 合并后进行 FPS 采样。
        """
        # --- 1. 基础参数设置 ---
        # fx, fy = intrinsic[0, 0], intrinsic[1, 1]
        # cx, cy = intrinsic[0, 2], intrinsic[1, 2]

        fx = self.intrinsics.fx
        fy = self.intrinsics.fy
        cx = self.intrinsics.ppx  # ppx 代表 principal point x
        cy = self.intrinsics.ppy  # ppy 代表 principal point y

        H, W = ori_depth_img.shape

        
        if mask.shape != (H, W):
            assert False, "Mask shape does not match depth image shape"
        mask = mask.astype(bool)
        
        # --- 2. 混合深度图生成 ---
     
        mixed_depth_img = ori_depth_img.astype(np.float32).copy()
        # Mask 区域使用修复后的深度
        mixed_depth_img[mask] = repaired_depth_img[mask].astype(np.float32)

        # 深度有效性过滤（基于混合深度图）
        z_cam_m = mixed_depth_img
        valid_depth_mask = (z_cam_m > min_depth_m) & (z_cam_m < max_depth_m)
        
        # 获取有效点的索引
        v, u = np.indices((H, W))
        
        u_valid = u[valid_depth_mask]
        v_valid = v[valid_depth_mask]
        z_valid_m = z_cam_m[valid_depth_mask]
        rgb_valid = rgb_img[valid_depth_mask]
        mask_valid = mask[valid_depth_mask] # 对应的 SAM Mask 状态

        # 反投影到相机坐标
        x_cam_valid = (u_valid - cx) * z_valid_m / fx
        y_cam_valid = (v_valid - cy) * z_valid_m / fy
        points_cam_xyz = np.stack([x_cam_valid, y_cam_valid, z_valid_m], axis=-1)

        # 转换到世界坐标
        R = extrinsic[:3, :3]
        t = extrinsic[:3, 3]
        points_world = points_cam_xyz @ R.T + t

        # self.visualize_point_cloud_(points_world, rgb_valid)


        # --- 3. 分离点云
        # Part A: Object
        points_obj = points_world[mask_valid]
        rgb_obj = rgb_valid[mask_valid]

        # obj aabb
        # points_obj, rgb_obj = self.aabb_filter(points_obj, rgb_obj)
        points_obj, rgb_obj = self.sor_filter(points_obj, rgb_obj)
        if debug:
            self.visualize_point_cloud_(points_obj, rgb_obj)
        # Part B: Background 
        points_bg = points_world[~mask_valid] 
        rgb_bg = rgb_valid[~mask_valid]

        if debug:
            print(f"Mask内点数 (使用修复深度): {len(points_obj)}, Mask外点数(过滤前, 使用原始深度): {len(points_bg)}")

        # --- 4. 处理 (背景点) ---
        if len(points_bg) > 0:
            # 4.1 AABB 过滤 
            points_bg, rgb_bg = self.aabb_filter(points_bg, rgb_bg)
            if debug:
                print(f"AABB 过滤后背景点数: {len(points_bg)}")
                

            # 4.2 随机下采样 
            points_bg, rgb_bg = self.random_downsample(points_bg, rgb_bg)
            if debug:
                print(f"随机下采样后背景点数: {len(points_bg)}")
            
            # 4.3 SOR
            points_bg, rgb_bg = self.sor_filter(points_bg, rgb_bg)
            if debug:
                print(f"SOR 过滤后背景点数: {len(points_bg)}")
                # self.visualize_point_cloud_(points_bg, rgb_bg)

            # 4.4 DBSCAN
            points_bg, rgb_bg = self.dbscan_filter(points_bg, rgb_bg)
            if debug:
                print(f"DBSCAN 过滤后背景点数: {len(points_bg)}")
                self.visualize_point_cloud_(points_bg, rgb_bg)
            

        # --- 5. 合并点云 ---
        points_final = np.vstack([points_obj, points_bg])
        rgb_final = np.vstack([rgb_obj, rgb_bg])
        
       
        # assert len(points_obj) > 0 and len(points_bg) > 0, "Object or background point cloud is empty"

        # --- 6. FPS 采样和补零 ---
        
        if len(points_final) > num_points:
            # 执行 FPS
            pcd_final = o3d.geometry.PointCloud()
            pcd_final.points = o3d.utility.Vector3dVector(points_final)
            pcd_final.colors = o3d.utility.Vector3dVector(rgb_final / 255.0)
            
            pcd_sampled = pcd_final.farthest_point_down_sample(num_points)
            
            points_return = np.asarray(pcd_sampled.points).astype(np.float32)
            rgb_return = (np.asarray(pcd_sampled.colors) * 255.0).astype(np.float32)
        
        elif len(points_final) < num_points:
            # 补零
            pad_num = num_points - len(points_final)
            points_return = np.vstack([points_final, np.zeros((pad_num, 3))]).astype(np.float32)
            rgb_return = np.vstack([rgb_final, np.zeros((pad_num, 3))]).astype(np.float32)
        else:
            points_return = points_final.astype(np.float32)
            rgb_return = rgb_final.astype(np.float32)

        # 调亮rgb
        rgb_return = self.adjust_saturation_brightness(rgb_return, 1.0, 1.2)
        if debug:
            self.visualize_point_cloud_(points_return, rgb_return)
        # self.visualize_point_cloud_(points_return, rgb_return)
        # breakpoint()
        return points_return, rgb_return

    def aabb_filter(self, points, rgb):
        xmin = 0.25
        xmax = 0.70
        ymin = -0.5
        ymax = 0.35
        # zmin = -0.085
        # zmin = -0.11
        zmin = -0.101
        zmax = 0.35
        
        aabb_mask = (points[:, 0] > xmin) & (points[:, 0] < xmax) & \
                    (points[:, 1] > ymin) & (points[:, 1] < ymax) & \
                    (points[:, 2] > zmin) & (points[:, 2] < zmax)
        return points[aabb_mask], rgb[aabb_mask]

    def random_downsample(self, points, rgb, num_points=10000):
        # 修复：如果当前点数少于目标点数，则不进行采样，直接返回
        if len(points) > num_points:
            indices = np.random.choice(len(points), num_points, replace=False)
            return points[indices], rgb[indices]
        else:
            return points, rgb

    def sor_filter(self, points, rgb):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(rgb / 255.0)
        pcd, ind_sor = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=0.2)
        return np.asarray(pcd.points), np.asarray(pcd.colors) * 255.0

    def dbscan_filter(self, points, rgb):
        if len(points) == 0:
            return points, rgb
            
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(rgb / 255.0)
        
        # 执行 DBSCAN
        # labels = np.array(pcd.cluster_dbscan(eps=0.015, min_points=100, print_progress=False))
        labels = np.array(pcd.cluster_dbscan(eps=0.012, min_points=100, print_progress=False))
        
        # 修复：根据 label 过滤点云
        # label == -1 表示噪声点，我们保留 label >= 0 的点
        max_label = labels.max()
        if max_label < 0: # 如果全是噪声，或者没聚成类
             # 策略1：返回空（视情况而定）
             # return np.array([]), np.array([])
             # 策略2：如果不聚类太严格，就返回原样（这里选策略1演示过滤）
             valid_mask = labels >= 0 
        else:
             valid_mask = labels >= 0
             
        # 应用 Mask
        points_filtered = np.asarray(pcd.points)[valid_mask]
        rgb_filtered = np.asarray(pcd.colors)[valid_mask] * 255.0
        
        return points_filtered, rgb_filtered



    def adjust_saturation_brightness(self, rgb_nx3, saturation_scale=1.0, brightness_scale=1.0):
        """
        输入: 
            rgb_nx3: (N, 3) np.uint8, 范围 [0, 255]  <-- 必须是 uint8
            saturation_scale: 饱和度倍率
            brightness_scale: 亮度倍率
        输出:
            adjusted_rgb: (N, 3) np.uint8, 范围 [0, 255] <-- 返回 uint8
        """
        # 0. 确保输入是 uint8，防止传入了错误类型导致计算前就溢出
        # 如果输入已经是 numpy array，这一步是零拷贝（如果类型匹配）
        rgb_data = rgb_nx3.astype(np.uint8) 

        # 1. RGB -> HSV
        # 必须先转 float 再除以 255.0
        hsv = mcolors.rgb_to_hsv(rgb_data.astype(np.float32) / 255.0)
        
        # 2. 调整饱和度 (Channel 1: Saturation)
        hsv[:, 1] *= saturation_scale
        
        # 3. 调整亮度 (Channel 2: Value)
        hsv[:, 2] *= brightness_scale
        
        # 4. 截断数值，防止超出 [0, 1] 范围
        # 这一步非常重要，否则下一步转 RGB 会报错或溢出
        hsv[:, 1:] = np.clip(hsv[:, 1:], 0, 1)
        
        # 5. HSV -> RGB
        adjusted_rgb = mcolors.hsv_to_rgb(hsv)
        
        # 6. 转回 0-255 并转为 uint8
        # 使用 np.round 四舍五入会比直接截断更精确一点
        return np.round(adjusted_rgb * 255.0).astype(np.uint8)

def main():
    camera = DepthCameraModule()
    try:
        rgb, depth = camera.get_frame()
        if rgb is not None and depth is not None:
            print(f"✓ 获取图像成功: RGB={rgb.shape}, Depth={depth.shape}")
            
            # 简单的测试：显示图像
            import cv2
            cv2.imshow('RGB Image', cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            cv2.imshow('Depth Image', (depth * 1000).astype(np.uint8))  # 深度图×1000显示
            print("按任意键退出...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        else:
            print("✗ 获取图像失败")
    finally:
        camera.stop()
        print("程序结束")

if __name__ == "__main__":
    main()