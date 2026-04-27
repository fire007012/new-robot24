# vision_pkg —— 越障车视觉模块（ROS1 Noetic）

## 项目简介

vision_pkg 是一个运行在 **Ubuntu 20.04 + ROS1 Noetic** 上的视觉处理模块，为越障车提供以下功能：

- **综合视觉显示**：一个节点集成 YOLOv8 检测 + 障碍物预警 + 距离显示，细白线辅助框设计
- **YOLOv8 目标检测**：基于 OpenCV DNN 加载 ONNX 模型，检测置信度阈值 0.7
- **物体 3D 位姿估计**：结合检测结果与深度图，计算物体 3D 位置（含深度滤波 + EMA 平滑）
- **障碍物预警**：深度图三区域（左/中/右）最近距离检测，细白线分割，距离文字显示
- **中心距离显示**：细白线十字准星，实时显示画面中心深度距离
- **双鱼眼显示/全景处理**：支持 `azimuthal` 方位等距圆盘、`stacked` 前后透视图上下拼接、`cropped_stacked` 原始鱼眼矩形裁剪后上下拼接；裁剪框可通过 launch 参数配置
- **RealSense D405**：paw_camera 提供彩色图、深度图和点云数据
- **Behind Camera**：支持 Astra+ 或 RealSense D435i（可通过 launch 参数或环境变量指定；也可用外部脚本自动检测后传参）
- **多路 USB 摄像头**：支持 forward/back/head/hand/arm 多路 USB 摄像头采集，其中前后鱼眼默认按 1920×1080@30 输入
- **最终视频流压缩**：支持对 `/panorama/panorama_image`、`/paw_vision/vision_image`、`/behind_vision/vision_image` 按需启动 H.265 编码节点，供上位机订阅压缩码流
- **热成像相机**：Xtherm T2S+ 热成像（独立包 `thermal_camera`，默认随 vision.launch 启动）

---

## 项目结构

```
vision_pkg/
├── CMakeLists.txt                          # 构建配置
├── package.xml                             # ROS 包描述
├── README.md                               # 本文件
├── include/vision_pkg/
│   ├── camerainit.h                        # USB 摄像头采集类（V4L2）
│   ├── realsense.h                         # RealSense 深度相机采集类
│   ├── fisheye.h                           # 鱼眼展开、透视重投影、原始鱼眼裁剪
│   ├── panorama.h                          # 双鱼眼显示管线（圆盘/透视拼接/裁剪拼接）
│   ├── yolov8.h                            # YOLOv8 目标检测封装
│   └── yolov8_utils.h                      # 检测结果数据结构与绘图工具
├── src/
│   ├── yolov8.cpp                          # YOLOv8 检测实现（LetterBox + NMS）
│   ├── yolov8_node.cpp                     # YOLOv8 ROS 节点
│   ├── object_pose_node.cpp                # 物体 3D 位姿估计节点
│   ├── distance_display_node.cpp           # 中心距离显示节点
│   ├── obstacle_warning_node.cpp           # 前方障碍物预警节点
│   ├── fisheye.cpp                         # 鱼眼展开 / 等距柱状 / 方位等距 / 透视重投影
│   ├── panorama.cpp                        # 全景管线实现
│   ├── panorama_node.cpp                   # 全景处理 ROS 节点
│   ├── h265_encoder_node.cpp               # 最终图像流 H.265 编码发布节点
│   └── vision_display_node.cpp             # 综合视觉显示节点
├── scripts/
│   └── detect_behind_camera.py             # behind_camera 类型自动检测脚本
├── launch/
│   └── vision.launch                       # 一键启动所有节点
├── msg/
│   ├── Detection.msg                       # 2D 检测结果
│   ├── DetectedObject3D.msg                # 单个物体 3D 位姿
│   ├── DetectedObject3DArray.msg           # 多物体 3D 位姿数组
│   └── ObstacleWarning.msg                 # 障碍物预警消息
├── model/
│   └── best.onnx                           # YOLOv8 ONNX 模型文件
└── third_party/                            # 第三方头文件库（可选）
    └── include/
        └── Eigen/                           # Eigen 矩阵库（header-only，仅部分模块用到）

thermal_camera/                              # 热成像独立包
├── CMakeLists.txt
├── package.xml
├── setup.py
├── scripts/
│   └── thermal_camera_node.py              # ROS 热成像节点
├── launch/
│   └── thermal_camera.launch
└── src/                                     # IR-Py-Thermal 库源码
    ├── irpythermal.py
    ├── pyplot.py
    ├── opencv.py
    ├── display.py
    ├── utils.py
    └── example_simple.py
```

---

## 一键启动

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vision_pkg vision.launch
```

启动前请确认 behind_camera 位置的相机类型是 RealSense 还是 Astra+。默认是 RealSense，需更改时参考下面的 `behind_camera_type` 参数。

### launch 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `enable_usb_cams` | `false` | 启用 USB 摄像头组：forward/back/head/hand/arm |
| `enable_paw_camera` | `true` | 启用 RealSense D405 |
| `enable_behind_camera` | `true` | 启用 behind_camera |
| `enable_thermal` | `true` | 启用热成像 |
| `behind_camera_type` | `realsense` | Behind 相机类型：`astra` 或 `realsense`（可用环境变量 `BEHIND_CAMERA_TYPE` 覆盖） |
| `paw_serial` | `130322273001` | RealSense D405 序列号 |
| `behind_serial` | `219323070286` | Astra+ 序列号 |
| `behind_realsense_serial` | `112222070518` | D435i 序列号 |
| `panorama_display_mode` | `azimuthal` | 全景输出模式：`azimuthal` 圆盘、`stacked` 去畸变透视上下拼接、`cropped_stacked` 原始鱼眼裁剪上下拼接 |
| `panorama_fisheye_source_fov_deg` | `180.0` | 前/后鱼眼相机的源视场角（°），用于展开 map 计算 |
| `panorama_operator_view_fov_deg` | `120.0` | `stacked` 模式下单路透视视场角 |
| `panorama_operator_view_width` | `640` | `stacked` 模式下单路输出宽度 |
| `panorama_operator_view_height` | `360` | `stacked` 模式下单路输出高度 |
| `panorama_front_rotate_180` | `false` | 前鱼眼输入是否旋转 180° |
| `panorama_back_rotate_180` | `true` | 后鱼眼输入是否旋转 180° |
| `panorama_input_transport` | `raw` | 全景输入图像传输方式，可改为 `compressed` |
| `panorama_tcp_nodelay` | `true` | 全景输入订阅启用 TCP_NODELAY，降低 TCPROS 延迟 |
| `panorama_output_queue_size` | `1` | 全景输出队列长度，低延迟建议保持 1 |
| `panorama_front_crop_ref_width` / `panorama_front_crop_ref_height` | `1920` / `1080` | 前鱼眼裁剪参数采集时的参考分辨率 |
| `panorama_front_crop_x` / `panorama_front_crop_y` | `525` / `276` | 前鱼眼裁剪框左上角 |
| `panorama_front_crop_width` / `panorama_front_crop_height` | `876` / `538` | 前鱼眼裁剪框宽高 |
| `panorama_back_crop_ref_width` / `panorama_back_crop_ref_height` | `1920` / `1080` | 后鱼眼裁剪参数采集时的参考分辨率 |
| `panorama_back_crop_x` / `panorama_back_crop_y` | `558` / `250` | 后鱼眼裁剪框左上角 |
| `panorama_back_crop_width` / `panorama_back_crop_height` | `876` / `538` | 后鱼眼裁剪框宽高 |
| `forward_cam_dev` / `back_cam_dev` | `""` / `""` | 前后 USB 鱼眼设备路径，如 `/dev/video0`、`/dev/video2` |
| `head_cam_dev` / `hand_cam_dev` / `arm_cam_dev` | `""` | 额外 3 路 USB 摄像头设备路径；为空时不会启动对应节点，避免空设备反复报错 |
| `forward_cam_width` / `forward_cam_height` | `1920` / `1080` | 前鱼眼采集分辨率 |
| `back_cam_width` / `back_cam_height` | `1920` / `1080` | 后鱼眼采集分辨率 |
| `forward_cam_framerate` / `back_cam_framerate` | `30` / `30` | 前后鱼眼采集帧率 |
| `paw_color_input_transport` / `behind_color_input_transport` | `raw` / `raw` | D405/behind 彩色图输入传输方式，可改为 `compressed` |
| `paw_depth_input_transport` / `behind_depth_input_transport` | `raw` / `raw` | D405/behind 深度图输入传输方式 |
| `paw_*_queue_size` / `behind_*_queue_size` | `1` | 彩色图、深度图、相机内参和输出图队列长度，低延迟建议保持 1 |
| `paw_enable_qrcode_detection` / `behind_enable_qrcode_detection` | `true` / `false` | 是否在综合视觉节点中启用二维码识别 |
| `paw_enable_motion_detection` / `behind_enable_motion_detection` | `true` / `false` | 是否在综合视觉节点中启用动态检测 |
| `paw_enable_motion_debug_images` / `behind_enable_motion_debug_images` | `false` / `false` | 是否发布动态检测九宫格调试中间图 |
| `paw_motion_depth_min_m` / `paw_motion_depth_max_m` | `0.0` / `0.7` | D405 动态检测深度过滤范围 |
| `paw_motion_min_area` | `80` | D405 动态检测最小轮廓面积 |
| `paw_motion_max_foreground_ratio` | `0.12` | 相机/场景整体运动占比阈值，超过后抑制运动标记 |
| `enable_panorama_h265` | `false` | 是否对 `/panorama/panorama_image` 额外发布 H.265 压缩码流 |
| `enable_paw_h265` | `false` | 是否对 `/paw_vision/vision_image` 额外发布 H.265 压缩码流 |
| `enable_behind_h265` | `false` | 是否对 `/behind_vision/vision_image` 额外发布 H.265 压缩码流 |
| `*_h265_bitrate_kbps` | `1200`~`1500` | H.265 输出目标码率 |
| `*_h265_fps` / `*_h265_max_input_fps` | `15` / `15.0` | H.265 输出帧率与输入限帧 |

```bash
# 示例：关闭热成像
roslaunch vision_pkg vision.launch enable_thermal:=false

# 只测试前后鱼眼裁剪上下拼接图
roslaunch vision_pkg vision.launch \
  enable_usb_cams:=true \
  enable_paw_camera:=false \
  enable_behind_camera:=false \
  enable_thermal:=false \
  forward_cam_dev:=/dev/video0 \
  back_cam_dev:=/dev/video2 \
  panorama_display_mode:=cropped_stacked

# 如果要给上位机订阅 H.265 压缩后的最终全景图
roslaunch vision_pkg vision.launch \
  enable_usb_cams:=true \
  forward_cam_dev:=/dev/video0 \
  back_cam_dev:=/dev/video2 \
  panorama_display_mode:=cropped_stacked \
  enable_panorama_h265:=true

# 手动指定 behind_camera 为 Astra+
roslaunch vision_pkg vision.launch behind_camera_type:=astra

# 手动指定 behind_camera 为 RealSense D435i
roslaunch vision_pkg vision.launch behind_camera_type:=realsense behind_realsense_serial:=XXXXXX

# 用环境变量指定（适合两台工控机固定配置）
export BEHIND_CAMERA_TYPE=astra 或 export BEHIND_CAMERA_TYPE=realsense
roslaunch vision_pkg vision.launch
```

---

## 所有话题一览 & 订阅方法

### 驱动层话题

#### 1. RealSense D405 (paw_camera)

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/paw_camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/paw_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |
| `/paw_camera/color/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| `/paw_camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |

```bash
# 查看彩色图像
rqt_image_view /paw_camera/color/image_raw

# 查看深度图
rqt_image_view /paw_camera/depth/image_rect_raw

# RViz 查看点云
rviz   # 添加 PointCloud2，话题选 /paw_camera/depth/color/points
```

#### 2. Behind Camera (Astra+ 或 D435i，按配置选择)

**当 behind_camera 为 Astra+ 时：**

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/behind_camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/behind_camera/depth/image_raw` | `sensor_msgs/Image` | 深度图像 |
| `/behind_camera/depth/points` | `sensor_msgs/PointCloud2` | 点云 |

**当 behind_camera 为 RealSense D435i 时：**

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/behind_camera/color/image_raw` | `sensor_msgs/Image` | 彩色图像 |
| `/behind_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 深度图像 |
| `/behind_camera/depth/color/points` | `sensor_msgs/PointCloud2` | 彩色点云 |

```bash
# 查看 behind_camera 彩色图像
rqt_image_view /behind_camera/color/image_raw

# 查看深度图
rqt_image_view /behind_camera/depth/image_raw
```

#### 3. USB 摄像头（默认关闭，需 `enable_usb_cams:=true`）

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/forward_camera/image_raw` | `sensor_msgs/Image` | 前鱼眼/前方摄像头，默认 1920×1080@30 |
| `/back_camera/image_raw` | `sensor_msgs/Image` | 后鱼眼/后方摄像头，默认 1920×1080@30 |
| `/head_camera/image_raw` | `sensor_msgs/Image` | 头部/预留 USB 摄像头，默认 640×480@30 |
| `/hand_camera/image_raw` | `sensor_msgs/Image` | 手部摄像头，默认 640×480 |
| `/arm_camera/image_raw` | `sensor_msgs/Image` | 机械臂摄像头，默认 640×480 |

```bash
roslaunch vision_pkg vision.launch enable_usb_cams:=true
rqt_image_view /forward_camera/image_raw
```

#### 4. 热成像 Xtherm T2S+

| 话题 | 消息类型 | 说明 |
|------|---------|------|
| `/thermal_camera/image_raw` | `sensor_msgs/Image` | 伪彩色热成像图像（plasma colormap，1024x768） |

```bash
# 查看热成像画面
rqt_image_view /thermal_camera/image_raw

# 单独启动热成像
roslaunch thermal_camera thermal_camera.launch

# 热成像参数
#   ~device       : 设备路径（空=自动检测）
#   ~raw_mode     : RAW 模式（默认 true，T2S+ V2 必须开启）
#   ~frame_rate   : 帧率（默认 25）
#   ~temp_offset  : 温度偏移（默认 0.0）
#   ~upscale      : 放大倍数（默认 4，原始 256x192 → 1024x768）
#   ~colormap     : matplotlib colormap（默认 plasma）
```

### 功能层话题

#### 5. vision_display_node（综合视觉显示）

运行两个实例：`paw_vision`（paw_camera）和 `behind_vision`（behind_camera）。

**paw_vision：**

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/paw_camera/color/image_raw` | `sensor_msgs/Image` | 订阅 | 彩色图像输入 |
| `/paw_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 订阅 | 深度图像输入 |
| `/paw_vision/vision_image` | `sensor_msgs/Image` | 发布 | YOLO标注 + 障碍物预警 + 距离显示 |
| `/paw_vision/detections` | `vision_pkg/Detection` | 发布 | 检测结果 |
| `/paw_vision/obstacle_warning` | `vision_pkg/ObstacleWarning` | 发布 | 障碍物预警 |

**behind_vision：**

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/behind_camera/color/image_raw` | `sensor_msgs/Image` | 订阅 | 彩色图像输入 |
| `/behind_camera/depth/image_raw` | `sensor_msgs/Image` | 订阅 | 深度图像输入 |
| `/behind_vision/vision_image` | `sensor_msgs/Image` | 发布 | YOLO标注 + 障碍物预警 + 距离显示 |
| `/behind_vision/detections` | `vision_pkg/Detection` | 发布 | 检测结果 |
| `/behind_vision/obstacle_warning` | `vision_pkg/ObstacleWarning` | 发布 | 障碍物预警 |

```bash
# 查看 paw_camera 综合视觉画面（YOLO + 障碍物预警 + 距离）
rqt_image_view /paw_vision/vision_image

# 查看 behind_camera 综合视觉画面
rqt_image_view /behind_vision/vision_image

# 订阅检测结果
rostopic echo /paw_vision/detections

# 订阅障碍物预警
rostopic echo /paw_vision/obstacle_warning
```

#### 6. object_pose_node（物体 3D 位姿估计）

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/paw_vision/detections` | `vision_pkg/Detection` | 订阅 | 检测结果输入 |
| `/paw_camera/depth/image_rect_raw` | `sensor_msgs/Image` | 订阅 | 深度图输入 |
| `/paw_camera/color/camera_info` | `sensor_msgs/CameraInfo` | 订阅 | 相机内参 |
| `/detected_object_pose` | `geometry_msgs/PoseStamped` | 发布 | 物体 3D 位姿（20Hz） |

```bash
# 订阅物体 3D 位姿
rostopic echo /detected_object_pose

# 在 RViz 中可视化位姿
rviz   # 添加 Pose 显示，话题选 /detected_object_pose
```

#### 7. panorama_node（双鱼眼显示/全景处理）

| 话题 | 消息类型 | 方向 | 说明 |
|------|---------|------|------|
| `/forward_camera/image_raw` | `sensor_msgs/Image` | 订阅 | 前方鱼眼图像（`cam1_topic` 参数） |
| `/back_camera/image_raw` | `sensor_msgs/Image` | 订阅 | 后方鱼眼图像（`cam2_topic` 参数） |
| `/panorama/panorama_image` | `sensor_msgs/Image` | 发布 | 主输出图像，内容由 `panorama_display_mode` 决定 |
| `/panorama/panorama_equirect` | `sensor_msgs/Image` | 发布 | 等距柱状全景（2:1 长条图，仅在有订阅者时发布） |
| `/panorama/panorama_image/h265` | `sensor_msgs/CompressedImage` | 发布 | 可选 H.265 码流，需 `enable_panorama_h265:=true` |

**主要参数：** `~cam1_topic` / `~cam2_topic` / `~display_mode` / `~fisheye_source_fov_deg` / `~input_transport` / `~front_crop_*` / `~back_crop_*`。

**输出模式：**

| 模式 | `/panorama/panorama_image` 内容 | 适用场景 |
|------|-------------------------------|----------|
| `azimuthal` | 方位等距圆盘图 | 需要保留完整 360° 环视关系 |
| `stacked` | 前后两个去畸变透视图上下拼接 | 操作手希望地板、边线等直线更接近现实直线 |
| `cropped_stacked` | 从原始鱼眼画面按配置矩形裁剪后上下拼接 | 低延迟、少计算、保留原始鱼眼局部画质 |

**管线说明：** `azimuthal`/`stacked` 会先将鱼眼图展开到等距柱状图，再渲染圆盘或前后透视图；`cropped_stacked` 不做展开，直接按前后裁剪框从原始图中取 ROI，上下拼接后发布，适合当前操作手查看需求。

```bash
# 查看当前主输出图
roslaunch vision_pkg vision.launch \
  enable_usb_cams:=true \
  forward_cam_dev:=/dev/video0 \
  back_cam_dev:=/dev/video2 \
  panorama_display_mode:=cropped_stacked
rqt_image_view /panorama/panorama_image

# 查看等距柱状全景（可直接导入 360° 全景播放器）
rqt_image_view /panorama/panorama_equirect

# 查看 H.265 码流是否发布
rostopic echo -n 1 /panorama/panorama_image/h265
```

---

## 话题速查表

```bash
# ===== 驱动层 =====
/paw_camera/color/image_raw            # D405 彩色图
/paw_camera/depth/image_rect_raw       # D405 深度图
/paw_camera/depth/color/points         # D405 点云
/behind_camera/color/image_raw         # Astra+/D435i 彩色图
/behind_camera/depth/image_raw         # Astra+/D435i 深度图
/forward_camera/image_raw              # USB 前方（需 enable）
/back_camera/image_raw                 # USB 后方（需 enable）
/head_camera/image_raw                 # USB 头部/预留（需 enable）
/hand_camera/image_raw                 # USB 手部（需 enable）
/arm_camera/image_raw                  # USB 机臂（需 enable）
/thermal_camera/image_raw              # 热成像伪彩色

# ===== 功能层 =====
/paw_vision/vision_image               # paw 综合视觉画面
/paw_vision/detections                 # paw YOLO 检测结果
/paw_vision/obstacle_warning           # paw 障碍物预警
/behind_vision/vision_image            # behind 综合视觉画面
/behind_vision/detections              # behind YOLO 检测结果
/behind_vision/obstacle_warning        # behind 障碍物预警
/detected_object_pose                  # 物体 3D 位姿
/panorama/panorama_image               # 全景/裁剪拼接主输出，取决于 panorama_display_mode
/panorama/panorama_equirect            # 等距柱状 2:1 全景（按需发布）
/panorama/panorama_image/h265          # 可选 H.265 压缩全景码流
/paw_vision/vision_image/h265          # 可选 H.265 压缩 D405 综合视觉码流
/behind_vision/vision_image/h265       # 可选 H.265 压缩 behind 综合视觉码流
```

**快速查看所有话题：**
```bash
rostopic list
rostopic list | grep image    # 只看图像话题
rostopic list | grep points   # 只看点云话题
```

---

## 自定义消息

### Detection.msg

```
int32   id
float32 confidence
int32   x
int32   y
int32   width
int32   height
string  class_name
```

### DetectedObject3D.msg / DetectedObject3DArray.msg

在 `object_pose_node` 中发布，由 2D 检测结果结合 depth + camera_info 计算得到，包含目标类别、置信度和相机坐标系下的 3D 位置。

### ObstacleWarning.msg

```
float32 left_dist
float32 center_dist
float32 right_dist
bool    left_warn
bool    center_warn
bool    right_warn
```

---

## 环境要求

| 项目 | 版本要求 |
|------|---------|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS1 Noetic（鱼香一键安装） |
| OpenCV | **4.10**（需从源码编译，替换系统自带 4.2） |
| librealsense2 | v2.50.0（paw_camera 使用） |
| realsense-ros | ROS1 分支（源码编译，放入 catkin_ws） |
| astra_camera | OrbbecSDK_ROS1 v1.5.8+（源码编译，behind_camera 使用） |
| scikit-image | `pip3 install scikit-image`（热成像节点使用） |
| matplotlib | 系统自带即可（热成像节点使用） |

---

## 安装步骤

### 第一步：安装 ROS Noetic（鱼香一键脚本）

如果尚未安装 ROS：

```bash
wget http://fishros.com/install -O fishros && . fishros
```

按提示选择安装 ROS1 Noetic。

### 第二步：安装基础依赖

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake git pkg-config \
    libegl1-mesa-dev libgl1-mesa-dev \
    libusb-1.0-0-dev libssl-dev libgtk-3-dev \
    libglfw3-dev libglu1-mesa-dev \
    python3-catkin-tools \
    ros-noetic-image-transport \
    ros-noetic-compressed-image-transport \
    ros-noetic-compressed-depth-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-usb-cam \
    ros-noetic-joy \
    ros-noetic-ddynamic-reconfigure \
    ros-noetic-rgbd-launch \
    ros-noetic-backward-ros \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    libopenni2-dev \
    libuvc-dev

pip3 install scikit-image
```

### 第三步：从源码编译安装 OpenCV 4.10

卸载系统自带 OpenCV 4.2 并编译安装 4.10：

```bash
# 安装编译依赖
sudo apt install -y \
    libavcodec-dev libavformat-dev libswscale-dev \
    libv4l-dev libxvidcore-dev libx264-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libatlas-base-dev gfortran \
    python3-numpy

# 下载 OpenCV 4.10 源码
cd ~
git clone -b 4.10.0 --depth 1 https://github.com/opencv/opencv.git
git clone -b 4.10.0 --depth 1 https://github.com/opencv/opencv_contrib.git

# 编译安装
cd ~/opencv
mkdir build && cd build
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D WITH_CUDA=OFF \
      -D BUILD_EXAMPLES=OFF \
      -D BUILD_TESTS=OFF \
      -D BUILD_PERF_TESTS=OFF \
      -D BUILD_opencv_python2=OFF \
      -D BUILD_opencv_python3=ON \
      ..

make -j$(nproc)
sudo make install
sudo ldconfig
echo 'export OpenCV_DIR=/usr/local/lib/cmake/opencv4' >> ~/.bashrc
source ~/.bashrc
```

> **注意**：编译安装 OpenCV 4.10 后，还需要重新编译 `cv_bridge` 以匹配新版本 OpenCV，否则 catkin_make 时可能出现链接错误。方法如下：

```bash
cd ~/catkin_ws/src
git clone -b noetic https://github.com/ros-perception/vision_opencv.git
```

这样 `cv_bridge` 会在工作空间内从源码编译，自动链接到新版 OpenCV。

### 第四步：安装 Intel RealSense SDK（librealsense2 v2.50.0）
从 Intel apt 源安装的 `librealsense2-dev` 的 cmake 配置中引用了 `fastcdr` 和 `fastrtps`（ROS2 DDS 依赖），
 会导致 `catkin_make` 时报错：
 The following imported targets are referenced, but are missing: fastcdr fastrtps
>解决办法：卸载 apt 版本，改用源码编译安装 librealsense2：
```
 # 卸载已有 apt 版本
 sudo apt remove librealsense2-dev librealsense2 librealsense2-utils
 sudo apt autoremove

 # 安装编译依赖
 sudo apt install -y git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev \
   libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

 # 源码编译（v2.50.0）
 cd ~
 git clone https://github.com/IntelRealSense/librealsense.git
 cd librealsense
 git checkout v2.50.0
 mkdir build && cd build
 cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false
 make -j$(nproc)
 sudo make install
```

### 第五步：获取 realsense-ros 源码包

将自己的 realsense-ros 包（ROS1 分支）放到工作空间中：

```bash
cd ~/catkin_ws/src

# 克隆 realsense-ros（ROS1 分支）
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros
git checkout `git tag | sort -V | grep -P "^2.\d+\.\d+" | tail -1`
cd ..
```

> 如果你已经有自己的 realsense-ros 包，直接将其复制到 `~/catkin_ws/src/` 下即可。

### 第五步 b：获取 Astra+ 相机驱动（behind_camera 使用）

**重要**：必须使用 v1.5.8 版本，v2.x 不支持 Astra+。

```bash
cd ~/catkin_ws/src
git clone --depth 1 --branch v1.5.8 https://github.com/orbbec/OrbbecSDK_ROS1.git
cd OrbbecSDK_ROS1
git checkout v1.5.8
cd ~/catkin_ws/src
```

安装 udev 规则（否则普通用户无权限访问设备）：

```bash
cd ~/catkin_ws/src/OrbbecSDK_ROS1/scripts
sudo cp 99-obsensor-ros1-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 第六步：（可选）放置 Eigen 头文件

Eigen 仅在部分模块中用到，若编译提示缺失再执行：

```bash
cd ~/catkin_ws/src/vision_pkg
mkdir -p third_party/include

# Eigen（apt 安装后软链接过来即可）
sudo apt install -y libeigen3-dev
ln -s /usr/include/eigen3/Eigen ~/catkin_ws/src/vision_pkg/third_party/include/Eigen
```

> 全景模块已改为纯 OpenCV 2D 实现，**不再依赖 GLM / OpenGL / EGL**，相关第三方库可省略。

### 第七步：编译工作空间

```bash
cd ~/catkin_ws

# 确保 source 了 ROS 环境
source ~/catkin_ws/devel/setup.bash

catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
```

编译成功后，配置环境变量：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 运行

### 一键启动所有节点

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch vision_pkg vision.launch
```

这将同时启动：
- RealSense 深度相机 paw_camera（含点云）
- Behind Camera（Astra+ 或 D435i，自动检测）
- 热成像 Xtherm T2S+
- YOLOv8 综合视觉显示节点（paw_vision + behind_vision）
- 物体 3D 位姿估计节点
- 双鱼眼显示/全景节点（输出内容由 `panorama_display_mode` 决定）
- forward/back/head/hand/arm USB 摄像头（默认关闭，需设置 `enable_usb_cams:=true`）
- 可选 H.265 最终图像流编码节点（默认关闭，需设置 `enable_panorama_h265` / `enable_paw_h265` / `enable_behind_h265`）

---

## 视频流低延迟与压缩建议

当前代码按低延迟优先做了几项处理：

- `panorama_node` 和 `vision_display_node` 使用 `image_transport` 订阅图像，`*_input_transport` 可从 `raw` 改成 `compressed`，用于跨机器传输时降低带宽。
- 主要图像输出队列默认是 `1`，旧帧会尽快丢弃，避免上位机看到明显滞后的画面。
- `panorama_node` 只有在 `/panorama/panorama_image` 或 `/panorama/panorama_equirect` 有订阅者时才处理图像，减少无订阅时的计算负载。
- `vision_display_node` 只有在最终图像、检测结果或调试图有订阅者时才执行对应计算，运动调试图默认关闭。
- H.265 节点只编码最终输出图，不把中间处理图都发到网络上。

常用启动方式：

```bash
# 本机调试：保留 raw，方便 rqt_image_view 查看
roslaunch vision_pkg vision.launch \
  enable_usb_cams:=true \
  forward_cam_dev:=/dev/video0 \
  back_cam_dev:=/dev/video2 \
  panorama_display_mode:=cropped_stacked

# 上位机订阅最终压缩码流：开启 H.265
roslaunch vision_pkg vision.launch \
  enable_usb_cams:=true \
  forward_cam_dev:=/dev/video0 \
  back_cam_dev:=/dev/video2 \
  panorama_display_mode:=cropped_stacked \
  enable_panorama_h265:=true \
  panorama_h265_bitrate_kbps:=1500 \
  panorama_h265_fps:=15

# 如果上游 camera raw 图像跨网线传输，可尝试 compressed 输入
roslaunch vision_pkg vision.launch \
  enable_usb_cams:=true \
  panorama_input_transport:=compressed \
  paw_color_input_transport:=compressed \
  behind_color_input_transport:=compressed
```

H.265 输出话题是 `sensor_msgs/CompressedImage`，`format` 字段为 `h265`。它保留给上位机通讯/解码使用，不能按普通 JPEG/PNG 压缩图直接用 `rqt_image_view` 查看。

---

## 注意事项

1. **OpenCV 版本**：本项目要求 OpenCV 4.10，系统自带的 4.2 版本不满足需求。编译安装后务必确认 `cv_bridge` 也使用新版本。
2. **RealSense 序列号**：launch 文件中 paw_camera 的 `serial_no` 需要替换为实际设备序列号，可通过 `rs-enumerate-devices | grep Serial` 获取。
3. **Astra+ 序列号**：launch 文件中 behind_camera 的 `serial_number` 需要替换为实际设备序列号。
4. **Behind Camera** 选择方式：vision.launch 通过 behind_camera_type 选择 astra 或 realsense。可手动传参
    （behind_camera_type:=astra/realsense）或设置环境变量 BEHIND_CAMERA_TYPE。若需自动检测，请在外部脚本检测后再传给
    roslaunch。
5. **USB 摄像头设备路径**：launch 文件中的设备路径需要根据实际硬件连接情况调整，可通过 `v4l2-ctl --list-devices` 查看可用设备；前后鱼眼常用 `/dev/video0` 和 `/dev/video2`，但以现场枚举结果为准。
6. **全景源 FOV**：`panorama_fisheye_source_fov_deg` 需匹配实际鱼眼镜头的视场角（默认 180°）。`cropped_stacked` 模式不依赖展开 FOV，只使用裁剪框。
7. **Astra+ USB 规则**：首次使用 Astra+ 需要配置 udev 规则，否则可能无权限访问设备，详见安装步骤。
8. **热成像依赖**：热成像节点需要 `scikit-image` 和 `matplotlib`，通过 `pip3 install scikit-image` 安装。
9. **H.265 查看方式**：H.265 话题是 `sensor_msgs/CompressedImage`，`format=h265`，不是普通 `image_transport/compressed` JPEG/PNG 图像；`rqt_image_view` 通常不能直接显示，需要上位机或自定义节点解码。
---

## 二维码与动态检测

当前二维码识别和动态检测都在 `vision_display_node` 中完成；`panorama_node` 只负责双鱼眼显示/全景处理，不再承担二维码或动态检测。

### 接入位置

| 节点 | 默认状态 | 输入 | 输出 |
|------|----------|------|------|
| `paw_vision` | 二维码开启，动态检测开启 | `/paw_camera/color/image_raw` + `/paw_camera/depth/image_rect_raw` | `/paw_vision/vision_image` |
| `behind_vision` | 二维码关闭，动态检测关闭 | behind 彩色图 + 深度图 | `/behind_vision/vision_image` |

### 常用参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `paw_enable_qrcode_detection` | `true` | 是否给 D405 画面开启二维码识别 |
| `behind_enable_qrcode_detection` | `false` | 是否给后方视觉画面开启二维码识别 |
| `paw_enable_motion_detection` | `true` | 是否给 D405 画面开启动态物体检测 |
| `behind_enable_motion_detection` | `false` | 是否给后方视觉画面开启动态物体检测 |
| `paw_enable_motion_depth_filter` | `true` | 是否按深度范围过滤 D405 动态检测结果 |
| `paw_motion_depth_min_m` / `paw_motion_depth_max_m` | `0.0` / `0.7` | 只保留 0~70cm 范围内的运动区域 |
| `paw_motion_min_area` | `80` | 动态检测保留的最小轮廓面积 |
| `paw_motion_canny_low_threshold` / `paw_motion_canny_high_threshold` | `50.0` / `150.0` | 用于细化运动物体边缘的 Canny 阈值 |
| `paw_motion_diff_threshold` | `18.0` | 帧差/背景差分阈值 |
| `paw_motion_learning_rate` | `0.01` | 动态背景更新学习率 |
| `paw_motion_max_foreground_ratio` | `0.12` | 前景占比过大时视为整机/场景运动，并抑制运动标记 |

### D405 测试启动

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
source ~/catkin_ws/devel/setup.bash

roslaunch vision_pkg vision.launch \
  paw_enable_qrcode_detection:=true \
  paw_enable_motion_detection:=true \
  paw_enable_motion_debug_images:=true \
  paw_enable_motion_depth_filter:=true \
  paw_motion_min_area:=80 \
  paw_motion_depth_min_m:=0.0 \
  paw_motion_depth_max_m:=0.7
```

验证主输出：

```bash
rostopic hz /paw_camera/color/image_raw
rosparam get /paw_vision/enable_qrcode_detection
rosparam get /paw_vision/enable_motion_detection
rqt_image_view /paw_vision/vision_image
```

动态检测调试图：

```bash
/paw_vision/debug/motion_gray
/paw_vision/debug/motion_frame_diff
/paw_vision/debug/motion_fg_raw
/paw_vision/debug/motion_fg_after_diff
/paw_vision/debug/motion_fg_after_open
/paw_vision/debug/motion_fg_after_close
/paw_vision/debug/motion_fg_after_dilate
/paw_vision/debug/motion_depth_mask
/paw_vision/debug/motion_fg_final
```

查看调试图示例：

```bash
rqt_image_view /paw_vision/debug/motion_fg_final
```

预期现象：

- 画面出现二维码时，会画出二维码边框，并在框附近标出解码内容。
- 画面中出现摆动、摇摆、来回移动的目标时，会对运动区域轮廓描边，并标注 `Moving`。
- 如果整幅画面变化占比过大，系统会按相机/场景整体运动处理，避免把全画面误标成目标运动。
