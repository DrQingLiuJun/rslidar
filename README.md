# RSLidar 三路感知工作空间

ROS 2 Humble 项目：两路 RS80、一路 RSBP 接收，三路 PV-RCNN 推理、目标跟踪与地理坐标标注。

## 目录

```text
src/
  rslidar_msg/        雷达消息定义
  rslidar_sdk/        C++ 驱动、接收配置、单独驱动入口
  lidar_perception/  Python 检测应用、模型配置、权重、三路启动入口
scripts/             环境、编译、运行、验证工具
deploy/              后台服务模板
docs/                部署与结构说明
tests/               配置和跟踪回归检查
vendor/OpenPCDet/    固定提交的第三方推理框架（Git 子模块）
requirements.txt     直接 Python 依赖
requirements-lock.txt 已验证环境的依赖快照
```

`.venv/` 保存本机 Python 环境，`.cache/colcon/` 集中保存构建、安装和日志产物；两者均不进入 Git。

## 获取与安装

```bash
git lfs install
git clone --recurse-submodules https://github.com/DrQingLiuJun/rslidar.git rslidar_ws
cd rslidar_ws
git lfs pull
```

需要 Ubuntu 22.04、ROS 2 Humble、NVIDIA 驱动、CUDA Toolkit 12.1。已验证显卡为 RTX 4090。

```bash
sudo apt install libpcap-dev libpcl-dev libyaml-cpp-dev python3-venv \
  ros-humble-pcl-conversions ros-humble-rmw-cyclonedds-cpp \
  ros-humble-ament-cmake-python
python3 -m venv --system-site-packages .venv
export PYTHONNOUSERSITE=1
.venv/bin/python -m pip install 'pip<25' 'setuptools<70' wheel
.venv/bin/python -m pip install -r requirements.txt
./scripts/build_openpcdet.sh
./scripts/build.sh
```

脚本从自身位置定位工作空间，不要求固定的用户名或安装目录。`CUDA_HOME` 默认 `/usr/local/cuda-12.1`；`TORCH_CUDA_ARCH_LIST` 默认 `8.9`，其他显卡应在编译 OpenPCDet 时覆盖这两个变量。

## 启动与验证

```bash
./scripts/run.sh           # 三路驱动 + TF + 检测
./scripts/test.sh          # 配置与跟踪回归检查
./scripts/smoke_test.sh    # 三份模型的合成点云 GPU 检查
```

后台运行：

```bash
./scripts/install_service.sh
systemctl --user status rslidar
journalctl --user -u rslidar -f
systemctl --user stop rslidar
```

服务安装脚本会启动或重启服务，默认不启用开机自启。前台运行前先停止后台服务，避免占用同一端口。

## 三路输入输出

| 通道 | 型号 | MSOP / DIFOP UDP | 输入点云 | 输出检测 |
|---|---|---|---|---|
| lidar_1 | RS80 | 6636 / 7736 | /rslidar_points_1 | /detection_result_1 |
| lidar_2 | RS80 | 6639 / 7739 | /rslidar_points_2 | /detection_result_2 |
| lidar_3 | RSBP | 6641 / 7741 | /rslidar_points_3 | /detection_result_3 |

- 接收参数：`src/rslidar_sdk/config/radars.yaml`。
- 模型选择、话题、安装标定和 TF 原点：`src/lidar_perception/config/sensors.yaml`。
- 模型结构与点云范围：`src/lidar_perception/config/models/`、`config/datasets/`。
- 权重：`src/lidar_perception/models/`，使用 Git LFS。

驱动监听所有 IPv4 网卡，雷达需发送到服务器实际接收网卡的可达 IP 和对应端口。未接入时 `MSOPTIMEOUT` 表示等待数据。默认 ROS_DOMAIN_ID=82、ROS_LOCALHOST_ONLY=1、CycloneDDS；可通过环境变量覆盖。

本次整理保留原模型、端口、标定、静态障碍过滤及跟踪算法。真实精度、帧率和标定仍需雷达接入后验证；合成点云检查只证明推理通路能运行。

详见 [部署说明](docs/deployment.md)、[架构与维护](docs/architecture.md)。
