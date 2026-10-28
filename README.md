# 三路激光雷达接收与 PV-RCNN 检测

基于 ROS 2 Humble、RoboSense rslidar_sdk 和 OpenPCDet，接收两路 RS80 与一路 RSBP 点云，运行三份独立 PV-RCNN 模型，发布检测框和跟踪标注。

## 获取源码及模型

```bash
git lfs install
git clone --recurse-submodules https://github.com/DrQingLiuJun/rslidar.git rslidar_ws
cd rslidar_ws
git lfs pull
```

三份 `.pth` 权重通过 Git LFS 保存。OpenPCDet 以子模块固定到已验证提交 `233f849829b6ac19afb8af8837a0246890908755`。RoboSense SDK 和 rs_driver 源码直接包含在 src 中，保留各自许可证。

## 已验证环境

- Ubuntu 22.04 / ROS 2 Humble / Python 3.10
- NVIDIA RTX 4090，CUDA Toolkit 12.1
- PyTorch 2.3.1，torchvision 0.18.1，spconv-cu120 2.3.6
- NumPy 1.26.4，Numba 0.59.1，llvmlite 0.42.0，Kornia 0.6.12

完整 Python 环境快照位于 `scripts/requirements-installed.txt`。系统依赖包括 libpcap-dev、libpcl-dev、libyaml-cpp-dev、python3-venv、ROS PCL 转换及 rmw-cyclonedds-cpp。

默认服务器部署路径为 `/home/user/carla-projects/rslidar_ws`。现有配置保留该路径；如安装到其他位置，需要修改 config 下 YAML 的绝对路径、launch 中的虚拟环境解释器路径、run-rslidar.sh 及 systemd 模板路径。

## 编译

在已安装 ROS 和 CUDA 的环境中：

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages .venv
export PYTHONNOUSERSITE=1
.venv/bin/python -m pip install 'pip<25' 'setuptools<70' wheel
.venv/bin/python -m pip install -r scripts/requirements-installed.txt
bash scripts/build-openpcdet.sh
CMAKE_BUILD_PARALLEL_LEVEL=6 colcon build \
  --paths src/rslidar_msg src/rslidar_sdk --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

CUDA 扩展脚本默认针对 RTX 4090 的 8.9 架构；其他显卡应调整 TORCH_CUDA_ARCH_LIST。

## 运行

```bash
./run-rslidar.sh
```

| 雷达 | 型号 | MSOP / DIFOP UDP 端口 | 点云话题 | 检测话题 |
|---|---|---|---|---|
| 1 | RS80 | 6636 / 7736 | /rslidar_points_1 | /detection_result_1 |
| 2 | RS80 | 6639 / 7739 | /rslidar_points_2 | /detection_result_2 |
| 3 | RSBP | 6641 / 7741 | /rslidar_points_3 | /detection_result_3 |

雷达需将数据发送到接收网卡可达的服务器 IP 和对应端口。配置见 `src/rslidar_sdk/config/config_multi.yaml`。未接收到雷达数据时会出现 MSOPTIMEOUT。

启动脚本使用 ROS_DOMAIN_ID=82、CycloneDDS 和 ROS_LOCALHOST_ONLY=1；雷达 UDP 接收监听所有 IPv4 接口。标定、地理坐标原点及模型参数保留项目原设置，换场地后需重新核实。

后台服务模板位于 `scripts/rslidar.service`。服务器现有服务可通过 `systemctl --user status rslidar` 和 `journalctl --user -u rslidar -f` 查看。

## 验证与部署记录

三份模型均完整加载 319/319 参数，并使用 20000 个合成点通过 GPU 推理检查；三路点云发布与检测订阅已连通。检查时雷达尚未发送数据，真实检测精度、标定与帧率仍需现场验证。

```bash
bash scripts/run-gpu-smoke.sh
```

服务器部署说明见 `DEPLOYMENT.md`。其中 runtime/deployment 下的历史日志、修改前备份和原始 ZIP 仅保存在服务器，未纳入 Git。虚拟环境、构建产物及运行数据同样不入库。
