# 部署说明

服务器工作空间：/home/user/carla-projects/rslidar_ws。

## 服务管理

入口 scripts/run.sh；模板 deploy/rslidar.service.in 安装时会替换为当前路径。

```bash
./scripts/build.sh
./scripts/install_service.sh
systemctl --user status rslidar
journalctl --user -u rslidar -f
```

ROS 环境由 scripts/env.sh 统一设置。不要同时启动前台和后台两套驱动，六个 UDP 端口不能重复占用。

## 网络

主板网口 eno1：172.16.4.81，用于管理和当前默认上网。USB 拓展坞网口 enx2c16dba65bb0：检查时为 192.168.0.111/16、1 Gbps，另有旧摄像头网络路由。

雷达发送地址应对应现场实际接入的网卡，不能仅凭管理 IP 推断。换线后检查可达性、三路点云、带宽和丢包；驱动未绑定某个网卡名称。

## 模型与验证

已验证 PyTorch 2.3.1、CUDA 12.1、spconv-cu120 2.3.6、NumPy 1.26.4、Numba 0.59.1、llvmlite 0.42.0、Kornia 0.6.12。依赖快照见 requirements-lock.txt。

模型权重重命名不改变内容，lidar_1、lidar_2、lidar_3 分别对应原 6636、6639、6641 检查点。三份模型均应完整加载 319/319 参数。

scripts/test.sh 检查配置匹配、路径独立性、跟踪关联和轨迹过期；scripts/smoke_test.sh 使用合成点云执行三份模型 GPU 前向推理，不评估精度。

雷达尚未发送数据时，检测器显示 Ready/waiting，驱动报告 MSOPTIMEOUT。此时真实标定、精度和实时帧率仍待验证。
