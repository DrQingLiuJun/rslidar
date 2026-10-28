# rslidar_ws 服务器部署

位置：`/home/user/carla-projects/rslidar_ws`，服务器：172.16.4.81。
原始压缩包保留在上级目录，SHA256：
`3574fc06adc700670b054a6fcd2b71b5969149719771826d396c42084c7fd2b7`。

## 运行管理

```bash
systemctl --user start rslidar
systemctl --user stop rslidar
systemctl --user restart rslidar
systemctl --user status rslidar
journalctl --user -u rslidar -f
```

已配置后台运行，当前未设置开机自启。前台调试使用 `./run-rslidar.sh`，先停止后台服务，避免端口冲突。

## 雷达连接

将各雷达的目标 IP 配置为服务器可达的网卡 IP。若雷达网络可以访问 172.16.4.81，则使用该地址。
服务器另有网卡 192.168.0.111/16，现场接线时需确认路由和雷达网段。

| 型号 | MSOP UDP | DIFOP UDP | 点云话题 | 检测结果 |
|---|---|---|---|---|
| RS80 | 6636 | 7736 | /rslidar_points_1 | /detection_result_1 |
| RS80 | 6639 | 7739 | /rslidar_points_2 | /detection_result_2 |
| RSBP | 6641 | 7741 | /rslidar_points_3 | /detection_result_3 |

配置文件：`src/rslidar_sdk/config/config_multi.yaml`。
雷达尚未配置发送时，MSOPTIMEOUT 表示等待数据。尚不能据此验证现场检测精度、标定或帧率。
ROS 2 使用 domain 82、CycloneDDS、仅本机发现；UDP 雷达接收仍监听网卡。
在其他终端查看话题时，先 source ROS 和 install 环境，并设置相同 ROS 环境变量，可参考 run-rslidar.sh。

## 软件环境与修改

ROS 2 Humble，Python 3.10 独立 .venv，PyTorch 2.3.1/CUDA 12.1，spconv-cu120 2.3.6。
OpenPCDet 源码位于 vendor/OpenPCDet，提交 233f849，CUDA 扩展针对 RTX 4090 的 sm_89 编译。
CUDA 12.1 编译工具独立安装，保留服务器现有 NVIDIA 驱动。

修复原项目绝对路径；将模型加入 ROS 安装资源；三路检测使用虚拟环境解释器；删除检测器中覆盖三路话题参数的重复固定订阅和发布器。
保留原始标定和检测阈值。源文件修改前备份位于 runtime/deployment/source-backup。
压缩包内旧 build/install/log 位于 runtime/deployment/original-*；服务器重新编译生成新的构建结果。
构建、依赖安装、验证记录位于 runtime/deployment。

## 验证结果（2026-10-28）

- ROS 包 rslidar_msg、rslidar_sdk 重新编译成功；OpenPCDet CUDA 扩展编译成功。
- 固定 numba 0.59.1、llvmlite 0.42.0、kornia 0.6.12 以解决运行时兼容性；完整依赖见 runtime/deployment/requirements-installed.txt。
- 三份模型均完整加载 319/319 参数；每份模型使用 20000 个合成点完成 GPU 前向推理，日志 gpu-smoke-v4.log。随机合成点未检出目标，此检查仅验证运行通路，不代表现场检测效果。
- rslidar 后台服务已运行，三路检测器初始化完成，六个 UDP 端口监听于 0.0.0.0。
- 三路点云各有一个发布器和一个检测订阅器，三路 detection_result 发布器已就绪。详见 graph-check.log。
- 雷达尚未配置发送，目前等待数据；现场点云、标定、检测效果及实时帧率尚待接入雷达后验证。
