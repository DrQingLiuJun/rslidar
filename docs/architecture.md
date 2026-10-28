# 架构与维护约定

## 职责分离

- rslidar_msg：驱动消息。
- rslidar_sdk：设备接收与 ROS 点云发布，内嵌 RoboSense rs_driver 解码核心。
- lidar_perception：模型推理、跟踪、坐标转换和 Marker 发布。

检测应用的 Python 模块：

| 模块 | 职责 |
|---|---|
| detector_node.py | ROS 参数、订阅、GPU 推理调度 |
| config.py | 按 YAML 所在位置解析数据集基配置 |
| dataset.py | 在线点云与 OpenPCDet 的适配 |
| preprocessing.py | 原项目的现场静态障碍过滤 |
| tracking.py | 最近邻关联、平滑、轨迹保留 |
| geometry.py | 四元数、安装位置和 TF 坐标计算 |
| visualization.py | 三维框、轨迹 ID、经纬度和航向文本 |

## 命名与路径

自有 Python 模块、脚本、配置使用小写和下划线。每路感知资源以 lidar_1、lidar_2、lidar_3 命名，端口独立保存在雷达配置中，不把端口或训练轮数写进资源名称。

_BASE_CONFIG_ 相对于模型 YAML 所在目录解析；权重和配置通过 ROS 包共享目录定位。入口为 scripts/run.sh；也可在加载 .cache/colcon/install/setup.bash 并激活 .venv 后直接运行 ROS launch。

产物集中在 .cache/colcon/{build,install,log}。驱动版本头和 CMake 配置写入构建目录，不再污染源码。

## 第三方代码与清理范围

OpenPCDet 固定到 Git 子模块提交。RoboSense C++ 接收和解码算法保留；精简为 ROS 2 构建，移除未使用的文档、演示和 ROS 1 文件，并调整生成文件位置。上游许可证保留，第三方文件名保持上游命名，方便对照更新。

已移除旧单路及重复多路检测脚本、历史启动入口、.bak、注释掉的历史实现、重复初始化、ROS 1 与重复 ROS 2 消息模板、旧打包工具、未使用的上游图片/教程/演示，以及源码中的生成文件。

旧版本通过 Git 提交保留；部署前服务器源码快照位于工作空间外的 archive 目录，项目内不继续堆积备份或历史安装日志。

退出流程使用仅设置标志的信号处理器，由主线程停止驱动后再关闭 ROS 上下文；重复 stop 时只回收仍可 join 的点云线程，避免历史版本停止服务时的异常退出。Python 节点也会检查上下文状态，避免重复 shutdown。
