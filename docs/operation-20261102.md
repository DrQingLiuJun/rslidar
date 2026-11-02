# 三路雷达检测修复与操作说明

服务器：172.16.4.81；账号：user。工作区：`/home/user/carla-projects/rslidar_ws`。检查日期：2026-11-02。

## 原因与修复

第二、三路无持续输出的主要问题是ROS点云传输接收缓冲不足，并非模型权重没有加载，也不是GPU推理死锁。两个进程调用栈位于`rmw_wait/rcl_wait`，表示等待输入。修复前三个检测进程的UDP数据套接字累计丢包分别为22443、25271、13366，系统`net.core.rmem_max`只有212992字节。三路大点云的分片接收因此不可靠。

修改内容：

- 新增`config/cyclonedds.xml`，要求DDS至少16MB接收缓冲；`scripts/env.sh`自动加载，保留外部CYCLONEDDS_URI覆盖方式。
- 新增系统`/etc/sysctl.d/90-rslidar-buffers.conf`，将rmem_max和wmem_max持久化为33554432字节。
- 检测节点BEST_EFFORT订阅深度由10改为1，保留最新点云，减少慢推理时旧帧积压。
- 检测框显示有效期由0.2秒改为0.6秒，匹配约3Hz输出间隔，减少RViz中框的闪烁。
- 新增`config/three_lidars.rviz`及`scripts/visualize.sh`。

没有修改模型权重、网络结构、分类阈值、坐标标定和目标跟踪算法。

修复后连续60秒：三路点云均10Hz，检测输出约3.05/3.06/3.12Hz，最大检测间隔约0.35秒。三个检测进程接收套接字丢包均为0。开启RViz后再测30秒，三路仍持续输出约3Hz，第二、三路有非空检测消息。5项项目回归检查通过。模型输出频率不等于检测准确率；本次没有人工标注精度评估。

## 地址自动恢复

接收网口`enx2c16dba65bb0`使用NetworkManager连接“有线连接3”，UUID为`639e4a07-7875-3dc1-ae8f-d68fa068e5b7`，其自动连接已启用。已在原地址之外持久添加`172.16.5.103/32`，保留原`192.168.0.111/16`、管理地址及默认路由。使用/32避免新增整个172.16.5.0/24的直连路由。

以后服务器重启或该网口重连，连接配置会恢复103地址，无需再次抓包寻找。没有为验证而重启整台服务器；已核对活动地址、持久连接属性和自动连接属性。

检查命令：

```bash
ip -br address show enx2c16dba65bb0
nmcli -g ipv4.addresses,connection.autoconnect connection show 639e4a07-7875-3dc1-ae8f-d68fa068e5b7
```

雷达36/39/41分别向103的6636/6639/6641发送点云，DIFOP端口为7736/7739/7741，原项目端口没有改变。链路仍需有线缆连接、雷达供电和交换网络可达。

## 启动、停止和查看日志

使用user账号，在服务器终端执行：

```bash
cd /home/user/carla-projects/rslidar_ws
systemctl --user start rslidar.service
systemctl --user status rslidar.service
journalctl --user -u rslidar.service -f
```

退出日志跟随按Ctrl+C，不会停止后台检测。停止检测：

```bash
systemctl --user stop rslidar.service
```

后台检测启动后不要再同时运行`./scripts/run.sh`，避免启动第二套驱动占用相同端口。此次只持久化网络及缓冲配置，没有更改检测服务的开机自启设置；按上述命令自行启动检测即可。

## RViz可视化

使用服务器本机桌面或远程桌面（向日葵/ToDesk等），在图形桌面的终端运行：

```bash
cd /home/user/carla-projects/rslidar_ws
./scripts/visualize.sh
```

脚本加载项目ROS域82、DDS环境和配置。普通SSH文字终端没有图形显示能力；请在服务器图形桌面运行，或自行配置图形转发。当前已在服务器桌面打开一个RViz窗口，无需重复打开。

配置使用Fixed Frame `map`，三路点云分别为红、绿、蓝色；左侧勾选对应`Lidar N points`和`Lidar N detections`即可单独查看各路。点云话题为`/rslidar_points_1`至`_3`，检测框与文字为`/detection_result_1`至`_3`，静态TF来自现有标定。空检测消息表示该帧没有通过现有检测与跟踪筛选的目标，不能据此判断节点停住。若现场无检测目标，仍可看到点云。

鼠标左键旋转、滚轮缩放、Shift+左键平移；配置已经把初始视点放到三台雷达的现场位置附近。若RViz无法连接，优先用此脚本而非直接从其他ROS环境启动rviz2。

## 回滚记录

备份位置见本目录`deployment.json`，服务器备份前缀为`/home/user/carla-services/backups/rslidar-transport-20261102-`。旧网络地址属性与旧sysctl值也保存在该备份中。恢复代码应先停止雷达服务；不要盲目覆盖其他服务或恢复整个服务器网络。

证据文件：`live-verification.json`、`rviz-load-verification.json`、`socket-drops-after.json`、`final-verification.json`。RViz进程、OpenGL日志及话题数据已核验；桌面截图导出被自动审批拒绝，因此没有导出截图或做截图目视验收。
