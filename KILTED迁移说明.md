# Kilted 工作空间

目标电脑：`blue@192.168.2.62`，Ubuntu 24.04、ROS 2 Kilted、Gazebo Ionic。

| 工作空间 | 当前适配分支 | 保留的 Humble 基线 |
| --- | --- | --- |
| `/home/blue/SentryMaster2026` | `kilted-rna` | `7566625` |
| `/home/blue/SentryMaster2026-nav2` | `kilted-nav2` | `749ea44` |

两个目录均是独立 Git 仓库，不依赖原电脑的 worktree 路径。原电脑保留 Humble 的两套框架分支，共用决策修复同步到四个分支；同步的 Kilted 工作目录分别为 `/home/wendyy/SentryMaster2026-kilted` 和 `/home/wendyy/SentryMaster2026-nav2-kilted`，原电脑仍是 Humble 环境，Kilted 目录用于编辑、保存与部署，不复用本机的 Humble 编译产物。迁移基线及完整历史另存于两台电脑的 `sentry-kilted-migration-20261016/`。代码仓库为 https://github.com/Rnaww/Sentrymaster.git ，默认分支 `kilted-rna`；另外保留 `kilted-nav2`、`sentrymaster-rna` 和 `nav2-experiment`。`sentry-kilted-migration-20261016/` 只是工作空间外的备份与检查资料目录，不是 ROS 包，也不需要放进 `src` 或随程序启动。

## 使用

新终端先加载 Kilted，再进入所选工作空间的 `scripts` 目录。两套框架分别运行，避免在同一 ROS domain 中同时启动比赛节点。

```bash
source /opt/ros/kilted/setup.bash
cd ~/SentryMaster2026/scripts
# Nav2 改为 cd ~/SentryMaster2026-nav2/scripts
./build.sh
./simulate.sh
```

通过 SSH 且没有图形会话时，使用 `./simulate.sh gui:=false rviz:=false`；默认界面启动适用于目标电脑的桌面终端。

实车启动使用 `./chassis.sh`；内录使用 `./simulate.sh record:=true` 或 `./chassis.sh record:=true`。脚本保留原来的 `cd ..`、colcon、source 和 launch 写法。不要复用 Humble 的 build/install/log，目标机已从源码重新编译。

实车总 launch 已启用 Odin 与 UART，`./chassis.sh` 同时启动传感器、串口、定位转换、导航和决策。实车总启动不打开 Odin 自带的 RViz，方便 SSH 和自启动；单独启动 Odin launch 时仍默认打开 RViz，可用 `rviz:=false` 关闭。串口沿用 `src/uartdriver/config/driver_config.yaml` 的端口、序列号和波特率配置；缺少串口设备时，UART 会按原有实现报错退出，接车前需核对实际设备和权限。缺少定位或裁判输入时保持停车。软件启动与仿真验证不能替代接设备后对 Odin、电控、串口权限、真实运动方向与速度的检查。

## 必要的适配

- 安装 Kilted 的 Nav2、ros_gz、py_trees、PCL 及 OMPL 等依赖；修正 package.xml 中失效的 ROS1、Classic Gazebo 依赖，并补充实际使用的依赖。
- Odin 主驱动设置基于 pkg-config 检测目录的运行时库搜索路径，解决海康 MVS 的旧 libusb 抢先加载、缺少 `libusb_interrupt_event_handler` 导致启动退出的问题。只修改该可执行文件的库加载设置，保留 MVS 安装及环境。
- UART 沿用现有串口配置和协议；修正读取超时后复用旧缓冲内容、退出时接收线程未回收的问题，不增加模拟串口或跳过串口发送。
- Nav2 自定义规划插件弱引用父节点，避免父节点与插件相互持有。规划插件改用带取消检查回调的接口，按 Kilted 的分类异常头文件编译；行为树使用 BehaviorTree.CPP 4 格式。
- 配置新版进度检查器参数，保留当前桥接节点使用的 Twist：`controller_server.enable_stamped_cmd_vel: false`。只启动当前使用的 NavigateToPose 导航器，内置 BT 插件由 Nav2 自动加载。
- Gazebo 使用官方 `ros_gz_sim/gz_server.launch.py` 原生服务器、`create` 和 `ros_gz_bridge`。资源通过包安装位置与 `GZ_SIM_RESOURCE_PATH`、`GZ_SIM_SYSTEM_PLUGIN_PATH` 获取。
- 新增的 `sentry_gazebo` 只提供 Ionic 平面运动接口，替代无法在 Kilted 加载的 Classic planar_move 插件。它接收原有车体系 Twist，只接管平移和 yaw，保留垂直运动、重力及碰撞；车体、轮子、场地网格仍使用原模型。不是电控驱动，也不参与实车启动。
- Ionic 里程计使用完整车体姿态及三维车体系速度；仿真桥接据此产生原 Odin 接口输入。实车定位转换、任务点、决策顺序、A*/MINCO、控制参数与右手坐标约定不作迁移性改写。
- GUI 配置基于 Gazebo Sim 9 随安装提供的官方 `gui.config`，使用与现有材质兼容的 Ogre 和全场俯视相机。GUI 与 RViz 可以关闭，不影响比赛链路。

## 检查记录

完整结果和当前轨迹见 `导航框架对比.md`。每个工作空间的 `stress_results/zero_wz_verified_20261017/` 保存当前禁转修复的整场结果、原始内录和角速度核对，`stress_results/kilted_native_record_verified_20261016/` 保存实际 rosbag、CSV 和通信检查结果，`stress_results/kilted_real_launch/` 保存无硬件实车模式检查。生成物按原有 gitignore 排除。

公共检查日志位于 `/home/blue/sentry-kilted-migration-20261016/`，包括编译、静态检查、MINCO 边界/坐标/取消回调、自由落体物理对照、GUI 与 RViz 启动退出检查。原生物体与平面运动接口物体的自由落体实测加速度均为约 -9.81 m/s²，接口不额外施加重力。原代码已有的 CMake 弃用及编译警告保留，没有为迁移进行无关格式改写。

版本依据：[Nav2 Kilted 迁移说明](https://docs.nav2.org/rolling/configuration_and_development/migration_guides/jazzy/Jazzy/)、[Gazebo 与 ROS 配套版本](https://gazebosim.org/docs/ionic/ros_installation/)、[Classic 到 ros_gz 的迁移说明](https://gazebosim.org/docs/all/migrating_gazebo_classic_ros2_packages/)。

Odin 主驱动另在两套工作空间的普通交互环境下实际启动检查通过：USB SDK 初始化、设备热插拔监听、里程计/点云/IMU 发布者注册及正常退出均通过。目标机当前没有 `2207:0019` Odin 设备，因此收到的数据为零，尚未验证实物取流。`senddepth: 0` 与 `sendreprojection: 0` 下，两个辅助节点按配置正常退出。结果与完整日志位于各自 `stress_results/kilted_odin_launch_fixed/`；两套工作空间的 12 个 C++ 可执行文件也通过 `ldd -r` 重定位符号检查。

实车总启动检查保存于 `stress_results/kilted_hardware_enabled/`：Odin 主驱动实际启动并监听 USB；无裁判输入时导航、决策输出为零。UART 在无物理端口时按原实现报错退出；另用虚拟串口验证只发送且无反馈时仍可发送、`vx/vy/wz` 和姿态报文字段、CRC16、裁判及底盘报文接收、正常退出。虚拟串口路径仅保存在忽略的检查资料中，没有写入实车 YAML。

Nav2 另用外部静止 Odin 测试消息检查实车总 launch，结果位于 `stress_results/kilted_hardware_enabled_with_pose/`。已知限制：目标机官方 Nav2 在完全没有定位、仍等待初始 TF 时直接 Ctrl+C，`planner_server` 会在 `Costmap2DROS` 析构阶段异常退出；调试记录在公共资料目录的 `planner-shutdown-gdb.log`。这项关闭异常尚未修复，不能把无定位检查记为全部通过。没有在生产代码中伪造 TF 或定位。
