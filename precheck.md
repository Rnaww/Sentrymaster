# 预检录检查说明

## 启动

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
cd scripts
./build.sh
./chassis.sh
```
## 临时测试内容

| Topic | 发布者 → 订阅者 | 当前测试内容 |
| --- | --- | --- |
| `/referee_data` | 测试输入→ 决策 | `game_time=1、sentry_posture=3` |
| `/chassis_data` | 测试输入→ 决策、导航 | `vx=1、vy=1 mm/s、wz=1 度/s` |
| `/odin1/odometry_highfreq` | 测试输入→ 决策、导航 | 零平移、单位位姿矩阵 |
| `/odin1/cloud_raw` | 测试输入→ 导航 | `PointCloud2`，点 `(1,1,1)`，坐标系 `odin1_base_link` |
| `/decision2nav` | 决策 → 导航 | `is_cross=false、aim_x=1、aim_y=1、move_v=1 mm/s` |
| `/decision2mcu` | 决策 → UART | `wz=1 度/s、sentry_posture=3` |
| `/navigation2mcu` | 导航 → UART | `vx=1、vy=1 mm/s` |

## 代码与配置位置

- 模拟测试：[test_input_node.py](src/sentry_test/sentry_test/test_input_node.py)；配置 [test_input.yaml](src/sentry_test/config/test_input.yaml)，决定模拟测试数据。
- 决策：[decision_node.py](src/sentry_decision_sys/sentry_decision_sys/decision_node.py)；发送固定测试值，不新增 YAML。
- 导航：[sentry_planner_node.cpp](src/sentry_planner/src/sentry_planner_node.cpp)；发送固定测试值，配置 [global_planning.yaml](src/sentry_planner/config/global_planning.yaml) 使用 `test_param`，启动时打印。
- 串口：[transfer.cpp](src/uartdriver/src/transfer.cpp)、[uart_driver_node.cpp](src/uartdriver/src/uart_driver_node.cpp)；配置 [driver_config.yaml](src/uartdriver/config/driver_config.yaml)。
- 当前未启动 UART