# 可选内录

在工作空间 scripts 目录：`./simulate.sh record:=true` 或 `./chassis.sh record:=true`。默认不启动录制。

输出到工作空间 `recordings/run_日期_时间/`，包含 `telemetry.csv` 和 `bag/`。使用 `record_directory:=/自选目录` 更改输出位置。

CSV 记录实际速度、世界系加速度、转速与角加速度、导航输出、决策目标和姿态、裁判时间、任务名称。单位写在列名中。世界系为里程计 header 的坐标系；加速度先把车体系速度旋转到该世界系，再按消息时间戳差分，避免把小陀螺旋转误当成平移加速度。

首帧及时间断开后的加速度留空。微分结果可能有噪声，分析时应结合原始速度；没有把命令速度当成实测速度。

rosbag 保留导航里程计、速度输出、决策、裁判、底盘反馈、TF、时钟和推导的 `/recorded_acceleration`。默认不录高流量点云和重复的大路径消息。回放：`ros2 bag play recordings/对应目录/bag --clock`。
