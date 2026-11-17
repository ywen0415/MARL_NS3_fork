# 实机部署运行依赖清单（初稿）

## 目的与范围

本清单记录当前仓库中，策略运行入口依赖哪些代码、模型、ROS 接口和外部软件，并区分实机侧、中心协调侧和仿真/研究用途。

已确认的目标拓扑：每台机器人各有一台 Jetson，其中一台机器人上的 Jetson 同时运行中心协调器。中心协调器汇集多台机器人状态、地图并分配 Nav2 目标；每台机器人自己的 Jetson 运行该车的传感器、底盘、SLAM 和 Nav2 节点。

本文件是基于仓库源码的静态清单，不代表已完成 Jetson 构建、实机联调或安全验证。尚未掌握的硬件信息在“待确认项”中列出，不按猜测填写。

## 当前版本基准

- 发布名：`MR3-RL-POLICY-v1.1.0`。
- 发布清单记录策略权重 SHA256：`6d4c51ba6f96737ccd89393c9738a888acb82606770483f97dd562051ee01d5d`。
- 当前策略接口按三台机器人 `tb1`、`tb2`、`tb3` 编写，地图编码分辨率为 0.05 m，任务 ROI 固定为 `[-5.8, -5.6, 7.25, 5.3]`。
- 历史 ROS/Gazebo 参考环境为 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic；这不是 Jetson 实机环境认证。
- `README.md` 和 `docs/SETUP.md` 明确限定：CPU 回放只验证已记录输入下的策略推理，不等于完整 ROS 导航或实机验证；当前发布没有硬件部署认证。

## 策略当前运行链

当前候选的 RL04 ROS 入口为 `workspace/rl04/ros_headquarters.py`。它继承 `workspace/rl01/ros_policy.py` 中的 ROS 适配器，并使用 `workspace/src/multi_robot_exploration/multi_robot_exploration/` 下的策略与分配实现。

| 仓库路径 | 当前作用 | 实机清单判断 |
|---|---|---|
| `workspace/rl04/ros_headquarters.py` | RL04 ROS 策略入口；加载权重、创建中心 ROS 节点、记录推理及 Nav2 Action 过程 | 入口逻辑需保留或整理成部署入口；当前还耦合研究校验与记录代码 |
| `workspace/rl01/ros_policy.py` | ROS 适配基类；接收地图、里程计、TF，跟踪 Nav2 目标生命周期 | 现有适配代码的依赖来源；整理前不可直接删 |
| `workspace/src/multi_robot_exploration/multi_robot_exploration/allocation.py` | 机器人状态、候选分配和任务状态管理 | 核心运行逻辑候选 |
| `.../frontier.py`、`.../joint.py` | Frontier 候选与联合动作/掩码逻辑 | 核心运行逻辑候选 |
| `.../observations.py` | 将收到的地图和机器人状态编码为策略输入 | 核心运行逻辑；输入格式和 ROI 固定，改动会改变策略接口 |
| `.../ppo.py` | ActorCritic、权重加载和动作选择 | 核心运行逻辑；需保留兼容权重的结构 |
| `.../control.py` | 现有 ROS/Nav2 控制适配，包括状态订阅与 NavigateToPose 请求 | 多处运行适配代码在此；其常规 `main()` 启动传统分配器，不能误当成 RL04 策略入口 |
| `.../evaluation.py`、`.../rl_ledger.py` | 事件评价和审计记录 | 当前运行代码会导入/调用；是否从精简部署包中移出，需在保持必要运行行为后再决定 |
| `policy/checkpoint-00512.pt` | 发布策略权重；本地文件哈希与发布清单所记哈希一致 | 中心协调器需要的模型文件 |

> `...` 表示 `workspace/src/multi_robot_exploration/multi_robot_exploration/`。

### 当前入口的研究代码耦合

`workspace/rl04/ros_headquarters.py` 目前还会导入：

- `workspace/rl02/diagnose.py`：推理记录中的观测信息整理。
- `workspace/rl02/maps.py`：哈希和记录相关辅助函数。
- `workspace/rl04/train.py`：checkpoint 完整性校验函数。
- `workspace/rl01/ros_policy.py`：该文件模块级导入 `workspace/rl01/completion.py`，并带有另一套默认 checkpoint 校验逻辑。
- 根目录 `runs/selection.json`，以及被选 checkpoint 旁边的 `summary.json`、`process-exit.json`：RL04 加载器会读取这些文件来验证所选模型的来源和训练结果。

因此，**当前入口不能只和 `policy/checkpoint-00512.pt` 一起单独拷走**。若目标是精简部署包，需要新增或整理部署加载器：保留必要的权重身份、模型结构和配置校验，同时解除对训练目录、实验诊断模块和原始运行记录路径的依赖。校验规则必须按发布的 512 步 checkpoint 设计；不要误用 RL01 默认加载器中针对 1024 步开发 checkpoint 的条件。

当前 `multi_robot_exploration/setup.py` 注册的 `control` 命令指向常规传统控制入口，并没有把 RL04 入口注册成独立 ROS 可执行程序。部署包需要明确的 RL 策略启动命令/launch。

## 中心协调器和各车接口

### 中心协调 Jetson

策略适配器当前使用以下 ROS 接口：

| 接口 | 当前代码期望 | 来源/用途 |
|---|---|---|
| `/merge_map` | `nav_msgs/OccupancyGrid`，frame 为 `merge_map` | 合并地图；策略候选和观测输入 |
| `/{robot}/map` | `nav_msgs/OccupancyGrid` | 各车本地地图 |
| `/{robot}/odom` | `nav_msgs/Odometry` | 机器人位姿和速度状态 |
| `/{robot}/tf`、`/{robot}/tf_static` | `tf2_msgs/TFMessage` | 各车坐标变换 |
| `/{robot}/cmd_vel` | `geometry_msgs/Twist` | 当前适配器订阅并记录运动状态；不是策略的 Nav2 目标接口 |
| `/{robot}/navigate_to_pose` | Nav2 `NavigateToPose` Action server | 中心协调器向每台机器人发送导航目标 |

其中 `{robot}` 当前固定为 `tb1`、`tb2`、`tb3`。真实机器人是否能直接提供这些名称、消息和 frame，尚未确认；后续可能通过 ROS remap、namespace 或单独适配节点对接。

### 每台机器人 Jetson

按上述接口，各车侧至少要提供本地传感器/底盘驱动、SLAM 和 Nav2，并让其地图、里程计、TF 与 NavigateToPose Action 在 ROS 网络中可被中心节点访问。仓库没有包含这几台真实底盘的硬件驱动，也没有 Jetson 间 ROS 网络发现配置。

`workspace/src/merge_map/merge_map/merge_map.py` 是仓库里的地图合并节点：订阅 `/tb1/map` 等本地地图，按显式 `map_poses` 参数变换后发布 `/merge_map`。按当前集中协调拓扑，它可运行在中心 Jetson；地图外参/位姿参数必须来自实际标定，当前仿真配置不能直接视为实机标定结果。

## 软件依赖

### 策略中心侧候选依赖

- ROS 2 Python：`rclpy`。
- ROS 消息与接口：`nav_msgs`、`geometry_msgs`、`nav2_msgs`、`action_msgs`、`tf2_msgs`、`tf2_ros`、`tf2_geometry_msgs`、`unique_identifier_msgs`。
- Python 科学计算与推理：NumPy、SciPy、PyTorch。
- 当前代码/历史配置使用 Ubuntu 22.04、ROS 2 Humble；`docs/SETUP.md` 记录的研究环境还包含自带修改的 `slam_toolbox` 和 `nav2_bringup` 版本。Jetson 上的 Python/PyTorch/ROS 二进制兼容性尚未验证。

### 每车侧候选依赖

- ROS 2、真实传感器和底盘驱动。
- `slam_toolbox`（若继续使用仓库当前 SLAM 路线）。
- Nav2、Nav2 bringup、各车导航参数。
- TF 和导航所需 ROS 消息/Action。

具体是否使用仓库内的 `workspace/src/slam_toolbox`、`workspace/src/nav2_bringup`，或换成 Jetson 系统上安装的匹配版本，需在核对代码差异和目标 ROS 版本后决定。

## 当前可排除的仿真/研究内容

以下内容不应进入实机启动依赖，但原仓库应保留，以便仿真复现和算法追溯：

- Gazebo Classic 世界、模型和仿真专用 launch。
- `workspace/src/base01_diff_drive`：CMake 明确依赖 Gazebo/Gazebo ROS，是 Gazebo 插件，不是真实底盘驱动。
- `workspace/src/multi_robot` 中依赖 `gazebo_ros_pkgs`、`turtlebot3_gazebo` 的仿真启动部分。
- Gazebo harness、训练、训练环境、仿真回放、绘图和离线评测代码（包括 `workspace/gz01`、`workspace/rl01..04` 中未被运行入口调用的部分）。
- 与部署运行无关的 `runs/` 训练历史、比较报告和证据文件。注意：当前 RL04 loader 仍显式读取部分 selection/summary/process-exit 文件；解除这项耦合前，不能整体排除这些具体文件。

## 下一步整理边界

1. 先按入口调用链确认完整的直接导入、间接导入和运行时文件读取关系；本清单中的“候选”标记在闭合追踪前不作为最终删除依据。
2. 给 RL 策略建立独立的部署启动入口，避免误启 `control` 传统分配器。
3. 把策略权重和必要配置显式作为部署参数，替换对 `runs/` 内研究文件路径的隐式依赖。
4. 将中心策略/地图合并与每车驱动/SLAM/Nav2 的安装清单分开。
5. 保持策略输入编码、候选规则、checkpoint、目标分配逻辑不变；接口适配和硬件参数单独记录。

以上只规划代码和依赖整理，不意味着可直接连接实机运行。真实网络、传感器、底盘控制和急停仍需在硬件确定后单独验证。

## 待确认项

这些信息不影响当前仓库静态依赖盘点，但在形成最终 Jetson 安装/启动清单前必须补齐：

- Jetson 的具体型号及 JetPack/L4T 版本。
- 每台机器人使用的底盘、雷达型号和真实 ROS 驱动包。
- 实机 ROS 发行版、各车当前发布的话题/Action 名称、frame 名和 QoS。
- 各车本地 SLAM/Nav2 是继续用仓库修改版，还是使用目标系统安装版。
- `map_poses`（本地地图到 `merge_map` 的变换）如何标定并由谁提供。
- 要部署的机器人数量是否固定为三台；当前策略实现和发布配置按三台机器人编写。
