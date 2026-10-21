# Lite_IK_node

Lite 双臂的独立运动学包。它提供可复用的 C++ 运动学库和 ROS 2 节点，将连续末端
位姿目标转换为平滑的关节目标，供 `lite_move_controller` 使用。

## 原理

节点以 `/joint_states` 作为当前构型和连续求解种子。每个目标周期执行闭环微分逆解：

```text
位姿误差 e
   │
   ▼
v = Kp · e
   │
   ▼
q̇ = Jᵀ(JJᵀ + λ²I)⁻¹v + (I - J⁺J)q̇_null
   │
   ▼
速度/加速度/单步幅度限制 → 关节目标
```

阻尼最小二乘项在奇异位形附近限制关节速度；零空间项在不改变末端任务的方向上
把关节拉向限位中部。微分解残差超出阈值时，节点尝试 MoveIt
`KinematicsBase` 插件；插件不可用时使用 KDL 后端。兜底结果还会经过关节跳变
检查，避免冗余机械臂切换到相距过远的构型。

包内安装以下模型资产，因此运行链路不依赖 `lite_moveit`：

```text
config/model/lite_000_asm.urdf
config/model/lite_000_asm.srdf
config/kinematics.yaml
```

## 包内容

- `lite_ik_node`：独立 ROS 2 可执行节点。
- `lite_ik_node_kinematics`：导出的 C++ 静态库，包含 KDL 与插件后端。
- `include/lite_ik_node/kinematics_types.hpp`：共享运动学类型。
- `differential_ik.hpp`：微分 IK、阻尼和零空间控制。
- `ik_backend.hpp`：多种子、限位处理和残差分类。

为保持现有控制器 ABI，公共 C++ 类型暂时沿用
`lite_move_controller` 命名空间；文件和构建依赖已经归属本包。

## 依赖与构建

ROS 2 包依赖包括 `move_msgs`、KDL、Eigen、MoveIt Core、pluginlib、SRDF/URDF
解析库。`pick_ik` 是默认插件；若系统未安装，节点仍可使用 KDL。

```bash
cd <workspace>
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to lite_ik_node
source install/setup.bash
```

单独启动：

```bash
ros2 launch lite_ik_node lite_ik_node.launch.py
```

控制器的 `move_controller.launch.py` 默认会自动启动本节点。

## 接口

| 方向 | 默认话题 | 类型 |
|---|---|---|
| 输入 | `/lite_move_controller/task_target` | `move_msgs/msg/MoveTargetArray` |
| 输入 | `/joint_states` | `sensor_msgs/msg/JointState` |
| 输出 | `/lite_move_controller/joint_target` | `move_msgs/msg/MoveTargetArray` |

若输入已经是 `TARGET_JOINTS`，节点会校验长度、更新连续求解缓存并透明转发；
`TARGET_POSE` 则进入微分 IK 链路。

## 模型更新

机器人构型变化后，应把同一版本的已解析 URDF 和 SRDF 覆盖到
`config/model/`，同步检查 `config/kinematics.yaml` 中的 group、超时和搜索步长，
然后重新构建本包。控制器使用的动力学参数也应通过 `lite_dyid` 重新辨识。

## 验证

```bash
./build/lite_ik_node/test_ik_orchestrator
./build/lite_ik_node/test_ik_kdl
```

## 注意事项

- URDF、SRDF 的根链接、末端链接和关节组必须相互一致。
- 节点只发布关节目标，不直接向电机或 `lite_bridge` 发命令。
