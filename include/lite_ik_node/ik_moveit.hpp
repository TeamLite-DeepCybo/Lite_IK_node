// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// ik_moveit.hpp —— IK 后端的**第二种具体实现**（MoveIt `kinematics::KinematicsBase` 插件）
//
// ===========================================================================
// 为什么要有这个后端（与 ik_kdl.hpp 顶部的说明对照着看）
// ===========================================================================
//
// `ik_kdl.hpp` 那时的结论是"用 KDL 自己的链式求解器，不走 MoveIt 机制"，
// 理由是本层只要一条链的 IK，而 MoveIt 的插件机制要一整个 RobotModel。
// 这个理由在**依赖轻**上成立，但"KDL 够用"这一点**不成立**：
//
//   · KDL 的 `ChainIkSolverPos_LMA` **完全不看关节限位**。它是个最小二乘，
//     只要残差下降就返回"成功"——越限的解由上层 `clampToLimits()` 裁回限位，
//     于是最终残差可以到几厘米、姿态差 0.2 rad 以上，而状态仍报"有解"
//     （实测：`~/ik_query` 不给种子时，两臂都是 LIMIT_CLAMPED，
//      位置差 4.7~7.3 cm、姿态差 0.23 rad）。这种"差不多"的解会被当成
//      残差解下发，把手臂带到**错的地方**。
//   · pick_ik 只在与目标的差落在阈值内（3e-4 m / 1e-3 rad）时才返回解，并且
//     优化目标里带关节限位项 ⇒ 要么给一个**真的**到位、在限位内的解，要么
//     明确失败（HoldLast，原地保持）。这是本层想要的"fail-closed"语义。
//   · ⚠️ 如实说清楚：**种子给得好时（= 实测关节角）两个后端都是精确的**
//     （残差 ~5e-10），pick_ik 不是"更准"；它的价值是"不糊弄" + 解在限位内。
//     代价是慢：贴种子 0.1 ms、典型 4 ms、不可达时会把超时用满（见下）。
//   · pick_ik / trac_ik **只有** KinematicsBase 插件形态（没有独立的库可以像
//     KDL 那样直接 new 出来），所以要用它们就**必须**走 MoveIt 机制：
//     pluginlib + RobotModel + SRDF。这一步是绕不过去的，不是"顺手多拉依赖"。
//
// 于是本文件把"走 MoveIt 机制"的成本**封装在一条臂一个对象**里：
//   · 对上层暴露的回调与 KDL 后端**逐字同签名**（`IkSolveFn` / `FkFn`），
//     所以 `IkOrchestrator`（多种子 / 限位裁剪 / 残差前进 / NaN 拦截）
//     **一行不改**，随时可以在两个后端之间切换；
//   · 载不出来 / RobotModel 建不起来 ⇒ `build()` 返回 false，调用方**回退 KDL
//     并打 WARN**（fail-closed：绝不让一个半残的后端留在链路上）；
//   · **KDL 后端不删**：它是默认可用、零外部插件依赖的兜底。pick_ik 没装、
//     SRDF 找不到、插件 ABI 不匹配时，链路必须还能跑。
//
// ===========================================================================
// 三个"不这样就会错"的关键点（都实测过）
// ===========================================================================
//
// ① **关节顺序重排**：插件返回的 `getJointNames()` 顺序来自它自己的
//    RobotModel（按 URDF 关节树遍历），与本层 canonical/臂的关节顺序
//    **不保证一致**。直接把种子喂进去 = "关节 3 的解写到关节 5 上"，而每个
//    数值看起来都合理 —— 本项目反复强调的最难查的一类故障。所以 build() 里
//    建一张"我们的下标 → 插件下标"的置换表，逐名校验，名字对不上就 fail。
//
// ② **正解不能用插件的 getPositionFK 就完事**：pick_ik 1.1.2 的
//    `getPositionFK` 是**空实现**（反汇编出来就是 `xor eax,eax; ret`，恒返回
//    false 且不打任何日志）。如果只写 getPositionFK，症状是"IK 解出来了但残差
//    永远算不出来 ⇒ 编排器判 FAILED ⇒ 一切目标都保持上一帧"，而且日志里一句
//    错都没有。所以：先试 `getPositionFK`（trac_ik 之类是真实现的），
//    不可用就回退到 `RobotState` 正解（同一个 RobotModel，数值等价），
//    并且在 build() 里**只 WARN 一次**（不能每帧刷）。
//
// ③ **种子必须钳到关节限位**（这一条是真机事故之后加的）：pick_ik 遇到
//    "种子贴限位或越限"时会**把种子换成随机的合法位形**再解
//    （日志原文：`Initial guess exceeds joint limits. Regenerating a random valid
//    configuration.`）。而我们的编排器 `clampToLimits()` 会把解裁到限位**上**，
//    于是下一拍的种子正好落在限位上 ⇒ 每一拍都触发随机重启 ⇒ 解不出来 ⇒ 连续
//    多拍走"预算用尽 → 保持上一帧"。真机日志里崩溃前那 4 连发
//    "本周期 IK 时间预算已用尽" 就是这条路径（左臂某个关节被裁到限位后一路卡住）。
//    所以 ik() 在喂种子之前把它钳进 `[lower+ε, upper-ε]`：插件**永远**不会走到
//    随机重启，IK 的确定性也不再被一个 RNG 破坏。
//
// ④ **插件精度必须与编排器的判据对齐**：pick_ik 默认
//    `position_threshold = 1 mm`、`orientation_threshold = 1 mrad`，即它认为
//    "误差 1 mm 以内就算解出来了"。而 `IkOrchestrator` 判 `Exact` 的门是
//    0.5 mm —— 于是一个**完全可达**的目标会被判成 `RESIDUAL`（并且向上游发
//    `exact=false`）。实测：中性种子反解时残差 7.6e-4 m，正好卡在这个缝里。
//    但阈值也不能一路收紧到 1e-4：pick_ik **达不到阈值时不是给近似解，而是直接
//    返回 false**，于是"太紧"会把本来能解的位形推给编排器的增量前进路径
//    （实测右臂：种子偏 0.05 rad 时 1e-4 有 34/150 次失败）。所以 build() 把
//    它们设成 **3e-4 m / 1e-3 rad**（标准 MoveIt 参数名
//    `robot_description_kinematics.<group>.position_threshold`）：
//    解一定落在 0.5 mm/5 mrad 内 ⇒ 判 EXACT，同时实测 150/150 成功、最差 14 ms。
//
// ===========================================================================
// 谁负责什么（与 IkOrchestrator 的分工）
// ===========================================================================
//
// 本文件只负责"给定 (种子, 目标) 调**一次**插件"，以及"给定 q 算一次正解"：
//   · 限位裁剪、多种子、残差线搜索、NaN 拦截**都不在这里**（在 IkOrchestrator）；
//   · 但**解的有限性 / 长度**要在这里先挡一层：插件是外部 .so，返回 NaN 或长度
//     不对是可能的，而 `IkOrchestrator` 只在"求解器给过非法解"时才会拒绝线
//     搜索 —— 两层都挡才安全。
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "lite_ik_node/ik_backend.hpp"

// ⚠️ 只前置声明：本头文件刻意不 include <rclcpp/rclcpp.hpp>（把 ROS 依赖关在
//    .cpp 里，和 ik_kdl.hpp 把 KDL 关在 .cpp 里同一个理由）。
//    `std::shared_ptr<rclcpp::Node>` 作为**空**成员析构时不需要完整类型
//    （删除器在构造时被类型擦除），所以这里前置声明是安全的。
namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace lite_move_controller
{

/// 一条臂的 MoveIt 插件运动学（内部持有 RobotModel + 插件实例）。
///
/// 不可拷贝（持 pimpl + 插件实例），可移动（要能放进 `std::vector<ArmRuntime>`）。
class MoveItArmKinematics
{
public:
  /// 建链参数（来自节点参数与本包模型配置）。
  struct Options
  {
    /// 插件名，例如 `pick_ik/PickIkPlugin`、`trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin`。
    std::string plugin{"pick_ik/PickIkPlugin"};
    /// SRDF 里的 group 名（`left_arm_group` / `right_arm_group`）。
    std::string group_name;
    /// 目标位姿所在的坐标系（= 链的根，本机是 `world_root`）。
    std::string base_frame;
    /// 末端 link（与 KDL 后端的 tip **必须**是同一个，否则两个后端算的不是一个东西）。
    std::string tip_frame;
    /// 期望的关节顺序（本层 canonical / 臂顺序）。插件内部顺序不同会自动重排。
    std::vector<std::string> joint_names;
    /// 冗余臂搜索步长（取自 kinematics.yaml 的 `kinematics_solver_search_resolution`）。
    double search_discretization{0.005};
    /// 单次求解超时（秒，取自 kinematics.yaml 的 `kinematics_solver_timeout`）。
    double timeout_sec{0.05};
    /// 插件要挂靠的 rclcpp 节点。**空 = 自建一个内部节点**（离线测试/库使用者用）。
    ///
    /// ⚠️ 正常路径（move_controller_node）传的是**主节点的 shared_ptr**，前提是
    ///    那个 shared_ptr 已经存在 —— 即节点把"建臂"从构造函数挪到 `initialize()`
    ///    之后（`shared_from_this()` 在构造函数里会抛 `bad_weak_ptr`）。
    ///    复用主节点的好处：ROS 图里不再多出两条 "_ik_<group>" 节点，而且
    ///    pick_ik 的参数（`robot_description_kinematics.<group>.*`）可以直接用
    ///    `ros2 param set` 调 —— 每个 group 一个前缀，两条臂不会撞名。
    ///    插件会给这个节点**声明一批参数**（pick_ik 约 25 个），所以传主节点
    ///    意味着 `/lite_move_controller` 上会多出这些参数，属预期行为。
    std::shared_ptr<rclcpp::Node> node{};
    /// 插件自身的收敛阈值（**只对 pick_ik 生效**）。
    ///
    /// ⚠️ 取值不是拍脑袋的，是量出来的（见 ik_moveit.cpp 顶部 ③ 与下面的实测表）：
    ///   · pick_ik 默认 1 mm / 1 mrad 太松 —— 编排器判 `Exact` 的门是 0.5 mm，
    ///     可达目标会被判成 RESIDUAL；
    ///   · 收到 1e-4 又太紧 —— 种子稍有偏差时插件会**直接返回 false**
    ///     （实测：种子 ±0.05 rad 时右臂 150 次里失败 34 次，平均 13 ms/次，
    ///     最差撞 50 ms 超时），于是编排器只好去走 1 mm 的增量前进；
    ///   · 3e-4 m / 1e-3 rad 两头都满足：解一定落在 0.5 mm / 5 mrad 之内（判
    ///     EXACT），实测种子 ±0.05 rad 时 150/150 全成功、平均 4.1 ms、最差 14 ms。
    double position_threshold{3e-4};
    double orientation_threshold{1e-3};
  };

  MoveItArmKinematics();
  /// ⚠️ 析构/移动**必须在 .cpp 里定义**（那里 Impl 完整）。理由同 `ik_kdl.hpp`：
  ///    在头文件里即使写 `= default` 也会实例化 `unique_ptr<Impl>` 的删除器，
  ///    而 Impl 在此处不完整 ⇒ "invalid application of sizeof to incomplete type"。
  ~MoveItArmKinematics();
  MoveItArmKinematics(const MoveItArmKinematics &) = delete;
  MoveItArmKinematics & operator=(const MoveItArmKinematics &) = delete;
  MoveItArmKinematics(MoveItArmKinematics &&) noexcept;
  MoveItArmKinematics & operator=(MoveItArmKinematics &&) noexcept;

  /// 建模型 + 载插件 + 初始化。
  ///
  /// @param urdf_xml 完整 URDF 文本（与 KDL 后端、limit_guard 必须是**同一份**）
  /// @param srdf_xml 完整 SRDF 文本（提供 group 定义；插件靠它找到关节集合）
  /// @param error_out 失败原因（**必须**被调用方打出来，不允许静默回退）
  /// @return false ⇒ 调用方回退 KDL 后端（本对象此后 ready()==false）
  bool build(const std::string & urdf_xml,
             const std::string & srdf_xml,
             const Options & opt,
             std::string & error_out);

  bool ready() const noexcept {return ready_;}
  std::size_t dof() const noexcept {return dof_;}
  /// 插件的关节顺序（排障用：与本层顺序不一致时看这个）。
  const std::vector<std::string> & solverJointNames() const noexcept {return solver_joint_names_;}
  /// 本项目顺序（`Options::joint_names`）。
  const std::vector<std::string> & jointNames() const noexcept {return joint_names_;}
  const std::string & pluginName() const noexcept {return plugin_name_;}
  const std::string & groupName() const noexcept {return group_name_;}
  const std::string & baseFrame() const noexcept {return base_frame_;}
  const std::string & tipFrame() const noexcept {return tip_frame_;}
  /// 插件的 `getPositionFK` 可用吗？（pick_ik 1.1.2 是空实现 ⇒ false）
  bool pluginFkUsable() const noexcept {return plugin_fk_usable_;}
  const char * lastError() const noexcept {return last_error_.c_str();}

  /// 正解：关节角（**本项目顺序**）→ 末端位置（m）与旋转向量（rad），在 `base_frame` 下。
  bool fk(const std::vector<double> & q, Vec3 & position, Vec3 & rotation) const;

  /// 逆解一次（不做多种子/裁剪/线搜索；那些在 `IkOrchestrator`）。
  bool ik(const Vec3 & position, const Vec3 & rotation,
          const std::vector<double> & seed, std::vector<double> & q_out) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joint_names_;         ///< 本项目顺序
  std::vector<std::string> solver_joint_names_;  ///< 插件顺序
  std::string plugin_name_;
  std::string group_name_;
  std::string base_frame_;
  std::string tip_frame_;
  std::string last_error_;
  std::size_t dof_{0};
  bool ready_{false};
  bool plugin_fk_usable_{false};
};

/// 把 MoveIt 插件后端适配成 `IkOrchestrator` 需要的两个回调。
/// ⚠️ 与 `makeKdlSolveFn/makeKdlFkFn` 一样捕获**裸指针**：调用方必须在对象
///    **最终落位之后**再绑定（见 move_controller_node.cpp 里 buildArm 的注释）。
IkSolveFn makeMoveItSolveFn(const MoveItArmKinematics * arm);

FkFn makeMoveItFkFn(const MoveItArmKinematics * arm);

/// 本包 SRDF 文件名（组定义 `left_arm_group` / `right_arm_group`）。
inline constexpr const char * kSrdfFileName = "lite_000_asm.srdf";

/// 读 SRDF 文本。
///
/// 按顺序找：
///   ① `explicit_path`（节点参数 `srdf_path`，非空则**只**用它）；
///   ② `extra_candidates`（节点给：本包 share 目录下的同名文件）；
///   ③ 源码树回退 `src/lite_ik_node/config/model/<kSrdfFileName>`。
///
/// ⚠️ 找不到时返回**空串**而**不是**空 SRDF：没有 SRDF 就没有 group 定义，
///    RobotModel 里的 group 会不存在 ⇒ 插件初始化必然失败。让调用方明确回退 KDL，
///    比"用一份瞎编的 SRDF 硬跑"安全得多（后者会静默解到别的关节集合上）。
/// @param tried_out 非空时填入"找过的路径"，供调用方在 WARN 里说清楚
std::string loadSrdfText(const std::string & explicit_path,
                         const std::vector<std::string> & extra_candidates,
                         std::vector<std::string> * tried_out);

}  // namespace lite_move_controller
