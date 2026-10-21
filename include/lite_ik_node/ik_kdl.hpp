// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// ik_kdl.hpp —— IK 后端的**具体实现**（KDL，进程内）
//
// ===========================================================================
// 为什么用 KDL 而不是 MoveIt 的 KinematicsBase 插件
// ===========================================================================
//
// 本包保存并读取自己的解算器配置：
//
//   ✅ 复用配置：group → 关节链、base/tip link、`kinematics.yaml` 里的
//      search_resolution / timeout。
//   ❌ 不复用机制：MoveIt 的 `KinematicsBase` 插件要求一个完整的
//      `moveit::core::RobotModel`（URDF+SRDF+Bond+规划场景），
//      对本层"只要一个链式 IK"的需求来说是**数百倍**的依赖。
//
// 所以：**用 KDL 自己的链式求解器**（`kdl_parser` + `orocos-kdl`），
// 参数统一取自本包的配置。这也是"算力尽可能轻"的落地。
//
// ⚠️ 如果将来要换 `pick_ik` / `trac_ik`：它们**只有** KinematicsBase 插件形态，
//    那时才需要走 MoveIt 机制。届时只需在 `IkSolverSet` 里换一个实现，
//    `IkOrchestrator`（多种子/限位裁剪/线搜索/NaN 拦截）**一行不改**。
//
// ===========================================================================
// 求解策略（与 IkOrchestrator 的分工）
// ===========================================================================
//
// 本文件只负责"给定 (种子, 目标) 解一次"：
//   · `ChainIkSolverPos_LMA`：Levenberg-Marquardt，对冗余臂比 NR-JL 稳；
//   · 目标位姿从"base frame"变到"tip link frame"；
//   · 超时由 `IkOrchestrator` 的迭代次数间接约束（LMA 是定步数的）。
//
// **限位裁剪、多种子、残差线搜索、NaN 拦截都不在这里** —— 那些在
// `IkOrchestrator` 里统一做，这样换求解器时安全逻辑不变。
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "lite_ik_node/ik_backend.hpp"

namespace KDL
{
class Chain;
class ChainFkSolverPos_recursive;
class ChainIkSolverPos_LMA;
class ChainJntToJacSolver;
class JntArray;
}  // namespace KDL

namespace lite_move_controller
{

/// 一条臂的 KDL 运动学链（不可拷贝：内部持有求解器）。
class KdlArmChain
{
public:
  KdlArmChain();
  // ⚠️ 析构**必须**在 .cpp 里定义（那里 Impl 是完整类型）。写在头文件里
  //    即使写 `= default` 也会在头文件中实例化 unique_ptr 的删除器，
  //    于是报 "invalid application of sizeof to incomplete type"。
  ~KdlArmChain();
  KdlArmChain(const KdlArmChain &) = delete;
  KdlArmChain & operator=(const KdlArmChain &) = delete;
  // ⚠️ 必须显式给移动：pimpl(unique_ptr) 让类型**不可拷贝**，而用户声明的
  //    析构函数会**抑制**隐式移动构造 —— 于是 `std::vector<ArmRuntime>` 里
  //    放一个含 KdlArmChain 的结构体就编不过（错误信息是
  //    "use of deleted function ... move constructor"，与真实原因隔了好几层）。
  // ⚠️ 移动操作也**只声明**（不 `= default`）：`= default` 在头文件里同样会
  //    实例化 unique_ptr<Impl> 的删除器，遇到 incomplete type 就编不过。
  //    真正的 `= default` 放在 .cpp（那里 Impl 完整）。
  KdlArmChain(KdlArmChain &&) noexcept;
  KdlArmChain & operator=(KdlArmChain &&) noexcept;

  /// 从 URDF 文本建链。
  ///
  /// @param urdf_xml  完整 URDF 文本
  /// @param root_link 链的根（通常是 `world_root` 或臂的基座 link）
  /// @param tip_link  链的末端（例如 `left_gripper_tip_middle_link`）
  /// @param joint_names 期望的关节顺序（长度 = 链的自由度数）。
  ///        ⚠️ 必须与 canonical 顺序一致 —— 不一致会让"关节 3 的解写到关节 5 上"，
  ///        而且每个数值看起来都合理，是本项目反复强调的最难查的一类故障。
  /// @param eps        LMA 收敛容差
  /// @param max_iter   LMA 最大迭代次数
  /// @param error_out  失败原因
  bool build(const std::string & urdf_xml,
             const std::string & root_link,
             const std::string & tip_link,
             const std::vector<std::string> & joint_names,
             double eps, int max_iter,
             std::string & error_out);

  bool ready() const noexcept {return ready_;}
  std::size_t dof() const noexcept {return dof_;}
  const std::vector<std::string> & jointNames() const noexcept {return joint_names_;}
  const std::string & rootLink() const noexcept {return root_link_;}
  const std::string & tipLink() const noexcept {return tip_link_;}

  /// 正解：关节角 → 末端位置（m）与旋转向量（rad），在 `root_link` 下。
  bool fk(const std::vector<double> & q, Vec3 & position, Vec3 & rotation) const;

  /// 逆解一次（不做多种子/裁剪/线搜索；那些在 `IkOrchestrator`）。
  bool ik(const Vec3 & position, const Vec3 & rotation,
          const std::vector<double> & seed, std::vector<double> & q_out) const;

  /// 底层 KDL 链（供 `KdlDynamics` 绑定）。⚠️ 只在 `ready()` 后有效。
  /// 返回 `const void*` 而不是 `const KDL::Chain&`：本头文件刻意**不引入 KDL 头**
  /// （把 KDL 依赖限制在 .cpp 里），调用方在 .cpp 中 static_cast。
  const void * chainOpaque() const noexcept;

  /// 链上第 i 段的**子连杆名**（用于把 params.json 的连杆参数按名映射）。
  /// 越界返回空串。`i` 是 segment 下标（含 fixed segment）。
  std::string segmentChildLink(std::size_t i) const;

  /// segment 数量。
  std::size_t segmentCount() const noexcept;

  /// 6×n 几何雅可比（末端在**世界/基座系**，base 系角速度约定）。
  ///
  /// ⚠️ 这条路线已**逐列严格验证**：`dR/dq_j·Rᵀ = [ω_col_j]×`，与数值微分
  ///    一致到 1.9e-08（见 `test_dynamics.cpp` 的历史版本）。
  ///    pinocchio 侧的 `computeJointJacobians` 将来接入后应给出同一结果；
  ///    在那之前这里就是雅可比的**单一事实来源**。
  bool jacobian(const std::vector<double> & q, Jacobian & J) const;

  /// 工具帧相对链末端的固定偏移（`tip_link` 与 KDL segment 末端的差）。
  /// ⚠️ 若 `tip_link` 不在 URDF 的链上（名字打错），`build` 会失败而不是静默用别的 link。
  const char * lastError() const noexcept {return last_error_.c_str();}

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  std::vector<std::string> joint_names_;
  std::string root_link_;
  std::string tip_link_;
  std::string last_error_;
  std::size_t dof_{0};
  bool ready_{false};
};

/// 把 KDL 链适配成 `IkOrchestrator` 需要的两个回调。
///
/// 用法：
/// ```cpp
/// KdlArmChain chain;
/// chain.build(...);
/// IkOrchestrator ik;
/// ik.setSolver(makeKdlSolveFn(&chain), makeKdlFkFn(&chain));
/// ```
IkSolveFn makeKdlSolveFn(const KdlArmChain * chain);

FkFn makeKdlFkFn(const KdlArmChain * chain);

/// 从本包 `kinematics.yaml` 解析出的求解参数。
struct KinematicsParams
{
  std::string plugin;                 ///< 例如 kdl_kinematics_plugin/KDLKinematicsPlugin
  double search_resolution{0.005};    ///< 冗余臂的搜索步长（这里映射到 LMA 容差）
  double timeout_sec{0.05};           ///< 超时（映射到 LMA 迭代上限）
  /// 由 timeout 反算的迭代上限：LMA 每步约 0.1 ms，取保守值。
  int max_iter{200};
};

/// 从 YAML 文本里取某个 group 的运动学参数。
///
/// `kinematics.yaml` 的格式：
/// ```yaml
/// left_arm_group:
///   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
///   kinematics_solver_search_resolution: 0.005
///   kinematics_solver_timeout: 0.05
/// ```
/// 返回 false ⇒ 该 group 没有配置（调用方必须报错，**不要**静默用默认值）。
bool loadKinematicsParams(const std::string & yaml_text, const std::string & group,
                          KinematicsParams & out, std::string & error_out);

}  // namespace lite_move_controller
