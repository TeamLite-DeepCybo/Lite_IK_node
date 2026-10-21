// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// ik_backend.hpp —— 逆解后端接口 + **限位感知 / 最近可达点** 的纯逻辑部分
//
// ===========================================================================
// 为什么把接口与"最近可达点"逻辑抽出来（而不是塞进 KDL 适配器）
// ===========================================================================
//
// 具体的数值求解器（KDL / pick_ik / trac_ik）是**可替换**的，而且它们的
// 共同短板是同一个：**无法返回"最近可达点"**。
//
//   · KDL `searchPositionIK` 只有成功/失败，失败时不给任何解；
//   · 7 自由度冗余臂上它**局部收敛**，种子差就失败，但目标其实可达。
//
// 用户明确要求：**无解时不要停止下发**，而是"规划到对应终端残差的解然后下发"。
// 这段逻辑与具体求解器无关 —— 它只需要"给我一个(种子, 目标)能不能解出来"，
// 所以放在这里可以被穷举单测（用一个**假的**求解器回调）。
//
// ===========================================================================
// 三层分类（设计文档 §4，三个**正交轴**）
// ===========================================================================
//
//   轴 A `SolverOutcome`：数值求解器给出什么
//         Solved         —— 成功且解在限位内
//         LimitClamped   —— 给了解但越限（我们裁到限位）
//         Failed         —— 失败 / 超时 / NaN
//
//   轴 B `TaskResidual`：任务目标打满了没
//         Exact          —— 残差在阈值内
//         Residual       —— 有可用解但残差超阈（目标在工作空间外一点点）
//
//   轴 C `IkAction`：本帧实际发什么
//         SendSolved / SendResidual / HoldLast
//
// ⚠️ **`Residual` 与 `Failed`/`HoldLast` 的本质区别**（最容易混的一对）：
//    · `Residual`   = 我**手上有解**，那个解**安全**，只是任务没打满 → 位置照常前进；
//    · `Failed`/`HoldLast` = 我**手上没有合法新解** → 位置**原地保持**，
//      **绝不用未验证的解去动关节**。
//
//   所以"NO_SOLUTION"这个词只作为人类可读的说明，**不作为状态字段**。
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "lite_ik_node/kinematics_types.hpp"

namespace lite_move_controller
{

/// 轴 A：数值求解器的结果。
enum class SolverOutcome : std::uint8_t
{
  Solved = 0,        ///< 成功，解在限位内
  LimitClamped = 1,  ///< 给了解但越限 → 已裁到限位
  Failed = 2,        ///< 失败 / 超时 / NaN / 所有种子都失败
};

/// 轴 B：任务残差。
enum class TaskResidual : std::uint8_t
{
  Exact = 0,
  Residual = 1,
};

/// 轴 C：本帧动作。
enum class IkAction : std::uint8_t
{
  SendSolved = 0,
  SendResidual = 1,
  HoldLast = 2,
};

inline const char * solverOutcomeName(SolverOutcome o) noexcept
{
  switch (o) {
    case SolverOutcome::Solved:       return "SOLVED";
    case SolverOutcome::LimitClamped: return "LIMIT_CLAMPED";
    case SolverOutcome::Failed:       return "FAILED";
  }
  return "UNKNOWN";
}

inline const char * taskResidualName(TaskResidual r) noexcept
{
  return (r == TaskResidual::Exact) ? "EXACT" : "RESIDUAL";
}

/// IK 配置。
struct IkConfig
{
  /// 位置残差阈值（m）：超过就不算 `Exact`。
  double residual_pos_m{0.0005};
  /// 姿态残差阈值（rad）。
  double residual_rot_rad{0.005};
  /// 限位余量（rad）：解落在这个余量内就算"贴限位"（`saturated_joints`）。
  double limit_margin{0.087};            // ≈5°
  /// 最近可达点"增量前进"的**起始步长**（m）。
  ///
  /// ⚠️ 实测标定（本机 7 轴臂 + KDL/LMA）：可靠单步只有 **1 mm**；
  ///    5 mm 就已经解不出来（收敛域很窄），1.7 m 完全不可能。
  ///    所以起点取 1 mm，并由 `walk` **自适应**（成功翻倍 / 失败减半）。
  double walk_step_m{0.001};
  /// 自适应步长的上限（m）：连续成功后最多放大到这里。
  double walk_step_max_m{0.02};
  /// 自适应步长的下限（m）：减到这里还失败就停下（再小没意义）。
  double walk_step_min_m{0.0001};
  /// 增量前进的**步数上限**（防病态输入炸时间）。
  ///
  /// 含义：走满这么多步还没到头就停下，返回已到达的最远可行点。
  /// 代价是"残差解可能离最远可达点还有距离"，但那是**保守方向**，
  /// 而且上游本来就可以用 `EeResidual` 迭代逼近（每周期前进一段）。
  /// 默认 64 步 × 1~20 mm ≈ 每周期最多前进 1.3 m，失败路径耗时 ≤ ~7 ms。
  std::size_t max_walk_steps{64};
  /// 是否允许用"限位裁剪解"作为残差解（关闭时贴限位的解也进 FAILED/HOLD_LAST）。
  bool allow_limit_clamped{true};
};

/// 一次求解的请求。
struct IkRequest
{
  /// 目标位置（在该臂的 base frame 下）
  Vec3 position{0, 0, 0};
  /// 目标姿态（旋转向量）
  Vec3 rotation{0, 0, 0};
  /// 种子关节角（长度必须 = 关节数）
  std::vector<double> seed;
  /// 上一帧的解（用于 `HoldLast`）
  std::vector<double> last_solution;
  bool has_last_solution{false};
};

/// 一次求解的结果。
struct IkResult
{
  SolverOutcome outcome{SolverOutcome::Failed};
  TaskResidual residual_kind{TaskResidual::Residual};
  IkAction action{IkAction::HoldLast};

  /// 关节解（`action != HoldLast` 时有效）
  std::vector<double> joints;

  /// 任务残差（target − 实际可达点）：位置（m）与旋转向量（rad）
  Vec3 position_residual{0, 0, 0};
  Vec3 rotation_residual{0, 0, 0};

  /// 被限位夹住的关节下标
  std::vector<std::size_t> saturated_joints;

  bool ok{false};                 ///< `action != HoldLast`（即"有可用解"）
  std::uint32_t iterations{0};
  std::uint32_t seed_tries{0};
  std::string note;               ///< 人类可读说明（"目标不可达：位置差 0.003 m"）

  double positionResidualNorm() const noexcept
  {
    return std::sqrt(position_residual[0] * position_residual[0] +
                     position_residual[1] * position_residual[1] +
                     position_residual[2] * position_residual[2]);
  }
  double rotationResidualNorm() const noexcept
  {
    return std::sqrt(rotation_residual[0] * rotation_residual[0] +
                     rotation_residual[1] * rotation_residual[1] +
                     rotation_residual[2] * rotation_residual[2]);
  }
};

/// 数值求解器的抽象：**给定种子与目标，能不能解出来**。
///
/// 返回 true 且填充 `joints` = 成功。返回 false = 失败（不给解）。
/// ⚠️ 实现方**不要**在这里做限位裁剪、重试、线搜索 —— 那些都在本文件里统一做，
///    这样换求解器（KDL → pick_ik）时那套安全逻辑**一行不改**。
using IkSolveFn = std::function<bool(const Vec3 & position, const Vec3 & rotation,
                                     const std::vector<double> & seed,
                                     std::vector<double> & joints)>;

/// 正解回调：给定关节角，算末端位置与旋转向量（用于残差与线搜索）。
using FkFn = std::function<bool(const std::vector<double> & joints,
                                Vec3 & position, Vec3 & rotation)>;

/// 位置 + 旋转向量的误差（简单相减即可：旋转向量在小幅下是线性空间）。
inline void poseError(const Vec3 & target_p, const Vec3 & target_r,
                      const Vec3 & actual_p, const Vec3 & actual_r,
                      Vec3 & dp, Vec3 & dr) noexcept
{
  for (std::size_t i = 0; i < 3; ++i) {
    dp[i] = target_p[i] - actual_p[i];
    dr[i] = target_r[i] - actual_r[i];
  }
}

/// 限位裁剪（返回是否发生了裁剪）。
inline bool clampToLimits(std::vector<double> & q, const JointLimits & lim,
                          std::vector<std::size_t> * clamped = nullptr) noexcept
{
  if (q.size() != lim.size()) {return false;}
  bool any = false;
  for (std::size_t j = 0; j < q.size(); ++j) {
    if (q[j] < lim.position_min[j]) {
      q[j] = lim.position_min[j]; any = true;
      if (clamped) {clamped->push_back(j);}
    } else if (q[j] > lim.position_max[j]) {
      q[j] = lim.position_max[j]; any = true;
      if (clamped) {clamped->push_back(j);}
    }
  }
  return any;
}

/// IK 编排器：把"可替换的数值求解器"包成"**永远给出可用解**"的接口。
///
/// 流程（`solve()`）：
///   1. 多种子尝试（上一帧解 → 给定种子 → 中性位形），任一成功即 `Solved`；
///   2. 全失败 → **任务空间线搜索**：沿 `Δx` 找最大可行步长 α*，
///      得到"最近可达点" → `Residual` + `SendResidual`；
///   3. 连 α→0 都不可行 → `Failed` + `HoldLast`（位置原地保持）。
///
/// ⚠️ 第 2 步是**用户明确要求**的："如果出现超出关节空间可行范围（或无解），
///    也不要停止发送结果，而是规划到对应终端残差的解然后下发。"
class IkOrchestrator
{
public:
  struct Config
  {
    IkConfig ik{};
    /// 中性位形（多种子用）。长度必须 = 关节数。
    std::vector<double> neutral;
    /// 单次求解的超时（秒）。由具体求解器使用。
    double solver_timeout_sec{0.05};
    /// 多种子最多尝试几个
    std::size_t max_seed_tries{3};
  };

  /// 配置。返回 false ⇒ 配置不可用（中性位形长度不符 / 阈值非有限），必须拒绝启动。
  bool configure(const Config & cfg, std::size_t n_joints) noexcept
  {
    cfg_ = cfg;
    n_joints_ = n_joints;
    if (n_joints_ == 0) {return false;}
    if (cfg_.neutral.size() != n_joints_) {return false;}
    if (!(cfg_.ik.residual_pos_m >= 0.0) || !std::isfinite(cfg_.ik.residual_pos_m)) {return false;}
    if (!(cfg_.ik.residual_rot_rad >= 0.0) || !std::isfinite(cfg_.ik.residual_rot_rad)) {return false;}
    if (!(cfg_.ik.limit_margin >= 0.0) || !std::isfinite(cfg_.ik.limit_margin)) {return false;}
    configured_ = true;
    return true;
  }

  /// 注入具体求解器（KDL / pick_ik）与正解。两者都必须就绪才能 `solve()`。
  void setSolver(IkSolveFn solve, FkFn fk) noexcept
  {
    solve_ = std::move(solve);
    fk_ = std::move(fk);
  }

  /// 注入限位（用于裁剪与"贴限位"判定）。
  void setLimits(const JointLimits & lim) noexcept {lim_ = lim;}

  bool ready() const noexcept
  {
    return configured_ && static_cast<bool>(solve_) && static_cast<bool>(fk_);
  }

  /// 求解一帧。
  IkResult solve(const IkRequest & req) noexcept
  {
    IkResult out;
    if (!ready() || req.seed.size() != n_joints_) {
      out.note = "IK 未就绪或种子长度不符";
      return out;
    }
    // ---- 1. 多种子尝试 ----
    std::vector<std::vector<double>> seeds;
    if (req.has_last_solution && req.last_solution.size() == n_joints_) {
      seeds.push_back(req.last_solution);
    }
    seeds.push_back(req.seed);
    if (cfg_.neutral.size() == n_joints_) {seeds.push_back(cfg_.neutral);}
    if (seeds.size() > cfg_.max_seed_tries) {seeds.resize(cfg_.max_seed_tries);}

    std::vector<double> joints;
    std::vector<std::size_t> clamped;
    //: 求解器是否给出过"非法解"（NaN / 长度不对）。
    //: ⚠️ 这决定"要不要再做线搜索"：求解器坏掉（一直给 NaN）时，
    //:    线搜索同样会拿到 NaN，而若线搜索不检查有限性就会**把 NaN 当成功**，
    //:    最终把 NaN 关节角发下去 —— 实测踩过。
    bool solver_gave_invalid = false;
    for (const auto & s : seeds) {
      ++out.seed_tries;
      joints.clear();
      if (!solve_(req.position, req.rotation, s, joints)) {continue;}
      if (joints.size() != n_joints_) {solver_gave_invalid = true; continue;}
      // 有限性
      bool finite = true;
      for (double v : joints) {if (!std::isfinite(v)) {finite = false; break;}}
      if (!finite) {solver_gave_invalid = true; continue;}
      clamped.clear();
      const bool any_clamp = clampToLimits(joints, lim_, &clamped);
      if (any_clamp && !cfg_.ik.allow_limit_clamped) {continue;}
      out.outcome = any_clamp ? SolverOutcome::LimitClamped : SolverOutcome::Solved;
      out.saturated_joints = clamped;
      break;
    }

    if (joints.empty()) {
      if (solver_gave_invalid) {
        // 求解器本身不可信（给 NaN / 长度错）⇒ **不要**再线搜索：
        // 那只会把 NaN 当成功。直接保持上一帧，并说明原因。
        out.outcome = SolverOutcome::Failed;
        out.action = IkAction::HoldLast;
        out.note = "求解器返回非法解（NaN/长度错）→ 保持上一帧";
        return out;
      }
      // 所有种子都失败（但求解器本身正常）→ 走线搜索求"最近可达点"
      return nearestReachable(req, out);
    }

    // ---- 2. 算残差，判定 Exact / Residual ----
    Vec3 ap{0, 0, 0}, ar{0, 0, 0};
    if (!checkedFk(joints, ap, ar)) {
      out.outcome = SolverOutcome::Failed;
      out.action = IkAction::HoldLast;
      out.note = "正解失败或非有限，无法判定残差 → 保持上一帧";
      return out;
    }
    poseError(req.position, req.rotation, ap, ar, out.position_residual, out.rotation_residual);
    const bool exact = out.positionResidualNorm() <= cfg_.ik.residual_pos_m &&
                       out.rotationResidualNorm() <= cfg_.ik.residual_rot_rad;
    out.residual_kind = exact ? TaskResidual::Exact : TaskResidual::Residual;
    out.action = exact ? IkAction::SendSolved : IkAction::SendResidual;
    out.joints = std::move(joints);
    out.ok = true;
    if (!exact) {
      out.note = "任务未打满（位置差 " + std::to_string(out.positionResidualNorm()) +
                 " m，姿态差 " + std::to_string(out.rotationResidualNorm()) + " rad）";
    }
    // 贴限位的关节（余量 < margin），供上游看
    if (lim_.size() == n_joints_) {
      for (std::size_t j = 0; j < n_joints_; ++j) {
        if (out.joints[j] - lim_.position_min[j] < cfg_.ik.limit_margin ||
            lim_.position_max[j] - out.joints[j] < cfg_.ik.limit_margin) {
          if (std::find(out.saturated_joints.begin(), out.saturated_joints.end(), j) ==
              out.saturated_joints.end()) {
            out.saturated_joints.push_back(j);
          }
        }
      }
    }
    return out;
  }

  const IkConfig & config() const noexcept {return cfg_.ik;}

private:
  /// **校验过的正解**：把 `fk_` 包一层，拒绝非有限的输出。
  ///
  /// ⚠️ 为什么必须包：`fk_` 的实现来自具体运动学后端（pinocchio/KDL），
  ///    它返回 true 但输出 NaN 是**可能**的（模型加载不全、关节名对不上等）。
  ///    如果这里不拦，NaN 会一路走到"残差比较"（NaN 比较恒为 false ⇒
  ///    被判成"残差很大"）然后**当成可用解发下去** —— 实测踩过：
  ///    真机上直接让整条控制链变成 NaN。
  bool checkedFk(const std::vector<double> & q, Vec3 & p, Vec3 & r) const noexcept
  {
    p = Vec3{0, 0, 0};
    r = Vec3{0, 0, 0};
    if (!fk_(q, p, r)) {return false;}
    return vec3_finite(p) && vec3_finite(r);
  }

  /// 任务空间**增量前进**：从当前可行解出发，朝目标一小步一小步走，
  /// 返回"走得最远的那个可行点"的解。
  ///
  /// ⚠️⚠️ 这里**不能**用"二分找最大可行 α"的写法。实测（本机 KDL/LMA，7 轴臂）：
  ///    目标离当前解 5 m 时，即使 α=0.05（目标只挪 13 cm），从**当前解**直接解
  ///    也**解不出来** —— LMA 的线性化在大姿态/大位移差下根本不收敛。
  ///    二分的前提"α 越小越可行"因此**不成立**，会把明明可达的点判成不可达。
  ///
  /// 正确做法是**增量**：`q ← ik(q, 目标方向上前进 1 步)`，用**上一步的解**当种子。
  /// 这样每一步的位移都很小（种子与目标近），LMA 收敛良好；
  /// 走到走不动为止，最后那个可行点就是"最近可达点"。
  ///
  /// 步长自适应：先用 1/`line_search_iters` 的份额，走不动就把份额减半重试
  /// （最多 `line_search_iters` 次细分）。所以总迭代量是 O(iters)，
  /// 每步一次 IK，**只在失败路径上触发**（正常运行 0 成本）。
  IkResult nearestReachable(const IkRequest & req, IkResult & out) noexcept
  {
    std::vector<double> q_cur = (req.has_last_solution &&
                                 req.last_solution.size() == n_joints_)
                                ? req.last_solution : req.seed;
    Vec3 cp{0, 0, 0}, cr{0, 0, 0};
    if (!checkedFk(q_cur, cp, cr)) {
      out.outcome = SolverOutcome::Failed;
      out.action = IkAction::HoldLast;
      out.note = "无解且正解失败/非有限 → 保持上一帧";
      return out;
    }
    // 目标相对当前解的总误差
    Vec3 dp{0, 0, 0}, dr{0, 0, 0};
    poseError(req.position, req.rotation, cp, cr, dp, dr);
    const double total_p = std::sqrt(dp[0] * dp[0] + dp[1] * dp[1] + dp[2] * dp[2]);

    // 自适应增量前进：成功就放大步长，失败就减半重试。
    // ⚠️ 为什么必须自适应：KDL/LMA 的收敛域**随位形变化**（实测同一条链上
    //    有的地方能迈 5 mm、有的地方 1 mm 都吃力）。固定步长要么太保守
    //    （远离目标）、要么直接解不动。
    double step = std::max(1e-5, cfg_.ik.walk_step_m);
    const double step_min = std::max(1e-6, cfg_.ik.walk_step_min_m);
    const double step_max = std::max(step, cfg_.ik.walk_step_max_m);
    const std::size_t max_steps = std::max<std::size_t>(1, cfg_.ik.max_walk_steps);

    std::vector<double> q_best = q_cur;          // 起点本身总是可行的
    bool moved = false;
    std::vector<std::size_t> cl_best;
    double remaining_p = total_p;
    for (std::size_t i = 0; i < max_steps && remaining_p > step_min; ) {
      // 朝目标方向走 step（位置），姿态按剩余比例同步推进
      const double a = std::min(1.0, step / std::max(1e-9, remaining_p));
      Vec3 tp{cp[0] + a * dp[0], cp[1] + a * dp[1], cp[2] + a * dp[2]};
      Vec3 tr{cr[0] + a * dr[0], cr[1] + a * dr[1], cr[2] + a * dr[2]};
      std::vector<double> sol;
      bool ok = solve_(tp, tr, q_cur, sol) && sol.size() == n_joints_;
      if (ok) {
        for (double v : sol) {if (!std::isfinite(v)) {ok = false; break;}}
      }
      std::vector<std::size_t> cl;
      if (ok) {
        clampToLimits(sol, lim_, &cl);
        if (!cl.empty() && !cfg_.ik.allow_limit_clamped) {ok = false;}
      }
      if (!ok) {
        // 失败：步长减半；已经很小就停
        step *= 0.5;
        if (step < step_min) {break;}
        continue;
      }
      // 成功：推进
      q_cur = std::move(sol);
      q_best = q_cur;
      cl_best = cl;
      moved = true;
      ++i;
      ++out.iterations;
      // 下一段的起点 = 这一步的目标（增量语义）
      cp = tp;
      cr = tr;
      // 重算到原始目标的剩余距离
      remaining_p = std::sqrt((req.position[0] - cp[0]) * (req.position[0] - cp[0]) +
                              (req.position[1] - cp[1]) * (req.position[1] - cp[1]) +
                              (req.position[2] - cp[2]) * (req.position[2] - cp[2]));
      // 成功 ⇒ 乐观放大（上限 walk_step_max_m）
      step = std::min(step_max, step * 2.0);
    }

    if (!moved) {
      out.outcome = SolverOutcome::Failed;
      out.action = IkAction::HoldLast;
      out.note = "无解：朝目标方向连最小一步都走不动 → 保持上一帧位置";
      return out;
    }

    // 用最终解算真实残差（相对原始目标）
    Vec3 ap{0, 0, 0}, ar{0, 0, 0};
    if (!checkedFk(q_best, ap, ar)) {
      out.outcome = SolverOutcome::Failed;
      out.action = IkAction::HoldLast;
      out.note = "残差解的正解失败/非有限 → 保持上一帧";
      return out;
    }
    poseError(req.position, req.rotation, ap, ar, out.position_residual, out.rotation_residual);
    out.outcome = cl_best.empty() ? SolverOutcome::Solved : SolverOutcome::LimitClamped;
    out.residual_kind = TaskResidual::Residual;
    out.action = IkAction::SendResidual;
    out.joints = std::move(q_best);
    out.saturated_joints = cl_best;
    out.ok = true;
    out.note = "目标不可达 → 自适应增量前进到最远可行点（残差 " +
               std::to_string(out.positionResidualNorm()) +
               " m / " + std::to_string(out.iterations) + " 步）";
    return out;
  }

  Config cfg_{};
  JointLimits lim_{};
  IkSolveFn solve_{};
  FkFn fk_{};
  std::size_t n_joints_{0};
  bool configured_{false};
};

}  // namespace lite_move_controller
