// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// differential_ik.hpp —— 微分逆解（CLIK / 阻尼最小二乘 + 零空间避限位）
//
// ===========================================================================
// 原理与优势
// ===========================================================================
//
// 1. 速度闭环 (Closed-loop Inverse Kinematics, CLIK):
//    v_cmd = K_p · [ e_pos ; e_rot ]
//
// 2. 阻尼最小二乘 (DLS, Damped Least Squares):
//    J⁺ = Jᵀ · (J·Jᵀ + λ²·I)⁻¹
//    在奇异点附近自动引入阻尼 λ，防止关节速度爆炸。
//
// 3. 零空间自运动投影 (Null-space Projection):
//    N = I - J⁺·J
//    q̇_null = -k_null · ∇ H(q)
//    H(q) = ∑ ((q_i - q_mid) / (q_max - q_min))²
//    完全不影响末端笛卡尔位姿的前提下，自动把各关节往中心拉，防止贴限位。
//
// 4. 连续性保证:
//    因为输出直接基于当前状态的小步长积分，数学上天然 C¹ 连续，
//    绝不发生肘部/肩部突跳翻转。
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "lite_ik_node/ik_backend.hpp"

namespace lite_move_controller
{

struct DiffIkConfig
{
  /// 阻尼因子 λ（奇异点保护）
  double lambda{0.05};
  /// 笛卡尔纠偏增益（单步积分比例系数）
  double kp_pos{1.0};
  double kp_rot{1.0};
  /// 零空间避限位增益
  double k_null{0.5};
  /// 关节最大允许速度（rad/s，默认 1.5 rad/s，拒绝全域飞车）
  double max_joint_vel{1.5};
  /// 关节最大允许加速度（rad/s^2，默认 12.0 rad/s^2）
  double max_joint_acc{12.0};
  /// 控制周期（s，默认 0.01s 即 100Hz）
  double control_dt{0.01};
  /// 单步关节角最大允许增量（rad，防止远距离目标飞车）
  double max_step_rad{0.03};
  /// 内部微迭代最大步数（用于初始大位移快速拉回）
  std::size_t max_iterations{10};
  /// 判定收敛的位置残差阈值（m）
  double residual_pos_m{0.001};
  /// 判定收敛的姿态残差阈值（rad）
  double residual_rot_rad{0.01};
  /// 关节限位安全缓冲裕度（rad）
  double limit_margin_rad{0.015};
};

struct DiffIkResult
{
  std::vector<double> joints;
  double position_residual{0.0};
  double rotation_residual{0.0};
  std::size_t iterations{0};
  bool ok{false};
  std::string note;
};

/// 微分逆解求解器
class DifferentialIkSolver
{
public:
  using FkFunction = std::function<bool(const std::vector<double> &, Vec3 &, Vec3 &)>;
  using JacFunction = std::function<bool(const std::vector<double> &, Jacobian &)>;

  DifferentialIkSolver() = default;

  bool configure(const DiffIkConfig & cfg, const JointLimits & limits, std::size_t n_joints)
  {
    cfg_ = cfg;
    limits_ = limits;
    n_joints_ = n_joints;
    if (n_joints_ == 0 || limits_.size() != n_joints_) {
      return false;
    }
    configured_ = true;
    return true;
  }

  void setKinematics(FkFunction fk, JacFunction jac)
  {
    fk_ = std::move(fk);
    jac_ = std::move(jac);
  }

  bool ready() const noexcept
  {
    return configured_ && static_cast<bool>(fk_) && static_cast<bool>(jac_);
  }

  /// 执行微分逆解
  ///
  /// @param target_pos 目标位置（m）
  /// @param target_rot 目标姿态（旋转向量，rad）
  /// @param seed       当前关节角状态
  DiffIkResult solve(const Vec3 & target_pos, const Vec3 & target_rot,
                     const std::vector<double> & seed) const
  {
    DiffIkResult res;
    if (!ready() || seed.size() != n_joints_) {
      res.note = "微分逆解未就绪或种子长度不匹配";
      return res;
    }

    std::vector<double> q = seed;
    // 确保初始在限位内
    for (std::size_t j = 0; j < n_joints_; ++j) {
      q[j] = std::max(limits_.position_min[j], std::min(limits_.position_max[j], q[j]));
    }

    std::size_t iter = 0;
    Jacobian J_kdl;
    Eigen::Matrix<double, 6, Eigen::Dynamic> J(6, n_joints_);
    Eigen::Matrix<double, 6, 6> JJt;
    Eigen::Matrix<double, 6, 6> I6 = Eigen::Matrix<double, 6, 6>::Identity();
    Eigen::MatrixXd In = Eigen::MatrixXd::Identity(n_joints_, n_joints_);

    double final_err_p = 0.0;
    double final_err_r = 0.0;

    for (; iter < cfg_.max_iterations; ++iter) {
      Vec3 cur_p{0, 0, 0}, cur_r{0, 0, 0};
      if (!fk_(q, cur_p, cur_r)) {
        res.note = "正解计算失败";
        return res;
      }

      // 计算误差向量 e = [e_pos; e_rot]
      Eigen::Vector<double, 6> e;
      e[0] = target_pos[0] - cur_p[0];
      e[1] = target_pos[1] - cur_p[1];
      e[2] = target_pos[2] - cur_p[2];

      // 姿态误差：利用李代数四元数对数映射计算世界系旋转误差向量 e_rot = log(R_tgt * R_cur^T)
      const Quat q_tgt = rotvec_to_quat(target_rot);
      const Quat q_cur = rotvec_to_quat(cur_r);
      const Quat q_cur_inv{-q_cur[0], -q_cur[1], -q_cur[2], q_cur[3]};
      const Quat q_err{
        q_tgt[3]*q_cur_inv[0] + q_tgt[0]*q_cur_inv[3] + q_tgt[1]*q_cur_inv[2] - q_tgt[2]*q_cur_inv[1],
        q_tgt[3]*q_cur_inv[1] - q_tgt[0]*q_cur_inv[2] + q_tgt[1]*q_cur_inv[3] + q_tgt[2]*q_cur_inv[0],
        q_tgt[3]*q_cur_inv[2] + q_tgt[0]*q_cur_inv[1] - q_tgt[1]*q_cur_inv[0] + q_tgt[2]*q_cur_inv[3],
        q_tgt[3]*q_cur_inv[3] - q_tgt[0]*q_cur_inv[0] - q_tgt[1]*q_cur_inv[1] - q_tgt[2]*q_cur_inv[2]
      };
      const Vec3 rot_err = quat_to_rotvec(q_err);
      e[3] = rot_err[0];
      e[4] = rot_err[1];
      e[5] = rot_err[2];

      final_err_p = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
      final_err_r = std::sqrt(e[3] * e[3] + e[4] * e[4] + e[5] * e[5]);

      if (final_err_p <= cfg_.residual_pos_m && final_err_r <= cfg_.residual_rot_rad) {
        break; // 已精确收敛
      }

      if (!jac_(q, J_kdl) || !J_kdl.valid() || J_kdl.cols != n_joints_) {
        res.note = "雅可比矩阵计算失败";
        return res;
      }

      // 拷贝到 Eigen 矩阵
      for (std::size_t r = 0; r < 6; ++r) {
        for (std::size_t c = 0; c < n_joints_; ++c) {
          J(r, c) = J_kdl.at(r, c);
        }
      }

      // 笛卡尔速度指令 v_cmd = K_p * e
      Eigen::Vector<double, 6> v_cmd;
      v_cmd.head<3>() = cfg_.kp_pos * e.head<3>();
      v_cmd.tail<3>() = cfg_.kp_rot * e.tail<3>();

      // 阻尼最小二乘 DLS: J_pinv = Jᵀ (J Jᵀ + λ² I)⁻¹
      JJt = J * J.transpose();
      JJt += (cfg_.lambda * cfg_.lambda) * I6;

      Eigen::Vector<double, 6> y = JJt.ldlt().solve(v_cmd);
      Eigen::VectorXd dq_main = J.transpose() * y;

      // 零空间投影: (I - J_pinv * J) * dq_null
      Eigen::VectorXd dq_null = Eigen::VectorXd::Zero(n_joints_);
      for (std::size_t j = 0; j < n_joints_; ++j) {
        const double q_mid = 0.5 * (limits_.position_max[j] + limits_.position_min[j]);
        const double range = std::max(0.1, limits_.position_max[j] - limits_.position_min[j]);
        // 避限位势能梯度
        dq_null[j] = -cfg_.k_null * (q[j] - q_mid) / (range * range);
      }

      // 零空间分量: dq_null_proj = dq_null - J_pinv * (J * dq_null)
      Eigen::Vector<double, 6> J_null = J * dq_null;
      Eigen::Vector<double, 6> y_null = JJt.ldlt().solve(J_null);
      Eigen::VectorXd dq_null_proj = dq_null - J.transpose() * y_null;

      Eigen::VectorXd dq_total = dq_main + dq_null_proj;

      // 单步最大速度与步长上限综合保护（拒绝全域突跳）
      const double max_allowed_step = std::min(cfg_.max_step_rad, cfg_.max_joint_vel * cfg_.control_dt);
      double max_dq = 0.0;
      for (std::size_t j = 0; j < n_joints_; ++j) {
        max_dq = std::max(max_dq, std::fabs(dq_total[j]));
      }
      if (max_dq > max_allowed_step && max_dq > 1e-9) {
        dq_total *= (max_allowed_step / max_dq);
      }

      // 积分更新并严格截断在安全缓冲限位以内
      for (std::size_t j = 0; j < n_joints_; ++j) {
        q[j] += dq_total[j];
        const double q_lo = limits_.position_min[j] + cfg_.limit_margin_rad;
        const double q_hi = limits_.position_max[j] - cfg_.limit_margin_rad;
        q[j] = std::max(q_lo, std::min(q_hi, q[j]));
      }
    }

    res.joints = std::move(q);
    res.iterations = iter;
    res.position_residual = final_err_p;
    res.rotation_residual = final_err_r;
    res.ok = (final_err_p <= cfg_.residual_pos_m && final_err_r <= cfg_.residual_rot_rad);
    if (!res.ok) {
      res.note = "微分逆解未能在步数限制内完全收敛（残差 pos: " +
                 std::to_string(final_err_p) + " m, rot: " +
                 std::to_string(final_err_r) + " rad）";
    }
    return res;
  }

private:
  DiffIkConfig cfg_{};
  JointLimits limits_{};
  std::size_t n_joints_{0};
  FkFunction fk_{};
  JacFunction jac_{};
  bool configured_{false};
};

}  // namespace lite_move_controller

