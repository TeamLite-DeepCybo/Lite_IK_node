// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// ik_kdl.cpp —— KDL 链式 IK/FK 的实现（见头文件顶部说明为什么是 KDL 而不是 MoveIt 插件）
#include "lite_ik_node/ik_kdl.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace lite_move_controller
{

// ---------------------------------------------------------------------------
// 内部实现（把 KDL 类型藏在 .cpp 里，头文件不泄漏 KDL 依赖）
// ---------------------------------------------------------------------------
struct KdlArmChain::Impl
{
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver;
  KDL::Chain chain;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
  std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik;
};

KdlArmChain::KdlArmChain() : impl_(std::make_unique<Impl>()) {}
KdlArmChain::~KdlArmChain() = default;
// 移动操作在这里（而非头文件）定义：Impl 在此处是完整类型
KdlArmChain::KdlArmChain(KdlArmChain &&) noexcept = default;
KdlArmChain & KdlArmChain::operator=(KdlArmChain &&) noexcept = default;

namespace
{

/// 旋转矩阵 → 旋转向量（axis·angle）。
Vec3 rotmatToRotvec(const KDL::Rotation & R)
{
  // 用四元数中转（KDL 自带 GetQuaternion），再走我们自己的 rotvec 转换，
  // 保证与 input_filter / shape_traj **同一套约定**（w<0 取反、最小表示）。
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
  R.GetQuaternion(x, y, z, w);
  return quat_to_rotvec(Quat{x, y, z, w});
}

/// 旋转向量 → 旋转矩阵。
KDL::Rotation rotvecToRotmat(const Vec3 & rv)
{
  const Quat q = rotvec_to_quat(rv);
  return KDL::Rotation::Quaternion(q[0], q[1], q[2], q[3]);
}

}  // namespace

bool KdlArmChain::build(const std::string & urdf_xml,
                        const std::string & root_link,
                        const std::string & tip_link,
                        const std::vector<std::string> & joint_names,
                        double eps, int max_iter,
                        std::string & error_out)
{
  ready_ = false;
  last_error_.clear();
  dof_ = 0;
  root_link_ = root_link;
  tip_link_ = tip_link;
  joint_names_.clear();

  if (urdf_xml.empty()) {
    error_out = "URDF 文本为空（robot_description 是否已发布？）";
    return false;
  }
  if (joint_names.empty()) {
    error_out = "joint_names 为空";
    return false;
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf_xml, tree)) {
    error_out = "kdl_parser::treeFromString 失败（URDF 解析不了）";
    return false;
  }
  if (!tree.getChain(root_link, tip_link, impl_->chain)) {
    error_out = "找不到链 " + root_link + " → " + tip_link +
                "（检查 base/tip link 名是否与 URDF 一致）";
    return false;
  }
  const std::size_t chain_dof = static_cast<std::size_t>(impl_->chain.getNrOfJoints());
  if (chain_dof != joint_names.size()) {
    error_out = "链的自由度(" + std::to_string(chain_dof) + ") 与给定关节数(" +
                std::to_string(joint_names.size()) + ") 不符 —— 关节集合配置错误";
    return false;
  }

  // ⚠️ 校验链上的关节名与期望顺序**逐个一致**。
  //    不一致会让"关节 3 的解写到关节 5 上"，每个数值看起来都合理 ——
  //    本项目反复强调的最难查的一类故障（见 lite_controller/joint_index_map.hpp）。
  std::vector<std::string> chain_joints;
  for (unsigned i = 0; i < impl_->chain.getNrOfSegments(); ++i) {
    const KDL::Joint & j = impl_->chain.getSegment(i).getJoint();
    if (j.getType() == KDL::Joint::None) {continue;}
    chain_joints.push_back(j.getName());
  }
  if (chain_joints.size() != joint_names.size()) {
    error_out = "链上实际关节数(" + std::to_string(chain_joints.size()) +
                ") 与期望(" + std::to_string(joint_names.size()) + ") 不符";
    return false;
  }
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    if (chain_joints[i] != joint_names[i]) {
      error_out = "链上的关节顺序与配置不符：第 " + std::to_string(i) +
                  " 个是 '" + chain_joints[i] + "'，期望 '" + joint_names[i] + "'";
      return false;
    }
  }

  impl_->fk = std::make_unique<KDL::ChainFkSolverPos_recursive>(impl_->chain);
  // ⚠️ 雅可比求解器**必须在这里建**。原先 `jac_solver` 只声明、从未初始化，
  //    而 `jacobian()` 无条件解引用它 —— 一旦走到动力学/外力估计（`dyn_ready`
  //    为真，例如带 params_json 启动），第一拍就 SIGSEGV（空指针）。
  //    不开动力学时 `jacobian()` 从不被调用，所以这个洞一直没暴露。
  impl_->jac_solver = std::make_unique<KDL::ChainJntToJacSolver>(impl_->chain);
  // ⚠️ LMA 的构造函数要求一个"权重向量"：它决定各方向误差的相对重要度。
  //    默认给 1（位置与姿态同等重要）。本层不做精细加权 —— 精细加权会让
  //    "位置准姿态偏"这类残差的含义变得难以解释，而上游正是按那个残差迭代修正的。
  Eigen::Matrix<double, 6, 1> weights;
  weights << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
  impl_->ik = std::make_unique<KDL::ChainIkSolverPos_LMA>(
      impl_->chain, weights, std::max(1e-9, eps), std::max(1, max_iter));

  joint_names_ = joint_names;
  dof_ = chain_dof;
  ready_ = true;
  return true;
}

const void * KdlArmChain::chainOpaque() const noexcept
{
  return ready_ ? static_cast<const void *>(&impl_->chain) : nullptr;
}

std::size_t KdlArmChain::segmentCount() const noexcept
{
  return ready_ ? static_cast<std::size_t>(impl_->chain.getNrOfSegments()) : 0;
}

std::string KdlArmChain::segmentChildLink(std::size_t i) const
{
  if (!ready_ || i >= static_cast<std::size_t>(impl_->chain.getNrOfSegments())) {return {};}
  return impl_->chain.getSegment(static_cast<unsigned>(i)).getName();
}

bool KdlArmChain::jacobian(const std::vector<double> & q, Jacobian & J_out) const
{
  if (!ready_ || q.size() != dof_) {return false;}
  KDL::JntArray qa(static_cast<unsigned>(dof_));
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!std::isfinite(q[i])) {return false;}
    qa(static_cast<unsigned>(i)) = q[i];
  }
  KDL::Jacobian kj(static_cast<unsigned>(dof_));
  if (impl_->jac_solver->JntToJac(qa, kj) < 0) {return false;}
  J_out.resize(6, dof_);
  for (std::size_t r = 0; r < 6; ++r) {
    for (std::size_t c = 0; c < dof_; ++c) {
      const double v = kj(static_cast<unsigned>(r), static_cast<unsigned>(c));
      if (!std::isfinite(v)) {return false;}
      J_out.at(r, c) = v;
    }
  }
  return true;
}

bool KdlArmChain::fk(const std::vector<double> & q, Vec3 & position, Vec3 & rotation) const
{
  if (!ready_ || q.size() != dof_) {return false;}
  KDL::JntArray qa(static_cast<unsigned>(dof_));
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!std::isfinite(q[i])) {return false;}
    qa(static_cast<unsigned>(i)) = q[i];
  }
  KDL::Frame T;
  if (impl_->fk->JntToCart(qa, T) < 0) {return false;}
  position = Vec3{T.p.x(), T.p.y(), T.p.z()};
  rotation = rotmatToRotvec(T.M);
  return vec3_finite(position) && vec3_finite(rotation);
}

bool KdlArmChain::ik(const Vec3 & position, const Vec3 & rotation,
                     const std::vector<double> & seed, std::vector<double> & q_out) const
{
  if (!ready_ || seed.size() != dof_) {return false;}
  if (!vec3_finite(position) || !vec3_finite(rotation)) {return false;}
  KDL::JntArray q_init(static_cast<unsigned>(dof_));
  for (std::size_t i = 0; i < dof_; ++i) {
    if (!std::isfinite(seed[i])) {return false;}
    q_init(static_cast<unsigned>(i)) = seed[i];
  }
  const KDL::Frame target(rotvecToRotmat(rotation), KDL::Vector(position[0], position[1], position[2]));
  KDL::JntArray q_sol(static_cast<unsigned>(dof_));
  const int rc = impl_->ik->CartToJnt(q_init, target, q_sol);
  if (rc < 0) {return false;}
  q_out.resize(dof_);
  for (std::size_t i = 0; i < dof_; ++i) {
    q_out[i] = q_sol(static_cast<unsigned>(i));
    if (!std::isfinite(q_out[i])) {return false;}
  }
  return true;
}

IkSolveFn makeKdlSolveFn(const KdlArmChain * chain)
{
  return [chain](const Vec3 & p, const Vec3 & r, const std::vector<double> & seed,
                 std::vector<double> & out) -> bool {
    if (chain == nullptr || !chain->ready()) {return false;}
    return chain->ik(p, r, seed, out);
  };
}

FkFn makeKdlFkFn(const KdlArmChain * chain)
{
  return [chain](const std::vector<double> & q, Vec3 & p, Vec3 & r) -> bool {
    if (chain == nullptr || !chain->ready()) {return false;}
    return chain->fk(q, p, r);
  };
}

// ---------------------------------------------------------------------------
// kinematics.yaml 解析
// ---------------------------------------------------------------------------
bool loadKinematicsParams(const std::string & yaml_text, const std::string & group,
                          KinematicsParams & out, std::string & error_out)
{
  if (yaml_text.empty()) {
    error_out = "kinematics.yaml 内容为空";
    return false;
  }
  try {
    const YAML::Node root = YAML::Load(yaml_text);
    const YAML::Node node = root[group];
    if (!node || !node.IsMap()) {
      error_out = "kinematics.yaml 里没有 group '" + group + "'";
      return false;
    }
    const auto plugin = node["kinematics_solver"];
    if (!plugin) {
      error_out = "group '" + group + "' 没有 kinematics_solver";
      return false;
    }
    out.plugin = plugin.as<std::string>();
    if (const auto v = node["kinematics_solver_search_resolution"]) {
      out.search_resolution = v.as<double>();
    }
    if (const auto v = node["kinematics_solver_timeout"]) {
      out.timeout_sec = v.as<double>();
    }
    if (!(out.search_resolution > 0.0) || !std::isfinite(out.search_resolution)) {
      error_out = "kinematics_solver_search_resolution 非法";
      return false;
    }
    if (!(out.timeout_sec > 0.0) || !std::isfinite(out.timeout_sec)) {
      error_out = "kinematics_solver_timeout 非法（单位是**秒**）";
      return false;
    }
    // LMA 每步约 0.1 ms（本机量级）⇒ 迭代上限 = timeout / 0.1ms，夹在 [20, 2000]
    const double steps = out.timeout_sec / 1e-4;
    out.max_iter = static_cast<int>(std::min(2000.0, std::max(20.0, steps)));
    return true;
  } catch (const std::exception & e) {
    error_out = std::string("解析 kinematics.yaml 异常：") + e.what();
    return false;
  }
}

}  // namespace lite_move_controller
