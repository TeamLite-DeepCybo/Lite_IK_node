// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// ik_node.cpp —— 独立 IK 节点（上游位姿目标 → 连续关节目标）
//
// ===========================================================================
// 职责与分工
// ===========================================================================
//
// 1. 输入：
//    · /lite_move_controller/task_target (MoveTargetArray: TARGET_POSE / TARGET_JOINTS)
//    · /joint_states (JointState: 实测关节角，作为跟踪基准与种子)
//
// 2. 解算策略：
//    · 主解算器：微分逆解 (Differential IK: DLS 阻尼最小二乘 + 零空间避限位)
//    · 兜底解算器：仅当微分逆解残差超阈值时，退化调用一次全局解算器 (PickIK / KDL)，
//                并强行施加距离当前位置的跳变惩罚（拒绝构型翻转）。
//
// 3. 输出：
//    · /lite_move_controller/joint_target (MoveTargetArray: 严格填充 TARGET_JOINTS)
//    · 供运控层 (lite_move_controller) 直接进行三次插值与力控映射下发。
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <move_msgs/msg/move_target.hpp>
#include <move_msgs/msg/move_target_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include "lite_ik_node/differential_ik.hpp"
#include "lite_ik_node/ik_kdl.hpp"
#include "lite_ik_node/ik_moveit.hpp"

namespace lite_move_controller
{

struct ArmIkContext
{
  std::string name;
  std::string tip_link;
  std::vector<std::string> joints;
  std::size_t offset{0};

  KdlArmChain chain;
  MoveItArmKinematics moveit;
  bool use_moveit{false};
  DifferentialIkSolver diff_ik;
  JointLimits limits;

  std::vector<double> q_current;
  std::vector<double> q_last_solved;
  std::vector<double> v_last_solved;
  bool have_q_current{false};
  bool have_last_solved{false};
  rclcpp::Time last_target_time{0, 0, RCL_ROS_TIME};

  // 统计指标
  std::uint64_t diff_ik_success_count{0};
  std::uint64_t fallback_count{0};
  std::uint64_t total_requests{0};
  double last_residual_pos{0.0};
  double last_residual_rot{0.0};
};

class IkNode : public rclcpp::Node
{
public:
  IkNode() : Node("lite_ik_node")
  {
    declareParameters();
  }

  void init()
  {
    loadConfiguration();
    setupInterfaces();
    setupArms();

    RCLCPP_INFO(get_logger(),
                "lite_ik_node 启动就绪：监听目标话题 '%s'，发布关节目标 '%s'，状态参考 '%s'",
                input_target_topic_.c_str(), output_joint_topic_.c_str(), joint_states_topic_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("input_target_topic", "/lite_move_controller/task_target");
    declare_parameter<std::string>("output_joint_topic", "/lite_move_controller/joint_target");
    declare_parameter<std::string>("joint_states_topic", "/joint_states");
    declare_parameter<std::string>("root_link", "world_root");
    declare_parameter<std::string>("tip_link_left", "left_gripper_tip_middle_link");
    declare_parameter<std::string>("tip_link_right", "right_gripper_tip_middle_link");
    declare_parameter<std::string>("urdf_path", "");
    declare_parameter<std::string>("srdf_path", "");
    declare_parameter<std::string>("ik.solver_plugin", "pick_ik/PickIkPlugin");
    declare_parameter<double>("max_joint_jump_rad", 0.35); // 兜底解算器单步跳变防护门限
    declare_parameter<double>("max_joint_vel", 2.0);       // 关节速度上限（rad/s）
    declare_parameter<double>("max_joint_acc", 12.0);      // 关节加速度上限（rad/s^2）

    // 默认关节列表
    declare_parameter<std::vector<std::string>>(
        "joints_left",
        {"left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
         "left_elbow_pitch", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch"});
    declare_parameter<std::vector<std::string>>(
        "joints_right",
        {"right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
         "right_elbow_pitch", "right_wrist_yaw", "right_wrist_roll", "right_wrist_pitch"});
  }

  void loadConfiguration()
  {
    input_target_topic_ = get_parameter("input_target_topic").as_string();
    output_joint_topic_ = get_parameter("output_joint_topic").as_string();
    joint_states_topic_ = get_parameter("joint_states_topic").as_string();
    root_link_ = get_parameter("root_link").as_string();
    tip_link_left_ = get_parameter("tip_link_left").as_string();
    tip_link_right_ = get_parameter("tip_link_right").as_string();
    urdf_path_ = get_parameter("urdf_path").as_string();
    srdf_path_ = get_parameter("srdf_path").as_string();
    solver_plugin_ = get_parameter("ik.solver_plugin").as_string();
    max_joint_jump_rad_ = get_parameter("max_joint_jump_rad").as_double();
    max_joint_vel_ = get_parameter("max_joint_vel").as_double();
    max_joint_acc_ = get_parameter("max_joint_acc").as_double();

    joints_left_ = get_parameter("joints_left").as_string_array();
    joints_right_ = get_parameter("joints_right").as_string_array();
  }

  std::string loadUrdf()
  {
    if (!urdf_path_.empty()) {
      std::ifstream f(urdf_path_);
      if (f.is_open()) {
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
      }
    }
    std::vector<std::string> candidates;
    try {
      candidates.push_back(
        ament_index_cpp::get_package_share_directory("lite_ik_node") +
        "/config/model/lite_000_asm.urdf");
    } catch (const std::exception &) {}
    candidates.push_back("src/lite_ik_node/config/model/lite_000_asm.urdf");
    for (const auto & p : candidates) {
      std::ifstream f(p);
      if (f.is_open()) {
        std::stringstream ss;
        ss << f.rdbuf();
        return ss.str();
      }
    }
    return {};
  }

  std::string loadSrdf()
  {
    std::vector<std::string> tried;
    std::vector<std::string> candidates;
    try {
      candidates.push_back(
        ament_index_cpp::get_package_share_directory("lite_ik_node") +
        "/config/model/lite_000_asm.srdf");
    } catch (const std::exception &) {}
    return loadSrdfText(srdf_path_, candidates, &tried);
  }

  void setupInterfaces()
  {
    sub_targets_ = create_subscription<move_msgs::msg::MoveTargetArray>(
        input_target_topic_, 10,
        std::bind(&IkNode::onTargetArray, this, std::placeholders::_1));

    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
        joint_states_topic_, 10,
        std::bind(&IkNode::onJointStates, this, std::placeholders::_1));

    pub_joint_targets_ = create_publisher<move_msgs::msg::MoveTargetArray>(
        output_joint_topic_, 10);
  }

  void setupArms()
  {
    const std::string urdf = loadUrdf();
    if (urdf.empty()) {
      RCLCPP_FATAL(get_logger(), "无法读取 URDF，IK 节点初始化终止");
      return;
    }
    const std::string srdf = loadSrdf();

    auto urdf_model = urdf::parseURDF(urdf);

    // The differential IK callbacks keep pointers to elements in this vector.
    // Reserve the complete, fixed arm count so the elements cannot move while
    // the node is running.
    arms_.reserve(2);

    auto init_arm = [&](const std::string & name, const std::string & tip,
                        const std::vector<std::string> & joints, std::size_t offset) {
      ArmIkContext arm;
      arm.name = name;
      arm.tip_link = tip;
      arm.joints = joints;
      arm.offset = offset;
      arm.q_current.assign(joints.size(), 0.0);
      arm.q_last_solved.assign(joints.size(), 0.0);

      // 限位构建
      arm.limits.position_min.assign(joints.size(), -3.14);
      arm.limits.position_max.assign(joints.size(), 3.14);
      arm.limits.velocity_max.assign(joints.size(), 5.0);
      arm.limits.torque_max.assign(joints.size(), 14.0);
      arm.limits.stiffness_max.assign(joints.size(), 50.0);
      arm.limits.damping_max.assign(joints.size(), 5.0);

      if (urdf_model) {
        for (std::size_t j = 0; j < joints.size(); ++j) {
          auto jt = urdf_model->getJoint(joints[j]);
          if (jt && jt->limits) {
            arm.limits.position_min[j] = jt->limits->lower;
            arm.limits.position_max[j] = jt->limits->upper;
            arm.limits.velocity_max[j] = jt->limits->velocity;
          }
        }
      }

      std::string err;
      if (!arm.chain.build(urdf, root_link_, tip, joints, 1e-5, 200, err)) {
        RCLCPP_ERROR(get_logger(), "臂 '%s' KDL 链构建失败: %s", name.c_str(), err.c_str());
        return;
      }

      // 配置微分逆解
      DiffIkConfig dcfg;
      dcfg.lambda = 0.04;
      dcfg.kp_pos = 1.0;
      dcfg.kp_rot = 1.0;
      dcfg.k_null = 0.4;
      dcfg.max_joint_vel = max_joint_vel_;
      dcfg.max_joint_acc = max_joint_acc_;
      dcfg.control_dt = 0.01;
      dcfg.max_step_rad = 0.03;
      dcfg.limit_margin_rad = 0.015;
      dcfg.max_iterations = 12;
      dcfg.residual_pos_m = 0.001;
      dcfg.residual_rot_rad = 0.01;

      arm.diff_ik.configure(dcfg, arm.limits, joints.size());
      // MoveIt 局部兜底插件尝试
      if (!srdf.empty() && !solver_plugin_.empty()) {
        MoveItArmKinematics::Options mo;
        mo.node = shared_from_this();
        mo.plugin = solver_plugin_;
        mo.group_name = (name == "left") ? "left_arm_group" : "right_arm_group";
        mo.base_frame = root_link_;
        mo.tip_frame = tip;
        mo.joint_names = joints;
        mo.timeout_sec = 0.02;
        mo.position_threshold = 0.001;
        mo.orientation_threshold = 0.01;
        std::string merr;
        if (arm.moveit.build(urdf, srdf, mo, merr)) {
          arm.use_moveit = true;
          RCLCPP_INFO(get_logger(), "臂 '%s' 兜底 MoveIt 求解器 (%s) 已就绪",
                      name.c_str(), solver_plugin_.c_str());
        }
      }
      arms_.push_back(std::move(arm));
      auto * stored_arm = &arms_.back();
      stored_arm->diff_ik.setKinematics(
          [stored_arm](const std::vector<double> & q, Vec3 & p, Vec3 & r) {
            return stored_arm->chain.fk(q, p, r);
          },
          [stored_arm](const std::vector<double> & q, Jacobian & J) {
            return stored_arm->chain.jacobian(q, J);
          });
    };

    init_arm("left", tip_link_left_, joints_left_, 0);
    init_arm("right", tip_link_right_, joints_right_, joints_left_.size());
  }

  void onJointStates(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (auto & arm : arms_) {
      bool all_found = true;
      std::vector<double> q_found(arm.joints.size(), 0.0);
      for (std::size_t j = 0; j < arm.joints.size(); ++j) {
        auto it = std::find(msg->name.begin(), msg->name.end(), arm.joints[j]);
        if (it != msg->name.end()) {
          auto idx = std::distance(msg->name.begin(), it);
          q_found[j] = msg->position[idx];
        } else {
          all_found = false;
          break;
        }
      }
      if (all_found) {
        arm.q_current = q_found;
        arm.have_q_current = true;
        const auto now_t = now();
        if (!arm.have_last_solved ||
            (now_t.nanoseconds() - arm.last_target_time.nanoseconds() > 300000000LL)) {
          arm.q_last_solved = q_found;
          arm.v_last_solved.assign(arm.joints.size(), 0.0);
          arm.have_last_solved = true;
        }
      }
    }
  }

  void onTargetArray(const move_msgs::msg::MoveTargetArray::SharedPtr msg)
  {
    move_msgs::msg::MoveTargetArray out_msg;
    out_msg.targets.reserve(msg->targets.size());

    for (const auto & in_target : msg->targets) {
      move_msgs::msg::MoveTarget out_target = in_target;
      out_target.target_type = move_msgs::msg::MoveTarget::TARGET_JOINTS;

      // 如果上游已经是 TARGET_JOINTS，更新本地解算缓存，并直接透明透传给运控层
      if (in_target.target_type == move_msgs::msg::MoveTarget::TARGET_JOINTS &&
          !in_target.joint_positions.empty()) {
        ArmIkContext * arm = nullptr;
        for (auto & a : arms_) {
          if (a.name == in_target.arm) {
            arm = &a;
            break;
          }
        }
        if (arm && in_target.joint_positions.size() == arm->joints.size()) {
          arm->q_last_solved = in_target.joint_positions;
          arm->v_last_solved.assign(arm->joints.size(), 0.0);
          arm->have_last_solved = true;
          arm->last_target_time = now();
        }
        out_msg.targets.push_back(out_target);
        continue;
      }

      // 否则为 TARGET_POSE，通过本节点解算
      ArmIkContext * arm = nullptr;
      for (auto & a : arms_) {
        if (a.name == in_target.arm) {
          arm = &a;
          break;
        }
      }
      if (!arm || !arm->diff_ik.ready()) {
        // 找不到对应的臂或未就绪
        continue;
      }

      arm->last_target_time = now();
      ++arm->total_requests;

      // 连续性安全防护：若历史解与当前实测偏差较大（>0.15 rad），重新锚定到实测当前位置
      if (arm->have_q_current && arm->have_last_solved) {
        double max_drift = 0.0;
        for (std::size_t j = 0; j < arm->joints.size(); ++j) {
          max_drift = std::max(max_drift, std::fabs(arm->q_last_solved[j] - arm->q_current[j]));
        }
        if (max_drift > 0.15) {
          arm->q_last_solved = arm->q_current;
          arm->v_last_solved.assign(arm->joints.size(), 0.0);
        }
      }

      Vec3 target_p{in_target.position.x, in_target.position.y, in_target.position.z};
      Quat q_tgt{in_target.orientation.x, in_target.orientation.y,
                 in_target.orientation.z, in_target.orientation.w};
      Vec3 target_r = quat_to_rotvec(q_tgt);

      // 种子选择：优先使用上一拍成功解，无解则使用当前实际关节角
      std::vector<double> seed = arm->have_last_solved ? arm->q_last_solved : arm->q_current;

      // ---- 1. 主解算器：微分逆解求解 ----
      auto diff_res = arm->diff_ik.solve(target_p, target_r, seed);
      std::vector<double> solved_q;
      bool success = false;

      if (diff_res.ok) {
        solved_q = diff_res.joints;
        success = true;
        ++arm->diff_ik_success_count;
        arm->last_residual_pos = diff_res.position_residual;
        arm->last_residual_rot = diff_res.rotation_residual;
      } else {
        // ---- 2. 兜底解算器：退化求解并施加位移惩罚 ----
        ++arm->fallback_count;
        std::vector<double> fb_q;
        bool fb_ok = false;
        if (arm->use_moveit) {
          fb_ok = arm->moveit.ik(target_p, target_r, seed, fb_q);
        } else {
          fb_ok = arm->chain.ik(target_p, target_r, seed, fb_q);
        }

        if (fb_ok && fb_q.size() == arm->joints.size()) {
          // 校验跳变惩罚：若与种子偏离过大，判定为跨引力盆翻转，进行保护性拒绝
          double max_jump = 0.0;
          for (std::size_t j = 0; j < arm->joints.size(); ++j) {
            max_jump = std::max(max_jump, std::fabs(fb_q[j] - seed[j]));
          }
          if (max_jump <= max_joint_jump_rad_) {
            solved_q = fb_q;
            success = true;
          } else {
            // 翻转跳变过大：限制步幅单向朝解靠近，防止突跳
            solved_q = seed;
            for (std::size_t j = 0; j < arm->joints.size(); ++j) {
              double dq = fb_q[j] - seed[j];
              dq = std::max(-max_joint_jump_rad_, std::min(max_joint_jump_rad_, dq));
              solved_q[j] += dq;
            }
            success = true;
          }
        } else {
          // 连兜底解算都无解：保持上一帧，并记录残差
          solved_q = seed;
        }
      }

      if (success) {
        // 严格施加关节速度与加速度物理极限平滑拦截（拒绝全域突跳与飞车）
        const double dt = 0.01;
        if (arm->have_last_solved) {
          for (std::size_t j = 0; j < arm->joints.size(); ++j) {
            double v = (solved_q[j] - arm->q_last_solved[j]) / dt;
            const double v_limit = (arm->limits.velocity_max[j] > 0.0) ?
                std::min(arm->limits.velocity_max[j], max_joint_vel_) : max_joint_vel_;
            if (std::fabs(v) > v_limit) {
              v = (v > 0.0 ? v_limit : -v_limit);
              solved_q[j] = arm->q_last_solved[j] + v * dt;
            }
            if (arm->v_last_solved.size() == arm->joints.size()) {
              double a = (v - arm->v_last_solved[j]) / dt;
              if (std::fabs(a) > max_joint_acc_) {
                a = (a > 0.0 ? max_joint_acc_ : -max_joint_acc_);
                v = arm->v_last_solved[j] + a * dt;
                solved_q[j] = arm->q_last_solved[j] + v * dt;
              }
            }
          }
        }
        if (arm->v_last_solved.size() != arm->joints.size()) {
          arm->v_last_solved.assign(arm->joints.size(), 0.0);
        }
        if (arm->have_last_solved) {
          for (std::size_t j = 0; j < arm->joints.size(); ++j) {
            arm->v_last_solved[j] = (solved_q[j] - arm->q_last_solved[j]) / dt;
          }
        }
        arm->q_last_solved = solved_q;
        arm->have_last_solved = true;
      }

      out_target.joint_positions = solved_q;
      out_msg.targets.push_back(out_target);
    }

    if (!out_msg.targets.empty()) {
      pub_joint_targets_->publish(out_msg);
    }
  }

  std::string input_target_topic_;
  std::string output_joint_topic_;
  std::string joint_states_topic_;
  std::string root_link_;
  std::string tip_link_left_;
  std::string tip_link_right_;
  std::string urdf_path_;
  std::string srdf_path_;
  std::string solver_plugin_;
  double max_joint_jump_rad_{0.35};
  double max_joint_vel_{2.0};
  double max_joint_acc_{12.0};

  std::vector<std::string> joints_left_;
  std::vector<std::string> joints_right_;

  std::vector<ArmIkContext> arms_;

  rclcpp::Subscription<move_msgs::msg::MoveTargetArray>::SharedPtr sub_targets_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
  rclcpp::Publisher<move_msgs::msg::MoveTargetArray>::SharedPtr pub_joint_targets_;
};

}  // namespace lite_move_controller

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lite_move_controller::IkNode>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

