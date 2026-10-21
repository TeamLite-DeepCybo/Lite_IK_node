// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// ik_moveit.cpp —— MoveIt `kinematics::KinematicsBase` 插件后端的实现
//                   （设计取舍与三个关键坑见 ik_moveit.hpp 顶部）
#include "lite_ik_node/ik_moveit.hpp"

#include <moveit/kinematics_base/kinematics_base.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

namespace lite_move_controller
{

// ---------------------------------------------------------------------------
// 内部实现（把 MoveIt / pluginlib 的头文件关在 .cpp 里，头文件只留 pimpl）
// ---------------------------------------------------------------------------
struct MoveItArmKinematics::Impl
{
  // ⚠️ **声明顺序 = 析构顺序的反序**，这里刻意排成：
  //      node → robot_model → urdf/srdf → loader → solver → state
  //    于是析构时 **solver 先于 loader** 销毁 —— pluginlib 的 ClassLoader
  //    析构时如果还有它创建的实例活着，会打
  //        "SEVERE WARNING!!! Attempting to unload library while objects
  //         created by this loader exist in the heap!"
  //    并**拒绝卸载**库（实测：每条臂一次，日志很吓人，而且库永远赖在内存里）。
  //    实测过反过来声明就会刷这两条警告。
  rclcpp::Node::SharedPtr node;
  moveit::core::RobotModelPtr robot_model;
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  std::unique_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase>> loader;
  std::shared_ptr<kinematics::KinematicsBase> solver;
  /// 复用同一个 RobotState 做正解：正解在 100 Hz 的实时路径上会被反复调用，
  /// 每次新建一个（14 个关节 + 连杆变换）没必要；**所有访问都在 mtx 下**。
  std::unique_ptr<moveit::core::RobotState> state;

  /// 我们的下标 j → 插件数组里的下标。
  std::vector<std::size_t> to_solver;
  /// 插件数组下标 → 我们的下标（解回来时用）。
  std::vector<std::size_t> from_solver;
  double timeout_sec{0.05};

  /// 每个关节的 URDF 限位（**本项目顺序**），只用于把种子钳到限位内。
  /// 连续关节（URDF limits 为空）用 ±inf，等于不钳。
  ///
  /// ⚠️ 为什么要钳（见头文件 ③）：pick_ik 见到"种子贴/越限位"会**换成随机种子**
  ///    再解。而编排器 `clampToLimits()` 会把解正好裁在限位**上**，于是下一拍
  ///    的种子落在限位上 ⇒ pick_ik 每拍都随机重启 ⇒ 连续解不出来 ⇒ 连续多拍
  ///    "预算用尽 → 保持上一帧"。真机崩溃前那 4 连发预算告警就是这条路径。
  ///    ⚠️ 钳的是**种子的副本**，不是解；解仍然由编排器统一裁剪（安全逻辑不变）。
  std::vector<double> seed_lo;
  std::vector<double> seed_hi;
  /// 钳进限位内侧的余量（rad）：足够让插件的比较浮点上也过，又不会明显挪动种子。
  double seed_eps{1e-6};

  // ---- "一拨调用"的时间预算（见 ik() 里的说明）---------------------------
  /// 同一"拨"里已经花掉的求解时间（ms）。
  mutable double burst_spent_ms{0.0};
  /// 上一拨的结束时刻。
  mutable std::chrono::steady_clock::time_point last_end{};
  mutable bool have_last_end{false};
  /// 空闲超过这么久就算"新的一拨"（= 新的控制周期）。本层控制周期 10 ms，
  /// 一拨内部的多次调用之间只隔几微秒，所以 1 ms 足够区分（实测）。
  double burst_idle_ms{1.0};
  /// **一拨**的总时间预算（ms）。<=0 = 不限制。
  ///
  /// ⚠️ 为什么必须有它（实测数据）：
  ///    `IkOrchestrator` 失败时走"自适应增量前进"（最多 64 步，每步还可能步长
  ///    减半重试好几次）。KDL 一步 ~1 ms 且几乎不失败，那条路径只要 20~44 ms；
  ///    而 pick_ik **达不到收敛阈值时会一直用满超时**，实测一个"5 m 外"的不可达
  ///    目标，一次 `IkOrchestrator::solve()` 要 **1.32 s**（66 次调用 × 20 ms）。
  ///    这远超桥的 `command_timeout_ms = 250`：桥会判本层"死了"并把命令全丢掉
  ///    （症状：目标还在流，机器人却不动了）。
  ///    预算用完就**快速失败**（返回 false），让编排器的失败路径立刻收敛到
  ///    HoldLast（= 继续下发上一帧位置，原地保持）。方向是保守的，与设计文档
  ///    §4 的 HoldLast 语义一致；代价只是"这一次的残差解走得少一点"。
  double burst_budget_ms{50.0};
  /// 预算第一次被打穿时 WARN 一次（不能每帧刷 rosout）。
  mutable bool budget_warned{false};

  /// ⚠️ 必须加锁：本节点用的是 `MultiThreadedExecutor` + **Reentrant** callback
  ///    group（见 move_controller_node.cpp 的 main/定时器注释），所以
  ///    `~/ik_query`（服务线程）与 100 Hz 控制 tick（定时器线程）**会并发**调到
  ///    同一条臂的同一个插件实例上。KDL 的 `CartToJnt` 同样不是线程安全的，
  ///    但既有代码只在一个线程里用；换成插件后必须显式串行化，否则插件内部的
  ///    工作区（pick_ik 的 robot_/fk_mutex_ 等）会被踩，症状是偶发 NaN 或
  ///    "同一条命令两次解出不同的解"，极难复现。
  mutable std::mutex mtx;
};

MoveItArmKinematics::MoveItArmKinematics() : impl_(std::make_unique<Impl>()) {}
MoveItArmKinematics::~MoveItArmKinematics() = default;
// 移动操作在这里（而非头文件）定义：Impl 在此处是完整类型
MoveItArmKinematics::MoveItArmKinematics(MoveItArmKinematics &&) noexcept = default;
MoveItArmKinematics & MoveItArmKinematics::operator=(MoveItArmKinematics &&) noexcept = default;

namespace
{

/// 位置 + 四元数（x,y,z,w）→ Vec3 位置 + 旋转向量（与内部约定一致）。
bool poseToVec(const geometry_msgs::msg::Pose & pose, Vec3 & position, Vec3 & rotation)
{
  position = Vec3{pose.position.x, pose.position.y, pose.position.z};
  rotation = quat_to_rotvec(Quat{pose.orientation.x, pose.orientation.y,
                                 pose.orientation.z, pose.orientation.w});
  return vec3_finite(position) && vec3_finite(rotation);
}

/// 位姿（Eigen）→ Vec3 位置 + 旋转向量。
/// 走 `quat_to_rotvec` 而不是自己写 atan2：保证与 input_filter / shape_traj
/// **同一套约定**（w<0 取反、最小表示），否则两个后端的"姿态残差"不是同一个量。
void isometryToVec(const Eigen::Isometry3d & T, Vec3 & position, Vec3 & rotation)
{
  position = Vec3{T.translation().x(), T.translation().y(), T.translation().z()};
  const Eigen::Quaterniond q(T.linear());
  rotation = quat_to_rotvec(Quat{q.x(), q.y(), q.z(), q.w()});
}

std::string readTextFile(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {return {};}
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

}  // namespace

bool MoveItArmKinematics::build(const std::string & urdf_xml,
                                const std::string & srdf_xml,
                                const Options & opt,
                                std::string & error_out)
{
  ready_ = false;
  plugin_fk_usable_ = false;
  dof_ = 0;
  last_error_.clear();
  joint_names_ = opt.joint_names;
  plugin_name_ = opt.plugin;
  group_name_ = opt.group_name;
  base_frame_ = opt.base_frame;
  tip_frame_ = opt.tip_frame;

  // ---- ① 参数自检（不完整就直接失败，不要"尽力而为"）----------------------
  if (urdf_xml.empty()) {
    error_out = "URDF 文本为空（robot_description 是否已发布？）";
    return false;
  }
  if (srdf_xml.empty()) {
    error_out = "SRDF 文本为空 —— 没有 group 定义，插件无法确定关节集合";
    return false;
  }
  if (opt.plugin.empty()) {
    error_out = "插件名为空（ik.solver_plugin 未配置）";
    return false;
  }
  if (opt.group_name.empty() || opt.base_frame.empty() || opt.tip_frame.empty()) {
    error_out = "group_name / base_frame / tip_frame 有空值";
    return false;
  }
  if (opt.joint_names.empty()) {
    error_out = "joint_names 为空";
    return false;
  }
  if (!(opt.timeout_sec > 0.0) || !std::isfinite(opt.timeout_sec)) {
    error_out = "timeout_sec 非法（单位是**秒**）";
    return false;
  }
  // 插件要在内部建 rclcpp 参数/日志，**必须**有一个已初始化的 context。
  // ⚠️ 这里不自己 `rclcpp::init()`：库代码初始化全局 context 会与应用抢所有权
  //    （重复 init、退出时 rclcpp::shutdown 顺序全乱）。调用方（本节点 / 离线
  //    测试程序）必须已经 init —— 没 init 就明确报错，而不是埋一个偶发崩溃。
  if (!rclcpp::ok()) {
    error_out = "rclcpp 未初始化（插件需要 rclcpp::Node 才能载入）";
    return false;
  }

  impl_->timeout_sec = opt.timeout_sec;

  // ---- ② URDF / SRDF → RobotModel ---------------------------------------
  impl_->urdf_model = urdf::parseURDF(urdf_xml);
  if (!impl_->urdf_model) {
    error_out = "urdf::parseURDF 失败（URDF 解析不了）";
    return false;
  }
  // ⚠️ tip link 先自己查一遍：让"名字打错"在这里就报出来，而不是等插件
  //    initialize 返回 false（那时错误信息是插件内部的，往往只有一句泛泛的
  //    "failed to initialize"）。
  if (!impl_->urdf_model->getLink(opt.tip_frame)) {
    error_out = "URDF 里没有 tip link '" + opt.tip_frame + "'";
    return false;
  }
  impl_->srdf_model = std::make_shared<srdf::Model>();
  if (!impl_->srdf_model->initString(*impl_->urdf_model, srdf_xml)) {
    error_out = "srdf::Model::initString 失败（SRDF 解析不了）";
    return false;
  }
  try {
    impl_->robot_model =
      std::make_shared<moveit::core::RobotModel>(impl_->urdf_model, impl_->srdf_model);
  } catch (const std::exception & e) {
    // RobotModel 的构造函数会抛 moveit::Exception（SRDF 引用了 URDF 里没有的
    // link/joint 时）。不接住的话异常会一路抛到节点构造函数里 → 整个节点起不来，
    // 而这里只是"某个后端不可用"，应该降级而不是致命。
    error_out = std::string("RobotModel 构造失败：") + e.what();
    return false;
  }
  const moveit::core::JointModelGroup * jmg = impl_->robot_model->getJointModelGroup(opt.group_name);
  if (jmg == nullptr) {
    std::string groups;
    for (const auto & g : impl_->robot_model->getJointModelGroupNames()) {
      groups += (groups.empty() ? "" : ", ") + g;
    }
    error_out = "SRDF 里没有 group '" + opt.group_name + "'（有：" + groups + "）";
    return false;
  }
  // 关节集合必须与配置**逐个对上**（顺序可以不同，名字必须一致）。
  // ⚠️ 这一层是"重排"的前提：名字集合不等 ⇒ 说明 URDF/SRDF 与我们配的
  //    joints_left/right 不是同一套东西，此时重排出来的就是垃圾。
  {
    std::vector<std::string> ours = opt.joint_names;
    std::vector<std::string> theirs = jmg->getVariableNames();
    std::sort(ours.begin(), ours.end());
    std::sort(theirs.begin(), theirs.end());
    if (ours != theirs) {
      error_out = "group '" + opt.group_name + "' 的关节集合(" +
                  std::to_string(theirs.size()) + " 个) 与配置的 joints(" +
                  std::to_string(ours.size()) + " 个) 不一致 —— 关节集合配置错误";
      return false;
    }
  }

  // ---- ③ 插件挂靠的节点 -------------------------------------------------
  // 优先复用调用方给的节点（节点侧传的是**主节点**的 shared_ptr）。这条路径成立
  // 的前提是"建臂"已经不在构造函数里了 —— 见 move_controller_node.cpp 的
  // `initialize()` 与 ik_moveit.hpp 里 `Options::node` 的说明。
  //
  // 只有"没给节点"时才自建一个（离线测试程序、库的其它使用者）：
  //   · 插件要往节点上声明参数（pick_ik：`robot_description_kinematics.<group>.*`
  //     共 25+ 个），挂主节点会让 `/lite_move_controller` 的参数表变长 —— 这是
  //     **预期**行为，而且换来"可以用 ros2 param set 调插件参数"；
  //   · 自建节点不会被 executor spin：插件只用它拿参数和 logger，不需要 spin。
  impl_->node = opt.node ? opt.node
                         : rclcpp::Node::make_shared("lite_move_controller_ik_" + opt.group_name);

  // ---- ④ 载插件 ---------------------------------------------------------
  try {
    // 基类包名是 `moveit_core`（`kinematics::KinematicsBase` 所在包）。
    // ⚠️ 写错成 `moveit_kinematics` 或漏掉 `kinematics::` 前缀都会在**运行时**
    //    才炸（"Unable to find class"），编译期毫无提示。
    impl_->loader = std::make_unique<pluginlib::ClassLoader<kinematics::KinematicsBase>>(
        "moveit_core", "kinematics::KinematicsBase");
    impl_->solver = impl_->loader->createSharedInstance(opt.plugin);
  } catch (const std::exception & e) {
    error_out = "载入插件 '" + opt.plugin + "' 失败：" + e.what() +
                "（是否装了 ros-jazzy-pick-ik / ros-jazzy-trac-ik-kinematics-plugin？）";
    return false;
  }
  if (!impl_->solver) {
    error_out = "插件 '" + opt.plugin + "' 创建后为空指针";
    return false;
  }

  // ---- ⑤ initialize（MoveIt 2 / Jazzy 的新签名：带 node + RobotModel）----
  bool ok = false;
  try {
    ok = impl_->solver->initialize(impl_->node, *impl_->robot_model, opt.group_name,
                                   opt.base_frame, {opt.tip_frame}, opt.search_discretization);
  } catch (const std::exception & e) {
    error_out = std::string("插件 initialize 抛异常：") + e.what();
    return false;
  }
  if (!ok) {
    error_out = "插件 '" + opt.plugin + "' initialize 返回 false（group=" + opt.group_name +
                ", base=" + opt.base_frame + ", tip=" + opt.tip_frame + "）";
    return false;
  }
  impl_->solver->setDefaultTimeout(opt.timeout_sec);

  // ---- ⑥ 插件精度与编排器判据对齐（见头文件 ③）--------------------------
  // 参数名是 MoveIt 2 的标准前缀（实测 pick_ik 用 `robot_description_kinematics.
  // <group>.position_threshold`），不是裸名。只在插件**已经声明**了该参数时
  // 才 set（否则 rclcpp 会抛 ParameterNotDeclaredException，把一条本来能用的臂
  // 直接弄死）；因此这里用 has_parameter 兜底，并对异常再包一层。
  //
  // 实测（本机、两臂各 150 个限位内位形，种子 = 真值 ±0.05 rad）：
  //   阈值 1e-4/1e-4 → 右臂 116/150 成功，平均 11.7 ms，最差 50 ms（撞超时）
  //   阈值 3e-4/1e-3 → 右臂 150/150 成功，平均  4.1 ms，最差 14 ms
  // 且 3e-4 的解仍落在编排器的 Exact 门（0.5 mm / 5 mrad）之内 ⇒ 判 EXACT。
  // ⚠️ 这就是"阈值既不能松（会被判 RESIDUAL）也不能紧（会 return false，把本来
  //    能解的位形推给增量前进）"的具体取值来源，别再凭感觉改。
  if (opt.plugin.find("pick_ik") != std::string::npos) {
    const std::string pre = "robot_description_kinematics." + opt.group_name + ".";
    const std::pair<const char *, double> knobs[] = {
      {"position_threshold", opt.position_threshold},
      {"orientation_threshold", opt.orientation_threshold},
    };
    for (const auto & kv : knobs) {
      const std::string name = pre + kv.first;
      if (!impl_->node->has_parameter(name)) {continue;}
      try {
        impl_->node->set_parameter(rclcpp::Parameter(name, kv.second));
      } catch (const std::exception &) {
        // 阈值不在插件允许的范围里时保持插件默认值 —— 精度差一点是**可接受**的
        // 降级（编排器会把可达目标判成 RESIDUAL），但把臂弄死不可接受。
      }
    }
  }

  // ---- ⑦ 关节顺序置换表 -------------------------------------------------
  solver_joint_names_ = impl_->solver->getJointNames();
  if (solver_joint_names_.size() != opt.joint_names.size()) {
    error_out = "插件报告 " + std::to_string(solver_joint_names_.size()) +
                " 个关节，配置的是 " + std::to_string(opt.joint_names.size()) + " 个";
    return false;
  }
  impl_->to_solver.assign(opt.joint_names.size(), 0);
  impl_->from_solver.assign(opt.joint_names.size(), 0);
  for (std::size_t j = 0; j < opt.joint_names.size(); ++j) {
    const auto it = std::find(solver_joint_names_.begin(), solver_joint_names_.end(),
                              opt.joint_names[j]);
    if (it == solver_joint_names_.end()) {
      error_out = "插件没有关节 '" + opt.joint_names[j] + "'（插件给出的是另一套关节）";
      return false;
    }
    const std::size_t k = static_cast<std::size_t>(it - solver_joint_names_.begin());
    impl_->to_solver[j] = k;
    impl_->from_solver[k] = j;
  }
  // link 也要对得上：插件只认它 initialize 时拿到的那个 tip。
  if (impl_->solver->getTipFrames().size() != 1 ||
      impl_->solver->getTipFrames().front() != opt.tip_frame) {
    error_out = "插件报告的 tip frame 与配置不符";
    return false;
  }

  dof_ = opt.joint_names.size();

  // ---- ⑦b 关节限位（把种子钳在限位内，见头文件 ③）-----------------------
  impl_->seed_lo.assign(dof_, -std::numeric_limits<double>::infinity());
  impl_->seed_hi.assign(dof_, std::numeric_limits<double>::infinity());
  for (std::size_t j = 0; j < dof_; ++j) {
    const auto joint = impl_->urdf_model->getJoint(opt.joint_names[j]);
    if (!joint || !joint->limits) {continue;}          // 连续关节 / 取不到 ⇒ 不钳
    const double lo = joint->limits->lower;
    const double hi = joint->limits->upper;
    if (!(hi > lo)) {continue;}
    impl_->seed_lo[j] = lo + impl_->seed_eps;
    impl_->seed_hi[j] = hi - impl_->seed_eps;
  }

  // ---- ⑧ 正解通道：插件 getPositionFK 可用吗？----------------------------
  // ⚠️ pick_ik 1.1.2 的这个函数是**空实现**（反汇编只有 `xor eax,eax; ret`，
  //    恒 false 且不打日志）。只试一次就定下策略，不能每帧试 —— 每帧试会在
  //    100 Hz 的控制循环里静默多花一次函数调用，而且失败时无从解释。
  try {
    std::vector<double> zeros(dof_, 0.0);
    std::vector<double> q_solver(dof_, 0.0);
    for (std::size_t j = 0; j < dof_; ++j) {q_solver[impl_->to_solver[j]] = zeros[j];}
    std::vector<geometry_msgs::msg::Pose> poses(1);
    plugin_fk_usable_ = impl_->solver->getPositionFK({opt.tip_frame}, q_solver, poses) &&
                        poses.size() == 1 &&
                        std::isfinite(poses[0].position.x) &&
                        std::isfinite(poses[0].orientation.w);
  } catch (const std::exception &) {
    plugin_fk_usable_ = false;
  }
  if (!plugin_fk_usable_) {
    // 只在 build 期 WARN 一次（RCLCPP_WARN 每帧打会把 rosout 刷满）。
    RCLCPP_WARN(impl_->node->get_logger(),
                "插件 %s 的 getPositionFK 不可用（pick_ik 1.1.2 是空实现）→ "
                "正解改用 RobotModel/RobotState 计算（同一个模型，数值等价）",
                opt.plugin.c_str());
  }

  // 正解用的 RobotState（默认值先填好；之后每次只覆盖本臂的 7 个变量）
  impl_->state = std::make_unique<moveit::core::RobotState>(impl_->robot_model);
  impl_->state->setToDefaultValues();

  ready_ = true;
  return true;
}

bool MoveItArmKinematics::fk(const std::vector<double> & q, Vec3 & position, Vec3 & rotation) const
{
  position = Vec3{0, 0, 0};
  rotation = Vec3{0, 0, 0};
  if (!ready_ || impl_ == nullptr || q.size() != dof_) {return false;}
  for (double v : q) {
    if (!std::isfinite(v)) {return false;}
  }
  std::vector<double> q_solver(dof_, 0.0);
  for (std::size_t j = 0; j < dof_; ++j) {q_solver[impl_->to_solver[j]] = q[j];}

  std::lock_guard<std::mutex> lock(impl_->mtx);
  if (plugin_fk_usable_) {
    // poses 必须**预先定长**：有的插件（如 kdl_kinematics_plugin）会自己 resize，
    // 有的（trac_ik 的老实现）只按 size() 写，空 vector 进去直接越界/返回 false。
    std::vector<geometry_msgs::msg::Pose> poses(1);
    if (!impl_->solver->getPositionFK({tip_frame_}, q_solver, poses) || poses.size() != 1) {
      return false;
    }
    if (!poseToVec(poses[0], position, rotation)) {return false;}
    return vec3_finite(position) && vec3_finite(rotation);
  }

  // 回退通道：同一个 RobotModel 的 RobotState 正解。
  // ⚠️ 目标位姿是相对 `base_frame` 的，而 `getGlobalLinkTransform` 给的是**模型
  //    根系**下的位姿 —— 本机两者都是 world_root，但写死"模型根系"会在将来换
  //    基座时静默错一个固定变换（数值看着都对，只是差一个平移）。
  try {
    impl_->state->setVariablePositions(solver_joint_names_, q_solver);
    impl_->state->updateLinkTransforms();
  } catch (const std::exception &) {
    // 变量名/长度对不上（理论上 ⑦ 已经挡住了）：宁可失败，也不要给半截位姿。
    return false;
  }
  bool base_found = false;
  bool tip_found = false;
  // ⚠️ 取**值**而不是 `const&`：`getFrameTransform` 返回的是 RobotState 内部
  //    `global_link_transforms_` 的引用，第二次调用/后续 update 都可能让它变
  //    （将来换 MoveIt 版本时更是如此）。多拷两个 4x4 矩阵的代价可以忽略，
  //    换来的是"这里永远不会有悬垂引用"。
  const Eigen::Isometry3d T_base = impl_->state->getFrameTransform(base_frame_, &base_found);
  const Eigen::Isometry3d T_tip = impl_->state->getFrameTransform(tip_frame_, &tip_found);
  if (!base_found || !tip_found) {return false;}
  isometryToVec(T_base.inverse() * T_tip, position, rotation);
  return vec3_finite(position) && vec3_finite(rotation);
}

bool MoveItArmKinematics::ik(const Vec3 & position, const Vec3 & rotation,
                             const std::vector<double> & seed,
                             std::vector<double> & q_out) const
{
  if (!ready_ || impl_ == nullptr || seed.size() != dof_) {return false;}
  if (!vec3_finite(position) || !vec3_finite(rotation)) {return false;}
  for (double v : seed) {
    if (!std::isfinite(v)) {return false;}
  }

  // 目标：位置 + 四元数（插件要的是 geometry_msgs::Pose，**不是**旋转向量）。
  // 旋转向量 → 四元数走 `rotvec_to_quat`，与内部约定统一。
  geometry_msgs::msg::Pose pose;
  pose.position.x = position[0];
  pose.position.y = position[1];
  pose.position.z = position[2];
  const Quat q = rotvec_to_quat(rotation);
  pose.orientation.x = q[0];
  pose.orientation.y = q[1];
  pose.orientation.z = q[2];
  pose.orientation.w = q[3];

  // 种子按插件顺序排好再喂（见头文件 ①），并**钳到限位内侧**（见头文件 ③：
  // 不钳的话 pick_ik 会把它换成随机种子，IK 就变成抽奖了）。
  std::vector<double> seed_solver(dof_, 0.0);
  for (std::size_t j = 0; j < dof_; ++j) {
    const double v = std::min(impl_->seed_hi[j], std::max(impl_->seed_lo[j], seed[j]));
    seed_solver[impl_->to_solver[j]] = v;
  }

  // ⚠️ 预先把解数组定长：`KinematicsBase::searchPositionIK` 的约定是"插件自己
  //    resize"，但外部 .so 里 `solution[i] = ...` 之前忘了 resize 是不会报错的
  //    （只会踩坏堆）。我们给足长度，让这种实现也落在自己的缓冲区内。
  std::vector<double> sol(dof_, 0.0);
  moveit_msgs::msg::MoveItErrorCodes ec;
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(impl_->mtx);
    // ---- "一拨调用"的时间预算（为什么要有：见 Impl::burst_budget_ms 的注释）----
    // 判断"新的一拨"用**距上次调用结束的空闲时间**，而不是两次进入的时间差：
    // 失败路径上单次调用本身就要 20 ms，用进入时间差会把它们全判成"新的一拨"，
    // 预算就永远重置、等于没有。而一拨内部的相邻调用只隔几微秒，控制周期之间
    // 隔 ~10 ms，1 ms 的门槛能把两者干净分开。
    const auto t_enter = std::chrono::steady_clock::now();
    if (impl_->have_last_end) {
      const double idle_ms = std::chrono::duration<double, std::milli>(
          t_enter - impl_->last_end).count();
      if (idle_ms > impl_->burst_idle_ms) {impl_->burst_spent_ms = 0.0;}
    }
    if (impl_->burst_budget_ms > 0.0 && impl_->burst_spent_ms > impl_->burst_budget_ms) {
      impl_->last_end = t_enter;          // 保持"仍在同一拨"的状态
      impl_->have_last_end = true;
      if (!impl_->budget_warned) {
        impl_->budget_warned = true;
        // 只 WARN 一次：这是"目标不可达/插件解不动"的**后果**，不是每帧都要喊的错。
        // 但必须留下线索 —— 否则现场只看到"帧没更新"，不知道为什么。
        RCLCPP_WARN(impl_->node->get_logger(),
                    "本周期 IK 时间预算已用尽（%.0f ms/拨）→ 本帧保持上一帧；"
                    "通常是目标不可达或种子离目标太远（详见 ik_moveit.cpp 的说明）",
                    impl_->burst_budget_ms);
      }
      return false;
    }
    try {
      ok = impl_->solver->searchPositionIK(pose, seed_solver, impl_->timeout_sec, sol, ec);
    } catch (const std::exception &) {
      // 插件抛异常（参数越界、内部 std::out_of_range 等）时**当成失败**：
      // 让编排器去试下一个种子 / 走残差前进，而不是把异常抛进控制定时器。
      impl_->burst_spent_ms += std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - t_enter).count();
      impl_->last_end = std::chrono::steady_clock::now();
      impl_->have_last_end = true;
      return false;
    }
    const auto t_after = std::chrono::steady_clock::now();
    impl_->burst_spent_ms += std::chrono::duration<double, std::milli>(t_after - t_enter).count();
    impl_->last_end = t_after;
    impl_->have_last_end = true;
    // 一次成功的求解说明"这个目标解得出" ⇒ 清掉一次性告警标记，下次还能再报。
    if (ok) {impl_->budget_warned = false;}
  }
  // 成功判据按本项目的口径收紧：插件说成功还不够，**解必须有限且长度正确**。
  // 插件是外部 .so，返回 NaN/短数组是可能的，而编排器只在"求解器给过非法解"
  // 时才拒绝线搜索 —— 这里先挡一层，NaN 就到不了下发路径。
  if (!ok || sol.size() != dof_) {return false;}
  q_out.assign(dof_, 0.0);
  for (std::size_t k = 0; k < dof_; ++k) {
    const double v = sol[k];
    if (!std::isfinite(v)) {
      q_out.clear();
      return false;
    }
    // 插件的第 k 个 → 我们的第 from_solver[k] 个
    q_out[impl_->from_solver[k]] = v;
  }
  return true;
}

IkSolveFn makeMoveItSolveFn(const MoveItArmKinematics * arm)
{
  return [arm](const Vec3 & p, const Vec3 & r, const std::vector<double> & seed,
               std::vector<double> & out) -> bool {
    if (arm == nullptr || !arm->ready()) {return false;}
    return arm->ik(p, r, seed, out);
  };
}

FkFn makeMoveItFkFn(const MoveItArmKinematics * arm)
{
  return [arm](const std::vector<double> & q, Vec3 & p, Vec3 & r) -> bool {
    if (arm == nullptr || !arm->ready()) {return false;}
    return arm->fk(q, p, r);
  };
}

// ---------------------------------------------------------------------------
// SRDF 文本定位
// ---------------------------------------------------------------------------
std::string loadSrdfText(const std::string & explicit_path,
                         const std::vector<std::string> & extra_candidates,
                         std::vector<std::string> * tried_out)
{
  std::vector<std::string> cands;
  // ① 显式参数：给了就**只**用它（避免"我明明指定了 A，它却偷偷用了 B"）
  if (!explicit_path.empty()) {
    cands.push_back(explicit_path);
  } else {
    // ② 调用方给的安装目录候选
    for (const auto & c : extra_candidates) {
      if (!c.empty()) {cands.push_back(c);}
    }
    // ③ 源码树回退（开发态未安装本包时使用）。
    cands.push_back(std::string("src/lite_ik_node/config/model/") + kSrdfFileName);
  }
  for (const auto & c : cands) {
    if (tried_out != nullptr) {tried_out->push_back(c);}
    std::string txt = readTextFile(c);
    if (!txt.empty()) {return txt;}
  }
  return {};
}

}  // namespace lite_move_controller
