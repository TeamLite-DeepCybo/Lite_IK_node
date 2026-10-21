// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// test_ik_kdl.cpp —— KDL IK 后端的**集成测试**（用本机真实 URDF）
//
// 这一层不能只靠假求解器测：要验证的正是"KDL 能不能解出我们这条链"，
// 所以用 src/lite_controller/config/gravity/lite_arm_model.urdf
// （与深度标定同源、含 7 轴左臂 + gripper_tip_middle_link）。
//
// ⚠️ 路径通过环境变量 `LITE_ARM_URDF` 传入（CMake/命令行给），
//    找不到就**跳过**（返回 0 并打印 SKIP），而不是假通过。
#include "lite_ik_node/ik_kdl.hpp"
#include "lite_ik_node/ik_backend.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace lite_move_controller;

static int g_fail = 0, g_n = 0;
#define CHECK(c, ...) do { ++g_n; if(!(c)) { ++g_fail; \
  std::printf("  FAIL %d  ", __LINE__); std::printf(__VA_ARGS__); std::printf("\n"); } } while(0)

static std::string readFile(const std::string & p)
{
  std::ifstream f(p);
  if (!f) {return {};}
  std::stringstream ss; ss << f.rdbuf();
  return ss.str();
}

int main()
{
  const char * env = std::getenv("LITE_ARM_URDF");
  const std::string urdf_path = env ? env : "";
  if (urdf_path.empty()) {
    std::printf("SKIP: 未设置 LITE_ARM_URDF\n");
    return 0;
  }
  const std::string urdf = readFile(urdf_path);
  if (urdf.empty()) {
    std::printf("SKIP: 读不到 %s\n", urdf_path.c_str());
    return 0;
  }
  std::printf("[ik_kdl] URDF = %s (%zu 字节)\n", urdf_path.c_str(), urdf.size());

  const std::vector<std::string> L = {
      "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
      "left_elbow_pitch", "left_wrist_yaw", "left_wrist_roll", "left_wrist_pitch"};

  // ---- ① 建链（真实 URDF）----
  KdlArmChain chain;
  std::string err;
  CHECK(chain.build(urdf, "world_root", "left_gripper_tip_middle_link", L,
                    1e-6, 500, err), "建链失败：%s", err.c_str());
  if (!chain.ready()) { std::printf("\n%d checks, %d failures\n", g_n, g_fail); return 1; }
  CHECK(chain.dof() == 7, "自由度应为 7，得到 %zu", chain.dof());

  // ---- ② 关节顺序不符必须被拒（本项目最难查的一类故障）----
  {
    KdlArmChain bad;
    std::string e2;
    std::vector<std::string> swapped = L;
    std::swap(swapped[1], swapped[2]);          // 交换 roll/yaw
    CHECK(!bad.build(urdf, "world_root", "left_gripper_tip_middle_link", swapped,
                     1e-6, 200, e2), "关节顺序不符必须被拒");
    CHECK(e2.find("顺序") != std::string::npos || e2.find("不符") != std::string::npos,
          "拒绝原因应提到顺序/不符：%s", e2.c_str());
  }
  // ---- ③ 链不存在必须被拒 ----
  {
    KdlArmChain bad;
    std::string e3;
    CHECK(!bad.build(urdf, "world_root", "no_such_link", L, 1e-6, 200, e3),
          "不存在的 tip link 必须被拒");
  }

  // ---- ④ 正解：给定关节角应给出有限位姿 ----
  std::vector<double> q0 = {0.1, -0.2, 0.3, -0.4, 0.2, -0.1, 0.15};
  Vec3 p0, r0;
  CHECK(chain.fk(q0, p0, r0), "正解应成功");
  const double reach = std::sqrt(p0[0]*p0[0] + p0[1]*p0[1] + p0[2]*p0[2]);
  std::printf("  FK: p=(%.4f, %.4f, %.4f) |p|=%.4f m  |r|=%.4f rad\n",
              p0[0], p0[1], p0[2], reach, std::sqrt(r0[0]*r0[0]+r0[1]*r0[1]+r0[2]*r0[2]));
  CHECK(reach > 0.05 && reach < 1.5, "末端应在合理工作空间内，得到 %.4f m", reach);

  // ---- ⑤ 往返：IK(FK(q)) 应回到 q（同一分支）----
  {
    std::vector<double> q_sol;
    CHECK(chain.ik(p0, r0, q0, q_sol), "逆解应成功");
    CHECK(q_sol.size() == 7, "解长度应为 7");
    // 再用正解验证"解出来的 q 给出的位姿"与目标一致（比逐关节比较更本质）
    Vec3 p1, r1;
    CHECK(chain.fk(q_sol, p1, r1), "解的正解应成功");
    const double dp = std::sqrt((p1[0]-p0[0])*(p1[0]-p0[0]) +
                                (p1[1]-p0[1])*(p1[1]-p0[1]) +
                                (p1[2]-p0[2])*(p1[2]-p0[2]));
    const double dr = std::sqrt((r1[0]-r0[0])*(r1[0]-r0[0]) +
                                (r1[1]-r0[1])*(r1[1]-r0[1]) +
                                (r1[2]-r0[2])*(r1[2]-r0[2]));
    std::printf("  往返残差: 位置 %.3e m  姿态 %.3e rad\n", dp, dr);
    CHECK(dp < 1e-4, "往返位置残差应 <0.1 mm，得到 %.3e m", dp);
    CHECK(dr < 1e-3, "往返姿态残差应 <1e-3 rad，得到 %.3e rad", dr);
  }

  // ---- ⑥ 多个随机位形的往返统计（7 轴冗余臂的局部收敛会有失败，统计成功率）----
  {
    int ok = 0, tried = 0;
    double worst_dp = 0.0;
    std::vector<double> seed(7, 0.0);
    for (int i = 0; i < 40; ++i) {
      std::vector<double> q(7);
      for (int j = 0; j < 7; ++j) {
        q[static_cast<std::size_t>(j)] = 0.6 * std::sin(0.7 * i + 1.3 * j);
      }
      Vec3 p, r;
      if (!chain.fk(q, p, r)) {continue;}
      ++tried;
      std::vector<double> sol;
      if (!chain.ik(p, r, q, sol)) {seed = q; continue;}   // 用真实 q 当种子应成功
      Vec3 p2, r2;
      if (!chain.fk(sol, p2, r2)) {continue;}
      const double dp = std::sqrt((p2[0]-p[0])*(p2[0]-p[0]) +
                                  (p2[1]-p[1])*(p2[1]-p[1]) +
                                  (p2[2]-p[2])*(p2[2]-p[2]));
      worst_dp = std::max(worst_dp, dp);
      if (dp < 1e-4) {++ok;}
    }
    std::printf("  随机位形往返: %d/%d 成功，最差位置残差 %.3e m\n", ok, tried, worst_dp);
    CHECK(tried > 30, "应至少试了 30 个位形，得到 %d", tried);
    CHECK(ok == tried, "用真值当种子时应**全部**成功（%d/%d）", ok, tried);
  }

  // ---- ⑦ 与 IkOrchestrator 串起来：不可达目标应给残差解，不停止下发 ----
  {
    IkOrchestrator ik;
    IkOrchestrator::Config cfg;
    cfg.neutral.assign(7, 0.0);
    CHECK(ik.configure(cfg, 7), "configure");
    JointLimits lim;
    lim.position_min = {-3.1, -1.5, -1.5, -2.3, -1.5, -0.69, -0.78};
    lim.position_max = { 0.78, 1.9,  1.5,  0.0,  1.5,  0.69,  0.78};
    lim.velocity_max.assign(7, 1.0);
    lim.torque_max = {7,7,7,4,1.5,1.8,1.8};
    lim.stiffness_max.assign(7, 50.0);
    lim.damping_max.assign(7, 5.0);
    ik.setLimits(lim);
    ik.setSolver(makeKdlSolveFn(&chain), makeKdlFkFn(&chain));
    CHECK(ik.ready(), "IK 编排器应就绪");

    // (a) 可达目标 ⇒ SOLVED / SEND_SOLVED，残差≈0
    IkRequest req;
    req.seed = q0;
    req.position = p0;
    req.rotation = r0;
    auto res = ik.solve(req);
    CHECK(res.ok, "可达目标应成功");
    CHECK(res.outcome == SolverOutcome::Solved, "应 SOLVED，得到 %s",
          solverOutcomeName(res.outcome));
    CHECK(res.action == IkAction::SendSolved, "动作应为 SendSolved，得到 %d", (int)res.action);
    CHECK(res.positionResidualNorm() < 1e-3, "残差应≈0，得到 %.3e", res.positionResidualNorm());

    // (b) 不可达目标（离基座 5 m）⇒ **必须仍给出可用解**（残差解），不停止下发
    req.position = Vec3{5.0, 0.0, 0.0};
    req.rotation = Vec3{0, 0, 0};
    req.has_last_solution = true;
    req.last_solution = q0;
    res = ik.solve(req);
    CHECK(res.ok, "不可达时**必须**仍给出可用解（不停止下发）");
    CHECK(res.action == IkAction::SendResidual, "动作应为 SendResidual，得到 %d", (int)res.action);
    CHECK(res.residual_kind == TaskResidual::Residual, "应标为 RESIDUAL");
    CHECK(res.joints.size() == 7, "残差解应有 7 个关节");
    // 残差应≈到目标的总距离 5 m 减去"走出去的那一段"（≥4 m 说明确实朝目标走了）
    CHECK(res.positionResidualNorm() > 4.0 && res.positionResidualNorm() < 5.0,
          "残差应在 4~5 m 之间（应朝目标前进），得到 %.3f", res.positionResidualNorm());
    std::printf("  5 m 外: action=%d 残差=%.3f m 走了 %u 步 note=%s\n",
                (int)res.action, res.positionResidualNorm(), res.iterations, res.note.c_str());
    // 解必须在限位内
    for (std::size_t j = 0; j < 7; ++j) {
      CHECK(res.joints[j] >= lim.position_min[j] - 1e-9 &&
            res.joints[j] <= lim.position_max[j] + 1e-9,
            "关节 %zu 必须在限位内，得到 %.4f", j, res.joints[j]);
    }

    // (c) **贴近可达边界**（最常见、也最需要残差解的场景）：
    //     从当前可达点再往外推 8 cm —— 超出工作空间，但应该能走过去一部分。
    //     ⚠️ 这才是本功能的主用例：目标"略微超出"时给上游一个可迭代的残差解。
    req.position = Vec3{p0[0] + 0.08, p0[1], p0[2]};
    req.rotation = r0;                       // 姿态保持不变（与当前一致）
    req.has_last_solution = true;
    req.last_solution = q0;
    res = ik.solve(req);
    CHECK(res.ok, "贴近边界的超程目标必须仍给出可用解");
    CHECK(res.joints.size() == 7, "应有 7 个关节");
    CHECK(res.positionResidualNorm() < 0.08,
          "应朝目标前进（残差 < 0.08 m），得到 %.4f", res.positionResidualNorm());
    std::printf("  边界外 8cm: action=%d 残差=%.4f m 走了 %u 步\n",
                (int)res.action, res.positionResidualNorm(), res.iterations);
    for (std::size_t j = 0; j < 7; ++j) {
      CHECK(res.joints[j] >= lim.position_min[j] - 1e-9 &&
            res.joints[j] <= lim.position_max[j] + 1e-9,
            "关节 %zu 必须在限位内，得到 %.4f", j, res.joints[j]);
    }
  }

  std::printf("\n%d checks, %d failures\n", g_n, g_fail);
  return g_fail == 0 ? 0 : 1;
}
