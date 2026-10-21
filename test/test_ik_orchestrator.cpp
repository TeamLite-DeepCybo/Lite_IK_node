// IK 编排器：用**假求解器**穷举三轴分类与"永不停止下发"
#include "lite_ik_node/ik_backend.hpp"
#include <cstdio>
#include <cmath>
using namespace lite_move_controller;
static int g_fail=0, g_n=0;
#define CHECK(c,...) do{++g_n; if(!(c)){++g_fail; std::printf("  FAIL %d  ",__LINE__);\
  std::printf(__VA_ARGS__); std::printf("\n");}}while(0)

static JointLimits mklim(std::size_t n=7){
  JointLimits L; L.position_min.assign(n,-1.5); L.position_max.assign(n,1.5);
  L.velocity_max.assign(n,1.0); L.torque_max.assign(n,7.0);
  L.stiffness_max.assign(n,50.0); L.damping_max.assign(n,5.0); return L;
}

int main(){
  // 假正解：末端位置 = 各关节角之和（便于构造"可达/不可达"）
  FkFn fk = [](const std::vector<double>& q, Vec3& p, Vec3& r){
    double s=0; for(double v:q) s+= v;
    p = Vec3{s, 0.0, 0.0}; r = Vec3{0,0,0}; return true; };

  IkOrchestrator ik;
  IkOrchestrator::Config cfg;
  cfg.neutral.assign(7, 0.0);
  cfg.max_seed_tries = 3;
  CHECK(ik.configure(cfg, 7), "configure");
  ik.setLimits(mklim());

  // 目标位置 = 0.7（7 个关节每个 0.1 即可）⇒ 求解器直接给解
  IkSolveFn exact = [](const Vec3& tgt, const Vec3&, const std::vector<double>&, std::vector<double>& out){
    out.assign(7, tgt[0]/7.0); return true; };
  ik.setSolver(exact, fk);
  IkRequest req; req.seed.assign(7, 0.0); req.position = Vec3{0.7,0,0};
  auto res = ik.solve(req);
  CHECK(res.ok, "应成功");
  CHECK(res.outcome==SolverOutcome::Solved, "应 SOLVED，得到 %s", solverOutcomeName(res.outcome));
  CHECK(res.residual_kind==TaskResidual::Exact, "残差应为 EXACT 得到 %s", taskResidualName(res.residual_kind));
  CHECK(res.action==IkAction::SendSolved, "动作应为 SendSolved");
  CHECK(res.positionResidualNorm() < 1e-9, "残差应≈0 得到 %.3e", res.positionResidualNorm());

  // 限位裁剪：解越限 ⇒ LimitClamped 且被记录
  IkSolveFn over = [](const Vec3&, const Vec3&, const std::vector<double>&, std::vector<double>& out){
    out.assign(7, 5.0); return true; };   // 远超上限 1.5
  ik.setSolver(over, fk);
  res = ik.solve(req);
  CHECK(res.ok, "裁剪后仍应可用");
  CHECK(res.outcome==SolverOutcome::LimitClamped, "应 LIMIT_CLAMPED 得到 %s",
        solverOutcomeName(res.outcome));
  CHECK(!res.saturated_joints.empty(), "应记录饱和关节");
  for(double v: res.joints) CHECK(v<=1.5+1e-12, "解必须被裁到限位内，得到 %.6f", v);

  // 求解器全失败 ⇒ 走线搜索；线搜索里也用同一求解器（对更近的目标成功）
  //   目标 10.5（每关节 1.5 正好在限位边界），种子 0 ⇒ 需要线搜索
  IkSolveFn only_near = [](const Vec3& tgt, const Vec3&, const std::vector<double>&, std::vector<double>& out){
    if (tgt[0] > 10.0) return false;            // 太远求解器失败
    out.assign(7, tgt[0]/7.0); return true; };
  ik.setSolver(only_near, fk);
  req.position = Vec3{100.0, 0, 0};             // 远超可达
  res = ik.solve(req);
  CHECK(res.ok, "无解时必须仍给出可用解（**不停止下发**）");
  CHECK(res.action==IkAction::SendResidual, "动作应为 SendResidual，得到 %d", (int)res.action);
  CHECK(res.residual_kind==TaskResidual::Residual, "应为 RESIDUAL");
  CHECK(res.positionResidualNorm() > 1.0, "残差应很大，得到 %.3f", res.positionResidualNorm());
  CHECK(!res.joints.empty(), "必须有解");

  // 连最小步都不行 ⇒ FAILED + HOLD_LAST（位置原地保持）
  IkSolveFn never = [](const Vec3&, const Vec3&, const std::vector<double>&, std::vector<double>&){ return false; };
  ik.setSolver(never, fk);
  res = ik.solve(req);
  CHECK(!res.ok, "全失败时 ok 应为 false");
  CHECK(res.action==IkAction::HoldLast, "动作应为 HoldLast，得到 %d", (int)res.action);
  CHECK(res.joints.empty(), "HoldLast 时不应给解（由外壳用上一帧）");
  CHECK(res.note.find("保持上一帧") != std::string::npos, "说明应提到保持上一帧：%s", res.note.c_str());

  // 非有限解必须被拒（不能把 NaN 关节角发下去）
  IkSolveFn nan_solver = [](const Vec3&, const Vec3&, const std::vector<double>&, std::vector<double>& out){
    out.assign(7, std::nan("")); return true; };
  ik.setSolver(nan_solver, fk);
  res = ik.solve(req);
  CHECK(res.action==IkAction::HoldLast, "NaN 解必须被拒 → HoldLast");

  // 多种子：第一个种子失败、第二个成功
  int calls = 0;
  IkSolveFn two_try = [&calls](const Vec3& tgt, const Vec3&, const std::vector<double>& seed,
                               std::vector<double>& out){
    ++calls;
    // 只有种子全为 0.5 时才成功（模拟"中性位形能解出"）
    bool is_neutral = true; for(double v:seed) if (std::fabs(v-0.5)>1e-12) {is_neutral=false;break;}
    if(!is_neutral) return false;
    out.assign(7, tgt[0]/7.0); return true; };
  cfg.neutral.assign(7, 0.5);
  CHECK(ik.configure(cfg, 7), "reconfigure");
  ik.setLimits(mklim());
  ik.setSolver(two_try, fk);
  req.seed.assign(7, 0.0); req.has_last_solution = false;
  res = ik.solve(req);
  CHECK(res.ok, "第二个种子应成功");
  CHECK(res.seed_tries >= 2, "应尝试 ≥2 个种子，得到 %u", res.seed_tries);
  CHECK(calls >= 2, "求解器应被调 ≥2 次，得到 %d", calls);

  // 配置校验
  IkOrchestrator bad;
  IkOrchestrator::Config bcfg; bcfg.neutral.assign(6,0.0);   // 长度不符
  CHECK(!bad.configure(bcfg, 7), "中性位形长度不符必须拒绝");
  IkOrchestrator bad2;
  IkOrchestrator::Config b2; b2.neutral.assign(7,0.0); b2.ik.residual_pos_m = -1.0;
  CHECK(!bad2.configure(b2, 7), "负阈值必须拒绝");

  // 未就绪时 solve 不得崩
  IkOrchestrator fresh;
  res = fresh.solve(req);
  CHECK(!res.ok && res.action==IkAction::HoldLast, "未就绪应 HoldLast");

  std::printf("\n%d checks, %d failures\n", g_n, g_fail);
  return g_fail==0?0:1;
}
