// Copyright (c) 2026
// Licensed under the Apache License, Version 2.0
//
// Shared, ROS-independent data types used by the IK package and its clients.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace lite_move_controller
{

using Vec3 = std::array<double, 3>;
using Quat = std::array<double, 4>;
using JointVec = std::vector<double>;

inline Vec3 vec3_zero() {return Vec3{0, 0, 0};}

inline bool vec3_finite(const Vec3 & v) noexcept
{
  return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline double quat_norm(const Quat & q) noexcept
{
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

inline double quat_dot(const Quat & a, const Quat & b) noexcept
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

inline bool quat_normalize(Quat & q) noexcept
{
  const double n = quat_norm(q);
  if (!(n > 1e-9) || !std::isfinite(n)) {return false;}
  q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
  return true;
}

inline Vec3 quat_to_rotvec(const Quat & q_in) noexcept
{
  Quat q = q_in;
  if (!quat_normalize(q)) {return vec3_zero();}
  if (q[3] < 0.0) {q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];}
  const double s = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2]);
  if (s < 1e-12) {return vec3_zero();}
  const double angle = 2.0 * std::atan2(s, std::min(1.0, std::max(-1.0, q[3])));
  const double k = angle / s;
  return Vec3{q[0] * k, q[1] * k, q[2] * k};
}

inline Quat rotvec_to_quat(const Vec3 & rv) noexcept
{
  const double theta = std::sqrt(rv[0] * rv[0] + rv[1] * rv[1] + rv[2] * rv[2]);
  if (theta < 1e-12) {return Quat{0.0, 0.0, 0.0, 1.0};}
  const double half = 0.5 * theta;
  const double s = std::sin(half) / theta;
  return Quat{rv[0] * s, rv[1] * s, rv[2] * s, std::cos(half)};
}

struct Jacobian
{
  std::size_t rows{6};
  std::size_t cols{0};
  std::vector<double> data;

  void resize(std::size_t r, std::size_t c)
  {
    rows = r; cols = c;
    data.assign(r * c, 0.0);
  }
  double & at(std::size_t r, std::size_t c) noexcept {return data[r * cols + c];}
  double at(std::size_t r, std::size_t c) const noexcept {return data[r * cols + c];}
  bool valid() const noexcept {return cols > 0 && data.size() == rows * cols;}
};

struct JointLimits
{
  std::vector<double> position_min;
  std::vector<double> position_max;
  std::vector<double> velocity_max;
  std::vector<double> torque_max;
  std::vector<double> stiffness_max;
  std::vector<double> damping_max;
  std::vector<double> position_soft_band;

  std::size_t size() const noexcept {return position_min.size();}
  double softBand(std::size_t j) const noexcept
  {
    return (position_soft_band.size() == position_min.size()) ? position_soft_band[j] : 0.0;
  }
  bool valid() const noexcept
  {
    const std::size_t n = size();
    const bool band_ok = position_soft_band.empty() ||
      (position_soft_band.size() == n && check_finite(position_soft_band) &&
       check_non_negative(position_soft_band));
    return n > 0 && position_max.size() == n && velocity_max.size() == n &&
           torque_max.size() == n && stiffness_max.size() == n &&
           damping_max.size() == n && band_ok && check_finite(position_min) &&
           check_finite(position_max) && check_positive(velocity_max) &&
           check_positive(torque_max) && check_positive(stiffness_max) &&
           check_positive(damping_max);
  }

private:
  static bool check_finite(const std::vector<double> & v)
  {
    for (double x : v) {if (!std::isfinite(x)) {return false;}}
    return true;
  }
  static bool check_positive(const std::vector<double> & v)
  {
    for (double x : v) {if (!std::isfinite(x) || x <= 0.0) {return false;}}
    return true;
  }
  static bool check_non_negative(const std::vector<double> & v)
  {
    for (double x : v) {if (!std::isfinite(x) || x < 0.0) {return false;}}
    return true;
  }
};

}  // namespace lite_move_controller
