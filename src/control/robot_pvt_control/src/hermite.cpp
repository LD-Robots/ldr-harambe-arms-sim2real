#include "robot_pvt_control/hermite.hpp"

#include <algorithm>
#include <cstddef>

namespace robot_pvt_control
{

void sample_trajectory(
  const Trajectory & traj, double t,
  std::vector<double> & p, std::vector<double> & v,
  std::vector<double> & a)
{
  if (traj.knots.size() == 1) {
    p = traj.knots[0].pos;
    v = traj.knots[0].vel;
    a = traj.knots[0].acc;
    return;
  }
  if (t <= traj.knots.front().t) {
    p = traj.knots.front().pos;
    v = traj.knots.front().vel;
    a = traj.knots.front().acc;
    return;
  }
  if (t >= traj.knots.back().t) {
    p = traj.knots.back().pos;
    v = traj.knots.back().vel;
    a = traj.knots.back().acc;
    return;
  }
  for (std::size_t i = 1; i < traj.knots.size(); ++i) {
    if (t <= traj.knots[i].t) {
      sample_segment(traj.knots[i - 1], traj.knots[i], t, p, v, a);
      return;
    }
  }
  p = traj.knots.back().pos;
  std::fill(v.begin(), v.end(), 0.0);
  std::fill(a.begin(), a.end(), 0.0);
}

void sample_segment(
  const Knot & a_k, const Knot & b_k, double t,
  std::vector<double> & p, std::vector<double> & v,
  std::vector<double> & a_out)
{
  const double h = b_k.t - a_k.t;
  if (h <= 0.0) {
    p = b_k.pos;
    v = b_k.vel;
    a_out = b_k.acc;
    return;
  }
  const double u = std::clamp((t - a_k.t) / h, 0.0, 1.0);
  const std::size_t n = a_k.pos.size();

  if (a_k.has_acc && b_k.has_acc) {
    // Quintic Hermite — matches (p, v, a) at both endpoints.
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double u4 = u3 * u;
    const double u5 = u4 * u;
    const double h0 = 1.0 - 10.0 * u3 + 15.0 * u4 - 6.0 * u5;
    const double h1 = u - 6.0 * u3 + 8.0 * u4 - 3.0 * u5;
    const double h2 = 0.5 * u2 - 1.5 * u3 + 1.5 * u4 - 0.5 * u5;
    const double h3 = 10.0 * u3 - 15.0 * u4 + 6.0 * u5;
    const double h4 = -4.0 * u3 + 7.0 * u4 - 3.0 * u5;
    const double h5 = 0.5 * u3 - u4 + 0.5 * u5;

    const double h0p = -30.0 * u2 + 60.0 * u3 - 30.0 * u4;
    const double h1p = 1.0 - 18.0 * u2 + 32.0 * u3 - 15.0 * u4;
    const double h2p = u - 4.5 * u2 + 6.0 * u3 - 2.5 * u4;
    const double h3p = 30.0 * u2 - 60.0 * u3 + 30.0 * u4;
    const double h4p = -12.0 * u2 + 28.0 * u3 - 15.0 * u4;
    const double h5p = 1.5 * u2 - 4.0 * u3 + 2.5 * u4;

    const double h0pp = -60.0 * u + 180.0 * u2 - 120.0 * u3;
    const double h1pp = -36.0 * u + 96.0 * u2 - 60.0 * u3;
    const double h2pp = 1.0 - 9.0 * u + 18.0 * u2 - 10.0 * u3;
    const double h3pp = 60.0 * u - 180.0 * u2 + 120.0 * u3;
    const double h4pp = -24.0 * u + 84.0 * u2 - 60.0 * u3;
    const double h5pp = 3.0 * u - 12.0 * u2 + 10.0 * u3;

    for (std::size_t j = 0; j < n; ++j) {
      p[j] = h0 * a_k.pos[j] + h1 * (a_k.vel[j] * h) + h2 * (a_k.acc[j] * h * h)
        + h3 * b_k.pos[j] + h4 * (b_k.vel[j] * h) + h5 * (b_k.acc[j] * h * h);
      v[j] = (h0p * a_k.pos[j] + h3p * b_k.pos[j]) / h
        + (h1p * a_k.vel[j] + h4p * b_k.vel[j])
        + (h2p * a_k.acc[j] + h5p * b_k.acc[j]) * h;
      a_out[j] = (h0pp * a_k.pos[j] + h3pp * b_k.pos[j]) / (h * h)
        + (h1pp * a_k.vel[j] + h4pp * b_k.vel[j]) / h
        + (h2pp * a_k.acc[j] + h5pp * b_k.acc[j]);
    }
  } else {
    // Cubic Hermite — matches (p, v) at both endpoints.
    const double u2 = u * u;
    const double u3 = u2 * u;
    const double h00 = 2.0 * u3 - 3.0 * u2 + 1.0;
    const double h10 = u3 - 2.0 * u2 + u;
    const double h01 = -2.0 * u3 + 3.0 * u2;
    const double h11 = u3 - u2;

    const double h00p = 6.0 * u2 - 6.0 * u;
    const double h10p = 3.0 * u2 - 4.0 * u + 1.0;
    const double h01p = -6.0 * u2 + 6.0 * u;
    const double h11p = 3.0 * u2 - 2.0 * u;

    const double h00pp = 12.0 * u - 6.0;
    const double h10pp = 6.0 * u - 4.0;
    const double h01pp = -12.0 * u + 6.0;
    const double h11pp = 6.0 * u - 2.0;

    for (std::size_t j = 0; j < n; ++j) {
      p[j] = h00 * a_k.pos[j] + h10 * (a_k.vel[j] * h)
        + h01 * b_k.pos[j] + h11 * (b_k.vel[j] * h);
      v[j] = (h00p * a_k.pos[j] + h01p * b_k.pos[j]) / h
        + h10p * a_k.vel[j] + h11p * b_k.vel[j];
      a_out[j] = (h00pp * a_k.pos[j] + h01pp * b_k.pos[j]) / (h * h)
        + (h10pp * a_k.vel[j] + h11pp * b_k.vel[j]) / h;
    }
  }
}

}  // namespace robot_pvt_control
