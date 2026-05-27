#ifndef ROBOT_PVT_CONTROL__HERMITE_HPP_
#define ROBOT_PVT_CONTROL__HERMITE_HPP_

#include <vector>

namespace robot_pvt_control
{

/// One waypoint in an action-driven trajectory. `pos / vel / acc` are sized
/// to the controller's joint roster. `has_acc` is per-knot — applies to all
/// joints uniformly. When both endpoints of a segment have_acc, the segment
/// is sampled with a quintic Hermite (matches p, v, a at both endpoints).
/// Otherwise a cubic Hermite is used (matches p, v only).
struct Knot
{
  double t{0.0};
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> acc;
  bool has_acc{false};
};

struct Trajectory
{
  std::vector<Knot> knots;
  double duration{0.0};
};

/// Sample one segment between `a` and `b` at absolute time `t`. Output
/// vectors `p`, `v`, `a_out` must be pre-sized to the joint count (this is
/// an RT-path call; it does not allocate). Boundary conditions: `t <= a.t`
/// returns endpoint a; `t >= b.t` returns endpoint b.
void sample_segment(
  const Knot & a, const Knot & b, double t,
  std::vector<double> & p, std::vector<double> & v,
  std::vector<double> & a_out);

/// Sample the trajectory at absolute time `t`. Binary-search-free (linear
/// scan) — fine for the small knot counts produced by the action server.
/// Out-of-range `t` clamps to the endpoints; single-knot trajectories
/// return that knot.
void sample_trajectory(
  const Trajectory & traj, double t,
  std::vector<double> & p, std::vector<double> & v,
  std::vector<double> & a);

}  // namespace robot_pvt_control

#endif  // ROBOT_PVT_CONTROL__HERMITE_HPP_
