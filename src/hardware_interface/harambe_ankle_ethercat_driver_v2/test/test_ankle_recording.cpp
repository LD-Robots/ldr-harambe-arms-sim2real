// Table-vs-recording check: validates the cubic against a REAL recording from
// the robot, captured later. The recording is a CSV given via the environment
// variable ANKLE_RECORDING_CSV; the test SKIPs cleanly when it is unset/missing,
// so it never blocks CI before any recording exists.
//
// CSV format (one sample per line, '#' comments and blank lines ignored):
//   thetaA_inside_deg, thetaB_outside_deg, measured_pitch_deg, measured_roll_deg
//
// Capture tip: republish /joint_states in degrees with
//   ros2 run ethercat_tools joint_states_deg
// and log the ankle motor angles alongside an independent foot pitch/roll.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "harambe_ankle_ethercat_driver_v2/ankle_kinematics.hpp"

using harambe_ankle_ethercat_driver_v2::AnkleKinematics;
using harambe_ankle_ethercat_driver_v2::AnkleParams;
using harambe_ankle_ethercat_driver_v2::FkResult;

namespace
{
constexpr double kDeg = M_PI / 180.0;

struct Sample { double a, b, pitch, roll; };

bool parse_csv(const std::string & path, std::vector<Sample> & out, std::string & err)
{
  std::ifstream f(path);
  if (!f.is_open()) { err = "cannot open " + path; return false; }
  std::string line;
  int lineno = 0;
  while (std::getline(f, line)) {
    ++lineno;
    auto hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);
    // trim
    size_t a = line.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) continue;
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream ss(line);
    Sample s;
    if (!(ss >> s.a >> s.b >> s.pitch >> s.roll)) {
      err = "malformed line " + std::to_string(lineno);
      return false;
    }
    out.push_back(s);
  }
  return true;
}
}  // namespace

TEST(AnkleRecordingV2, CubicMatchesRecording)
{
  const char * env = std::getenv("ANKLE_RECORDING_CSV");
  if (!env || std::string(env).empty()) {
    GTEST_SKIP() << "ANKLE_RECORDING_CSV not set — no recording to compare yet";
  }
  std::vector<Sample> samples;
  std::string err;
  if (!parse_csv(env, samples, err)) {
    GTEST_SKIP() << "recording unreadable (" << err << ") — skipping";
  }
  ASSERT_FALSE(samples.empty()) << "recording " << env << " has no samples";

  auto k = AnkleKinematics::create("cubic");
  k->set_params(AnkleParams{});

  double sp = 0, sr = 0, mp = 0, mr = 0;
  for (const auto & s : samples) {
    const FkResult f = k->fk(s.a * kDeg, s.b * kDeg,
                             std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN());
    const double ep = f.pitch / kDeg - s.pitch;
    const double er = f.roll / kDeg - s.roll;
    sp += ep * ep; sr += er * er;
    mp = std::max(mp, std::abs(ep)); mr = std::max(mr, std::abs(er));
  }
  const double rms_p = std::sqrt(sp / samples.size());
  const double rms_r = std::sqrt(sr / samples.size());
  std::printf("[recording %s, %zu samples] pitch RMS=%.3f max=%.3f | roll RMS=%.3f max=%.3f\n",
              env, samples.size(), rms_p, mp, rms_r, mr);
  // Generous bound: this is real hardware with mounting/measurement noise.
  EXPECT_LT(rms_p, 2.0);
  EXPECT_LT(rms_r, 2.0);
}
