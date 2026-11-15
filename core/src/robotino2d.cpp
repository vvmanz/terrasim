// robotino2d — planar surrogate of a three-omni-wheel robot (Robotino) for fast calibration.
// Drives exactly the same SimLogic/OmniLogic (contact law, motor + controller, experiment schedule,
// dataset-format logging) through the same WheelIO interface as the Gazebo plugins; only the rigid-body
// integration is replaced:
//   chassis: x, y, yaw (+ velocities); wheel spins; normal loads from quasi-static equilibrium of the
//   tilting moments (CoM offset and height, load transfer by acceleration), passed to the penalty
//   contact through the wheel-centre height.
//
// usage: robotino2d <config.ini> [--dt 0.001]          (logs as in the INI, or TM_LOG / TM_DONE)
#include <cstdio>
#include <cstring>
#include <iostream>
#include "terramech/sim_logic.hpp"
using namespace terramech;


static bool solve3(double A[3][3], double b[3], double x[3]) {
  double M[3][4];
  for (int i = 0; i < 3; ++i) { for (int j = 0; j < 3; ++j) M[i][j] = A[i][j]; M[i][3] = b[i]; }
  for (int i = 0; i < 3; ++i) {
    int p = i; for (int r = i + 1; r < 3; ++r) if (std::fabs(M[r][i]) > std::fabs(M[p][i])) p = r;
    if (std::fabs(M[p][i]) < 1e-12) return false;
    for (int c = 0; c < 4; ++c) std::swap(M[i][c], M[p][c]);
    for (int r = 0; r < 3; ++r) if (r != i) { double f = M[r][i] / M[i][i]; for (int c = 0; c < 4; ++c) M[r][c] -= f * M[i][c]; }
  }
  for (int i = 0; i < 3; ++i) x[i] = M[i][3] / M[i][i];
  return true;
}

int main(int argc, char **argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: robotino2d config.ini [--dt 0.001]\n"); return 1; }
  double dt = 0.001;
  for (int k = 2; k + 1 < argc; ++k) if (!std::strcmp(argv[k], "--dt")) dt = std::atof(argv[k + 1]);
  SimLogic S; std::string err;
  if (!S.configure(argv[1], err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
  if (!S.omni_on) { std::fprintf(stderr, "robotino2d: config is not [robot] type = omni\n"); return 1; }
  OmniLogic &L = S.omni;
  const Config &cfg = S.cfg;
  auto *G = cfg.find("general");
  auto *RB = cfg.find("robot");
  const double m = Config::num(RB, "mass", 11.0), Iz = Config::num(RB, "Iz", 0.15), g = Config::num(G, "gravity", 9.81);
  const double cx = Config::num(RB, "com_x", 0.0), cy = Config::num(RB, "com_y", 0.0), hc = Config::num(RB, "com_h", 0.08);
  const double Jw = Config::num(RB, "Jw", 2.0e-3);
  const size_t nw = L.wheels.size();
  // state (world): position of the geometric centre, yaw; velocities of the CoM; wheel spins
  double X = 0, Y = 0, psi = 0, vX = 0, vY = 0, wz = 0;
  std::vector<double> om(nw, 0.0);
  std::vector<WheelIO> in(nw);
  double ax_r = 0, ay_r = 0;     // last acceleration of the CoM (robot frame) for load transfer
  const double kn = L.surf.kn;
  for (double t = 0; t <= L.t_end + 1e-9; t += dt) {
    const double c = std::cos(psi), s = std::sin(psi);
    // normal loads: sum N = m g ; sum (y_i - cy) N_i = -h m ay ; sum (x_i - cx) N_i = -h m ax  (robot frame)
    double A[3][3], b[3], N[3] = {m * g / 3, m * g / 3, m * g / 3};
    if (nw == 3) {
      for (size_t i = 0; i < 3; ++i) {
        double xi = L.R * std::cos(L.wheels[i].theta), yi = L.R * std::sin(L.wheels[i].theta);
        A[0][i] = 1; A[1][i] = yi - cy; A[2][i] = xi - cx;
      }
      b[0] = m * g; b[1] = -hc * m * ay_r; b[2] = -hc * m * ax_r;
      solve3(A, b, N);
    }
    // CoM velocity -> geometric-centre velocity (rigid body)
    const double rcx = c * cx - s * cy, rcy = s * cx + c * cy;   // CoM offset, world
    const double vcX = vX + wz * rcy, vcY = vY - wz * rcx;       // v_centre = v_com + w x (centre - com)
    for (size_t i = 0; i < nw; ++i) {
      const double th = L.wheels[i].theta;
      const double px = L.R * std::cos(th), py = L.R * std::sin(th);       // robot frame
      const double pX = c * px - s * py, pY = s * px + c * py;              // world offset from centre
      // wheel centre: in its Bekker rut (deformable surfaces) + penalty compression for the quasi-static load
      in[i].center = Vec3(X + pX, Y + pY, L.ground_z - L.contacts[i].z_filt + L.r - std::max(N[i], 0.0) / kn);
      in[i].vel = Vec3(vcX - wz * pY, vcY + wz * pX, 0.0);
      // axle points to the robot centre: positive spin drives along (-sin th, cos th) in the robot frame
      in[i].axle_left = Vec3(-std::cos(th + psi), -std::sin(th + psi), 0.0);
      in[i].angvel = in[i].axle_left * om[i];
      in[i].joint_vel = om[i];
    }
    double ch[6] = {X, Y, 0, 0, 0, psi};
    double dtq = 0, bf = 0;
    S.step(t, dt, in, ch, nullptr, nullptr, dtq, nullptr, bf);
    // dynamics
    double FX = 0, FY = 0, MZ = 0;
    for (size_t i = 0; i < nw; ++i) {
      const Vec3 &F = in[i].force;
      FX += F.x; FY += F.y;
      const double rX = in[i].center.x - (X + rcx), rY = in[i].center.y - (Y + rcy);
      MZ += rX * F.y - rY * F.x + in[i].torque.z;
      const double tau = in[i].torque.dot(in[i].axle_left) + in[i].motor_torque;
      om[i] += dt * tau / Jw;
    }
    const double aX = FX / m, aY = FY / m;
    vX += dt * aX; vY += dt * aY; wz += dt * MZ / Iz;
    ax_r = c * aX + s * aY; ay_r = -s * aX + c * aY;
    // integrate the geometric centre with its own velocity (semi-implicit)
    const double c2 = std::cos(psi), s2 = std::sin(psi);
    const double rcx2 = c2 * cx - s2 * cy, rcy2 = s2 * cx + c2 * cy;
    X += dt * (vX + wz * rcy2); Y += dt * (vY - wz * rcx2); psi += dt * wz;
  }
  return 0;
}
