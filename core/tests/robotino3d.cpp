// robotino3d — 3D rigid-body check of the plugin path for omni robots, without Gazebo.
// Chassis: 6 DOF (quaternion), wheels: spin DOF on fixed axles (as revolute joints in the SDF).
// Every step does exactly what the Gazebo wrappers do: fill WheelIO from the link kinematics, call
// SimLogic::step, apply io.force at the wheel centre + io.torque to the wheel link, apply motor_torque on
// the joint (and its reaction on the chassis). Normal loads come out of the penalty contact (no
// quasi-static shortcut as in robotino2d), pitch/roll and motor reaction torques are included.
// Purpose: catch sign / frame / stability errors of the plugin path before running Gazebo; the run
// means must agree with robotino2d.
//
// usage: robotino3d <config.ini> [--dt 0.001]
#include <cstdio>
#include <cstring>
#include "terramech/sim_logic.hpp"
using namespace terramech;

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
  Vec3 rot(const Vec3 &v) const {       // q v q*
    Vec3 u(x, y, z); Vec3 t = u.cross(v) * 2.0;
    return v + t * w + u.cross(t);
  }
  Vec3 inv_rot(const Vec3 &v) const { Quat c{w, -x, -y, -z}; return c.rot(v); }
  void integrate(const Vec3 &om, double dt) {   // world angular velocity
    Quat d{0, om.x, om.y, om.z};
    Quat r{-d.x * x - d.y * y - d.z * z, d.w * x + d.x * w + d.y * z - d.z * y,
           d.w * y - d.x * z + d.y * w + d.z * x, d.w * z + d.x * y - d.y * x + d.z * w};
    w += 0.5 * dt * r.w; x += 0.5 * dt * r.x; y += 0.5 * dt * r.y; z += 0.5 * dt * r.z;
    double n = std::sqrt(w * w + x * x + y * y + z * z); w /= n; x /= n; y /= n; z /= n;
  }
  void euler(double &roll, double &pitch, double &yaw) const {
    roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    pitch = std::asin(clamp(2 * (w * y - z * x), -1.0, 1.0));
    yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }
  static Quat from_axes(const Vec3 &X, const Vec3 &Y, const Vec3 &Z) {   // columns of the rotation matrix
    Quat q; const double tr = X.x + Y.y + Z.z;
    if (tr > 0) { const double s = 2 * std::sqrt(tr + 1); q.w = 0.25 * s; q.x = (Y.z - Z.y) / s; q.y = (Z.x - X.z) / s; q.z = (X.y - Y.x) / s; }
    else if (X.x > Y.y && X.x > Z.z) { const double s = 2 * std::sqrt(1 + X.x - Y.y - Z.z); q.w = (Y.z - Z.y) / s; q.x = 0.25 * s; q.y = (Y.x + X.y) / s; q.z = (Z.x + X.z) / s; }
    else if (Y.y > Z.z) { const double s = 2 * std::sqrt(1 + Y.y - X.x - Z.z); q.w = (Z.x - X.z) / s; q.x = (Y.x + X.y) / s; q.y = 0.25 * s; q.z = (Z.y + Y.z) / s; }
    else { const double s = 2 * std::sqrt(1 + Z.z - X.x - Y.y); q.w = (X.y - Y.x) / s; q.x = (Z.x + X.z) / s; q.y = (Z.y + Y.z) / s; q.z = 0.25 * s; }
    const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z); q.w /= n; q.x /= n; q.y /= n; q.z /= n; return q;
  }
};

int main(int argc, char **argv) {
  if (argc < 2) { std::fprintf(stderr, "usage: robotino3d config.ini [--dt 0.001]\n"); return 1; }
  double dt = 0.001;
  for (int k = 2; k + 1 < argc; ++k) if (!std::strcmp(argv[k], "--dt")) dt = std::atof(argv[k + 1]);
  SimLogic S; std::string err;
  if (!S.configure(argv[1], err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
  if (!S.omni_on) { std::fprintf(stderr, "robotino3d: config is not [robot] type = omni\n"); return 1; }
  OmniLogic &L = S.omni;
  auto *RB = S.cfg.find("robot");
  const double m = Config::num(RB, "mass", 11.0), Iz = Config::num(RB, "Iz", 0.15), g = 9.81;
  const double cx = Config::num(RB, "com_x", 0.0), cy = Config::num(RB, "com_y", 0.0), hc = Config::num(RB, "com_h", 0.08);
  const double Ixy = 0.6 * Iz;                       // as in the SDF
  const double Jw = Config::num(RB, "Jw", 2.0e-3);
  const size_t nw = L.wheels.size();
  const Vec3 com_b(cx, cy, hc);                      // CoM in the base frame (origin: centre on the floor)
  // state
  // rest pose: base tilted to the plane through the ground under the three wheels (a map may start on a slope)
  double hw[3]; Vec3 nn;
  for (int i = 0; i < 3 && i < (int)nw; ++i) L.ground_at(L.R * std::cos(L.wheels[i].theta), L.R * std::sin(L.wheels[i].theta), hw[i], nn);
  Vec3 np(0, 0, 1); double zp = hw[0];
  if (nw >= 3) {
    const Vec3 p0(L.R * std::cos(L.wheels[0].theta), L.R * std::sin(L.wheels[0].theta), hw[0]);
    const Vec3 p1(L.R * std::cos(L.wheels[1].theta), L.R * std::sin(L.wheels[1].theta), hw[1]);
    const Vec3 p2(L.R * std::cos(L.wheels[2].theta), L.R * std::sin(L.wheels[2].theta), hw[2]);
    np = (p1 - p0).cross(p2 - p0).normalized(); if (np.z < 0) np = np * -1.0;
    zp = p0.z - (np.x * (0 - p0.x) + np.y * (0 - p0.y)) / np.z;           // plane height at the base centre (0, 0)
  }
  const double z0 = zp - m * g / (3 * L.surf.kn) / np.z;                 // base origin height at rest
  const Vec3 ex = (Vec3(1, 0, 0) - np * np.x).normalized(), ey = np.cross(ex);
  Quat q = Quat::from_axes(ex, ey, np);
  Vec3 pc = Vec3(0, 0, z0) + q.rot(com_b), v(0, 0, 0), w(0, 0, 0);
  std::vector<double> om(nw, 0.0);
  std::vector<WheelIO> io(nw);
  double max_tilt = 0, min_N = 1e9, max_N = 0;
  for (double t = 0; t <= L.t_end + 1e-9; t += dt) {
    const Vec3 base = pc - q.rot(com_b);
    std::vector<Vec3> rw(nw), ax(nw);
    for (size_t i = 0; i < nw; ++i) {
      const double th = L.wheels[i].theta;
      const Vec3 pb(L.R * std::cos(th), L.R * std::sin(th), L.r);
      const Vec3 pw = base + q.rot(pb);
      rw[i] = pw - pc;
      ax[i] = q.rot(Vec3(-std::cos(th), -std::sin(th), 0));
      io[i].center = pw; io[i].vel = v + w.cross(rw[i]);
      io[i].axle_left = ax[i]; io[i].angvel = w + ax[i] * om[i]; io[i].joint_vel = om[i];
    }
    double r_, p_, y_; q.euler(r_, p_, y_);
    double ch[6] = {base.x, base.y, base.z, r_, p_, y_};
    const Vec3 vb = v + w.cross(base - pc);       // chassis-link origin velocity, as the engines report it
    S.chassis_vel[0] = vb.x; S.chassis_vel[1] = vb.y; S.chassis_vel[2] = vb.z;
    S.chassis_vel[3] = w.x; S.chassis_vel[4] = w.y; S.chassis_vel[5] = w.z; S.has_chassis_vel = true;
    double dtq = 0, bf = 0;
    S.step(t, dt, io, ch, nullptr, nullptr, dtq, nullptr, bf);
    Vec3 F(0, 0, -m * g), T(0, 0, 0);
    for (size_t i = 0; i < nw; ++i) {
      F += io[i].force;
      T += rw[i].cross(io[i].force);
      const double ts = io[i].torque.dot(ax[i]);
      T += io[i].torque - ax[i] * ts;                // transmitted through the bearings
      T += ax[i] * (-io[i].motor_torque);            // motor reaction on the chassis
      om[i] += dt * (ts + io[i].motor_torque) / Jw;
      const double N = io[i].force.z; if (t > 0.5) { min_N = std::min(min_N, N); max_N = std::max(max_N, N); }
    }
    // rigid body (world frame inertia)
    const Vec3 Lb = q.inv_rot(w);
    const Vec3 Iw_b(Ixy * Lb.x, Ixy * Lb.y, Iz * Lb.z);
    const Vec3 Hw = q.rot(Iw_b);
    const Vec3 rhs_b = q.inv_rot(T - w.cross(Hw));
    const Vec3 alpha = q.rot(Vec3(rhs_b.x / Ixy, rhs_b.y / Ixy, rhs_b.z / Iz));
    v += F * (dt / m); w += alpha * dt;
    pc += v * dt; q.integrate(w, dt);
    max_tilt = std::max(max_tilt, std::max(std::fabs(r_), std::fabs(p_)));
    double hg; Vec3 ng; L.ground_at(pc.x, pc.y, hg, ng);
    if (!std::isfinite(pc.x) || !std::isfinite(pc.z) || std::fabs(pc.z - hg) > 1.0) { std::fprintf(stderr, "robotino3d: diverged at t=%.3f\n", t); return 2; }
  }
  std::printf("robotino3d: OK, max tilt %.3f deg, wheel normal force %.1f .. %.1f N (static %.1f)\n",
              max_tilt * 180 / kPi, min_N, max_N, m * g / 3);
  return 0;
}
