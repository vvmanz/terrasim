// vehicle3d — 3D rigid-body emulator of the plugin path for any robot in vehicle mode ([robot] type = vehicle):
// chassis 6 DOF (quaternion) + wheel spins on fixed axles; per step exactly what the Gazebo wrappers do.
// Wheel spins are absolute (as in the engines): the joint speed is spin - chassis rate about the axle, so a
// motor torque turns the wheel and, in reaction, the chassis. Casters ([wheel] J > 0) spin with their own
// inertia; their swivel is not modelled here (fixed axle), its torque is dropped.
// (WheelIO from link kinematics, chassis velocities, SimLogic::step, wheel wrench + motor torque with its
// reaction on the chassis, body-ground contact wrench of [body]). Suspension (rocker-bogie of the rover) is NOT
// modelled here: the chassis is rigid; Gazebo runs the real articulated model.
//
// usage: vehicle3d <config.ini> [--dt 0.001]
#include <cstdio>
#include <cstring>
#include "terramech/sim_logic.hpp"
using namespace terramech;

struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
  Vec3 rot(const Vec3 &v) const { Vec3 u(x, y, z); Vec3 t = u.cross(v) * 2.0; return v + t * w + u.cross(t); }
  Vec3 inv_rot(const Vec3 &v) const { Quat c{w, -x, -y, -z}; return c.rot(v); }
  void integrate(const Vec3 &om, double dt) {
    Quat d{0, om.x, om.y, om.z};
    Quat r{-d.x * x - d.y * y - d.z * z, d.w * x + d.x * w + d.y * z - d.z * y, d.w * y - d.x * z + d.y * w + d.z * x, d.w * z + d.x * y - d.y * x + d.z * w};
    w += 0.5 * dt * r.w; x += 0.5 * dt * r.x; y += 0.5 * dt * r.y; z += 0.5 * dt * r.z;
    double n = std::sqrt(w * w + x * x + y * y + z * z); w /= n; x /= n; y /= n; z /= n;
  }
  void euler(double &roll, double &pitch, double &yaw) const {
    roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    pitch = std::asin(clamp(2 * (w * y - z * x), -1.0, 1.0));
    yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }
  static Quat yaw(double a) { Quat q; q.w = std::cos(a / 2); q.z = std::sin(a / 2); return q; }
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
  if (argc < 2) { std::fprintf(stderr, "usage: vehicle3d config.ini [--dt 0.001]\n"); return 1; }
  double dt = 0.001;
  for (int k = 2; k + 1 < argc; ++k) if (!std::strcmp(argv[k], "--dt")) dt = std::atof(argv[k + 1]);
  SimLogic S; std::string err;
  if (!S.configure(argv[1], err)) { std::fprintf(stderr, "%s\n", err.c_str()); return 1; }
  if (!S.veh_on) { std::fprintf(stderr, "vehicle3d: config is not [robot] type = vehicle\n"); return 1; }
  VehicleLogic &L = S.veh;
  auto *RB = S.cfg.find("robot"); auto *VE = S.cfg.find("vehicle");
  const double m = Config::num(RB, "mass", 10.0), g = 9.81;
  const double Ixx = Config::num(RB, "Ixx", 0.2), Iyy = Config::num(RB, "Iyy", 0.2), Izz = Config::num(RB, "Izz", 0.3);
  const double Jw = Config::num(RB, "Jw", 2e-3);
  const size_t nw = L.wheels.size();
  const Vec3 com_b(L.com_x, L.com_y, L.com_h);
  const double sx = Config::num(VE, "spawn_x", 0), sy = Config::num(VE, "spawn_y", 0), syaw = deg2rad(Config::num(VE, "spawn_yaw_deg", 0));
  // place the base so that the wheels rest on the terrain under the spawn point: the chassis is tilted to the
  // least-squares plane through the ground under the wheels (on a slope a level spawn would push the uphill
  // wheels deep into the ground and throw the robot off)
  double h0; Vec3 n0; L.terrain.ground(sx, sy, h0, n0);
  double kn = L.terrain.flat.base.kn;
  Vec3 np(0, 0, 1); double zp = h0;
  {
    double A[3][3] = {{0}}, B[3] = {0};
    for (size_t i = 0; i < nw; ++i) {
      const auto &W = L.wheels[i];
      const double wx = sx + std::cos(syaw) * W.x - std::sin(syaw) * W.y, wy = sy + std::sin(syaw) * W.x + std::cos(syaw) * W.y;
      double hi; Vec3 ni; L.terrain.ground(wx, wy, hi, ni);
      const double r3[3] = {1.0, wx - sx, wy - sy};
      for (int a = 0; a < 3; ++a) { B[a] += r3[a] * hi; for (int b = 0; b < 3; ++b) A[a][b] += r3[a] * r3[b]; }
    }
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (nw >= 3 && std::fabs(det) > 1e-9) {
      auto solve = [&](int k) { double M[3][3]; for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) M[a][b] = (b == k) ? B[a] : A[a][b];
        return (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) + M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det; };
      zp = solve(0); np = Vec3(-solve(1), -solve(2), 1.0).normalized();
    } else { np = n0; }
  }
  double z_base = zp + ((L.wheels[0].g.r - L.wheels[0].z) - m * g / (nw * kn)) / np.z;   // chassis-link origin height
  const Vec3 hd(std::cos(syaw), std::sin(syaw), 0.0);
  const Vec3 ex = (hd - np * hd.dot(np)).normalized(), ez = np, ey = ez.cross(ex);
  Quat q = Quat::from_axes(ex, ey, ez);
  {   // on curved ground the plane leaves some wheels in the ground: lift until none penetrates (stiff contact)
    double lift = 0;
    for (size_t i = 0; i < nw; ++i) {
      const auto &W = L.wheels[i];
      const Vec3 pw = Vec3(sx, sy, z_base) + q.rot(Vec3(W.x, W.y, W.z));
      double hi; Vec3 ni; L.terrain.ground(pw.x, pw.y, hi, ni);
      lift = std::max(lift, hi + W.g.r / std::max(ni.z, 0.2) - pw.z);
    }
    z_base += lift;
  }
  Vec3 pc = Vec3(sx, sy, z_base) + q.rot(com_b), v(0, 0, 0), w(0, 0, 0);
  std::vector<double> om(nw, 0.0);
  {   // initial motion ([vehicle] v0_x, v0_y, v0_wz in the chassis frame): a replayed recording may start while driving
    const double v0x = Config::num(VE, "v0_x", 0), v0y = Config::num(VE, "v0_y", 0), v0w = Config::num(VE, "v0_wz", 0);
    if (v0x != 0 || v0y != 0 || v0w != 0) {
      w = ez * v0w;
      v = q.rot(Vec3(v0x, v0y, 0)) + w.cross(q.rot(com_b));
      for (size_t i = 0; i < nw; ++i) {
        const auto &W = L.wheels[i];
        const Vec3 ax = q.rot(Vec3(-std::sin(W.heading), std::cos(W.heading), 0));
        double wj = 0;
        if (W.g.type == 2) { wj = (v0x * std::cos(W.heading) + v0y * std::sin(W.heading) - v0w * W.y) / W.g.r; }
        else if (i < L.J.size()) wj = L.J[i][0] * v0x + L.J[i][1] * v0y + L.J[i][2] * v0w;
        om[i] = wj + w.dot(ax);
      }
    }
  }
  std::vector<WheelIO> io(nw);
  double max_tilt = 0, min_N = 1e9, max_N = 0;
  for (double t = 0; t <= L.t_end + 1e-9 && !L.done_written; t += dt) {
    const Vec3 base = pc - q.rot(com_b);
    std::vector<Vec3> rw(nw), ax(nw);
    for (size_t i = 0; i < nw; ++i) {
      const auto &W = L.wheels[i];
      const Vec3 pw = base + q.rot(Vec3(W.x, W.y, W.z));
      rw[i] = pw - pc;
      ax[i] = q.rot(Vec3(-std::sin(W.heading), std::cos(W.heading), 0));
      io[i].center = pw; io[i].vel = v + w.cross(rw[i]);
      const double wrel = om[i] - w.dot(ax[i]);           // om = absolute spin about the axle
      io[i].axle_left = ax[i]; io[i].angvel = w - ax[i] * w.dot(ax[i]) + ax[i] * om[i]; io[i].joint_vel = wrel;
    }
    double r_, p_, y_; q.euler(r_, p_, y_);
    double ch[6] = {base.x, base.y, base.z, r_, p_, y_};
    const Vec3 vb = v + w.cross(base - pc);
    S.chassis_vel[0] = vb.x; S.chassis_vel[1] = vb.y; S.chassis_vel[2] = vb.z;
    S.chassis_vel[3] = w.x; S.chassis_vel[4] = w.y; S.chassis_vel[5] = w.z; S.has_chassis_vel = true;
    S.chassis_q[0] = q.w; S.chassis_q[1] = q.x; S.chassis_q[2] = q.y; S.chassis_q[3] = q.z; S.has_chassis_q = true;
    double dtq = 0, bf = 0;
    S.step(t, dt, io, ch, nullptr, nullptr, dtq, nullptr, bf);
    Vec3 F(0, 0, -m * g), T(0, 0, 0);
    for (size_t i = 0; i < nw; ++i) {
      F += io[i].force;
      T += rw[i].cross(io[i].force);
      const double ts = io[i].torque.dot(ax[i]);
      const bool caster = L.wheels[i].g.type == 2;
      if (!caster) T += io[i].torque - ax[i] * ts;          // caster: the swivel torque stays in the caster
      T += ax[i] * (-io[i].motor_torque);
      const double Ji = caster && L.wheels[i].g.Jc > 0 ? L.wheels[i].g.Jc : Jw;
      om[i] += dt * (ts + io[i].motor_torque) / Ji;
      if (t > 0.5) { min_N = std::min(min_N, io[i].force.z); max_N = std::max(max_N, io[i].force.z); }
    }
    if (S.body_on()) { F += S.body_force; T += S.body_torque + (base - pc).cross(S.body_force); }   // torque about the link origin -> CoM
    const Vec3 Lb = q.inv_rot(w);
    const Vec3 Hw = q.rot(Vec3(Ixx * Lb.x, Iyy * Lb.y, Izz * Lb.z));
    const Vec3 rb = q.inv_rot(T - w.cross(Hw));
    const Vec3 alpha = q.rot(Vec3(rb.x / Ixx, rb.y / Iyy, rb.z / Izz));
    v += F * (dt / m); w += alpha * dt;
    pc += v * dt; q.integrate(w, dt);
    max_tilt = std::max(max_tilt, std::max(std::fabs(r_), std::fabs(p_)));
    double hg; Vec3 ng; L.terrain.ground(pc.x, pc.y, hg, ng);        // far above / below the ground under the CoM
    if (!std::isfinite(pc.x) || !std::isfinite(pc.z) || std::fabs(pc.z - hg) > 5.0) { std::fprintf(stderr, "vehicle3d: diverged at t=%.3f\n", t); return 2; }
  }
  if (!L.done_written) { std::ofstream f(L.done_file); f << L.t_end << "\n"; }
  std::printf("vehicle3d: OK (%s), max tilt %.2f deg, wheel normal force %.1f .. %.1f N (static %.1f)\n",
              L.name.c_str(), max_tilt * 180 / kPi, min_N, max_N, m * g / nw);
  return 0;
}
