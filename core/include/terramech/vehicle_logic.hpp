// vehicle_logic.hpp — any wheeled robot (Robotino, KUKA youBot, Husky, MarsSim rover, ...) driving a
// trajectory over a terrain map: wheel–surface contact (omni.hpp: Janosi shear + Bekker layer, roller /
// mecanum / ordinary wheels), DC motors with speed loops (motor groups for chain-coupled sides),
// trajectory tracking (open loop or closed loop on truth / wheel odometry / odometry + IMU),
// IMU at the centre of mass with noise and bias, encoder odometry, one log per run.
// Selected in the INI by  [robot] type = vehicle. Engine-agnostic (Gazebo Harmonic plugin, offline core vehicle3d).
//
//   [robot]       name, mass, com_x, com_y, com_h, Jw, drive = holonomic|diff, encoder_ticks,
//                 odom_yaw_scale (skid steer: 1/wheel_separation_multiplier, applied to commands and odometry)
//   [wheel k]     link, joint, x, y, heading_deg (rolling direction for positive spin, body frame),
//                 r, b, patch, mu_rb (roller bearing friction, overrides the surface), type = omni|mecanum|standard|caster, gamma_deg, group (motor, -1 = none),
//                 zmax_r (sinkage limit / r, default 0.6), z (emulator only)
//   [motor]       MotorParams (omni.hpp);  coupling = chain damping between wheels of one group (N m s)
//   [omni_contact] contact options;  [surface X] library;  [vehicle] surface = default floor
//   [map]         dtm_file (heights + surface types)
//   [trajectory]  path_file (s,x,y,yaw,kappa in the start frame), speed, accel, heading = tangent|fixed, t_start
//   [controller]  type = open|closed, localization = truth|odom|odom_imu, motor = speed|voltage, rate,
//                 kp_xy, k_yaw, kx, ky, kth, v_max, w_max
//   [imu]         rate, gyro_sigma, gyro_bias, gyro_rw, acc_sigma, acc_bias, acc_rw, seed, calibrate (gyro bias at standstill)
#pragma once
#include <array>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <cstdio>
#include <fstream>
#include <memory>
#include <sstream>
#include "terramech/omni.hpp"
#include "terramech/sensors.hpp"

namespace terramech {

// ------------------------------------------------------------------------------- terrain (map or flat)
class TerrainMap {
 public:
  std::unique_ptr<DTM> dtm;
  std::vector<SurfaceMap> tmaps;
  std::vector<std::string> names;
  SurfaceMap flat; std::string flat_name = "floor";
  double ground_z = 0.0;

  bool configure(const Config &cfg, const std::string &surface, std::string &err) {
    const Config::Section *SS = cfg.find("surface", surface);
    if (!SS) { err = "vehicle: no [surface " + surface + "]"; return false; }
    flat.init(firm_from_section(SS)); flat_name = surface;
    const std::string mapf = cfg.path(Config::str(cfg.find("map"), "dtm_file", ""));
    if (mapf.empty()) return true;
    dtm.reset(new DTM());
    if (!dtm->load(mapf, &err)) return false;
    for (auto &tn : dtm->type_names) {
      const Config::Section *TS = cfg.find("surface", tn);
      if (!TS) { err = "vehicle: map type '" + tn + "' has no [surface " + tn + "] section"; return false; }
      SurfaceMap m; m.init(firm_from_section(TS)); tmaps.push_back(m); names.push_back(tn);
    }
    return true;
  }
  double res() const { return dtm ? dtm->res : 1e9; }
  void ground(double x, double y, double &h, Vec3 &n) const {
    if (!dtm) { h = ground_z; n = Vec3(0, 0, 1); return; }
    const double e = dtm->res;
    h = dtm->height(x, y);
    const double hx = (dtm->height(x + e, y) - dtm->height(x - e, y)) / (2 * e), hy = (dtm->height(x, y + e) - dtm->height(x, y - e)) / (2 * e);
    n = Vec3(-hx, -hy, 1.0).normalized();
  }
  int dominant(double x, double y) const {
    if (!dtm) return -1;
    int i, j; double u, v; dtm->cell(x, y, i, j, u, v);
    i += u > 0.5 ? 1 : 0; j += v > 0.5 ? 1 : 0;
    i = std::max(0, std::min(dtm->nx - 1, i)); j = std::max(0, std::min(dtm->ny - 1, j));
    return dtm->T[(size_t)j * dtm->nx + i];
  }
  const std::string &name_at(double x, double y) const { int k = dominant(x, y); return k < 0 ? flat_name : names[(size_t)k]; }
  static FirmSurface blend(const std::vector<std::pair<FirmSurface, double>> &P) {
    double W = 0; size_t dom = 0;
    for (size_t k = 0; k < P.size(); ++k) { W += P[k].second; if (P[k].second > P[dom].second) dom = k; }
    FirmSurface s = P[dom].first;
    if (P.size() == 1 || W <= 0) return s;
    auto mix = [&](auto get) { double a = 0; for (auto &p : P) a += p.second * get(p.first); return a / W; };
    s.phi = std::atan(mix([](const FirmSurface &f) { return std::tan(f.phi); }));
    s.c = mix([](const FirmSurface &f) { return f.c; }); s.K = mix([](const FirmSurface &f) { return f.K; });
    s.patch_len = mix([](const FirmSurface &f) { return f.patch_len; });
    s.f_roll = mix([](const FirmSurface &f) { return f.f_roll; }); s.f_lat = mix([](const FirmSurface &f) { return f.f_lat; });
    s.mu_rb = mix([](const FirmSurface &f) { return f.mu_rb; });
    s.kn = mix([](const FirmSurface &f) { return f.kn; }); s.cn = mix([](const FirmSurface &f) { return f.cn; });
    double comp = 0, wd = 0, nb = 0, lay = 0, reb = 0, b = 0;
    for (auto &p : P) if (p.first.deformable()) {
      const double ks = p.first.kc + p.first.b_contact * p.first.kphi;
      comp += p.second / ks; wd += p.second; nb += p.second * p.first.n_bek; lay += p.second * p.first.layer;
      reb += p.second * p.first.rebound; b += p.second * p.first.b_contact;
    }
    if (wd > 0) { s.b_contact = b / wd; s.kc = 0.0; s.kphi = (W / comp) / s.b_contact; s.n_bek = nb / wd; s.layer = lay / wd; s.rebound = reb / wd; }
    else { s.kc = 0.0; s.kphi = 0.0; }
    return s;
  }
  FirmSurface surface(double x, double y) const {
    if (!dtm) return flat.local(x, y);
    int i, j; double u, v; dtm->cell(x, y, i, j, u, v);
    auto T = [&](int a, int b) { a = std::max(0, std::min(dtm->nx - 1, a)); b = std::max(0, std::min(dtm->ny - 1, b)); return dtm->T[(size_t)b * dtm->nx + a]; };
    const int t[4] = {T(i, j), T(i + 1, j), T(i, j + 1), T(i + 1, j + 1)};
    const double w[4] = {(1 - u) * (1 - v), u * (1 - v), (1 - u) * v, u * v};
    std::vector<std::pair<FirmSurface, double>> P;
    for (int k = 0; k < 4; ++k) {
      bool seen = false;
      for (int m = 0; m < k; ++m) if (t[m] == t[k]) seen = true;
      if (seen) continue;
      double wk = 0; for (int m = 0; m < 4; ++m) if (t[m] == t[k]) wk += w[m];
      P.push_back({tmaps[(size_t)t[k]].local(x, y), wk});
    }
    return blend(P);
  }
};

// ------------------------------------------------------------------------------- reference path
class RefPath {
 public:
  std::vector<double> s, x, y, yaw, k;
  bool load(const std::string &file, std::string &err) {
    std::ifstream f(file);
    if (!f) { err = "vehicle: cannot open path " + file; return false; }
    std::string line; std::getline(f, line);           // header s,x,y,yaw,kappa
    while (std::getline(f, line)) {
      double a[5]; char c;
      std::istringstream is(line);
      if (!(is >> a[0] >> c >> a[1] >> c >> a[2] >> c >> a[3] >> c >> a[4])) continue;
      s.push_back(a[0]); x.push_back(a[1]); y.push_back(a[2]); yaw.push_back(a[3]); k.push_back(a[4]);
    }
    if (s.size() < 2) { err = "vehicle: path " + file + " has < 2 points"; return false; }
    for (size_t i = 1; i < yaw.size(); ++i) {             // unwrap
      while (yaw[i] - yaw[i - 1] > kPi) yaw[i] -= 2 * kPi;
      while (yaw[i] - yaw[i - 1] < -kPi) yaw[i] += 2 * kPi;
    }
    return true;
  }
  double length() const { return s.empty() ? 0.0 : s.back(); }
  void at(double si, double &px, double &py, double &pyaw, double &pk) const {
    si = clamp(si, s.front(), s.back());
    size_t lo = 0, hi = s.size() - 1;
    while (hi - lo > 1) { size_t m = (lo + hi) / 2; if (s[m] <= si) lo = m; else hi = m; }
    const double u = (s[hi] > s[lo]) ? (si - s[lo]) / (s[hi] - s[lo]) : 0.0;
    px = x[lo] + u * (x[hi] - x[lo]); py = y[lo] + u * (y[hi] - y[lo]);
    pyaw = yaw[lo] + u * (yaw[hi] - yaw[lo]); pk = k[lo] + u * (k[hi] - k[lo]);
  }
};

// trapezoidal speed profile along the path: s(t), v(t)
struct SpeedProfile {
  double L = 0, v = 0.1, a = 0.5, t_acc = 0, s_acc = 0, t_cruise = 0, T = 0, vpk = 0;
  void init(double length, double speed, double accel) {
    L = length; v = speed; a = std::max(accel, 1e-6);
    vpk = v; t_acc = v / a; s_acc = 0.5 * a * t_acc * t_acc;
    if (2 * s_acc > L) { vpk = std::sqrt(a * L); t_acc = vpk / a; s_acc = 0.5 * L; t_cruise = 0; }
    else t_cruise = (L - 2 * s_acc) / v;
    T = 2 * t_acc + t_cruise;
  }
  void at(double t, double &s, double &vel) const {
    if (t <= 0) { s = 0; vel = 0; return; }
    if (t < t_acc) { s = 0.5 * a * t * t; vel = a * t; return; }
    if (t < t_acc + t_cruise) { s = s_acc + vpk * (t - t_acc); vel = vpk; return; }
    if (t < T) { const double td = T - t; s = L - 0.5 * a * td * td; vel = a * td; return; }
    s = L; vel = 0;
  }
};

// recorded commands (dataset replay): CSV with a header; column 0 = time (s from the start of the motion),
// the other columns = values interpolated linearly in time (after the last row: 0)
struct TimeTable {
  std::vector<std::string> names; std::vector<double> t; std::vector<std::vector<double>> col;
  bool load(const std::string &file, std::string &err) {
    std::ifstream f(file);
    if (!f) { err = "vehicle: cannot open " + file; return false; }
    std::string line; std::getline(f, line);
    { std::istringstream is(line); std::string n; while (std::getline(is, n, ',')) names.push_back(n); }
    if (names.size() < 2) { err = "vehicle: " + file + ": need t + at least one column"; return false; }
    col.assign(names.size() - 1, {});
    while (std::getline(f, line)) {
      std::istringstream is(line); std::string v; std::vector<double> row;
      while (std::getline(is, v, ',')) row.push_back(std::atof(v.c_str()));
      if (row.size() != names.size()) continue;
      t.push_back(row[0]); for (size_t k = 1; k < row.size(); ++k) col[k - 1].push_back(row[k]);
    }
    if (t.size() < 2) { err = "vehicle: " + file + " has < 2 rows"; return false; }
    return true;
  }
  int index(const std::string &n) const { for (size_t k = 1; k < names.size(); ++k) if (names[k] == n) return (int)k - 1; return -1; }
  double T() const { return t.empty() ? 0.0 : t.back(); }
  double at(size_t c, double tau, bool zero_after = true) const {
    if (t.empty()) return 0.0;
    if (tau <= t.front()) return col[c].front();
    if (tau >= t.back()) return zero_after ? 0.0 : col[c].back();
    size_t lo = 0, hi = t.size() - 1;
    while (hi - lo > 1) { size_t m = (lo + hi) / 2; if (t[m] <= tau) lo = m; else hi = m; }
    const double u = (tau - t[lo]) / std::max(t[hi] - t[lo], 1e-12);
    return col[c][lo] + u * (col[c][hi] - col[c][lo]);
  }
};

struct VehWheelIn { Vec3 center, vel, angvel, axle; double joint_vel = 0; };
struct VehWheelOut { Vec3 force, torque; double motor_torque = 0; };

class VehicleLogic {
 public:
  struct WheelDef { std::string label, link, joint; double x = 0, y = 0, z = 0, heading = 0; OmniGeom g; int group = 0; };
  std::vector<WheelDef> wheels;
  std::vector<OmniContact> contacts;
  std::vector<OmniOut> last_c;
  TerrainMap terrain;
  OmniOpts oo;
  MotorParams mp;
  std::vector<DCMotor> motors;               // one per group
  std::vector<std::vector<size_t>> groups;
  std::vector<MotorOut> last_m;
  double couple_c = -1, Jw = 0.002;
  bool motor_implicit = true;                // [motor] implicit: torque at the speed predicted for the end of the step
  // robot
  std::string name = "robot";
  double com_x = 0, com_y = 0, com_h = 0.1;
  bool holonomic = true; double odom_yaw_scale = 1.0, enc_ticks = 0;
  // kinematics: w_i = J_i . (vx, vy, wz);  odometry: twist = Jp * w
  std::vector<std::array<double, 3>> J;
  double Jp[3][16] = {{0}};
  // trajectory + control
  RefPath path; SpeedProfile prof;
  bool heading_tangent = true, closed = true, motor_speed_loop = true;
  // replay of recorded commands: [controller] type = replay, replay_file, replay_mode = wheels | twist;
  // [trajectory] truth_file (t, x, y, yaw in the start frame) is the time-indexed reference for the logs
  bool replay = false; int replay_mode = 0; TimeTable rp, truth; std::vector<int> rp_col;
  int loc = 0;                               // 0 truth, 1 odom, 2 odom_imu
  double t_start = 1.0, t_settle = 2.0, ctrl_rate = 50, next_ctrl = -1;
  double kp_xy = 1.5, k_yaw = 2.0, kx = 1.0, ky = 8.0, kth = 3.0, v_max = 1.0, w_max = 2.0;
  bool started = false, finished = false, done_written = false;
  double sx0 = 0, sy0 = 0, syaw0 = 0, t_finish = 1e9, t_end = 1e9;
  double cmd[3] = {0, 0, 0};
  std::vector<double> wset;
  double ref[5] = {0, 0, 0, 0, 0};           // x, y, yaw, v, w (world)
  // estimation
  Imu imu; bool imu_on = true;
  bool gyro_cal = true; double gcal_sum = 0, gcal_n = 0, gyro_bias_est = 0;   // gyro bias averaged at standstill before t_start
  double odo[3] = {0, 0, 0}, fus[3] = {0, 0, 0};
  std::vector<double> enc_ang, enc_prev_cnt, w_enc;
  double prev_ctrl_t = -1;
  ChassisKinematics kin; BodyState bs;
  Odometer odo_ideal_est; double odo_ideal[3] = {0, 0, 0};   // exact joint speeds, no quantisation (ideal wheel odometry)
  double energy = 0, power = 0;
  FullLog full;
  // logging
  std::string log_file, done_file; std::unique_ptr<std::ofstream> flog;
  double log_rate = 50, next_log = -1;
  bool flush_each = false;          // [general] flush_log = 1 (or env TM_FLUSH=1): every log row reaches the file at once (live ROS 2 bridge)
  std::vector<FirmSurface> surf_cache; std::vector<Vec3> surf_cache_p;
  double ch_vel[6] = {0, 0, 0, 0, 0, 0}; bool has_ch_vel = false; double prev_ch[6] = {0}; bool have_prev_ch = false;
  double ch_q[4] = {1, 0, 0, 0}; bool has_ch_q = false;     // chassis orientation (w, x, y, z) from the engine, if given
  // body-ground contact ([body]): penalty spring-damper + stick-slip Coulomb friction at points of the body
  // (collision shape of the chassis and the outer wheel faces, chassis-link frame) against the same terrain map as
  // the wheels. Off: the body passes through the ground (terrasim <= 8.3).
  struct Body {
    bool on = false; std::vector<Vec3> pts; double k = 0, c = 0, kt = 0, ct = 0, mu = 0.6;
    std::vector<Vec3> anchor; std::vector<char> stick; std::vector<double> next_t;   // next_t: points well above the ground are re-checked later
    Vec3 F, T; double N = 0; int n = 0;                     // wrench on the chassis (world; torque about the link origin)
  } body;
  double tip_deg = 50.0, tilt_rel = 0.0, t_tip = -1.0; bool tip_stop = true, tipped = false;

  static double wrap(double a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }

  void set_chassis_quat(const double q[4]) { for (int k = 0; k < 4; ++k) ch_q[k] = q[k]; has_ch_q = true; }

  Vec3 chassis_rot(const double ch[6], const Vec3 &v) const {
    if (has_ch_q) {                                   // q (w, x, y, z) * v * q^-1
      const Vec3 u(ch_q[1], ch_q[2], ch_q[3]); const Vec3 t = u.cross(v) * 2.0;
      return v + t * ch_q[0] + u.cross(t);
    }
    const double cr = std::cos(ch[3]), sr = std::sin(ch[3]), cp = std::cos(ch[4]), sp = std::sin(ch[4]), cy = std::cos(ch[5]), sy = std::sin(ch[5]);
    return Vec3(cy * cp * v.x + (cy * sp * sr - sy * cr) * v.y + (cy * sp * cr + sy * sr) * v.z,
                sy * cp * v.x + (sy * sp * sr + cy * cr) * v.y + (sy * sp * cr - cy * sr) * v.z,
                -sp * v.x + cp * sr * v.y + cp * cr * v.z);
  }

  static bool parse_points(const std::string &txt, std::vector<Vec3> &P) {
    P.clear(); std::stringstream ss(txt); std::string item;
    while (std::getline(ss, item, '/')) {                 // x,y,z / x,y,z / ...  (';' starts an INI comment)
      for (auto &ch : item) if (ch == ',') ch = ' ';
      std::istringstream is(item); double x, y, z;
      if (is >> x >> y >> z) P.push_back(Vec3(x, y, z));
    }
    return !P.empty();
  }

  // tilt of the chassis z axis to the ground normal; body contact wrench (world, torque about the link origin)
  void body_step(double t, const double ch[6]) {
    body.F = Vec3(); body.T = Vec3(); body.N = 0; body.n = 0;
    const Vec3 o(ch[0], ch[1], ch[2]);
    {
      double h; Vec3 n; terrain.ground(o.x, o.y, h, n);
      tilt_rel = std::acos(clamp(chassis_rot(ch, Vec3(0, 0, 1)).dot(n), -1.0, 1.0)) * 180.0 / kPi;
      if (!tipped && tilt_rel > tip_deg) { tipped = true; t_tip = t; }
    }
    if (!body.on) return;
    const Vec3 vo(ch_vel[0], ch_vel[1], ch_vel[2]), w(ch_vel[3], ch_vel[4], ch_vel[5]);
    for (size_t k = 0; k < body.pts.size(); ++k) {
      if (t < body.next_t[k]) continue;
      const Vec3 r = chassis_rot(ch, body.pts[k]), p = o + r;
      double h; Vec3 n; terrain.ground(p.x, p.y, h, n);
      const double d = (h - p.z) * n.z;                         // depth below the (tilted) ground plane
      const Vec3 vp = vo + w.cross(r);
      if (d <= 0) {                                             // clear of the ground: skip it while it cannot reach the ground
        body.stick[k] = 0;
        body.next_t[k] = t + std::min(0.05, -d / (vp.norm() + 0.5));
        continue;
      }
      const double vn = vp.dot(n);
      const double Fn = std::max(0.0, body.k * d - body.c * vn);
      if (!body.stick[k]) { body.anchor[k] = p; body.stick[k] = 1; }
      Vec3 e = p - body.anchor[k]; e = e - n * e.dot(n);
      const Vec3 vt = vp - n * vn;
      Vec3 Ft = e * (-body.kt) - vt * body.ct;                  // sticking: tangential spring-damper to the anchor
      const double ft = Ft.norm(), lim = body.mu * Fn;
      if (ft > lim) {                                           // sliding: Coulomb limit, the anchor follows
        Ft = ft > 1e-12 ? Ft * (lim / ft) : Vec3();
        body.anchor[k] = p + (Ft + vt * body.ct) * (1.0 / std::max(body.kt, 1e-9));
      }
      const Vec3 Fk = n * Fn + Ft;
      body.F += Fk; body.T += r.cross(Fk); body.N += Fn; body.n += 1;
    }
  }

  bool configure(const Config &cfg, const std::string &log_path, const std::string &done_path, std::string &err) {
    auto *RB = cfg.find("robot");
    {
      auto *BD = cfg.find("body");
      body.on = Config::num(BD, "contact", 0) > 0.5;
      if (body.on && !parse_points(Config::str(BD, "points", ""), body.pts)) { err = "vehicle: [body] contact = 1 needs points = x,y,z / x,y,z / ..."; return false; }
      body.k = Config::num(BD, "k", 1e5); body.c = Config::num(BD, "c", 500); body.mu = Config::num(BD, "mu", 0.6);
      body.kt = Config::num(BD, "kt", body.k); body.ct = Config::num(BD, "ct", body.c);
      body.anchor.assign(body.pts.size(), Vec3()); body.stick.assign(body.pts.size(), 0); body.next_t.assign(body.pts.size(), -1.0);
      tip_deg = Config::num(BD, "tip_deg", 50.0);
      tip_stop = Config::str(BD, "on_tip", "stop") != "continue";
    }
    name = Config::str(RB, "name", name);
    com_x = Config::num(RB, "com_x", 0); com_y = Config::num(RB, "com_y", 0); com_h = Config::num(RB, "com_h", 0.1);
    Jw = Config::num(RB, "Jw", Jw);
    holonomic = Config::str(RB, "drive", "holonomic") != "diff";
    odom_yaw_scale = Config::num(RB, "odom_yaw_scale", 1.0); enc_ticks = Config::num(RB, "encoder_ticks", 0);
    for (auto *ws : cfg.all("wheel")) {
      WheelDef w; w.label = ws->label; w.link = Config::str(ws, "link", ""); w.joint = Config::str(ws, "joint", "");
      w.x = Config::num(ws, "x", 0); w.y = Config::num(ws, "y", 0);
      w.heading = deg2rad(Config::num(ws, "heading_deg", 0));
      w.g.r = Config::num(ws, "r", 0.05);
      w.z = Config::num(ws, "z", w.g.r);                         // wheel centre height above the chassis-link origin
      w.g.b = Config::num(ws, "b", 0); w.g.patch = Config::num(ws, "patch", 0); w.g.mu_rb = Config::num(ws, "mu_rb", -1); w.g.lat_grip = Config::num(ws, "lat_grip", 1.0); w.g.zmax_r = Config::num(ws, "zmax_r", 0.6);
      w.g.Jc = Config::num(ws, "J", 0.0); w.g.swivel = Config::num(ws, "swivel", 0.0) > 0.5;      // casters
      const std::string ty = Config::str(ws, "type", "standard");
      w.g.type = ty == "standard" ? 1 : (ty == "caster" ? 2 : 0);
      w.g.gamma = deg2rad(Config::num(ws, "gamma_deg", ty == "mecanum" ? 45.0 : 90.0));
      w.group = (int)Config::num(ws, "group", w.g.type == 2 ? -1.0 : (double)wheels.size());   // -1: passive (no motor)
      wheels.push_back(w);
    }
    if (wheels.empty() || wheels.size() > 16) { err = "vehicle: need 1..16 [wheel] sections"; return false; }
    // kinematic matrix
    for (auto &w : wheels) {
      const double ex = std::cos(w.heading), ey = std::sin(w.heading);
      const double axx = -ey, axy = ex;                          // axle = e rotated +90 deg (points left)
      double ax_ = ex, ay_ = ey, ea = 1.0;                       // constrained direction a and e.a
      if (w.g.type == 0) { const double s = std::sin(w.g.gamma), c = std::cos(w.g.gamma); ax_ = ex * s + axx * c; ay_ = ey * s + axy * c; ea = s; }
      if (w.g.type == 2) { J.push_back({0.0, 0.0, 0.0}); continue; }   // caster: no kinematic constraint, no motor
      J.push_back({ax_ / (w.g.r * ea), ay_ / (w.g.r * ea), (-ax_ * w.y + ay_ * w.x) / (w.g.r * ea)});
    }
    // odometry pseudo-inverse (J^T J + lambda I)^-1 J^T
    {
      double A[3][3] = {{0}};
      for (auto &r : J) for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) A[a][b] += r[a] * r[b];
      for (int a = 0; a < 3; ++a) A[a][a] += 1e-9;
      double I[3][3]; invert3(A, I);
      for (size_t i = 0; i < J.size(); ++i) for (int a = 0; a < 3; ++a) { double v = 0; for (int b = 0; b < 3; ++b) v += I[a][b] * J[i][b]; Jp[a][i] = v; }
    }
    int ng = 0; for (auto &w : wheels) ng = std::max(ng, w.group + 1);
    groups.assign(ng, {});
    for (size_t i = 0; i < wheels.size(); ++i) if (wheels[i].group >= 0) groups[(size_t)wheels[i].group].push_back(i);
    odo_ideal_est.init(J, odom_yaw_scale);
    mp = motor_from_section(cfg.find("motor"));
    motor_implicit = Config::num(cfg.find("motor"), "implicit", 1.0) > 0.5;
    couple_c = Config::num(cfg.find("motor"), "coupling", -1);
    motors.assign(groups.size(), DCMotor()); last_m.assign(groups.size(), MotorOut());
    auto *OC = cfg.find("omni_contact");
    oo.slip_v_eps = Config::num(OC, "slip_v_eps", oo.slip_v_eps); oo.slip_v_ramp = Config::num(OC, "slip_v_ramp", oo.slip_v_ramp);
    oo.slip_relax_len = Config::num(OC, "slip_relax_len", oo.slip_relax_len); oo.slip_tau_min = Config::num(OC, "slip_tau_min", oo.slip_tau_min);
    oo.v_eps = Config::num(OC, "v_eps", oo.v_eps); oo.slip_norm = (int)Config::num(OC, "slip_norm", oo.slip_norm);
    oo.slip_max = Config::num(OC, "slip_max", oo.slip_max);
    oo.tilt_wheel = body.on;
    if (!terrain.configure(cfg, Config::str(cfg.find("vehicle"), "surface", "floor"), err)) return false;
    contacts.assign(wheels.size(), OmniContact());
    for (size_t i = 0; i < wheels.size(); ++i) contacts[i].geom = wheels[i].g;
    last_c.assign(wheels.size(), OmniOut()); wset.assign(wheels.size(), 0.0);
    surf_cache.assign(wheels.size(), terrain.flat.base); surf_cache_p.assign(wheels.size(), Vec3(1e9, 1e9, 0));
    enc_ang.assign(wheels.size(), 0.0); enc_prev_cnt.assign(wheels.size(), 0.0); w_enc.assign(wheels.size(), 0.0);
    // trajectory and controller
    auto *TR = cfg.find("trajectory");
    const std::string pf = cfg.path(Config::str(TR, "path_file", ""));
    if (pf.empty()) { err = "vehicle: [trajectory] path_file missing"; return false; }
    if (!path.load(pf, err)) return false;
    prof.init(path.length(), Config::num(TR, "speed", 0.1), Config::num(TR, "accel", 0.5));
    {
      auto *CT0 = cfg.find("controller");
      replay = Config::str(CT0, "type", "closed") == "replay";
      if (replay) {
        if (!rp.load(cfg.path(Config::str(CT0, "replay_file", "")), err)) return false;
        replay_mode = Config::str(CT0, "replay_mode", "wheels") == "twist" ? 1 : 0;
        rp_col.clear();
        if (replay_mode == 1) {
          for (const char *n : {"vx", "vy", "wz"}) rp_col.push_back(rp.index(n));
          if (rp_col[0] < 0 || rp_col[2] < 0) { err = "vehicle: replay twist needs columns vx, (vy), wz"; return false; }
        } else {                                           // one column per motor group: g0, g1, ... (rad/s at the wheel)
          int ng = 0; for (auto &w : wheels) ng = std::max(ng, w.group + 1);
          for (int g = 0; g < ng; ++g) {
            const int c = rp.index("g" + std::to_string(g));
            if (c < 0) { err = "vehicle: replay wheels needs column g" + std::to_string(g); return false; }
            rp_col.push_back(c);
          }
        }
        prof.T = rp.T();
        const std::string tf = cfg.path(Config::str(TR, "truth_file", ""));
        if (!tf.empty() && !truth.load(tf, err)) return false;
      }
    }
    heading_tangent = Config::str(TR, "heading", "tangent") != "fixed";
    if (!holonomic) heading_tangent = true;
    t_start = Config::num(TR, "t_start", 1.0); t_settle = Config::num(TR, "t_settle", 2.0);
    auto *CT = cfg.find("controller");
    closed = Config::str(CT, "type", "closed") != "open";
    const std::string L = Config::str(CT, "localization", "truth");
    loc = L == "odom" ? 1 : (L == "odom_imu" ? 2 : 0);
    motor_speed_loop = Config::str(CT, "motor", "speed") != "voltage";
    if (!motor_speed_loop) { mp.kp = 0; mp.ki = 0; mp.ff = 1.0; }
    ctrl_rate = Config::num(CT, "rate", 50);
    kp_xy = Config::num(CT, "kp_xy", kp_xy); k_yaw = Config::num(CT, "k_yaw", k_yaw);
    kx = Config::num(CT, "kx", kx); ky = Config::num(CT, "ky", ky); kth = Config::num(CT, "kth", kth);
    v_max = Config::num(CT, "v_max", v_max); w_max = Config::num(CT, "w_max", w_max);
    auto *IM = cfg.find("imu");
    ImuParams ip = imu_from_section(IM);
    gyro_cal = Config::num(IM, "calibrate", 1) > 0.5;
    imu.init(ip);
    t_end = t_start + prof.T + t_settle + 0.05;
    // log
    log_file = log_path; done_file = done_path;
    log_rate = Config::num(cfg.find("general"), "log_rate", log_rate);
    flush_each = Config::num(cfg.find("general"), "flush_log", 0) > 0 || (std::getenv("TM_FLUSH") && std::string(std::getenv("TM_FLUSH")) == "1");
    flog.reset(new std::ofstream(log_file));
    if (!*flog) { err = "vehicle: cannot open log " + log_file; return false; }
    {
      auto *G = cfg.find("general");
      full.rate = Config::num(G, "full_rate", 50.0);
      std::string ff = cfg.path(Config::str(G, "full_log_file", ""));
      if (ff.empty()) ff = full_log_path(log_file);
      std::vector<std::string> labels; for (auto &w : wheels) labels.push_back(w.label);
      if (full.rate > 0 && !full.open(ff, wheels.size(), labels)) { err = "vehicle: cannot open " + ff; return false; }
    }
    kin.com = Vec3(com_x, com_y, com_h);
    *flog << "t,phase,ref_x,ref_y,ref_yaw,ref_v,ref_w,x,y,z,roll,pitch,yaw,vx,vy,wz,odo_x,odo_y,odo_yaw,fus_x,fus_y,fus_yaw,"
             "imu_gx,imu_gy,imu_gz,imu_ax,imu_ay,imu_az,cmd_vx,cmd_vy,cmd_wz,e_along,e_cross,e_yaw,surf";
    for (size_t i = 1; i <= wheels.size(); ++i)
      for (const char *k : {"wset", "wenc", "cur", "vol", "pow", "tau", "N", "Fx", "Fy", "slip", "slip_lat", "sink", "surf"}) *flog << ",w" << i << "_" << k;
    *flog << ",tilt_rel,body_N,body_pts\n";
    return true;
  }

  static void invert3(const double A[3][3], double I[3][3]) {
    const double d = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                     A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    const double k = 1.0 / d;
    I[0][0] = k * (A[1][1] * A[2][2] - A[1][2] * A[2][1]); I[0][1] = k * (A[0][2] * A[2][1] - A[0][1] * A[2][2]); I[0][2] = k * (A[0][1] * A[1][2] - A[0][2] * A[1][1]);
    I[1][0] = k * (A[1][2] * A[2][0] - A[1][0] * A[2][2]); I[1][1] = k * (A[0][0] * A[2][2] - A[0][2] * A[2][0]); I[1][2] = k * (A[0][2] * A[1][0] - A[0][0] * A[1][2]);
    I[2][0] = k * (A[1][0] * A[2][1] - A[1][1] * A[2][0]); I[2][1] = k * (A[0][1] * A[2][0] - A[0][0] * A[2][1]); I[2][2] = k * (A[0][0] * A[1][1] - A[0][1] * A[1][0]);
  }

  // wheel set speeds from a body twist
  void twist_to_wheels(const double tw[3], std::vector<double> &w) const {
    w.resize(J.size());
    // skid-steer: the yaw rate needs more wheel-speed difference than the geometric track gives
    // (lateral scrubbing); odom_yaw_scale (= 1 / wheel_separation_multiplier of ROS diff_drive_controller)
    // is used both ways, as the real Husky controller does: wheels from (v, w / scale), odometry w = scale * w_wheels
    const double wz = holonomic ? tw[2] : tw[2] / std::max(odom_yaw_scale, 1e-3);
    for (size_t i = 0; i < J.size(); ++i) w[i] = J[i][0] * tw[0] + J[i][1] * tw[1] + J[i][2] * wz;
  }

  // reference (world) at time t
  void reference(double t, double r[5]) const {
    if (replay && !truth.t.empty()) {                 // recorded trajectory, indexed by time
      const double tau = t - t_start, c = std::cos(syaw0), sn = std::sin(syaw0);
      const double px = truth.at(0, tau, false), py = truth.at(1, tau, false), pyaw = truth.at(2, tau, false);
      r[0] = sx0 + c * px - sn * py; r[1] = sy0 + sn * px + c * py; r[2] = syaw0 + pyaw; r[3] = 0; r[4] = 0;
      vel_dir = r[2];
      return;
    }
    double s, v; prof.at(t - t_start, s, v);
    double px, py, pyaw, pk; path.at(s, px, py, pyaw, pk);
    const double c = std::cos(syaw0), sn = std::sin(syaw0);
    r[0] = sx0 + c * px - sn * py; r[1] = sy0 + sn * px + c * py;
    r[2] = heading_tangent ? syaw0 + pyaw : syaw0;     // path yaw is relative to the start frame
    r[3] = v; r[4] = heading_tangent ? v * pk : 0.0;
    vel_dir = syaw0 + pyaw;
  }
  mutable double vel_dir = 0;

  void control(double t, const double ch[6]) {
    reference(t, ref);
    const double est[3] = {loc == 0 ? ch[0] : (loc == 1 ? odo[0] : fus[0]), loc == 0 ? ch[1] : (loc == 1 ? odo[1] : fus[1]),
                           loc == 0 ? ch[5] : (loc == 1 ? odo[2] : fus[2])};
    double vx = 0, vy = 0, wz = 0;
    if (replay) {                                       // recorded commands, open loop
      cmd[0] = cmd[1] = cmd[2] = 0;
      if (t < t_start || finished) return;
      const double tau = t - t_start;
      if (replay_mode == 1) { cmd[0] = rp.at((size_t)rp_col[0], tau); cmd[1] = rp_col[1] >= 0 ? rp.at((size_t)rp_col[1], tau) : 0.0; cmd[2] = rp.at((size_t)rp_col[2], tau); }
      return;
    }
    if (t < t_start || (finished && !closed)) { cmd[0] = cmd[1] = cmd[2] = 0; return; }   // closed loop keeps holding the end point
    const double vwx = ref[3] * std::cos(vel_dir), vwy = ref[3] * std::sin(vel_dir);   // reference velocity, world
    if (holonomic) {
      double gx = vwx, gy = vwy, w = ref[4];
      double yaw_used = closed ? est[2] : ref[2];
      if (closed) { gx += kp_xy * (ref[0] - est[0]); gy += kp_xy * (ref[1] - est[1]); w += k_yaw * wrap(ref[2] - est[2]); }
      const double c = std::cos(yaw_used), s = std::sin(yaw_used);
      vx = c * gx + s * gy; vy = -s * gx + c * gy; wz = w;
      const double n = std::hypot(vx, vy);
      if (n > v_max) { vx *= v_max / n; vy *= v_max / n; }
    } else {
      double v = ref[3], w = ref[4];
      if (closed) {
        const double c = std::cos(est[2]), s = std::sin(est[2]);
        const double ex = c * (ref[0] - est[0]) + s * (ref[1] - est[1]), ey = -s * (ref[0] - est[0]) + c * (ref[1] - est[1]);
        const double eth = wrap(ref[2] - est[2]);
        v = ref[3] * std::cos(eth) + kx * ex;
        w = ref[4] + ref[3] * (ky * ey + kth * std::sin(eth));
      }
      vx = clamp(v, -v_max, v_max);
      wz = w;
    }
    wz = clamp(wz, -w_max, w_max);
    cmd[0] = vx; cmd[1] = vy; cmd[2] = wz;
  }

  // encoders + odometry + IMU fusion at the control rate
  void estimate(double t, double dt_ctrl) {
    double tw[3] = {0, 0, 0};
    for (size_t i = 0; i < wheels.size(); ++i) {
      double w = w_enc[i];
      for (int a = 0; a < 3; ++a) tw[a] += Jp[a][i] * w;
    }
    tw[2] *= odom_yaw_scale;
    auto integ = [&](double p[3], double yaw_rate) {
      const double ym = p[2] + 0.5 * yaw_rate * dt_ctrl;
      p[0] += (std::cos(ym) * tw[0] - std::sin(ym) * tw[1]) * dt_ctrl;
      p[1] += (std::sin(ym) * tw[0] + std::cos(ym) * tw[1]) * dt_ctrl;
      p[2] += yaw_rate * dt_ctrl;
    };
    integ(odo, tw[2]);
    // odometry + IMU: heading from the gyro (yaw rate about the vertical, roll/pitch neglected)
    integ(fus, imu.gyro.z - gyro_bias_est);
    (void)t;
  }

  void step(double t, double dt, const std::vector<VehWheelIn> &in, std::vector<VehWheelOut> &out, const double ch[6]) {
    out.assign(in.size(), VehWheelOut());
    // chassis velocities (from the engine or by differences)
    if (!has_ch_vel && have_prev_ch && dt > 0) {
      for (int k = 0; k < 3; ++k) ch_vel[k] = (ch[k] - prev_ch[k]) / dt;
      ch_vel[3] = 0; ch_vel[4] = 0; ch_vel[5] = wrap(ch[5] - prev_ch[5]) / dt;
    }
    for (int k = 0; k < 6; ++k) prev_ch[k] = ch[k];
    have_prev_ch = true;
    if (!started && t >= t_start - 1e-9) {
      started = true; sx0 = ch[0]; sy0 = ch[1]; syaw0 = ch[5];
      if (gyro_cal && gcal_n > 0) gyro_bias_est = gcal_sum / gcal_n;
      odo[0] = fus[0] = odo_ideal[0] = ch[0]; odo[1] = fus[1] = odo_ideal[1] = ch[1]; odo[2] = fus[2] = odo_ideal[2] = ch[5];
    }
    if (started && !finished && t - t_start >= prof.T) { finished = true; t_finish = t; }
    // IMU at the CoM (body rates, specific force from the chassis motion)
    for (int k = 0; k < 6; ++k) kin.vel_in[k] = ch_vel[k];
    kin.have_vel = true;
    bs = kin.update(ch, dt);
    imu.accumulate(t, dt, bs.w_body, bs.f_body);
    if (!started && imu.fresh && t > 0.5 * t_start) { gcal_sum += imu.gyro.z; gcal_n += 1; }
    // contacts
    const double refresh = std::pow(0.05 * std::min(terrain.flat.base.corr_len, terrain.res()), 2);
    for (size_t i = 0; i < in.size() && i < wheels.size(); ++i) {
      OmniIn ci; ci.center = in[i].center; ci.vel = in[i].vel; ci.axle = in[i].axle;
      ci.omega = in[i].angvel.dot(in[i].axle.normalized()); ci.angvel = in[i].angvel;
      terrain.ground(in[i].center.x, in[i].center.y, ci.ground_z, ci.n);
      const Vec3 dp = in[i].center - surf_cache_p[i];
      if (dp.x * dp.x + dp.y * dp.y > refresh) { surf_cache[i] = terrain.surface(in[i].center.x, in[i].center.y); surf_cache_p[i] = in[i].center; }
      last_c[i] = contacts[i].step(ci, surf_cache[i], oo, dt);
      out[i].force = last_c[i].force; out[i].torque = last_c[i].torque;
      enc_ang[i] += in[i].joint_vel * dt;
    }
    body_step(t, ch);
    if (started) {                           // ideal wheel odometry: exact wheel speeds at every step
      std::vector<double> wj(wheels.size());
      for (size_t i = 0; i < wheels.size() && i < in.size(); ++i) wj[i] = in[i].joint_vel;
      double tw[3]; odo_ideal_est.twist(wj, tw);
      Odometer::integrate(odo_ideal, tw, tw[2], dt);
    }
    // control tick: encoders -> odometry, IMU fusion, controller -> wheel set speeds
    if (t + 1e-12 >= next_ctrl) {
      const double Tc = 1.0 / ctrl_rate;
      const double dtc = prev_ctrl_t < 0 ? Tc : t - prev_ctrl_t;
      for (size_t i = 0; i < wheels.size(); ++i) {
        if (enc_ticks > 0) {        // encoder counts on the motor shaft
          const double step_ang = 2 * kPi / (enc_ticks * mp.gear);
          const double cnt = std::floor(enc_ang[i] / step_ang);
          w_enc[i] = (cnt - enc_prev_cnt[i]) * step_ang / std::max(dtc, 1e-6); enc_prev_cnt[i] = cnt;
        } else {
          w_enc[i] = (enc_ang[i] - enc_prev_cnt[i]) / std::max(dtc, 1e-6); enc_prev_cnt[i] = enc_ang[i];
        }
      }
      if (started) estimate(t, dtc);
      control(t, ch);
      twist_to_wheels(cmd, wset);
      if (replay && replay_mode == 0) {                 // recorded wheel speeds per motor group
        const bool on = t >= t_start && !finished;
        for (size_t i = 0; i < wheels.size(); ++i)
          wset[i] = (on && wheels[i].group >= 0) ? rp.at((size_t)rp_col[(size_t)wheels[i].group], t - t_start) : 0.0;
        double tw[3] = {0, 0, 0};                       // the same as a twist, for the log
        for (size_t i = 0; i < wheels.size(); ++i) for (int a = 0; a < 3; ++a) tw[a] += Jp[a][i] * wset[i];
        cmd[0] = tw[0]; cmd[1] = tw[1]; cmd[2] = tw[2] * (holonomic ? 1.0 : odom_yaw_scale);
      }
      prev_ctrl_t = t; next_ctrl = (next_ctrl < 0 ? t : next_ctrl) + Tc;
    }
    // motors (one per group) + chain coupling inside the group
    if (couple_c < 0) couple_c = 0.3 * Jw / std::max(dt, 1e-4);
    power = 0;
    for (size_t g = 0; g < groups.size(); ++g) {
      if (groups[g].empty()) continue;
      double ws = 0, wm = 0;
      for (size_t i : groups[g]) { ws += wset[i]; wm += in[i].joint_vel; }
      ws /= groups[g].size(); wm /= groups[g].size();
      if (motor_implicit) {          // stiff speed loop: torque at the predicted speed (see DCMotor::step_implicit)
        double te = 0;
        for (size_t i : groups[g]) te += out[i].torque.dot(in[i].axle.normalized());
        last_m[g] = motors[g].step_implicit(mp, ws, wm, dt, Jw * groups[g].size(), te);
      } else {
        last_m[g] = motors[g].step(mp, ws, wm, dt);
      }
      for (size_t i : groups[g])
        out[i].motor_torque = last_m[g].tau_wheel / groups[g].size() + (groups[g].size() > 1 ? couple_c * (wm - in[i].joint_vel) : 0.0);
      power += std::fabs((last_m[g].I * mp.R + mp.Ce * last_m[g].w_motor) * last_m[g].I);
    }
    energy += power * dt;
    log_step(t, ch, in);
    if (full.due(t)) full_step(t, ch, in, out);
    if (finished && !done_written && t >= t_finish + t_settle) {
      if (flog) flog->flush();
      full.flush();
      std::ofstream f(done_file); f << t << "\n"; done_written = true;
    }
    if (tipped && tip_stop && !done_written && t >= t_tip + 0.2) {   // [body] on_tip = stop: end the run shortly after tipping over
      if (flog) flog->flush();
      full.flush();
      std::ofstream f(done_file); f << t << "\n"; done_written = true;
    }
  }

  void log_step(double t, const double ch[6], const std::vector<VehWheelIn> &in) {
    if (t + 1e-12 < next_log) return;
    next_log = (next_log < 0 ? t : next_log) + 1.0 / log_rate;
    // tracking error w.r.t. the reference point (along / cross track in the reference frame)
    const double dx = ch[0] - ref[0], dy = ch[1] - ref[1];
    const double ea = std::cos(vel_dir) * dx + std::sin(vel_dir) * dy, ec = -std::sin(vel_dir) * dx + std::cos(vel_dir) * dy;
    const double cy = std::cos(ch[5]), sy = std::sin(ch[5]);
    const double vxb = cy * ch_vel[0] + sy * ch_vel[1], vyb = -sy * ch_vel[0] + cy * ch_vel[1];
    char b[64];
    auto put = [&](double v) { std::snprintf(b, sizeof b, ",%.6g", v); *flog << b; };
    std::snprintf(b, sizeof b, "%.4f", t); *flog << b << "," << (t < t_start ? 0 : (finished ? 2 : 1));
    for (double v : {ref[0], ref[1], ref[2], ref[3], ref[4], ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], vxb, vyb, ch_vel[5],
                     odo[0], odo[1], odo[2], fus[0], fus[1], fus[2], imu.gyro.x, imu.gyro.y, imu.gyro.z, imu.acc.x, imu.acc.y, imu.acc.z,
                     cmd[0], cmd[1], cmd[2], ea, ec, wrap(ch[5] - ref[2])}) put(v);
    *flog << "," << terrain.name_at(ch[0], ch[1]);
    for (size_t i = 0; i < wheels.size() && i < in.size(); ++i) {
      // cur, vol, pow: the motor driving this wheel (wheels of one group show the same motor);
      // tau: this wheel's share of the motor torque
      const auto &c = last_c[i]; const bool driven = wheels[i].group >= 0;
      const MotorOut m = driven ? last_m[(size_t)wheels[i].group] : MotorOut();
      const double n_g = driven ? groups[(size_t)wheels[i].group].size() : 1.0;
      const double U = m.I * mp.R + mp.Ce * m.w_motor;
      for (double v : {wset[i], w_enc[i], std::fabs(m.I), U, std::fabs(U * m.I), m.tau_wheel / n_g,
                       c.N, c.H, c.F_free, c.s, c.s_lat, c.z}) put(v);
      *flog << "," << terrain.dominant(in[i].center.x, in[i].center.y);
    }
    put(tilt_rel); put(body.N); *flog << "," << body.n << "\n";
    if (flush_each) flog->flush();
  }

  // full per-run log row (sensors.hpp FullLog): truth, IMU (noisy + ideal), odometries, per-wheel everything
  void full_step(double t, const double ch[6], const std::vector<VehWheelIn> &in, const std::vector<VehWheelOut> &out) {
    FullFrame F; F.t = t; F.phase = t < t_start ? 0 : (finished ? 2 : 1); F.run = 0; F.b = bs;
    if (started) { F.ref[0] = ref[0]; F.ref[1] = ref[1]; F.ref[2] = ref[2]; }
    for (int k = 0; k < 3; ++k) { F.cmd[k] = cmd[k]; F.odo_ideal[k] = odo_ideal[k]; F.odo_enc[k] = odo[k]; F.odo_imu[k] = fus[k]; }
    if (!started) for (int k = 0; k < 3; ++k) F.odo_ideal[k] = F.odo_enc[k] = F.odo_imu[k] = (k < 2 ? ch[k] : ch[5]);
    F.imu_g = imu.gyro; F.imu_a = imu.acc; F.imu_g_true = imu.gyro_true; F.imu_a_true = imu.acc_true;
    F.power = power; F.energy = energy; F.surf = terrain.name_at(ch[0], ch[1]);
    std::vector<FullWheel> W(wheels.size());
    for (size_t i = 0; i < wheels.size() && i < in.size(); ++i) {
      FullWheel &w = W[i]; const auto &c = last_c[i]; const auto &S = surf_cache[i];
      w.angle = enc_ang[i]; w.omega = in[i].joint_vel; w.omega_set = wset[i]; w.omega_enc = w_enc[i];
      w.omega_eff = J[i][0] * bs.v_body.x + J[i][1] * bs.v_body.y + J[i][2] * bs.w_body.z;
      if (wheels[i].group >= 0) {
        const auto &m = last_m[(size_t)wheels[i].group];
        w.cur = m.I; w.volt = m.I * mp.R + mp.Ce * m.w_motor; w.power = std::fabs(w.volt * w.cur);
      }
      w.tau_motor = out[i].motor_torque; w.tau_contact = out[i].torque.dot(in[i].axle.normalized());
      w.N = c.N; w.F_trac = c.H; w.F_free = c.F_free; w.Fx = c.force.x; w.Fy = c.force.y; w.Fz = c.force.z;
      w.slip = c.s; w.slip_lat = c.s_lat; w.slip_kin = wheels[i].g.type == 2 ? 0.0 : kinematic_slip(w.omega_eff, w.omega); w.sink = c.z;
      w.cx = in[i].center.x; w.cy = in[i].center.y;
      w.mu = std::tan(S.phi); w.K = S.K; w.f_roll = S.f_roll; w.f_lat = S.f_lat; w.mu_rb = S.mu_rb; w.kc = S.kc; w.kphi = S.kphi;
      w.surf = terrain.name_at(in[i].center.x, in[i].center.y);
    }
    full.write(F, W);
  }

  void set_chassis_vel(const double v[6]) { for (int k = 0; k < 6; ++k) ch_vel[k] = v[k]; has_ch_vel = true; }
};

}  // namespace terramech
