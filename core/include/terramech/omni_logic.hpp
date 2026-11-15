// omni_logic.hpp — engine-agnostic logic for omni-wheel robots (Robotino) used by both Gazebo plugins
// and by the planar surrogate. Selected in the INI by  [robot] type = omni.
//
//   [robot]       type = omni, R = 0.125 (wheel distance from centre, kinematics), r = 0.04, gamma_deg = 90
//   [wheel k]     link, joint, theta_deg       (k = 1..3, dataset numbering)
//   [surface X]   FirmSurface parameters (phi_deg|mu, c, K, patch_len, f_roll, f_lat, mu_rb, kn, cn,
//                 cv_res, cv_grip, corr_len, field_seed — spatial inhomogeneity)
//   [omni_contact] slip_relax_len, ...
//   [motor]       MotorParams
//   [experiment]  surface = X, speed = 0.1, dir_start = 0, dir_step = 5, n_dirs = 72, t_run = 18, t_pause = 1.5,
//                 t_start = 1.0, camera_rate = 30, return_to_start = 0|1 (drive back to the star centre after every
//                 ray: return_speed, return_tol, t_return_max)
//   [map]         dtm_file = map.dtm  (heights + surface type per node; type names = [surface ...] sections).
//                 Without [map] the floor is flat (ground_z) and made of [experiment] surface.
// Logs:
//   log_file (log.csv)          — exactly the columns of dataset_belyaev.csv (t in ms, reset every run), so the
//                                 same analysis code processes reality and simulation;
//   detail_file (log_detail.csv) — forces, normal loads, slips, motor quantities.
#pragma once
#include <cstdio>
#include <fstream>
#include "terramech/omni.hpp"
#include "terramech/sensors.hpp"

namespace terramech {

struct OmniWheelIn {           // filled by the engine wrapper
  Vec3 center, vel, angvel, axle;
  double joint_vel = 0;        // wheel speed relative to the chassis (encoder), rad/s about axle
};
struct OmniWheelOut { Vec3 force, torque; double motor_torque = 0; };

class OmniLogic {
 public:
  struct WheelDef { std::string label, link, joint; double theta = 0; };
  std::vector<WheelDef> wheels;
  std::vector<OmniContact> contacts;
  std::vector<DCMotor> motors;
  FirmSurface surf;
  SurfaceMap smap;
  // terrain map (optional): heights + surface types; parameters blended between nodes as in MarsSim eq. (3)
  std::unique_ptr<DTM> dtm;
  std::vector<SurfaceMap> tmaps;
  // run state machine: 0 settle / pause, 1 drive (logged), 2 return to the start
  bool ret_on = false; double ret_speed = 0.1, ret_tol = 0.01, ret_tmax = 30, ret_kp = 2.0;
  int st_run = -1, st_phase = 0; double st_t0 = 0; bool finished = false;
  double start_x = 0, start_y = 0, start_yaw = 0;
  OmniOpts oo;
  MotorParams mp;
  double R = 0.125, r = 0.04, gamma = deg2rad(90.0), ground_z = 0.0, Jw = 2e-3;
  bool motor_implicit = false;
  // experiment
  double speed = 0.1, dir_start = 0, dir_step = 5, t_run = 18, t_pause = 1.5, t_start = 1.0, cam_rate = 30;
  double Ke_In = 0.17;   // no-load current in the energy coefficient Ke (dataset: 0.17 A)
  int n_dirs = 72;
  std::vector<double> dirs;
  double t_end = 1e9;
  // logging
  std::string log_file, detail_file, done_file, surf_label;
  std::unique_ptr<std::ofstream> flog, fdet;
  double next_cam = -1, det_rate = 30, next_det = -1;
  int cam_run = -1;
  double cam_prev[4] = {0, 0, 0, 0};   // t, x, y, yaw of the previous camera frame
  bool cam_have_prev = false, done_written = false;
  // last outputs per wheel (for logging)
  std::vector<OmniOut> last_c;
  std::vector<MotorOut> last_m;
  std::vector<double> last_wset;
  std::vector<FirmSurface> surf_cache; std::vector<Vec3> surf_cache_p;   // field lookup cached per wheel
  // full per-run log (sensors.hpp): truth, IMU at the CoM, wheel odometry (ideal / encoder / encoder + gyro),
  // everything per wheel. Odometry is re-anchored to the true pose at the start of every ray.
  Imu imu; ChassisKinematics kin; BodyState bs; Odometer odo_est; FullLog full;
  double ch_vel[6] = {0, 0, 0, 0, 0, 0}; bool has_ch_vel = false;
  double odo_ideal[3] = {0, 0, 0}, odo_enc[3] = {0, 0, 0}, odo_imu[3] = {0, 0, 0};
  std::vector<double> enc_ang, enc_prev_cnt, w_enc, wheel_angle;
  double enc_ticks = 71, est_rate = 100, next_est = -1, prev_est_t = -1;
  bool gyro_cal = true; double gcal_sum = 0, gcal_n = 0, gyro_bias_est = 0;
  double energy = 0, power = 0; int odo_run = -2;
  std::vector<std::string> type_names;

  static double ang_wrap(double a) { while (a > M_PI) a -= 2 * M_PI; while (a < -M_PI) a += 2 * M_PI; return a; }

  bool configure(const Config &cfg, const std::string &log_path, const std::string &done_path, std::string &err) {
    auto *RB = cfg.find("robot");
    R = Config::num(RB, "R", R); r = Config::num(RB, "r", r);
    gamma = deg2rad(Config::num(RB, "gamma_deg", 90.0));
    ground_z = Config::num(RB, "ground_z", 0.0);
    Jw = Config::num(RB, "Jw", Jw);                       // wheel spin inertia incl. reflected rotor (as in the SDF)
    motor_implicit = Config::num(cfg.find("motor"), "implicit", 0) > 0.5;   // explicit is exact at 1 ms
    for (auto *ws : cfg.all("wheel")) {
      WheelDef w; w.label = ws->label; w.link = Config::str(ws, "link", ""); w.joint = Config::str(ws, "joint", "");
      w.theta = deg2rad(Config::num(ws, "theta_deg", 0)); wheels.push_back(w);
    }
    if (wheels.empty()) { err = "omni: no [wheel] sections"; return false; }
    auto *EX = cfg.find("experiment");
    surf_label = Config::str(EX, "surface", "floor");
    const Config::Section *SS = cfg.find("surface", surf_label);
    if (!SS) { err = "omni: no [surface " + surf_label + "]"; return false; }
    surf_label = Config::str(EX, "label", surf_label);       // value of the 'surf' column (map runs: map name)
    surf = firm_from_section(SS);
    smap.init(surf);
    auto *MP = cfg.find("map");
    const std::string mapf = cfg.path(Config::str(MP, "dtm_file", ""));
    if (!mapf.empty()) {
      dtm.reset(new DTM());
      if (!dtm->load(mapf, &err)) return false;
      for (auto &tn : dtm->type_names) {
        const Config::Section *TS = cfg.find("surface", tn);
        if (!TS) { err = "omni: map type '" + tn + "' has no [surface " + tn + "] section"; return false; }
        SurfaceMap m; m.init(firm_from_section(TS)); tmaps.push_back(m);
      }
    }
    auto *OC = cfg.find("omni_contact");
    oo.slip_v_eps = Config::num(OC, "slip_v_eps", oo.slip_v_eps);
    oo.slip_v_ramp = Config::num(OC, "slip_v_ramp", oo.slip_v_ramp);
    oo.slip_relax_len = Config::num(OC, "slip_relax_len", oo.slip_relax_len);
    oo.slip_tau_min = Config::num(OC, "slip_tau_min", oo.slip_tau_min);
    oo.v_eps = Config::num(OC, "v_eps", oo.v_eps);
    oo.slip_norm = (int)Config::num(OC, "slip_norm", oo.slip_norm);
    oo.slip_max = Config::num(OC, "slip_max", oo.slip_max);
    mp = motor_from_section(cfg.find("motor"));
    speed = Config::num(EX, "speed", speed);
    dir_start = Config::num(EX, "dir_start", dir_start); dir_step = Config::num(EX, "dir_step", dir_step);
    n_dirs = (int)Config::num(EX, "n_dirs", n_dirs);
    t_run = Config::num(EX, "t_run", t_run); t_pause = Config::num(EX, "t_pause", t_pause);
    t_start = Config::num(EX, "t_start", t_start); cam_rate = Config::num(EX, "camera_rate", cam_rate);
    Ke_In = Config::num(EX, "Ke_In", mp.I0);
    det_rate = Config::num(EX, "detail_rate", cam_rate);
    auto dl = Config::list(EX, "directions");
    if (!dl.empty()) dirs = dl; else for (int k = 0; k < n_dirs; ++k) dirs.push_back(dir_start + k * dir_step);
    ret_on = Config::num(EX, "return_to_start", 0) > 0.5;
    ret_speed = Config::num(EX, "return_speed", speed); ret_tol = Config::num(EX, "return_tol", ret_tol);
    ret_tmax = Config::num(EX, "t_return_max", 3.0 * speed * t_run / std::max(ret_speed, 1e-3) + 3.0);
    t_end = t_start + dirs.size() * (t_run + t_pause + (ret_on ? ret_tmax : 0.0)) + 0.05;   // upper bound
    contacts.assign(wheels.size(), OmniContact());
    for (auto &c : contacts) { c.geom.r = r; c.geom.gamma = gamma; }
    motors.assign(wheels.size(), DCMotor());
    last_c.assign(wheels.size(), OmniOut()); last_m.assign(wheels.size(), MotorOut()); last_wset.assign(wheels.size(), 0.0);
    surf_cache.assign(wheels.size(), surf); surf_cache_p.assign(wheels.size(), Vec3(1e9, 1e9, 0));
    {
      std::vector<std::array<double, 3>> rows;
      for (auto &w : wheels) rows.push_back({-std::sin(w.theta) / r, std::cos(w.theta) / r, R / r});
      odo_est.init(rows, 1.0);
      enc_ang.assign(wheels.size(), 0.0); enc_prev_cnt.assign(wheels.size(), 0.0); w_enc.assign(wheels.size(), 0.0); wheel_angle.assign(wheels.size(), 0.0);
      enc_ticks = Config::num(RB, "encoder_ticks", enc_ticks); est_rate = std::max(1.0, Config::num(RB, "odom_rate", est_rate));
      kin.com = Vec3(Config::num(RB, "com_x", 0.0), Config::num(RB, "com_y", 0.0), Config::num(RB, "com_h", 0.08));
      auto *IM = cfg.find("imu");
      imu.init(imu_from_section(IM)); gyro_cal = Config::num(IM, "calibrate", 1) > 0.5;
      if (dtm) type_names = dtm->type_names;
    }
    log_file = log_path; done_file = done_path;
    auto *G = cfg.find("general");
    detail_file = cfg.path(Config::str(G, "detail_file", ""));
    if (detail_file.empty()) {
      size_t p = log_file.find_last_of('.');
      detail_file = (p == std::string::npos ? log_file : log_file.substr(0, p)) + "_detail.csv";
    }
    flog.reset(new std::ofstream(log_file));
    fdet.reset(new std::ofstream(detail_file));
    if (!*flog || !*fdet) { err = "omni: cannot open logs " + log_file; return false; }
    full.rate = Config::num(G, "full_rate", 50.0);
    if (full.rate > 0) {
      std::string ff = cfg.path(Config::str(G, "full_log_file", ""));
      if (ff.empty()) ff = full_log_path(log_file);
      std::vector<std::string> labels; for (auto &w : wheels) labels.push_back(w.label);
      if (!full.open(ff, wheels.size(), labels)) { err = "omni: cannot open " + ff; return false; }
    }
    *flog << "t,surf,movedir,speedamp,xsetspeed,ysetspeed";
    for (const char *k : {"setvel", "vel", "cur", "vol", "pow"}) for (size_t i = 1; i <= wheels.size(); ++i) *flog << ",m" << i << k;
    *flog << ",xpos,ypos,ang,vx,vy,omega,rpower,Ke";
    for (const char *k : {"vel", "effvel", "slip", "linslip"}) for (size_t i = 1; i <= wheels.size(); ++i) *flog << ",w" << i << k;
    // surface under every wheel at this sample (the heterogeneity field and map borders make these vary in time):
    // contact point (world, m) and the local parameters used by the contact model at that instant
    for (size_t i = 1; i <= wheels.size(); ++i)
      for (const char *k : {"x", "y", "mu", "K", "f_roll", "f_lat", "mu_rb", "kc", "kphi"}) *flog << ",w" << i << "_" << k;
    *flog << "\n";
    *fdet << "t,run,dir,phase,x,y,z,yaw";
    for (size_t i = 1; i <= wheels.size(); ++i)
      for (const char *k : {"N", "H", "Ffree", "Mroll", "s", "vx", "vy", "I", "V", "w", "wset", "tau", "z", "Rc"}) *fdet << "," << k << i;
    *fdet << "\n";
    return true;
  }

  // run state machine (replaces the fixed schedule so that a ray may be followed by a return to the centre)
  void update_state(double t, const double ch[6]) {
    if (finished) return;
    if (st_run < 0) {
      if (t < t_start) return;
      start_x = ch[0]; start_y = ch[1]; start_yaw = ch[5];
      st_run = 0; st_phase = 1; st_t0 = t; return;
    }
    if (st_phase == 1 && t - st_t0 >= t_run) { st_phase = ret_on ? 2 : 0; st_t0 = t; return; }
    if (st_phase == 2) {
      const double d = std::hypot(ch[0] - start_x, ch[1] - start_y);
      if ((d < ret_tol && std::fabs(ang_wrap(ch[5] - start_yaw)) < 0.02) || t - st_t0 > ret_tmax) { st_phase = 0; st_t0 = t; }
      return;
    }
    if (st_phase == 0 && t - st_t0 >= t_pause) {
      ++st_run;
      if (st_run >= (int)dirs.size()) { finished = true; st_phase = 0; return; }
      st_phase = 1; st_t0 = t;
    }
  }

  // commanded body velocity (robot frame)
  void command(const double ch[6], double &vx, double &vy, double &wz, double &alpha) const {
    vx = vy = wz = alpha = 0;
    if (st_phase == 1) { alpha = deg2rad(dirs[st_run]); vx = speed * std::cos(alpha); vy = speed * std::sin(alpha); }
    else if (st_phase == 2) {
      double ex = ret_kp * (start_x - ch[0]), ey = ret_kp * (start_y - ch[1]);
      const double n = std::hypot(ex, ey);
      if (n > ret_speed) { ex *= ret_speed / n; ey *= ret_speed / n; }
      const double c = std::cos(ch[5]), s = std::sin(ch[5]);
      vx = c * ex + s * ey; vy = -s * ex + c * ey;
      wz = clamp(-2.0 * ang_wrap(ch[5] - start_yaw), -0.5, 0.5);
    }
  }

  // floor under a point: height, normal, surface parameters
  void ground_at(double x, double y, double &h, Vec3 &n) const {
    if (!dtm) { h = ground_z; n = Vec3(0, 0, 1); return; }
    const double e = dtm->res;
    h = dtm->height(x, y);
    const double hx = (dtm->height(x + e, y) - dtm->height(x - e, y)) / (2 * e), hy = (dtm->height(x, y + e) - dtm->height(x, y - e)) / (2 * e);
    n = Vec3(-hx, -hy, 1.0).normalized();
  }
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
    // deformable layer: stiffness k_s = kc + b kphi blended through the compliance 1/k_s (a firm floor has zero
    // compliance), so the sinkage fades to 0 towards a firm neighbour; n, layer, rebound over deformable nodes
    double comp = 0, wd = 0, nb = 0, lay = 0, reb = 0, b = 0;
    for (auto &p : P) if (p.first.deformable()) {
      const double ks = p.first.kc + p.first.b_contact * p.first.kphi;
      comp += p.second / ks; wd += p.second; nb += p.second * p.first.n_bek; lay += p.second * p.first.layer;
      reb += p.second * p.first.rebound; b += p.second * p.first.b_contact;
    }
    if (wd > 0) {
      s.b_contact = b / wd; s.kc = 0.0; s.kphi = (W / comp) / s.b_contact;
      s.n_bek = nb / wd; s.layer = lay / wd; s.rebound = reb / wd;
    } else { s.kc = 0.0; s.kphi = 0.0; }
    return s;
  }
  FirmSurface surface_at(double x, double y) const {
    if (!dtm) return smap.local(x, y);
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

  // body velocity command (robot frame) -> wheel set speeds; dataset kinematics
  //   w_i = (1/r) (-sin th_i vx + cos th_i vy + R wz)
  double wheel_set(size_t i, double vx, double vy, double wz) const {
    return (-std::sin(wheels[i].theta) * vx + std::cos(wheels[i].theta) * vy + R * wz) / r;
  }

  // chassis: x, y, z, roll, pitch, yaw (world)
  void step(double t, double dt, const std::vector<OmniWheelIn> &in, std::vector<OmniWheelOut> &out, const double chassis[6]) {
    out.assign(in.size(), OmniWheelOut());
    update_state(t, chassis);
    const int run = finished ? (int)dirs.size() : st_run, phase = st_phase;
    const double t_in = t - st_t0;
    double vx = 0, vy = 0, wz = 0, alpha = 0;
    command(chassis, vx, vy, wz, alpha);
    const double refresh = std::pow(0.05 * std::min(surf.corr_len, dtm ? dtm->res : 1e9), 2);
    for (size_t i = 0; i < in.size() && i < wheels.size(); ++i) {
      OmniIn ci; ci.center = in[i].center; ci.vel = in[i].vel; ci.axle = in[i].axle;
      ci.omega = in[i].angvel.dot(in[i].axle.normalized());
      ground_at(in[i].center.x, in[i].center.y, ci.ground_z, ci.n);
      // surface parameters at the contact (smooth on corr_len / map resolution: refresh every 1/20 of it)
      const Vec3 dp = in[i].center - surf_cache_p[i];
      if (dp.x * dp.x + dp.y * dp.y > refresh) {
        surf_cache[i] = surface_at(in[i].center.x, in[i].center.y); surf_cache_p[i] = in[i].center;
      }
      OmniOut co = contacts[i].step(ci, surf_cache[i], oo, dt);
      double ws = wheel_set(i, vx, vy, wz);
      const double tau_c = co.torque.dot(in[i].axle.normalized());
      MotorOut mo = motor_implicit ? motors[i].step_implicit(mp, ws, in[i].joint_vel, dt, Jw, tau_c)
                                   : motors[i].step(mp, ws, in[i].joint_vel, dt);
      out[i].force = co.force; out[i].torque = co.torque; out[i].motor_torque = mo.tau_wheel;
      last_c[i] = co; last_m[i] = mo; last_wset[i] = ws;
    }
    log_step(t, run, phase, t_in, vx, vy, alpha, in, chassis);
    sense(t, dt, run, phase, vx, vy, wz, in, out, chassis);
  }

  std::string surf_name_at(double x, double y) const {
    if (!dtm || type_names.empty()) return surf.name;
    int i, j; double u, v; dtm->cell(x, y, i, j, u, v);
    i += u > 0.5 ? 1 : 0; j += v > 0.5 ? 1 : 0;
    i = std::max(0, std::min(dtm->nx - 1, i)); j = std::max(0, std::min(dtm->ny - 1, j));
    const int k = dtm->T[(size_t)j * dtm->nx + i];
    return k >= 0 && k < (int)type_names.size() ? type_names[(size_t)k] : surf.name;
  }

  // IMU, wheel odometry (ideal / encoder / encoder + gyro), energy, full per-run log
  void sense(double t, double dt, int run, int phase, double vx, double vy, double wz, const std::vector<OmniWheelIn> &in,
             const std::vector<OmniWheelOut> &out, const double ch[6]) {
    const size_t nw = std::min(in.size(), wheels.size());
    for (int k = 0; k < 6; ++k) kin.vel_in[k] = ch_vel[k];
    kin.have_vel = has_ch_vel;
    bs = kin.update(ch, dt);
    imu.accumulate(t, dt, bs.w_body, bs.f_body);
    if (st_run < 0 && imu.fresh && t > 0.5 * t_start) { gcal_sum += imu.gyro.z; gcal_n += 1; }
    for (size_t i = 0; i < nw; ++i) { wheel_angle[i] += in[i].joint_vel * dt; enc_ang[i] += in[i].joint_vel * dt; }
    if (phase == 1 && run != odo_run) {           // a new ray: odometry re-anchored to the true pose
      odo_run = run;
      odo_ideal[0] = odo_enc[0] = odo_imu[0] = ch[0]; odo_ideal[1] = odo_enc[1] = odo_imu[1] = ch[1]; odo_ideal[2] = odo_enc[2] = odo_imu[2] = ch[5];
      if (gyro_cal && gcal_n > 0) gyro_bias_est = gcal_sum / gcal_n;
    }
    if (odo_run >= 0) {
      std::vector<double> wj(nw); for (size_t i = 0; i < nw; ++i) wj[i] = in[i].joint_vel;
      double tw[3]; odo_est.twist(wj, tw); Odometer::integrate(odo_ideal, tw, tw[2], dt);
    }
    if (t + 1e-12 >= next_est) {
      const double Tc = 1.0 / est_rate, dtc = prev_est_t < 0 ? Tc : t - prev_est_t;
      for (size_t i = 0; i < nw; ++i) {
        const double step_ang = 2 * kPi / (std::max(enc_ticks, 1.0) * mp.gear);
        const double cnt = std::floor(enc_ang[i] / step_ang);
        w_enc[i] = (cnt - enc_prev_cnt[i]) * step_ang / std::max(dtc, 1e-6); enc_prev_cnt[i] = cnt;
      }
      if (odo_run >= 0) {
        double tw[3]; odo_est.twist(w_enc, tw);
        Odometer::integrate(odo_enc, tw, tw[2], dtc);
        Odometer::integrate(odo_imu, tw, imu.gyro.z - gyro_bias_est, dtc);
      }
      prev_est_t = t; next_est = (next_est < 0 ? t : next_est) + Tc;
    }
    power = 0;
    for (size_t i = 0; i < nw; ++i) power += std::fabs((last_m[i].I * mp.R + mp.Ce * in[i].joint_vel * mp.gear) * last_m[i].I);
    energy += power * dt;
    if (!full.due(t)) return;
    FullFrame F; F.t = t; F.phase = phase; F.run = run; F.dir = (run >= 0 && run < (int)dirs.size()) ? dirs[(size_t)run] : NAN; F.b = bs;
    F.cmd[0] = vx; F.cmd[1] = vy; F.cmd[2] = wz;
    for (int k = 0; k < 3; ++k) { F.odo_ideal[k] = odo_ideal[k]; F.odo_enc[k] = odo_enc[k]; F.odo_imu[k] = odo_imu[k]; }
    if (odo_run < 0) for (int k = 0; k < 3; ++k) F.odo_ideal[k] = F.odo_enc[k] = F.odo_imu[k] = (k < 2 ? ch[k] : ch[5]);
    F.imu_g = imu.gyro; F.imu_a = imu.acc; F.imu_g_true = imu.gyro_true; F.imu_a_true = imu.acc_true;
    F.power = power; F.energy = energy; F.surf = surf_name_at(ch[0], ch[1]);
    std::vector<FullWheel> W(nw);
    for (size_t i = 0; i < nw; ++i) {
      FullWheel &w = W[i]; const auto &c = last_c[i]; const auto &S = surf_cache[i]; const auto &m = last_m[i];
      w.angle = wheel_angle[i]; w.omega = in[i].joint_vel; w.omega_set = last_wset[i]; w.omega_enc = w_enc[i];
      w.omega_eff = wheel_set(i, bs.v_body.x, bs.v_body.y, bs.w_body.z);
      w.cur = m.I; w.volt = m.I * mp.R + mp.Ce * in[i].joint_vel * mp.gear; w.power = std::fabs(w.volt * w.cur);
      w.tau_motor = out[i].motor_torque; w.tau_contact = out[i].torque.dot(in[i].axle.normalized());
      w.N = c.N; w.F_trac = c.H; w.F_free = c.F_free; w.Fx = c.force.x; w.Fy = c.force.y; w.Fz = c.force.z;
      w.slip = c.s; w.slip_lat = c.s_lat; w.slip_kin = kinematic_slip(w.omega_eff, w.omega); w.sink = c.z;
      w.cx = in[i].center.x; w.cy = in[i].center.y;
      w.mu = std::tan(S.phi); w.K = S.K; w.f_roll = S.f_roll; w.f_lat = S.f_lat; w.mu_rb = S.mu_rb; w.kc = S.kc; w.kphi = S.kphi;
      w.surf = surf_name_at(in[i].center.x, in[i].center.y);
    }
    full.write(F, W);
  }

  void set_chassis_vel(const double v[6]) { for (int k = 0; k < 6; ++k) ch_vel[k] = v[k]; has_ch_vel = true; }

  void log_step(double t, int run, int phase, double t_in, double vx_set, double vy_set, double alpha,
                const std::vector<OmniWheelIn> &in, const double ch[6]) {
    const size_t nw = std::min(in.size(), wheels.size());
    if (t >= next_det) {
      *fdet << t << "," << run << "," << (run >= 0 && run < (int)dirs.size() ? dirs[run] : -1) << "," << phase << ","
            << ch[0] << "," << ch[1] << "," << ch[2] << "," << ch[5];
      for (size_t i = 0; i < nw; ++i) {
        const auto &c = last_c[i]; const auto &m = last_m[i];
        *fdet << "," << c.N << "," << c.H << "," << c.F_free << "," << c.M_roll << "," << c.s << "," << c.v_x << "," << c.v_y
              << "," << m.I << "," << m.V << "," << in[i].joint_vel << "," << last_wset[i] << "," << m.tau_wheel
              << "," << c.z << "," << c.R_comp;
      }
      *fdet << "\n";
      next_det = (next_det < 0 ? t : next_det) + 1.0 / det_rate;
    }
    // ---- camera-rate dataset log, only while driving (like the real runs)
    if (phase != 1) { cam_have_prev = false; return; }
    if (run != cam_run) { cam_run = run; cam_have_prev = false; next_cam = t; }
    if (t + 1e-9 < next_cam) return;
    next_cam += 1.0 / cam_rate;
    double cvx = 0, cvy = 0, com = 0;
    if (cam_have_prev) {
      double dtc = t - cam_prev[0];
      double dxw = (ch[0] - cam_prev[1]) / dtc, dyw = (ch[1] - cam_prev[2]) / dtc;
      double yaw_m = cam_prev[3] + 0.5 * ang_wrap(ch[5] - cam_prev[3]);
      cvx = std::cos(yaw_m) * dxw + std::sin(yaw_m) * dyw;
      cvy = -std::sin(yaw_m) * dxw + std::cos(yaw_m) * dyw;
      com = ang_wrap(ch[5] - cam_prev[3]) / dtc;
    }
    cam_prev[0] = t; cam_prev[1] = ch[0]; cam_prev[2] = ch[1]; cam_prev[3] = ch[5]; cam_have_prev = true;
    char buf[64];
    auto put = [&](double v) { std::snprintf(buf, sizeof(buf), ",%.7g", v); *flog << buf; };
    std::snprintf(buf, sizeof(buf), "%.3f", t_in * 1000.0); *flog << buf << "," << surf_label;
    put(dirs[run]); put(speed); put(vx_set); put(vy_set);
    for (size_t i = 0; i < nw; ++i) put(last_wset[i] * mp.gear);
    for (size_t i = 0; i < nw; ++i) put(in[i].joint_vel * mp.gear);
    for (size_t i = 0; i < nw; ++i) put(std::fabs(last_m[i].I));
    std::vector<double> U(nw);
    for (size_t i = 0; i < nw; ++i) { U[i] = last_m[i].I * mp.R + mp.Ce * in[i].joint_vel * mp.gear; put(U[i]); }
    double P_real = 0, U_abs = 0;
    for (size_t i = 0; i < nw; ++i) { put(std::fabs(U[i] * last_m[i].I)); P_real += std::fabs(U[i] * last_m[i].I); U_abs += std::fabs(U[i]); }
    double yaw_deg = std::fmod(rad2deg(ch[5]) + 720.0, 360.0);
    put(ch[0]); put(ch[1]); put(yaw_deg); put(cvx); put(cvy); put(com);
    put(P_real); put(U_abs > 1e-9 ? P_real / (Ke_In * U_abs) : 0.0);    // rpower, Ke (Robotics 2025, 14, 130, eq. 1)
    std::vector<double> we(nw), wv(nw);
    for (size_t i = 0; i < nw; ++i) { wv[i] = in[i].joint_vel; we[i] = wheel_set(i, cvx, cvy, com); }
    for (size_t i = 0; i < nw; ++i) put(wv[i]);
    for (size_t i = 0; i < nw; ++i) put(we[i]);
    std::vector<double> s(nw);
    for (size_t i = 0; i < nw; ++i) { s[i] = std::fabs(wv[i]) > 1e-6 ? 1.0 - we[i] / wv[i] : 0.0; put(s[i]); }
    for (size_t i = 0; i < nw; ++i) put(std::fabs(std::sin(alpha - wheels[i].theta)) * s[i]);
    for (size_t i = 0; i < nw; ++i) {
      const auto &S = surf_cache[i];
      for (double v : {in[i].center.x, in[i].center.y, std::tan(S.phi), S.K, S.f_roll, S.f_lat, S.mu_rb, S.kc, S.kphi}) put(v);
    }
    *flog << "\n";
  }

  void finish(double t) {
    if (done_written || (!finished && t < t_end)) return;
    if (flog) flog->flush();
    if (fdet) fdet->flush();
    full.flush();
    std::ofstream f(done_file); f << t << "\n";
    done_written = true;
  }
};

}  // namespace terramech
