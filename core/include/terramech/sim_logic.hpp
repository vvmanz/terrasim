// sim_logic.hpp — engine-agnostic part of the Gazebo Harmonic (gz-sim 8) system plugin and the offline core.
// The engine wrapper fills WheelIO / JointIO each step, calls SimLogic::step(), and applies
// the returned wrenches/torques. Everything else (config, terramechanics, motor control,
// differential, single-wheel bench, logging) lives here and is identical for both engines.
#pragma once
#include <cstdlib>
#include "terramech/terramech.hpp"
#include "terramech/omni_logic.hpp"
#include "terramech/vehicle_logic.hpp"

namespace terramech {

struct WheelIO {
  // ---- filled by engine
  Vec3 center, vel, angvel;   // wheel link origin (= wheel centre), world
  Vec3 axle_left;             // world direction of the wheel axle pointing to the left
  Vec3 parent_x;              // x axis of the carrier (parent) link, world
  Vec3 hub_force;             // force the wheel exerts on its joint/carrier, world (if available)
  bool has_hub = false;
  double joint_vel = 0;       // motor speed (rad/s) about axle_left
  // ---- set by SimLogic
  Vec3 force, torque;         // terrain wrench to apply at wheel centre, world
  double motor_torque = 0;    // about axle_left
};

struct WheelCfg {
  std::string label, link, joint;
  double axle_sign = 1.0;     // multiply joint axis by this to get axle_left
  Vec3 axle_local{0, 1, 0};   // axle in wheel-link frame (pointing left)
  WheelGeom geom;
  double r_collision = 0.145;
  double mass = 4.0;
};

struct JointIO { double pos = 0, vel = 0, effort = 0; };

class SimLogic {
 public:
  Config cfg;
  DTM dtm;
  ModelOpts mo;
  ContactOpts co;
  std::vector<WheelCfg> wheels;
  std::vector<WheelContact> contacts;
  std::vector<RigidContact> rcontacts;
  std::vector<RigidBox> boxes;
  RigidOpts ro;
  std::vector<PIController> pids;
  Schedule omega_sched;
  bool terramech_on = true;
  bool drive_on = true;
  double log_rate = 100, next_log = -1e9, t_end = 1e9, gravity = 9.81;
  std::string log_file, done_file, chassis_link;
  bool done_written = false;
  Csv log;
  std::vector<std::vector<double>> last;  // last per-wheel log values
  // differential
  bool diff_on = false; std::string diff_left, diff_right; double diff_k = 2e4, diff_c = 500;
  // bench (single wheel testbed)
  bool bench_on = false; std::string bench_joint; Schedule slip_sched; PIController bench_pid; double bench_vmax = 1.0;
  double bench_force = 0;
  // omni-wheel robot mode ([robot] type = omni): all physics/logging in OmniLogic
  bool omni_on = false; OmniLogic omni;
  std::vector<OmniWheelIn> omni_in; std::vector<OmniWheelOut> omni_out;
  // any wheeled robot on a trajectory ([robot] type = vehicle): all physics/control/logging in VehicleLogic
  bool veh_on = false; VehicleLogic veh;
  std::vector<VehWheelIn> veh_in; std::vector<VehWheelOut> veh_out;
  // chassis linear + angular velocity (world), set by the engine wrapper before step() when available
  double chassis_vel[6] = {0, 0, 0, 0, 0, 0}; bool has_chassis_vel = false;
  double chassis_q[4] = {1, 0, 0, 0}; bool has_chassis_q = false;   // chassis orientation (w, x, y, z), world
  // body-ground contact wrench of the vehicle mode, to apply to the chassis link: world frame, the force at the link
  // origin and the torque about the link origin (gz::sim::Link::AddWorldWrench semantics)
  Vec3 body_force, body_torque; bool body_on() const { return veh_on && veh.body.on; }

  static std::string env(const char *k, const std::string &d) { const char *v = std::getenv(k); return v && *v ? std::string(v) : d; }

  bool configure(const std::string &ini, std::string &err) {
    std::string path = env("TM_CONFIG", ini);
    if (!cfg.load(path)) { err = "terramech: cannot read config " + path; return false; }
    auto *G = cfg.find("general");
    std::string dtmf = cfg.path(Config::str(G, "dtm_file", ""));
    if (!dtmf.empty() && !dtm.load(dtmf, &err)) return false;
    for (auto *ts : cfg.all("terrain_type"))
      for (size_t k = 0; k < dtm.type_names.size(); ++k)
        if (dtm.type_names[k] == ts->label) dtm.types[k] = soil_from_section(ts, dtm.types[k]);
    terramech_on = Config::num(G, "terramechanics", 1) > 0.5;
    log_rate = Config::num(G, "log_rate", 100);
    t_end = Config::num(G, "t_end", 1e9);
    gravity = Config::num(G, "gravity", 9.81);
    log_file = env("TM_LOG", cfg.path(Config::str(G, "log_file", "terramech_log.csv")));
    done_file = env("TM_DONE", log_file + ".done");
    chassis_link = Config::str(G, "chassis_link", "chassis");
    if (Config::str(cfg.find("robot"), "type", "") == "omni") {
      omni_on = true; drive_on = true; terramech_on = false;
      for (auto *ws : cfg.all("wheel")) {
        WheelCfg w; w.label = ws->label; w.link = Config::str(ws, "link", ""); w.joint = Config::str(ws, "joint", "");
        w.axle_sign = Config::num(ws, "axle_sign", 1.0); wheels.push_back(w);
      }
      if (!omni.configure(cfg, log_file, done_file, err)) return false;
      t_end = omni.t_end;
      return true;
    }
    if (Config::str(cfg.find("robot"), "type", "") == "vehicle") {
      veh_on = true; drive_on = true; terramech_on = false;
      for (auto *ws : cfg.all("wheel")) {
        WheelCfg w; w.label = ws->label; w.link = Config::str(ws, "link", ""); w.joint = Config::str(ws, "joint", "");
        w.axle_sign = Config::num(ws, "axle_sign", 1.0); wheels.push_back(w);
      }
      auto *DFv = cfg.find("differential");
      if (DFv) { diff_on = true; diff_left = Config::str(DFv, "left_joint", ""); diff_right = Config::str(DFv, "right_joint", "");
                 diff_k = Config::num(DFv, "k", diff_k); diff_c = Config::num(DFv, "c", diff_c); }
      if (!veh.configure(cfg, log_file, done_file, err)) return false;
      t_end = veh.t_end;
      return true;
    }
    mo = model_from_section(cfg.find("model"));
    auto *CO = cfg.find("contact");
    co.v_eps = Config::num(CO, "v_eps", co.v_eps);
    co.normal_damping = Config::num(CO, "normal_damping", 1500);
    co.tangential_damping = Config::num(CO, "tangential_damping", 0);
    co.use_three_point_plane = Config::num(CO, "three_point_plane", 1) > 0.5;
    co.theta1_init = Config::num(CO, "theta1_init", 0.1);
    co.slip_tau = Config::num(CO, "slip_tau", co.slip_tau);
    co.slip_relax_len = Config::num(CO, "slip_relax_len", co.slip_relax_len);
    co.slip_tau_min = Config::num(CO, "slip_tau_min", co.slip_tau_min);
    auto *RC = cfg.find("rigid_contact");
    ro.kn = Config::num(RC, "kn", ro.kn); ro.cn = Config::num(RC, "cn", ro.cn);
    ro.kt = Config::num(RC, "kt", ro.kt); ro.ct = Config::num(RC, "ct", ro.ct);
    ro.tau_relax = Config::num(RC, "tau_relax", ro.tau_relax);
    for (auto *bs : cfg.all("rigid_box")) {
      RigidBox bx; bx.x0 = Config::num(bs, "x0", 0); bx.x1 = Config::num(bs, "x1", 0);
      bx.y0 = Config::num(bs, "y0", -1e3); bx.y1 = Config::num(bs, "y1", 1e3);
      bx.ztop = Config::num(bs, "ztop", 0); bx.zbot = Config::num(bs, "zbot", -0.1); bx.mu = Config::num(bs, "mu", 0.6);
      boxes.push_back(bx);
    }
    co.slip_v_ramp = Config::num(CO, "slip_v_ramp", co.slip_v_ramp);
    auto *WD = cfg.find("wheel_defaults");
    for (auto *ws : cfg.all("wheel")) {
      WheelCfg w;
      w.label = ws->label;
      w.link = Config::str(ws, "link", "");
      w.joint = Config::str(ws, "joint", "");
      auto num = [&](const char *k, double d) { return Config::num(ws, k, Config::num(WD, k, d)); };
      w.axle_sign = num("axle_sign", 1.0);
      w.geom.r = num("r", 0.14); w.geom.b = num("b", 0.2); w.geom.h = num("h", 0.01);
      w.geom.lambda = num("lambda", 0.0); w.geom.Rj = num("Rj", -1);
      w.r_collision = num("r_collision", w.geom.r + 0.5 * w.geom.h);
      w.mass = num("mass", 4.0);
      wheels.push_back(w);
    }
    contacts.resize(wheels.size());
    rcontacts.resize(wheels.size());
    for (size_t i = 0; i < wheels.size(); ++i) { contacts[i].geom = wheels[i].geom; contacts[i].theta1 = co.theta1_init; }
    auto *DR = cfg.find("drive");
    drive_on = DR != nullptr;
    omega_sched.parse(Config::list(DR, "omega_schedule"));
    pids.resize(wheels.size());
    for (auto &p : pids) {
      p.kp = Config::num(DR, "kp", 50); p.ki = Config::num(DR, "ki", 500);
      p.umax = Config::num(DR, "max_torque", 200); p.imax = Config::num(DR, "integral_max", 1e3);
    }
    auto *DF = cfg.find("differential");
    if (DF) { diff_on = true; diff_left = Config::str(DF, "left_joint", ""); diff_right = Config::str(DF, "right_joint", "");
              diff_k = Config::num(DF, "k", diff_k); diff_c = Config::num(DF, "c", diff_c); }
    auto *BE = cfg.find("bench");
    if (BE) {
      bench_on = true; bench_joint = Config::str(BE, "carriage_joint", "carriage_joint");
      slip_sched.parse(Config::list(BE, "slip_schedule")); slip_sched.step = true;
      bench_pid.kp = Config::num(BE, "kp", 2000); bench_pid.ki = Config::num(BE, "ki", 2e4);
      bench_pid.umax = Config::num(BE, "max_force", 500); bench_pid.imax = 1e3;
    }
    std::vector<std::string> hdr = {"t", "cx", "cy", "cz", "roll", "pitch", "yaw", "bench_force", "bench_v"};
    for (auto &w : wheels)
      for (const char *k : {"xc", "yc", "zc", "omega", "omega_cmd", "s", "beta", "sink", "soilw", "on_rigid", "FN", "FDP", "FS", "MR",
                            "Fx_soil", "Fy_soil", "Fz_soil", "hub_x", "hub_y", "hub_z", "FDP_leg", "FDP_hub_fwd", "torque",
                            "Fn_rigid", "Ft_rigid"})
        hdr.push_back(std::string(k) + "_" + w.label);
    if (!log.open(log_file, hdr)) { err = "terramech: cannot open log " + log_file; return false; }
    last.assign(wheels.size(), std::vector<double>(25, 0.0));
    return true;
  }

  double omega_cmd(double t) const { return omega_sched.at(t); }

  // chassis pose for logging (x,y,z,roll,pitch,yaw); bench_v: carriage velocity
  void step(double t, double dt, std::vector<WheelIO> &io, const double chassis[6], const JointIO *diffL, const JointIO *diffR,
            double &diff_torque, const JointIO *bench, double &bench_cmd_force) {
    diff_torque = 0; bench_cmd_force = 0;
    if (omni_on) {
      omni_in.resize(io.size());
      for (size_t i = 0; i < io.size(); ++i) {
        omni_in[i].center = io[i].center; omni_in[i].vel = io[i].vel; omni_in[i].angvel = io[i].angvel;
        omni_in[i].axle = io[i].axle_left; omni_in[i].joint_vel = io[i].joint_vel;
      }
      if (has_chassis_vel) omni.set_chassis_vel(chassis_vel);
      omni.step(t, dt, omni_in, omni_out, chassis);
      for (size_t i = 0; i < io.size(); ++i) {
        io[i].force = omni_out[i].force; io[i].torque = omni_out[i].torque; io[i].motor_torque = omni_out[i].motor_torque;
      }
      omni.finish(t);
      return;
    }
    if (veh_on) {
      veh_in.resize(io.size());
      for (size_t i = 0; i < io.size(); ++i) {
        veh_in[i].center = io[i].center; veh_in[i].vel = io[i].vel; veh_in[i].angvel = io[i].angvel;
        veh_in[i].axle = io[i].axle_left; veh_in[i].joint_vel = io[i].joint_vel;
      }
      if (has_chassis_vel) veh.set_chassis_vel(chassis_vel);
      if (has_chassis_q) veh.set_chassis_quat(chassis_q);
      veh.step(t, dt, veh_in, veh_out, chassis);
      body_force = veh.body.F; body_torque = veh.body.T;
      for (size_t i = 0; i < io.size(); ++i) {
        io[i].force = veh_out[i].force; io[i].torque = veh_out[i].torque; io[i].motor_torque = veh_out[i].motor_torque;
      }
      if (diff_on && diffL && diffR) diff_torque = -diff_k * (diffL->pos + diffR->pos) - diff_c * (diffL->vel + diffR->vel);
      return;
    }
    const double wcmd = omega_cmd(t);
    for (size_t i = 0; i < io.size() && i < wheels.size(); ++i) {
      WheelIO &w = io[i];
      WheelContactOutput o;
      w.force = Vec3(); w.torque = Vec3();
      double omega_abs = w.angvel.dot(w.axle_left.normalized());
      if (terramech_on) {
        WheelContactInput in; in.center = w.center; in.axle_left = w.axle_left; in.vel = w.vel; in.omega = omega_abs;
        in.yaw_rate = w.angvel.z;
        o = contacts[i].step(in, dtm, mo, co, dt);
        if (o.contact) { w.force = o.force; w.torque = o.torque; }
      }
      RigidOut ro_out;
      if (!boxes.empty()) {
        ro_out = rcontacts[i].step(w.center, w.vel, w.angvel, w.axle_left, wheels[i].r_collision, wheels[i].geom.b, boxes, ro, dt);
        if (ro_out.contact) { w.force += ro_out.force; w.torque += ro_out.torque; }
      }
      double tau = 0;
      if (drive_on) tau = pids[i].update(wcmd - w.joint_vel, dt);
      w.motor_torque = tau;
      // logging quantities
      double H = dtm.height(w.center.x, w.center.y);
      double on_rigid = ro_out.contact ? 1.0 : ((o.soil_w < 0.5 && (w.center.z - H) < wheels[i].r_collision + 0.01) ? 1.0 : 0.0);
      Vec3 hub = w.has_hub ? w.hub_force : (w.force + Vec3(0, 0, -wheels[i].mass * gravity));
      Vec3 px = w.parent_x.normalized();
      Vec3 fwd = Vec3(px.x, px.y, 0).normalized();
      last[i] = {w.center.x, w.center.y, w.center.z, w.joint_vel, wcmd, o.s, o.beta, o.z, o.soil_w, on_rigid,
                 o.contact ? o.FN_applied : 0.0, o.contact ? o.FDP_applied : 0.0, o.contact ? o.f.FS * o.ramp : 0.0,
                 o.contact ? o.f.MR * o.ramp : 0.0, o.force.x, o.force.y, o.force.z, hub.x, hub.y, hub.z,
                 hub.dot(px), hub.dot(fwd), tau, ro_out.fn, ro_out.ft};
    }
    if (diff_on && diffL && diffR) diff_torque = -diff_k * (diffL->pos + diffR->pos) - diff_c * (diffL->vel + diffR->vel);
    double bv = 0;
    if (bench_on && bench) {
      // target carriage speed for desired slip, using wheel 0 speed
      double s = slip_sched.at(t);
      double om = io.empty() ? 0 : io[0].joint_vel;
      double rs = wheels.empty() ? 0.14 : wheels[0].geom.rs();
      double vt = s >= 0 ? om * rs * (1.0 - s) : om * rs / (1.0 + s);
      bv = bench->vel;
      bench_cmd_force = bench_pid.update(vt - bv, dt);
      bench_force = bench_cmd_force;
    }
    if (t >= next_log - 1e-9) {
      std::vector<double> row = {t, chassis[0], chassis[1], chassis[2], chassis[3], chassis[4], chassis[5], bench_force, bv};
      for (auto &v : last) row.insert(row.end(), v.begin(), v.end());
      log.row(row);
      next_log = (next_log < -1e8 ? t : next_log) + 1.0 / log_rate;
    }
    if (!done_written && t >= t_end) {
      log.flush();
      std::ofstream f(done_file); f << t << "\n";
      done_written = true;
    }
  }
};

}  // namespace terramech
