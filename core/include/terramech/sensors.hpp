// sensors.hpp — what both robot adapters (omni_logic: Robotino star, vehicle_logic: sandbox) need besides
// physics: the IMU at the centre of mass (with its noise-free reference), rigid-body kinematics of the chassis
// (body rates, specific force, quaternion), wheel odometry (ideal = exact joint speeds, encoder = counted
// ticks) and the full per-run log (one row = everything known about the robot at that time).
#pragma once
#include <array>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include "terramech/omni.hpp"

namespace terramech {

// ------------------------------------------------------------------------------- IMU at the CoM
// Gyro and accelerometer (specific force) in the body frame, sampled at `rate` (true signal averaged over
// the sample), + white noise (sigma per sample) + constant bias (drawn once, sigma *_bias) + bias random
// walk (*_rw per sqrt(s)). gyro_true / acc_true = the same sample without noise and bias (ideal IMU).
// Deterministic for a given seed (own xorshift + Box–Muller).
struct ImuParams { double rate = 100, g_sigma = 0.002, g_bias = 0.005, g_rw = 2e-5, a_sigma = 0.03, a_bias = 0.1, a_rw = 2e-4; int seed = 11; };   // consumer MEMS
class Imu {
 public:
  ImuParams p; Vec3 bg, ba, gyro, acc, gyro_true, acc_true; bool fresh = false;
  void init(const ImuParams &q) { p = q; st = 0x2545F4914F6CDD1DULL ^ (uint64_t)(q.seed * 7919 + 1); bg = nvec(p.g_bias); ba = nvec(p.a_bias); }
  void accumulate(double t, double dt, const Vec3 &w_body, const Vec3 &f_body) {
    sw += w_body * dt; sf += f_body * dt; acc_t += dt; fresh = false;
    if (t + 1e-12 < next) return;
    const double Ts = 1.0 / p.rate;
    if (acc_t > 0) {
      gyro_true = sw * (1.0 / acc_t); acc_true = sf * (1.0 / acc_t);
      gyro = gyro_true + bg + nvec(p.g_sigma); acc = acc_true + ba + nvec(p.a_sigma);
      bg += nvec(p.g_rw * std::sqrt(Ts)); ba += nvec(p.a_rw * std::sqrt(Ts));
      fresh = true;
    }
    sw = Vec3(); sf = Vec3(); acc_t = 0; next = (next < 0 ? t : next) + Ts;
  }
 private:
  uint64_t st = 1; Vec3 sw, sf; double acc_t = 0, next = -1;
  double u01() { st ^= st << 13; st ^= st >> 7; st ^= st << 17; return ((st >> 11) + 0.5) * (1.0 / 9007199254740992.0); }
  double nrm() { return std::sqrt(-2.0 * std::log(u01())) * std::cos(2 * kPi * u01()); }
  Vec3 nvec(double s) { return Vec3(s * nrm(), s * nrm(), s * nrm()); }
};

inline ImuParams imu_from_section(const Config::Section *IM) {
  ImuParams ip; ip.rate = Config::num(IM, "rate", ip.rate);
  ip.g_sigma = Config::num(IM, "gyro_sigma", ip.g_sigma); ip.g_bias = Config::num(IM, "gyro_bias", ip.g_bias); ip.g_rw = Config::num(IM, "gyro_rw", ip.g_rw);
  ip.a_sigma = Config::num(IM, "acc_sigma", ip.a_sigma); ip.a_bias = Config::num(IM, "acc_bias", ip.a_bias); ip.a_rw = Config::num(IM, "acc_rw", ip.a_rw);
  ip.seed = (int)Config::num(IM, "seed", ip.seed);
  return ip;
}

// ------------------------------------------------------------------------------- chassis kinematics
// From the chassis pose (x, y, z, roll, pitch, yaw) and its world velocities (linear of the link origin,
// angular) -> body rates, body velocity, specific force at the CoM (finite difference of the CoM velocity),
// quaternion. Without engine velocities they are differentiated from the poses.
struct BodyState {
  double pose[6] = {0, 0, 0, 0, 0, 0}, vel[6] = {0, 0, 0, 0, 0, 0};   // world
  Vec3 w_body, v_body, f_body;                                        // body frame
  double quat[4] = {1, 0, 0, 0};                                      // w, x, y, z
  double R[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};
class ChassisKinematics {
 public:
  Vec3 com;                       // CoM in the chassis frame
  bool have_vel = false; double vel_in[6] = {0, 0, 0, 0, 0, 0};
  static double wrap(double a) { while (a > kPi) a -= 2 * kPi; while (a < -kPi) a += 2 * kPi; return a; }
  BodyState update(const double ch[6], double dt) {
    BodyState b; for (int k = 0; k < 6; ++k) b.pose[k] = ch[k];
    if (have_vel) for (int k = 0; k < 6; ++k) b.vel[k] = vel_in[k];
    else if (have_prev && dt > 0) {
      for (int k = 0; k < 3; ++k) b.vel[k] = (ch[k] - prev[k]) / dt;
      b.vel[5] = wrap(ch[5] - prev[5]) / dt;      // roll/pitch rates neglected without engine velocities
    }
    for (int k = 0; k < 6; ++k) prev[k] = ch[k];
    have_prev = true;
    const double cr = std::cos(ch[3]), sr = std::sin(ch[3]), cp = std::cos(ch[4]), sp = std::sin(ch[4]), cy = std::cos(ch[5]), sy = std::sin(ch[5]);
    const double R[3][3] = {{cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr}, {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr}, {-sp, cp * sr, cp * cr}};
    for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) b.R[i][j] = R[i][j];
    auto toBody = [&](const Vec3 &v) { return Vec3(R[0][0] * v.x + R[1][0] * v.y + R[2][0] * v.z, R[0][1] * v.x + R[1][1] * v.y + R[2][1] * v.z,
                                                   R[0][2] * v.x + R[1][2] * v.y + R[2][2] * v.z); };
    const Vec3 w(b.vel[3], b.vel[4], b.vel[5]);
    const Vec3 rcom(R[0][0] * com.x + R[0][1] * com.y + R[0][2] * com.z, R[1][0] * com.x + R[1][1] * com.y + R[1][2] * com.z,
                    R[2][0] * com.x + R[2][1] * com.y + R[2][2] * com.z);
    const Vec3 vbase(b.vel[0], b.vel[1], b.vel[2]);
    const Vec3 vcom = vbase + w.cross(rcom);
    Vec3 a(0, 0, 0);
    if (have_vprev && dt > 0) a = (vcom - vcom_prev) * (1.0 / dt);
    vcom_prev = vcom; have_vprev = true;
    b.w_body = toBody(w); b.v_body = toBody(vbase); b.f_body = toBody(a + Vec3(0, 0, 9.81));
    // quaternion (ZYX)
    const double hr = 0.5 * ch[3], hp = 0.5 * ch[4], hy = 0.5 * ch[5];
    b.quat[0] = std::cos(hr) * std::cos(hp) * std::cos(hy) + std::sin(hr) * std::sin(hp) * std::sin(hy);
    b.quat[1] = std::sin(hr) * std::cos(hp) * std::cos(hy) - std::cos(hr) * std::sin(hp) * std::sin(hy);
    b.quat[2] = std::cos(hr) * std::sin(hp) * std::cos(hy) + std::sin(hr) * std::cos(hp) * std::sin(hy);
    b.quat[3] = std::cos(hr) * std::cos(hp) * std::sin(hy) - std::sin(hr) * std::sin(hp) * std::cos(hy);
    return b;
  }
 private:
  double prev[6] = {0, 0, 0, 0, 0, 0}; bool have_prev = false;
  Vec3 vcom_prev; bool have_vprev = false;
};

// ------------------------------------------------------------------------------- wheel odometry
// Rows J_i: w_i = J_i . (vx, vy, wz). Body twist from wheel speeds by least squares (J^T J)^-1 J^T, rows of
// passive wheels (casters) are zero and drop out. yaw_scale: skid steer (1 / wheel_separation_multiplier).
class Odometer {
 public:
  std::vector<std::array<double, 3>> J;
  double Jp[3][16] = {{0}};
  double yaw_scale = 1.0;
  void init(const std::vector<std::array<double, 3>> &rows, double ys = 1.0) {
    J = rows; yaw_scale = ys;
    double A[3][3] = {{0}};
    for (auto &r : J) for (int a = 0; a < 3; ++a) for (int b = 0; b < 3; ++b) A[a][b] += r[a] * r[b];
    for (int a = 0; a < 3; ++a) A[a][a] += 1e-9;
    double I[3][3]; invert3(A, I);
    for (size_t i = 0; i < J.size() && i < 16; ++i)
      for (int a = 0; a < 3; ++a) { double v = 0; for (int b = 0; b < 3; ++b) v += I[a][b] * J[i][b]; Jp[a][i] = v; }
  }
  void twist(const std::vector<double> &w, double tw[3]) const {
    tw[0] = tw[1] = tw[2] = 0;
    for (size_t i = 0; i < w.size() && i < J.size(); ++i) for (int a = 0; a < 3; ++a) tw[a] += Jp[a][i] * w[i];
    tw[2] *= yaw_scale;
  }
  // midpoint integration of a body twist; yaw_rate replaces tw[2] for the heading (gyro fusion)
  static void integrate(double p[3], const double tw[3], double yaw_rate, double dt) {
    const double ym = p[2] + 0.5 * yaw_rate * dt;
    p[0] += (std::cos(ym) * tw[0] - std::sin(ym) * tw[1]) * dt;
    p[1] += (std::sin(ym) * tw[0] + std::cos(ym) * tw[1]) * dt;
    p[2] += yaw_rate * dt;
  }
  static void invert3(const double A[3][3], double I[3][3]) {
    const double d = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                     A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    const double id = std::fabs(d) > 1e-30 ? 1.0 / d : 0.0;
    I[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * id; I[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * id; I[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * id;
    I[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * id; I[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * id; I[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * id;
    I[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * id; I[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * id; I[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * id;
  }
};

// ------------------------------------------------------------------------------- full per-run log
// One row per 1/rate s. Column reference: docs/en/OUTPUTS.md (full log). nan = not applicable.
struct FullWheel {
  double angle = 0, omega = 0, omega_set = 0, omega_enc = 0, omega_eff = 0;   // rad, rad/s (wheel)
  double cur = 0, volt = 0, power = 0, tau_motor = 0, tau_contact = 0;          // A (signed), V, W, N m
  double N = 0, F_trac = 0, F_free = 0, Fx = 0, Fy = 0, Fz = 0;                  // N (Fx, Fy, Fz world)
  double slip = 0, slip_lat = 0, slip_kin = 0, sink = 0;                        // -, -, -, m
  double cx = 0, cy = 0, mu = 0, K = 0, f_roll = 0, f_lat = 0;                    // contact point (world), local surface
  double mu_rb = 0, kc = 0, kphi = 0;                                             // local surface (roller friction, Bekker moduli)
  std::string surf;
};
struct FullFrame {
  double t = 0; int phase = 0, run = 0; double dir = NAN;
  BodyState b;
  double ref[3] = {NAN, NAN, NAN}, cmd[3] = {0, 0, 0};
  double odo_ideal[3] = {0, 0, 0}, odo_enc[3] = {0, 0, 0}, odo_imu[3] = {0, 0, 0};
  Vec3 imu_g, imu_a, imu_g_true, imu_a_true;
  double power = 0, energy = 0;
  std::string surf;
};
class FullLog {
 public:
  std::unique_ptr<std::ofstream> f; size_t nw = 0; double rate = 50, next = -1;
  bool open(const std::string &path, size_t n_wheels, const std::vector<std::string> &labels) {
    nw = n_wheels; f.reset(new std::ofstream(path));
    if (!*f) return false;
    *f << "t,phase,run,dir,x_true,y_true,z_true,roll_true,pitch_true,yaw_true,qw_true,qx_true,qy_true,qz_true,"
          "vx_true,vy_true,vz_true,vx_body,vy_body,vz_body,wx_true,wy_true,wz_true,"
          "imu_ax,imu_ay,imu_az,imu_gx,imu_gy,imu_gz,imu_ax_true,imu_ay_true,imu_az_true,imu_gx_true,imu_gy_true,imu_gz_true,"
          "odo_ideal_x,odo_ideal_y,odo_ideal_yaw,odo_enc_x,odo_enc_y,odo_enc_yaw,odo_imu_x,odo_imu_y,odo_imu_yaw,"
          "ref_x,ref_y,ref_yaw,cmd_vx,cmd_vy,cmd_wz,power_W,energy_J,surf";
    for (size_t i = 0; i < nw; ++i) {
      const std::string p = "w" + (i < labels.size() && !labels[i].empty() ? labels[i] : std::to_string(i + 1)) + "_";
      for (const char *k : {"angle", "omega", "omega_set", "omega_enc", "omega_eff", "cur", "volt", "power", "tau_motor", "tau_contact",
                            "N", "F_trac", "F_free", "Fx", "Fy", "Fz", "slip", "slip_lat", "slip_kin", "sink", "cx", "cy",
                            "mu", "K", "f_roll", "f_lat", "mu_rb", "kc", "kphi", "surf"}) *f << "," << p << k;
    }
    *f << "\n";
    return true;
  }
  bool due(double t) const { return f && t + 1e-12 >= next; }
  void write(const FullFrame &F, const std::vector<FullWheel> &W) {
    if (!f) return;
    next = (next < 0 ? F.t : next) + 1.0 / rate;
    char b[48];
    auto put = [&](double v) { if (std::isfinite(v)) { std::snprintf(b, sizeof b, ",%.7g", v); *f << b; } else *f << ",nan"; };
    std::snprintf(b, sizeof b, "%.4f", F.t); *f << b << "," << F.phase << "," << F.run;
    put(F.dir);
    for (int k = 0; k < 6; ++k) put(F.b.pose[k]);
    for (int k = 0; k < 4; ++k) put(F.b.quat[k]);
    for (int k = 0; k < 3; ++k) put(F.b.vel[k]);
    for (double v : {F.b.v_body.x, F.b.v_body.y, F.b.v_body.z, F.b.w_body.x, F.b.w_body.y, F.b.w_body.z,
                     F.imu_a.x, F.imu_a.y, F.imu_a.z, F.imu_g.x, F.imu_g.y, F.imu_g.z,
                     F.imu_a_true.x, F.imu_a_true.y, F.imu_a_true.z, F.imu_g_true.x, F.imu_g_true.y, F.imu_g_true.z}) put(v);
    for (int k = 0; k < 3; ++k) put(F.odo_ideal[k]);
    for (int k = 0; k < 3; ++k) put(F.odo_enc[k]);
    for (int k = 0; k < 3; ++k) put(F.odo_imu[k]);
    for (int k = 0; k < 3; ++k) put(F.ref[k]);
    for (int k = 0; k < 3; ++k) put(F.cmd[k]);
    put(F.power); put(F.energy);
    *f << "," << F.surf;
    for (size_t i = 0; i < nw && i < W.size(); ++i) {
      const auto &w = W[i];
      for (double v : {w.angle, w.omega, w.omega_set, w.omega_enc, w.omega_eff, w.cur, w.volt, w.power, w.tau_motor, w.tau_contact,
                       w.N, w.F_trac, w.F_free, w.Fx, w.Fy, w.Fz, w.slip, w.slip_lat, w.slip_kin, w.sink, w.cx, w.cy,
                       w.mu, w.K, w.f_roll, w.f_lat, w.mu_rb, w.kc, w.kphi}) put(v);
      *f << "," << w.surf;
    }
    *f << "\n";
  }
  void flush() { if (f) f->flush(); }
};

// dataset-style kinematic slip of a wheel: 1 - w_eff / w (0 when the wheel does not turn)
inline double kinematic_slip(double w_eff, double w, double eps = 1e-3) { return std::fabs(w) > eps ? 1.0 - w_eff / w : 0.0; }

// default path of the full log next to the main log: log.csv -> log_full.csv
inline std::string full_log_path(const std::string &log_file) {
  const size_t p = log_file.find_last_of('.');
  return (p == std::string::npos ? log_file : log_file.substr(0, p)) + "_full.csv";
}

}  // namespace terramech
