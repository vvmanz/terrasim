// terramech.hpp — header-only wheel–terrain interaction core (MarsSim-style).
//
// Shared by: the gz-sim 8 (Gazebo Harmonic) system plugin, the offline 3D core (vehicle3d, robotino3d),
// C API (libterramech_c.so, used from Python via ctypes) and the planar surrogate.
//
// Models implemented
//   * CLOSED_FORM  — decoupled closed-form model used in MarsSim (Zhou et al. 2023, eq. 9-15),
//                    i.e. Ding's linearised Bekker–Wong–Janosi stress distribution.
//                    Misprints of the paper are fixed here:
//                      E = 1 - exp(-j_m/K) (undefined in the paper);
//                      j_m = r_s[(th1' - thm) - (1-s)(sin th1' - sin thm)];
//                      c is in Pa (not kPa); c_p form selectable.
//   * WONG_REECE   — classical Bekker pressure–sinkage + Wong–Reece stress distribution
//                    + Janosi–Hanamoto shear, integrated numerically over the contact arc.
//
// Units: SI everywhere (m, N, Pa, rad, s).  kc [Pa/m^(n-1)], kphi [Pa/m^n].
#pragma once
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace terramech {

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double sgn(double v) { return (v > 0) - (v < 0); }

// ----------------------------------------------------------------------------- math
struct Vec3 {
  double x = 0, y = 0, z = 0;
  Vec3() = default;
  Vec3(double a, double b, double c) : x(a), y(b), z(c) {}
  Vec3 operator+(const Vec3 &o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3 &o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator*(double k) const { return {x * k, y * k, z * k}; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  Vec3 &operator+=(const Vec3 &o) { x += o.x; y += o.y; z += o.z; return *this; }
  double dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }
  Vec3 cross(const Vec3 &o) const { return {y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x}; }
  double norm() const { return std::sqrt(dot(*this)); }
  Vec3 normalized() const { double n = norm(); return n > 1e-15 ? (*this) * (1.0 / n) : Vec3(0, 0, 0); }
};
inline Vec3 operator*(double k, const Vec3 &v) { return v * k; }

// ----------------------------------------------------------------------------- params
struct Soil {
  double kc = 13.6e3;       // Pa/m^(n-1)
  double kphi = 2259.1e3;   // Pa/m^n
  double n0 = 0.92;         // static sinkage exponent
  double n1 = 0.5;          // dynamic sinkage exponent (N = n0 + n1*s)
  double c = 462.3;         // cohesion, Pa
  double phi = deg2rad(35.0);  // internal friction angle, rad
  double K = 0.015;         // shear deformation modulus, m
  double Ky = 0.015;        // lateral shear deformation modulus, m
  double a0 = 0.40;         // Wong–Reece: theta_m = (a0 + a1*s)*theta1
  double a1 = 0.15;
  double soil = 1.0;        // 1 = deformable soil, 0 = rigid (handled by physics engine contact)

  Soil scaled(double w) const {
    Soil o; o.kc = kc * w; o.kphi = kphi * w; o.n0 = n0 * w; o.n1 = n1 * w; o.c = c * w; o.phi = phi * w;
    o.K = K * w; o.Ky = Ky * w; o.a0 = a0 * w; o.a1 = a1 * w; o.soil = soil * w; return o;
  }
  Soil &accum(const Soil &s) {
    kc += s.kc; kphi += s.kphi; n0 += s.n0; n1 += s.n1; c += s.c; phi += s.phi; K += s.K; Ky += s.Ky;
    a0 += s.a0; a1 += s.a1; soil += s.soil; return *this;
  }
  static Soil zero() { Soil z; z = z.scaled(0.0); return z; }
};

// MarsSim Table I (single-wheel testbed soil) and Table II (rover soft soil); c converted to Pa.
inline Soil soil_marssim_table1() { Soil s; s.kc = 15.6e3; s.kphi = 2407.6e3; s.n0 = 0.895; s.n1 = 0.995; s.c = 250.0; s.phi = deg2rad(31.9); s.K = 0.01; s.Ky = 0.01; return s; }
inline Soil soil_marssim_table2() { Soil s; s.kc = 13.6e3; s.kphi = 2259.1e3; s.n0 = 0.92; s.n1 = 0.5; s.c = 462.3; s.phi = deg2rad(35.0); s.K = 0.015; s.Ky = 0.015; return s; }

struct WheelGeom {
  double r = 0.14;      // wheel (rim) radius used by the soil model, m
  double b = 0.20;      // width, m
  double h = 0.01;      // lug height, m
  double lambda = 0.0;  // r_s = r + lambda*h  (shearing radius)
  double Rj = -1.0;     // equivalent radius for theta1' (<=0 -> r + h)
  double rs() const { return r + lambda * h; }
  double rj() const { return Rj > 0 ? Rj : r + h; }
};

enum ModelType { CLOSED_FORM = 0, WONG_REECE = 1 };

struct ModelOpts {
  int model = CLOSED_FORM;
  // Correction terms of MarsSim eq. (9). cp_form: 0 -> c_p = 1 (default, recommended);
  // 1 -> c_p = 1 + cp1 + cp2*s (as printed); 2 -> c_p = 1 + cp1*s + cp2*s^2.
  int cp_form = 0;
  double cp1 = 0.0, cp2 = 0.0, cp3 = 0.0, cM = 0.0;
  double W_bar = 0.0;      // average normal force for correction terms; <= 0 -> use F_N (terms vanish)
  int n_integ = 64;        // Wong–Reece integration points
  double max_sink_frac = 0.9;  // clamp z to this fraction of r
  // Slip used in the slip-sinkage exponent N = n0 + n1*clamp(s, sN_lo, sN_hi). The paper uses the
  // raw s (validated only for -0.03 <= s <= 0.5). For skid (s<0) the raw formula makes the support
  // force explode (N -> n0-n1), so by default skid uses the static exponent n0.
  double sN_lo = 0.0, sN_hi = 0.9;
};

struct Forces {
  bool contact = false;
  double FN = 0, FDP = 0, FS = 0, MR = 0, MO = 0, MS = 0;
  double theta1 = 0, thetam = 0, sigma_m = 0, tau_m = 0, jm = 0, N = 0;
};

// Unified slip ratio (MarsSim eq. 6): s = 1 - vx/(w rs) when slipping, w rs/vx - 1 when skidding.
inline double slip_ratio(double vx, double omega, double rs, double v_eps = 1e-4) {
  double vw = omega * rs;
  double den = std::max(std::max(std::fabs(vw), std::fabs(vx)), v_eps);
  return clamp((vw - vx) / den, -1.0, 1.0);
}
inline double side_slip(double vx, double vy, double v_eps = 1e-4) {
  if (std::fabs(vx) + std::fabs(vy) < v_eps) return 0.0;
  return std::atan2(vy, std::fabs(vx));
}

inline double signed_shear(double j, double K) {  // 1 - exp(-j/K), odd-symmetric (skid safe)
  return sgn(j) * (1.0 - std::exp(-std::fabs(j) / std::max(K, 1e-9)));
}

// ----------------------------------------------------------------------------- closed form (MarsSim)
// Inputs: slip s (forward rolling), sinkage z (>0), side-slip angle beta (rad).
inline Forces closed_form(const WheelGeom &w, const Soil &so, const ModelOpts &m, double s, double z, double beta) {
  Forces R;
  if (z <= 1e-7 || so.soil <= 1e-6) return R;
  s = clamp(s, -0.99, 0.99);
  const double r = w.r, b = w.b, rs = w.rs(), Rj = w.rj();
  z = std::min(z, m.max_sink_frac * r);
  const double th1 = std::acos(clamp(1.0 - z / r, -1.0, 1.0));
  const double th2 = 0.0, thm = 0.5 * (th1 + th2);
  if (th1 < 1e-5) return R;
  const double A = (std::cos(thm) - std::cos(th2)) / (thm - th2) + (std::cos(thm) - std::cos(th1)) / (th1 - thm);
  const double B = (std::sin(thm) - std::sin(th2)) / (thm - th2) + (std::sin(thm) - std::sin(th1)) / (th1 - thm);
  const double C = 0.5 * (th1 - th2);
  const double N = std::max(0.05, so.n0 + so.n1 * clamp(s, m.sN_lo, m.sN_hi));
  const double sigma_m = (so.kc / b + so.kphi) * std::pow(std::max(r * (std::cos(thm) - std::cos(th1)), 0.0), N);
  const double th1p = std::acos(clamp((r - z) / Rj, -1.0, 1.0));
  const double jm = rs * ((th1p - thm) - (1.0 - s) * (std::sin(th1p) - std::sin(thm)));
  const double E = signed_shear(jm, so.K);
  const double tp = std::tan(so.phi);
  const double tau_m = (so.c + sigma_m * tp) * E;
  const double FN = r * b * A * sigma_m + rs * b * B * tau_m;
  const double Wb = m.W_bar > 0 ? m.W_bar : FN;
  const double dW = Wb > 1e-9 ? (Wb - FN) / Wb : 0.0;
  const double MR = rs * rs * C * E * (b * so.c + (1.0 + m.cM * dW) * FN * tp / (r * A)) /
                    (1.0 + rs * B * E * tp / (r * A));
  double cp = 1.0;
  if (m.cp_form == 1) cp = 1.0 + m.cp1 + m.cp2 * s;
  else if (m.cp_form == 2) cp = 1.0 + m.cp1 * s + m.cp2 * s * s;
  const double FDP = (MR * (A * A + B * B) / (rs * A * C) - B * FN / A) * cp * (1.0 + m.cp3 * dW);
  // lateral force opposes side slip (beta = atan2(vy,|vx|))
  const double FSmag = rs * b * std::fabs(tau_m) * C *
                       (1.0 - std::exp(-r * std::max(1.0 - s, 0.0) * (th1 - thm) * std::fabs(std::tan(beta)) / std::max(so.Ky, 1e-9)));
  R.contact = true; R.FN = FN; R.FDP = FDP; R.MR = MR;
  R.FS = -sgn(beta) * FSmag;
  R.MO = R.FS * r;
  R.MS = std::fabs(R.FS) * r * std::sin(thm);  // magnitude; sign applied by caller (opposes yaw rate)
  R.theta1 = th1; R.thetam = thm; R.sigma_m = sigma_m; R.tau_m = tau_m; R.jm = jm; R.N = N;
  return R;
}

// ----------------------------------------------------------------------------- Wong–Reece integral
inline Forces wong_reece(const WheelGeom &w, const Soil &so, const ModelOpts &m, double s, double z, double beta) {
  Forces R;
  if (z <= 1e-7 || so.soil <= 1e-6) return R;
  s = clamp(s, -0.99, 0.99);
  const double r = w.r, b = w.b, rs = w.rs();
  z = std::min(z, m.max_sink_frac * r);
  const double th1 = std::acos(clamp(1.0 - z / r, -1.0, 1.0));
  if (th1 < 1e-5) return R;
  const double th2 = 0.0;
  const double thm = clamp(so.a0 + so.a1 * std::fabs(s), 0.05, 0.95) * th1;
  const double N = std::max(0.05, so.n0 + so.n1 * clamp(s, m.sN_lo, m.sN_hi));
  const double kk = (so.kc / b + so.kphi) * std::pow(r, N);
  const double tp = std::tan(so.phi), tb = std::tan(beta);
  const int n = std::max(8, m.n_integ);
  const double dth = (th1 - th2) / n;
  double FN = 0, FDP = 0, MR = 0, FSm = 0, smax = 0, tmax = 0;
  for (int i = 0; i < n; ++i) {
    const double th = th2 + (i + 0.5) * dth;
    double sig;
    if (th >= thm) sig = kk * std::pow(std::max(std::cos(th) - std::cos(th1), 0.0), N);
    else {
      const double the = th1 - (th - th2) / (thm - th2) * (th1 - thm);
      sig = kk * std::pow(std::max(std::cos(the) - std::cos(th1), 0.0), N);
    }
    const double j = rs * ((th1 - th) - (1.0 - s) * (std::sin(th1) - std::sin(th)));
    const double tau = (so.c + sig * tp) * signed_shear(j, so.K);
    const double jy = r * std::max(1.0 - s, 0.0) * (th1 - th) * std::fabs(tb);
    const double tauy = (so.c + sig * tp) * (1.0 - std::exp(-jy / std::max(so.Ky, 1e-9)));
    FN += (sig * std::cos(th) + tau * std::sin(th)) * dth;
    FDP += (tau * std::cos(th) - sig * std::sin(th)) * dth;
    MR += tau * dth;
    FSm += tauy * dth;
    smax = std::max(smax, sig); tmax = std::max(tmax, std::fabs(tau));
  }
  R.contact = true;
  R.FN = r * b * FN; R.FDP = r * b * FDP; R.MR = rs * rs * b * MR;
  R.FS = -sgn(beta) * r * b * FSm;
  R.MO = R.FS * r; R.MS = std::fabs(R.FS) * r * std::sin(thm);
  R.theta1 = th1; R.thetam = thm; R.sigma_m = smax; R.tau_m = tmax; R.N = N;
  return R;
}

inline Forces wheel_forces(const WheelGeom &w, const Soil &so, const ModelOpts &m, double s, double z, double beta) {
  Forces f = (m.model == WONG_REECE) ? wong_reece(w, so, m, s, z, beta) : closed_form(w, so, m, s, z, beta);
  if (so.soil < 1.0 && f.contact) {  // partial soil (soil/rigid boundary): scale forces
    const double k = clamp(so.soil, 0.0, 1.0);
    f.FN *= k; f.FDP *= k; f.FS *= k; f.MR *= k; f.MO *= k; f.MS *= k;
  }
  return f;
}

// Static sinkage: F_N(z, s) = W  (bisection, monotone in z)
inline double solve_sinkage(const WheelGeom &w, const Soil &so, const ModelOpts &m, double s, double W, double beta = 0.0) {
  double lo = 1e-7, hi = m.max_sink_frac * w.r;
  if (wheel_forces(w, so, m, s, hi, beta).FN < W) return hi;
  for (int i = 0; i < 80; ++i) {
    double mid = 0.5 * (lo + hi);
    if (wheel_forces(w, so, m, s, mid, beta).FN < W) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);
}

// ----------------------------------------------------------------------------- INI config
// [section] or [section label]; key = value; '#' or ';' comments.
class Config {
 public:
  struct Section { std::string name, label; std::map<std::string, std::string> kv; };
  std::vector<Section> sections;
  std::string base_dir;

  static std::string trim(const std::string &s) {
    size_t a = s.find_first_not_of(" \t\r\n"), b = s.find_last_not_of(" \t\r\n");
    return a == std::string::npos ? "" : s.substr(a, b - a + 1);
  }
  bool load(const std::string &path) {
    std::ifstream f(path);
    if (!f) return false;
    size_t p = path.find_last_of('/');
    base_dir = (p == std::string::npos) ? "." : path.substr(0, p);
    sections.clear();
    sections.push_back({"", "", {}});
    std::string line;
    while (std::getline(f, line)) {
      size_t c = line.find_first_of("#;");
      if (c != std::string::npos) line = line.substr(0, c);
      line = trim(line);
      if (line.empty()) continue;
      if (line.front() == '[' && line.back() == ']') {
        std::string inner = trim(line.substr(1, line.size() - 2));
        size_t sp = inner.find(' ');
        Section s;
        s.name = sp == std::string::npos ? inner : inner.substr(0, sp);
        s.label = sp == std::string::npos ? "" : trim(inner.substr(sp + 1));
        sections.push_back(s);
        continue;
      }
      size_t eq = line.find('=');
      if (eq == std::string::npos) continue;
      sections.back().kv[trim(line.substr(0, eq))] = trim(line.substr(eq + 1));
    }
    return true;
  }
  const Section *find(const std::string &name, const std::string &label = "") const {
    for (auto &s : sections) if (s.name == name && (label.empty() || s.label == label)) return &s;
    return nullptr;
  }
  std::vector<const Section *> all(const std::string &name) const {
    std::vector<const Section *> v;
    for (auto &s : sections) if (s.name == name) v.push_back(&s);
    return v;
  }
  static bool has(const Section *s, const std::string &k) { return s && s->kv.count(k); }
  static std::string str(const Section *s, const std::string &k, const std::string &d = "") {
    if (!s) return d;
    auto it = s->kv.find(k);
    return it == s->kv.end() ? d : it->second;
  }
  static double num(const Section *s, const std::string &k, double d) {
    std::string v = str(s, k, "");
    if (v.empty()) return d;
    try { return std::stod(v); } catch (...) { return d; }
  }
  std::string path(const std::string &p) const {
    if (p.empty() || p[0] == '/') return p;
    return base_dir + "/" + p;
  }
  static std::vector<double> list(const Section *s, const std::string &k) {
    std::vector<double> out; std::string v = str(s, k, "");
    for (char &ch : v) if (ch == ',' || ch == ':' || ch == ';') ch = ' ';
    std::stringstream ss(v); double d; while (ss >> d) out.push_back(d);
    return out;
  }
};

inline Soil soil_from_section(const Config::Section *s, Soil base = Soil()) {
  if (!s) return base;
  const std::string preset = Config::str(s, "preset", "");
  if (preset == "marssim_table1") base = soil_marssim_table1();
  if (preset == "marssim_table2") base = soil_marssim_table2();
  base.kc = Config::num(s, "kc", base.kc);
  base.kphi = Config::num(s, "kphi", base.kphi);
  base.n0 = Config::num(s, "n0", base.n0);
  base.n1 = Config::num(s, "n1", base.n1);
  base.c = Config::num(s, "c", base.c);
  if (Config::has(s, "phi_deg")) base.phi = deg2rad(Config::num(s, "phi_deg", 35));
  base.K = Config::num(s, "K", base.K);
  base.Ky = Config::num(s, "Ky", Config::has(s, "K") ? base.K : base.Ky);
  base.a0 = Config::num(s, "a0", base.a0);
  base.a1 = Config::num(s, "a1", base.a1);
  base.soil = Config::num(s, "soil", base.soil);
  return base;
}
inline ModelOpts model_from_section(const Config::Section *s, ModelOpts m = ModelOpts()) {
  if (!s) return m;
  std::string t = Config::str(s, "model", "closed_form");
  m.model = (t == "wong_reece" || t == "bekker_wong") ? WONG_REECE : CLOSED_FORM;
  m.cp_form = (int)Config::num(s, "cp_form", m.cp_form);
  m.cp1 = Config::num(s, "cp1", m.cp1); m.cp2 = Config::num(s, "cp2", m.cp2);
  m.cp3 = Config::num(s, "cp3", m.cp3); m.cM = Config::num(s, "cM", m.cM);
  m.W_bar = Config::num(s, "W_bar", m.W_bar);
  m.n_integ = (int)Config::num(s, "n_integ", m.n_integ);
  m.sN_lo = Config::num(s, "sN_lo", m.sN_lo);
  m.sN_hi = Config::num(s, "sN_hi", m.sN_hi);
  return m;
}

// ----------------------------------------------------------------------------- DTM
// Grid terrain: elevation H and terrain-type index per node; terrain types carry Soil params.
// File format (text):
//   terramech_dtm 1
//   nx ny x0 y0 res
//   ntypes
//   <name> kc kphi n0 n1 c phi_deg K Ky soil      (ntypes lines; soil=1 deformable, 0 rigid)
//   H: ny lines of nx floats (row j = y0 + j*res, col i = x0 + i*res)
//   T: ny lines of nx ints
class DTM {
 public:
  int nx = 0, ny = 0;
  double x0 = 0, y0 = 0, res = 0.1;
  std::vector<std::string> type_names;
  std::vector<Soil> types;
  std::vector<double> H;
  std::vector<int> T;
  double default_h = 0.0;

  bool load(const std::string &file, std::string *err = nullptr) {
    std::ifstream f(file);
    if (!f) { if (err) *err = "cannot open " + file; return false; }
    std::string magic; int ver;
    f >> magic >> ver;
    if (magic != "terramech_dtm") { if (err) *err = "bad magic in " + file; return false; }
    int nt;
    f >> nx >> ny >> x0 >> y0 >> res >> nt;
    types.clear(); type_names.clear();
    for (int k = 0; k < nt; ++k) {
      std::string name; Soil s; double phid;
      f >> name >> s.kc >> s.kphi >> s.n0 >> s.n1 >> s.c >> phid >> s.K >> s.Ky >> s.soil;
      s.phi = deg2rad(phid);
      types.push_back(s); type_names.push_back(name);
    }
    H.assign((size_t)nx * ny, 0.0); T.assign((size_t)nx * ny, 0);
    for (auto &h : H) f >> h;
    for (auto &t : T) f >> t;
    if (!f) { if (err) *err = "truncated DTM " + file; return false; }
    return true;
  }
  // override soil parameters of a type (e.g. from config) by name
  void set_type(const std::string &name, const Soil &s) {
    for (size_t k = 0; k < types.size(); ++k) if (type_names[k] == name) types[k] = s;
  }
  double h_at(int i, int j) const { i = std::max(0, std::min(nx - 1, i)); j = std::max(0, std::min(ny - 1, j)); return H[(size_t)j * nx + i]; }
  const Soil &t_at(int i, int j) const { i = std::max(0, std::min(nx - 1, i)); j = std::max(0, std::min(ny - 1, j)); return types[(size_t)T[(size_t)j * nx + i]]; }
  void cell(double x, double y, int &i, int &j, double &u, double &v) const {
    double fx = (x - x0) / res, fy = (y - y0) / res;
    i = (int)std::floor(fx); j = (int)std::floor(fy);
    u = clamp(fx - i, 0.0, 1.0); v = clamp(fy - j, 0.0, 1.0);
  }
  double height(double x, double y) const {
    if (nx == 0) return default_h;
    int i, j; double u, v; cell(x, y, i, j, u, v);
    return (1 - u) * (1 - v) * h_at(i, j) + u * (1 - v) * h_at(i + 1, j) + (1 - u) * v * h_at(i, j + 1) + u * v * h_at(i + 1, j + 1);
  }
  // MarsSim eq. (3): bilinear interpolation of parameter vectors. Parameters are averaged over
  // deformable nodes only; the returned .soil is the interpolated deformable fraction (0..1).
  Soil soil(double x, double y) const {
    if (nx == 0 || types.empty()) return Soil();
    int i, j; double u, v; cell(x, y, i, j, u, v);
    std::vector<std::pair<Soil, double>> parts = {
        {t_at(i, j), (1 - u) * (1 - v)}, {t_at(i + 1, j), u * (1 - v)},
        {t_at(i, j + 1), (1 - u) * v}, {t_at(i + 1, j + 1), u * v}};
    return mix(parts);
  }
  static Soil mix(const std::vector<std::pair<Soil, double>> &parts) {
    Soil acc = Soil::zero(); double ws = 0, wt = 0;
    for (auto &p : parts) { double w = p.second * p.first.soil; if (w > 0) { acc.accum(p.first.scaled(w)); ws += w; } wt += p.second; }
    if (ws <= 1e-12) { Soil s = parts.empty() ? Soil() : parts[0].first; s.soil = 0.0; return s; }
    Soil s = acc.scaled(1.0 / ws);
    s.soil = wt > 0 ? ws / wt : 0.0;
    return s;
  }
};

// ----------------------------------------------------------------------------- contact solver
struct WheelContactInput {
  Vec3 center;      // wheel centre, world
  Vec3 axle_left;   // unit axle direction pointing to the wheel's left (+y_w), world
  Vec3 vel;         // linear velocity of wheel centre, world
  double omega = 0; // spin rate about axle_left (positive = forward rolling)
  double yaw_rate = 0;  // angular velocity about the contact normal (for M_S sign)
};

struct WheelContactOutput {
  bool contact = false;
  Vec3 force, torque;   // to apply at wheel centre, world
  Vec3 xi, yi, zi;      // interaction frame
  double s = 0, beta = 0, z = 0, vx = 0, vy = 0, vz = 0, soil_w = 0, ramp = 0;
  Forces f;             // raw model output (before ramp/damping)
  double FN_applied = 0, FDP_applied = 0;
};

struct ContactOpts {
  double v_eps = 1e-3;         // low-speed regularisation (m/s)
  double slip_v_eps = 1e-4;    // slip-ratio denominator floor
  double normal_damping = 0.0; // N s/m along contact normal
  double tangential_damping = 0.0;  // N s/m (stationary friction model helper)
  double theta1_init = 0.1;    // rad (MarsSim: 0.1 initially)
  double slip_tau = 0.05;      // s, used only if slip_relax_len <= 0 (fixed-time filter)
  // Relaxation length (as in tyre models): slip and side-slip respond with a first-order lag
  // tau = slip_relax_len / max(|vx|, |w rs|). Physically: the stress state under the wheel is renewed
  // only after the wheel rolls over its contact length; at 16 mm/s this is ~3 s. Without it the
  // slip-sinkage exponent couples slip to the support force instantaneously and the six-wheel rover
  // develops a roll/stick-slip limit cycle at low speed.
  double slip_relax_len = 0.05;  // m
  double slip_tau_min = 0.02;    // s
  double slip_v_ramp = 2e-3;   // m/s, slip -> 0 when both |vx| and |w rs| are below this (undefined at rest)
  bool use_three_point_plane = true;
  double max_force = 1e5;
};

class WheelContact {
 public:
  WheelGeom geom;
  double theta1 = 0.1;  // carried between steps (MarsSim)
  double s_filt = 0.0;  // filtered slip ratio
  double b_filt = 0.0;  // filtered side-slip angle

  WheelContactOutput step(const WheelContactInput &in, const DTM &dtm, const ModelOpts &mo, const ContactOpts &co, double dt) {
    WheelContactOutput out;
    const double r = geom.r, b = geom.b;
    const Vec3 up(0, 0, 1);
    Vec3 yw = in.axle_left.normalized();
    Vec3 xw = yw.cross(up).normalized();
    if (xw.norm() < 0.5) xw = Vec3(1, 0, 0);  // axle vertical: degenerate
    Vec3 zw = xw.cross(yw).normalized();
    // three points (MarsSim eq. 1) using theta1 of previous step
    double th = clamp(theta1, 0.02, 1.2);
    Vec3 p0 = in.center + xw * (r * std::sin(th)) + yw * (0.5 * b) - zw * (r * std::cos(th));
    Vec3 p1 = in.center + xw * (r * std::sin(th)) - yw * (0.5 * b) - zw * (r * std::cos(th));
    Vec3 p2 = in.center - xw * (r * std::sin(th)) - zw * (r * std::cos(th));
    Vec3 A(p0.x, p0.y, dtm.height(p0.x, p0.y));
    Vec3 B(p1.x, p1.y, dtm.height(p1.x, p1.y));
    Vec3 C(p2.x, p2.y, dtm.height(p2.x, p2.y));
    Vec3 n = co.use_three_point_plane ? (A - C).cross(B - C) : up;
    if (n.z < 0) n = -n;
    n = n.normalized();
    if (n.norm() < 0.5) n = up;
    double dist = (in.center - A).dot(n);
    double z = r - dist;
    // terrain parameters: mean of the three interpolated points (MarsSim)
    Soil so = DTM::mix({{dtm.soil(A.x, A.y), 1.0}, {dtm.soil(B.x, B.y), 1.0}, {dtm.soil(C.x, C.y), 1.0}});
    out.soil_w = so.soil;
    // interaction frame
    Vec3 xi = (xw - n * xw.dot(n)).normalized();
    Vec3 zi = n;
    Vec3 yi = zi.cross(xi);
    out.xi = xi; out.yi = yi; out.zi = zi;
    out.vx = in.vel.dot(xi); out.vy = in.vel.dot(yi); out.vz = in.vel.dot(zi);
    out.z = z;
    if (z <= 0 || so.soil <= 1e-6) {
      theta1 = co.theta1_init;
      s_filt = 0.0; b_filt = 0.0;
      return out;
    }
    // Slip is filtered in the fixed (unmirrored) wheel frame: s_u = (w rs - vx)/max(|w rs|,|vx|).
    // For reverse driving the model is evaluated mirrored (dir = -1); keeping the filter state in the
    // fixed frame avoids a sign error (and a runaway) whenever the driving direction flips.
    const double rs = geom.rs();
    const double vw = in.omega * rs, vx = out.vx, vy = out.vy;
    double vref0 = std::max(std::fabs(vx), std::fabs(vw));
    double s_u = clamp((vw - vx) / std::max(vref0, co.slip_v_eps), -1.0, 1.0);
    s_u *= clamp(vref0 / std::max(co.slip_v_ramp, 1e-9), 0.0, 1.0);
    double tau = co.slip_relax_len > 0 ? std::max(co.slip_tau_min, co.slip_relax_len / std::max(vref0, 1e-3)) : co.slip_tau;
    double a = tau > 0 ? clamp(dt / tau, 0.0, 1.0) : 1.0;
    s_filt += a * (s_u - s_filt);
    double b_raw = std::atan2(vy, std::max(vref0, co.slip_v_eps));  // regularised side slip
    b_filt += a * (b_raw - b_filt);
    const double dir = (vw + vx) >= 0 ? 1.0 : -1.0;
    out.s = dir * s_filt;   // slip in the driving direction (model convention)
    out.beta = b_filt;
    Forces f = wheel_forces(geom, so, mo, out.s, z, out.beta);
    out.f = f;
    theta1 = f.theta1 > 0 ? f.theta1 : co.theta1_init;
    if (!f.contact) return out;
    out.contact = true;
    // low-speed regularisation ("stationary friction"): rolling resistance vanishes at rest
    double vref = vref0;
    double ramp = clamp(vref / std::max(co.v_eps, 1e-9), 0.0, 1.0);
    out.ramp = ramp;
    double FDP = dir * f.FDP * ramp;
    double MR = dir * f.MR * ramp;
    double FS = f.FS * ramp;
    double FN = f.FN - co.normal_damping * out.vz;   // damping term (MarsSim: suppress oscillations)
    if (FN < 0) FN = 0;
    FDP -= co.tangential_damping * out.vx * (1.0 - ramp);
    FS -= co.tangential_damping * out.vy * (1.0 - ramp);
    FN = clamp(FN, 0.0, co.max_force); FDP = clamp(FDP, -co.max_force, co.max_force);
    out.FN_applied = FN; out.FDP_applied = FDP;
    out.force = xi * FDP + yi * FS + zi * FN;
    double MS = -sgn(in.yaw_rate) * f.MS * ramp;
    // torque about axle: resistance moment opposes spin; axle_left == yw
    out.torque = yw * (-MR) + xi * (f.MO * ramp) + zi * MS;
    return out;
  }
};

// ----------------------------------------------------------------------------- rigid contact (plugin-side)
// Axis-aligned rigid boxes (bedrock slabs, steps). Normal: penalty spring-damper. Tangential and
// lateral: Maxwell (elastic + creep) element with Coulomb limit mu*fn. The creep lets two wheels that
// are both on rock share traction instead of "freezing in" an arbitrary internal force (what a purely
// rigid Coulomb contact + independent wheel PI controllers do).
struct RigidBox { double x0 = 0, x1 = 0, y0 = -1e3, y1 = 1e3, ztop = 0, zbot = -0.1, mu = 0.6; };
struct RigidOpts { double kn = 3e5, cn = 2e3, kt = 5e4, ct = 1e3, tau_relax = 1.0; };
struct RigidOut { bool contact = false; Vec3 force, torque, n, p; double fn = 0, ft = 0, fl = 0, pen = 0; };

class RigidContact {
 public:
  double zt = 0, zl = 0;
  RigidOut step(const Vec3 &c, const Vec3 &vel, const Vec3 &angvel, const Vec3 &axle_left, double rc, double b,
                const std::vector<RigidBox> &boxes, const RigidOpts &o, double dt) {
    RigidOut out;
    const Vec3 up(0, 0, 1);
    Vec3 yw = axle_left.normalized();
    Vec3 xw = yw.cross(up); xw.z = 0; xw = xw.normalized();
    if (xw.norm() < 0.5) return out;
    double best = 0; const RigidBox *bb = nullptr; double bu = 0, bz = 0, nu = 0, nz = 1, frac = 0;
    for (const auto &bx : boxes) {
      // lateral overlap of the wheel width with the box (axle assumed close to horizontal)
      double hy = 0.5 * b * std::fabs(yw.y);   // half-width of the wheel along world y
      double ov = std::min(c.y + hy, bx.y1) - std::max(c.y - hy, bx.y0);
      double f = hy > 1e-6 ? clamp(ov / (2 * hy), 0.0, 1.0) : ((c.y >= bx.y0 && c.y <= bx.y1) ? 1.0 : 0.0);
      if (f <= 0) continue;
      // section of the box footprint along the wheel heading line c + u*xw
      double u0 = -1e9, u1 = 1e9;
      auto slab = [&](double p, double d, double lo, double hi) {
        if (std::fabs(d) < 1e-9) { if (p < lo || p > hi) { u0 = 1; u1 = 0; } return; }
        double a = (lo - p) / d, e = (hi - p) / d; if (a > e) std::swap(a, e);
        u0 = std::max(u0, a); u1 = std::min(u1, e);
      };
      slab(c.x, xw.x, bx.x0, bx.x1);
      slab(c.y, xw.y, bx.y0 - hy, bx.y1 + hy);
      if (u0 > u1) continue;
      double qu = clamp(0.0, u0, u1), qz = clamp(c.z, bx.zbot, bx.ztop);
      double du = -qu, dz = c.z - qz, d = std::sqrt(du * du + dz * dz);
      double pen, n_u, n_z;
      if (d >= rc) continue;
      if (d < 1e-9) { n_u = 0; n_z = 1; pen = rc + bx.ztop - c.z; }
      else { n_u = du / d; n_z = dz / d; pen = rc - d; }
      if (pen > best) { best = pen; bb = &bx; bu = qu; bz = qz; nu = n_u; nz = n_z; frac = f; }
    }
    if (!bb) { zt = zl = 0; return out; }
    Vec3 n = xw * nu + up * nz;
    Vec3 p = Vec3(c.x, c.y, 0) + xw * bu + up * bz;
    Vec3 r = p - c;
    Vec3 vp = vel + angvel.cross(r);
    Vec3 t = xw * nz - up * nu;                       // in-plane tangent
    Vec3 l = (yw - n * yw.dot(n)).normalized();       // lateral
    double vn = vp.dot(n), vt = vp.dot(t), vl = vp.dot(l);
    double fn = std::max(0.0, frac * o.kn * best - o.cn * vn);
    zt += (vt - zt / o.tau_relax) * dt;
    zl += (vl - zl / o.tau_relax) * dt;
    double ft = -(o.kt * zt + o.ct * vt), fl = -(o.kt * zl + o.ct * vl);
    double lim = bb->mu * fn, mag = std::sqrt(ft * ft + fl * fl);
    if (mag > lim && mag > 1e-12) {
      double k = lim / mag; ft *= k; fl *= k;
      zt = clamp(zt, -lim / o.kt, lim / o.kt); zl = clamp(zl, -lim / o.kt, lim / o.kt);
    }
    out.contact = true; out.fn = fn; out.ft = ft; out.fl = fl; out.pen = best; out.n = n; out.p = p;
    out.force = n * fn + t * ft + l * fl;
    out.torque = r.cross(out.force);
    return out;
  }
};

// ----------------------------------------------------------------------------- control helpers
struct PIController {
  double kp = 50, ki = 200, imax = 1e9, umax = 1e9, integ = 0;
  double update(double err, double dt) {
    integ = clamp(integ + err * dt, -imax, imax);
    return clamp(kp * err + ki * integ, -umax, umax);
  }
};

// piecewise-linear schedule "t0:v0, t1:v1, ..." (hold last value)
struct Schedule {
  std::vector<double> t, v;
  bool step = false;  // piecewise constant if true
  void parse(const std::vector<double> &tv) {
    t.clear(); v.clear();
    for (size_t i = 0; i + 1 < tv.size(); i += 2) { t.push_back(tv[i]); v.push_back(tv[i + 1]); }
  }
  double at(double x) const {
    if (t.empty()) return 0.0;
    if (x <= t.front()) return v.front();
    for (size_t i = 1; i < t.size(); ++i)
      if (x < t[i]) {
        if (step) return v[i - 1];
        double a = (x - t[i - 1]) / std::max(t[i] - t[i - 1], 1e-12);
        return v[i - 1] + a * (v[i] - v[i - 1]);
      }
    return v.back();
  }
};

// simple CSV writer
class Csv {
 public:
  std::unique_ptr<std::ofstream> f;
  bool open(const std::string &path, const std::vector<std::string> &hdr) {
    f.reset(new std::ofstream(path));
    if (!*f) { f.reset(); return false; }
    for (size_t i = 0; i < hdr.size(); ++i) *f << (i ? "," : "") << hdr[i];
    *f << "\n";
    return true;
  }
  void row(const std::vector<double> &v) {
    if (!f) return;
    char buf[64];
    for (size_t i = 0; i < v.size(); ++i) { std::snprintf(buf, sizeof(buf), "%.7g", v[i]); *f << (i ? "," : "") << buf; }
    *f << "\n";
  }
  void flush() { if (f) f->flush(); }
};

}  // namespace terramech
