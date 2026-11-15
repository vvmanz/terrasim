// omni.hpp — omni / mecanum wheel on firm ground, DC motor with the Robotino speed controller.
//
// Same terramechanics family as the MarsSim soft-soil model (terramech.hpp), hard-surface limit:
//   * traction  : Janosi shear law tau = (c + sigma tan(phi)) (1 - exp(-j/K)) integrated over the
//                 contact patch with j = s*x (uniform pressure, Janosi–Hanamoto / Bekker tractive
//                 effort–slip relation, Wong "Theory of Ground Vehicles" eq. 2.56). It acts only along
//                 the roller axis a_r — the direction in which a roller cannot roll.
//   * resistance: on firm floors Bekker compaction is negligible; rolling resistance is the
//                 hysteresis of roller tread + floor, expressed (as in Wong ch.1 for tyres on hard
//                 ground) by coefficients: f_roll for rolling of the wheel (moment on the wheel, through
//                 the motor), f_lat for rolling of the rollers (force on the hub, not through the motor),
//                 both scaled by the share of the rolling velocity in that direction.
//   * normal    : penalty spring-damper (roller tread + floor compliance).
// The roller angle gamma (angle between roller axis and wheel axle) makes the same law cover
//   omni wheels (gamma = 90 deg: Robotino, Festo, TurtleBot-omni), mecanum (45 deg: KUKA youBot) and an
//   ordinary rubber wheel (set f_lat -> large and phi_lat... use the soft-soil WheelContact instead).
#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include "terramech/terramech.hpp"

namespace terramech {

inline double rad2deg(double r) { return r * 180.0 / kPi; }

struct FirmSurface {
  std::string name = "floor";
  double phi = deg2rad(35.0);  // friction angle: mu = tan(phi)
  double c = 0.0;              // adhesion (cohesion) Pa, acts on patch area
  double K = 0.003;            // shear deformation modulus, m
  double patch_len = 0.010;    // contact patch length along the constrained direction, m
  double patch_w = 0.010;      // contact patch width, m (only for c*A)
  double f_roll = 0.02;        // rolling-resistance coefficient of the wheel (moment arm / r)
  double f_lat = 0.05;         // rolling-resistance coefficient of the rollers
  double mu_rb = 0.0;          // roller bearing friction (Coulomb, along the free direction)
  double kn = 2.0e5, cn = 300; // normal stiffness N/m, damping N s/m
  // spatial inhomogeneity (the firm-ground analogue of MarsSim's soil-parameter map on the DTM):
  // resistance coefficients f_roll, f_lat, mu_rb are multiplied by (1 + cv_res g1(x, y)) and the shear
  // modulus K by (1 + cv_grip g2(x, y)); g1, g2 are independent unit-variance Gaussian random fields with
  // Gaussian correlation of length corr_len (deterministic for a given field_seed).
  double cv_res = 0.0, cv_grip = 0.0, corr_len = 0.02;
  int field_seed = 7;
  // Bekker deformable layer (off while kc = kphi = 0): pressure-sinkage p = (kc/b + kphi) z^n under the
  // wheel -> static sinkage of a rigid wheel (Bekker), compaction resistance b (kc/b + kphi) z^(n+1)/(n+1)
  // against the motion, contact (shear) length sqrt(2 r z) and cohesive area b L. For coverings on a rigid
  // floor (carpet pile, foam) the sinkage is limited by the layer thickness.
  double kc = 0.0, kphi = 0.0, n_bek = 1.0;  // N/m^(n+1), N/m^(n+2), -
  double b_contact = 0.03;   // tread width across the rolling direction, m
  double layer = 1e9;        // thickness of the deformable layer above a rigid base, m
  double rebound = 0.0;      // elastic recovery of the layer behind the wheel: 0 = plastic (soils), 0.8 = foam
                             // (only the hysteresis share 1 - rebound of the compaction work is lost)
  double cv_soil = 0.0;      // spatial variation of kc, kphi (third field)
  int slip_norm = -1;        // -1: robot default ([omni_contact]); 0: wheel element (soils); 1: ground element (coverings)
  bool deformable() const { return kc > 0 || kphi > 0; }
};

// Unit-variance stationary Gaussian random field on the plane, random Fourier features
// g(p) = sqrt(2/M) sum cos(k_m . p + phi_m),  k_m ~ N(0, I / l^2):  cov(d) = exp(-|d|^2 / (2 l^2)).
class RandomField2D {
 public:
  void init(int seed, double corr_len, int modes = 96) {
    kx.assign(modes, 0); ky.assign(modes, 0); ph.assign(modes, 0);
    uint64_t st = 0x9E3779B97F4A7C15ULL ^ (uint64_t)(seed * 2654435761ULL + 12345);
    auto u01 = [&]() { st ^= st << 13; st ^= st >> 7; st ^= st << 17; return ((st >> 11) + 0.5) * (1.0 / 9007199254740992.0); };
    for (int m = 0; m < modes; ++m) {
      double r = std::sqrt(-2.0 * std::log(u01())), a = 2 * kPi * u01();
      kx[m] = r * std::cos(a) / corr_len; ky[m] = r * std::sin(a) / corr_len; ph[m] = 2 * kPi * u01();
    }
    norm = std::sqrt(2.0 / modes);
  }
  double operator()(double x, double y) const {
    double s = 0;
    for (size_t m = 0; m < kx.size(); ++m) s += std::cos(kx[m] * x + ky[m] * y + ph[m]);
    return norm * s;
  }
  bool empty() const { return kx.empty(); }
 private:
  std::vector<double> kx, ky, ph;
  double norm = 0;
};

// Surface with its two fields; local(x, y) returns the parameters at a point.
class SurfaceMap {
 public:
  FirmSurface base;
  void init(const FirmSurface &S) {
    base = S; on = S.cv_res > 0 || S.cv_grip > 0 || (S.cv_soil > 0 && S.deformable());
    if (S.cv_res > 0) g1.init(S.field_seed, S.corr_len);
    if (S.cv_grip > 0) g2.init(S.field_seed + 1000, S.corr_len);
    if (S.cv_soil > 0 && S.deformable()) g3.init(S.field_seed + 2000, S.corr_len);
  }
  FirmSurface local(double x, double y) const {
    if (!on) return base;
    FirmSurface s = base;
    if (!g1.empty()) {
      const double m = std::max(0.05, 1.0 + base.cv_res * g1(x, y));
      s.f_roll *= m; s.f_lat *= m; s.mu_rb *= m;
    }
    if (!g2.empty()) s.K *= std::max(0.2, 1.0 + base.cv_grip * g2(x, y));
    if (!g3.empty()) { const double m = std::max(0.2, 1.0 + base.cv_soil * g3(x, y)); s.kc *= m; s.kphi *= m; }
    return s;
  }
 private:
  bool on = false;
  RandomField2D g1, g2, g3;
};

inline FirmSurface firm_from_section(const Config::Section *s, FirmSurface b = FirmSurface()) {
  if (!s) return b;
  b.name = s->label.empty() ? b.name : s->label;
  if (Config::has(s, "phi_deg")) b.phi = deg2rad(Config::num(s, "phi_deg", 35));
  if (Config::has(s, "mu")) b.phi = std::atan(Config::num(s, "mu", 0.7));
  b.c = Config::num(s, "c", b.c);
  b.K = Config::num(s, "K", b.K);
  b.patch_len = Config::num(s, "patch_len", b.patch_len);
  b.patch_w = Config::num(s, "patch_w", b.patch_w);
  b.f_roll = Config::num(s, "f_roll", b.f_roll);
  b.f_lat = Config::num(s, "f_lat", b.f_lat);
  b.mu_rb = Config::num(s, "mu_rb", b.mu_rb);
  b.kn = Config::num(s, "kn", b.kn);
  b.cn = Config::num(s, "cn", b.cn);
  b.cv_res = Config::num(s, "cv_res", b.cv_res);
  b.cv_grip = Config::num(s, "cv_grip", b.cv_grip);
  b.corr_len = Config::num(s, "corr_len", b.corr_len);
  b.field_seed = (int)Config::num(s, "field_seed", b.field_seed);
  b.kc = Config::num(s, "kc", b.kc); b.kphi = Config::num(s, "kphi", b.kphi); b.n_bek = Config::num(s, "n", b.n_bek);
  b.b_contact = Config::num(s, "b_contact", b.b_contact); b.layer = Config::num(s, "layer", b.layer);
  b.slip_norm = (int)Config::num(s, "slip_norm", b.slip_norm);
  b.rebound = Config::num(s, "rebound", b.rebound); b.cv_soil = Config::num(s, "cv_soil", b.cv_soil);
  return b;
}

// Janosi–Hanamoto: shear over a patch of length L with uniform pressure, j(x) = s x.
//   H/Hmax = 1 - K/(sL) (1 - exp(-sL/K))
inline double janosi_patch_ratio(double s, double L, double K) {
  double u = std::fabs(s) * L / std::max(K, 1e-9);
  double g = u < 1e-4 ? 0.5 * u - u * u / 6.0 : 1.0 - (1.0 - std::exp(-u)) / u;
  return s >= 0 ? g : -g;
}

struct OmniGeom {
  double r = 0.040;                 // wheel radius (roller envelope)
  double gamma = deg2rad(90.0);     // roller axis vs. wheel axle: 90 = omni, 45 = mecanum
  int type = 0;                     // 0 = roller wheel (omni / mecanum), 1 = ordinary wheel or tyre (no free direction),
                                    // 2 = caster (swivel wheel / ball: free in every direction, passive)
  double b = 0.0;                   // tread width, m (> 0 overrides the surface's b_contact)
  double patch = 0.0;               // contact length on firm ground, m (> 0 overrides the surface's patch_len)
  double lat_grip = 1.0;            // ordinary wheel (tyre): lateral / longitudinal grip ratio (friction ellipse). Lugged
                                    // tyres on grass grip less sideways than along the tread: the lateral resistance
                                    // coefficient of skid-steer models; 1 = isotropic (v7.5 and older)
  double mu_rb = -1.0;              // roller bearing friction of THIS wheel's rollers (>= 0 overrides the surface's
                                    // mu_rb: it is a property of the wheel, the surface value is the Robotino fit)
  double zmax_r = 0.6;              // sinkage limit as a fraction of r (validity of the rigid-wheel Bekker formula)
  double Jc = 0.0;                  // caster: spin inertia of the ball / wheel (> 0: the caster is turned by the floor)
  bool swivel = false;              // caster: mounted on a free swivel (aligns its rolling direction with the motion)
};

struct OmniOpts {
  double slip_v_eps = 0.005;     // m/s, slip denominator floor
  double slip_v_ramp = 0.01;     // m/s, traction ramp-in at very low speed
  double slip_relax_len = 0.010; // m, relaxation length (~patch length)
  double slip_tau_min = 0.005;   // s
  double v_eps = 0.008;          // m/s, regularisation of rolling-direction shares (explicit stability at 1-2 ms)
  // Normalisation of the slip in the Janosi shear displacement j = s x:
  //   0 = wheel element  : s = (Vd - Vg) / max(|Vd|, |v_c|) — time a point of the wheel/roller surface
  //                        spends in the patch (classical rigid-wheel / track derivation, Wong eq. 2.53);
  //   1 = ground element : s = (Vd - Vg) / |v_g| — time a point of the floor spends under the patch, the
  //                        patch moving over the floor with the hub ground speed |v_g|. Appropriate when the
  //                        sheared material is the floor covering itself (carpet pile, artificial grass,
  //                        foam); identified on the Robotino dataset (driven-wheel slip ~independent of load).
  int slip_norm = 0;
  double slip_max = 10.0;        // clamp of the normalised slip (ground normalisation of a stalled robot -> inf)
  bool tilt_wheel = false;       // a wheel tilted about the ground: rim radius r*sqrt(1 - (axle.n)^2); lying on its side
                                 // (|axle.n| > 0.97) it has no rim contact (the wheel faces are body contact points)
  double caster_tau = 0.01;      // s, time in which the floor brings a caster's spin to rolling
  double caster_wn = 30.0;       // rad/s, swivel alignment (trail effect) natural frequency
};

struct OmniIn {
  Vec3 center, vel;          // wheel centre position / velocity, world
  Vec3 axle;                 // wheel axle, world (positive spin about it drives along e_x = axle x n)
  double omega = 0;          // wheel spin about axle, rad/s
  Vec3 angvel;               // wheel angular velocity, world (casters: swivel rate)
  double ground_z = 0;       // flat floor height
  Vec3 n{0, 0, 1};           // floor normal
};

struct OmniOut {
  bool contact = false;
  Vec3 force, torque;        // wrench on the wheel at its centre, world
  double N = 0, H = 0, F_free = 0, M_roll = 0;  // normal, traction (along a_r), free-direction force, rolling moment
  double z = 0, R_comp = 0;                      // Bekker sinkage, compaction resistance (deformable surfaces)
  double s_lat = 0;                              // lateral slip (ordinary wheels)
  double s = 0, s_raw = 0, v_x = 0, v_y = 0, delta = 0;
};

class OmniContact {
 public:
  OmniGeom geom;
  double s_filt = 0, z_filt = 0, slat_filt = 0;

  // static Bekker sinkage of a rigid wheel of diameter D = 2r and tread width b under the load N
  static double bekker_sinkage(const FirmSurface &S, double N, double r, double b = -1.0, double zmax_r = 0.6) {
    if (!S.deformable() || N <= 0) return 0.0;
    if (b <= 0) b = S.b_contact;
    const double ks = S.kc + b * S.kphi, n = clamp(S.n_bek, 0.05, 2.9);
    const double z = std::pow(3.0 * N / ((3.0 - n) * ks * std::sqrt(2.0 * r)), 2.0 / (2.0 * n + 1.0));
    return std::min(std::min(z, S.layer), zmax_r * r);
  }

  OmniOut step(const OmniIn &in, const FirmSurface &S, const OmniOpts &o, double dt) {
    OmniOut out;
    const Vec3 n = in.n.normalized();
    const double ca = o.tilt_wheel && geom.type != 2 ? in.axle.normalized().dot(n) : 0.0;   // casters: balls / swivel wheels
    if (std::fabs(ca) > 0.97) { s_filt *= 0.9; return out; }
    Vec3 ax = (in.axle - n * in.axle.dot(n)).normalized();
    const Vec3 ex = ax.cross(n).normalized();   // wheel rolling direction for positive spin
    const Vec3 ey = ax;                          // along the axle
    // roller axis at the contact and the free (rolling) direction of the roller
    const double cg = std::cos(geom.gamma), sg = std::sin(geom.gamma);
    const bool tyre = geom.type == 1;            // ordinary wheel: traction along e_x, lateral shear along e_y
    const Vec3 ar = tyre ? ex : ex * sg + ey * cg;   // gamma=90: ar = ex (omni); gamma=45: diagonal (mecanum)
    const double bw = geom.b > 0 ? geom.b : S.b_contact;
    const Vec3 ef = n.cross(ar).normalized();    // free direction (perpendicular to ar in the floor plane)
    const bool soft = S.deformable();
    const double h = (in.center.z - (in.ground_z - z_filt)) * n.z;   // distance to the (tilted) floor plane; rut below
    out.delta = (ca != 0.0 ? geom.r * std::sqrt(1.0 - ca * ca) : geom.r) - h;
    out.v_x = in.vel.dot(ex); out.v_y = in.vel.dot(ey);
    const double vn = in.vel.dot(n);
    if (out.delta <= 0) { s_filt *= 0.9; return out; }
    out.contact = true;
    double N = S.kn * out.delta - S.cn * vn;
    N = std::max(N, 0.0);
    out.N = N;
    {
      // rut depth follows the load: sinking with tau = 50 ms; climbing out of the rut (softer -> firmer
      // ground) at most like up a ramp of the rut wall (~27 deg), never as a vertical jump of the floor
      const double zt = soft ? bekker_sinkage(S, N, geom.r, bw, geom.zmax_r) : 0.0;
      double dz = clamp(dt / 0.05, 0.0, 1.0) * (zt - z_filt);
      const double vh = (in.vel - n * vn).norm();
      if (dz < 0) dz = std::max(dz, -dt * (0.5 * vh + 0.002));
      z_filt = std::max(0.0, z_filt + dz);
      out.z = z_filt;
    }
    const double Lp0 = geom.patch > 0 ? geom.patch : S.patch_len;
    const double Lp = soft ? std::max(Lp0, std::sqrt(2.0 * geom.r * z_filt)) : Lp0;
    const double Ac = (soft || geom.b > 0) ? bw * Lp : S.patch_len * S.patch_w;
    const int sn = S.slip_norm >= 0 ? S.slip_norm : o.slip_norm;
    if (geom.type == 2) {
      // caster (swivel wheel or ball): rolls freely in any direction, so no traction and no moment on its axle;
      // resistance against the horizontal motion = rolling resistance f_roll N + Bekker compaction of its rut
      const Vec3 vh = in.vel - n * vn;
      const double vh_n = vh.norm(), vv = std::sqrt(vh_n * vh_n + o.v_eps * o.v_eps);
      double R = S.f_roll * N;
      if (soft && z_filt > 0) {
        const double zn1 = (1.0 - clamp(S.rebound, 0.0, 1.0)) * std::pow(z_filt, S.n_bek + 1.0) / (S.n_bek + 1.0);
        out.R_comp = (S.kc + bw * S.kphi) * zn1; R += out.R_comp;
      }
      out.F_free = -R * vh_n / vv;
      out.force = n * N + vh * (-R / vv);
      out.torque = Vec3(0, 0, 0);
      if (geom.Jc > 0) {
        // The caster's own rotation. The floor turns it by friction at the rim: the rim slip speed s = v_e - r w
        // is removed in caster_tau; the same friction force acts back on the hub (spinning the caster up costs
        // J a / r^2 of drag, nothing in steady rolling). The resistance above already is the ROLLING resistance
        // of a caster that rolls (f_roll N), not sliding friction (mu N).
        const double sr = vh.dot(ex) - geom.r * in.omega;
        const double cf = geom.Jc / (geom.r * geom.r * std::max(o.caster_tau, 2.0 * dt));
        const double Fmax = std::tan(S.phi) * N;
        const double Ff = clamp(cf * sr, -Fmax, Fmax);
        out.force = out.force - ex * Ff;
        out.torque = ax * (geom.r * Ff);
        out.s = sr;
        if (geom.swivel && vh_n > 1e-3) {
          // swivel: trail aligns the rolling direction with the motion (either way round: minimal turn)
          double d = std::atan2(ex.cross(vh).dot(n), ex.dot(vh));
          if (d > 0.5 * kPi) d -= kPi;
          if (d < -0.5 * kPi) d += kPi;
          const double k = geom.Jc * o.caster_wn * o.caster_wn, c = 1.4 * geom.Jc * o.caster_wn;
          const double tz = clamp(k * d - c * in.angvel.dot(n), -Fmax * geom.r, Fmax * geom.r);
          out.torque = out.torque + n * tz;
        }
      }
      return out;
    }
    // --- slip along the constrained direction a_r
    // Janosi over the patch: a ground element stays in the patch for L/|v_c|, where v_c is the full
    // planar velocity of the contact (wheel rolling + roller rolling), and is sheared along a_r at the
    // slip velocity Vd - Vg. Hence j = s x with s = (Vd - Vg)/max(|Vd|, |v_c|); for an ordinary wheel
    // (v_c along e_x) this is the usual slip ratio.
    const double Vd = in.omega * geom.r * ex.dot(ar);   // drive speed of the rim along a_r
    const double Vg = in.vel.dot(ar);                    // ground speed of the hub along a_r
    const Vec3 vpl = in.vel - n * vn;
    const double vmax = std::max(std::fabs(Vd), std::max(std::fabs(Vg), vpl.norm()));
    const double vref = sn == 1 ? vpl.norm() : vmax;
    const double smax = sn == 1 ? o.slip_max : 1.0;
    double s_u = clamp((Vd - Vg) / std::max(vref, o.slip_v_eps), -smax, smax);
    s_u *= clamp(vmax / std::max(o.slip_v_ramp, 1e-9), 0.0, 1.0);
    const double tau = o.slip_relax_len > 0 ? std::max(o.slip_tau_min, o.slip_relax_len / std::max(vmax, 1e-3)) : 0.0;
    const double a = tau > 0 ? clamp(dt / tau, 0.0, 1.0) : 1.0;
    s_filt += a * (s_u - s_filt);
    out.s_raw = s_u; out.s = s_filt;
    const double Hmax = S.c * Ac + N * std::tan(S.phi);
    double H = Hmax * janosi_patch_ratio(s_filt, Lp, S.K), F_lat = 0.0;
    if (tyre) {
      // ordinary wheel: lateral slip -v_y/v_ref sheared by the same Janosi law (K_y = K); traction and
      // side force share one friction limit (resultant slip |s|, direction of the slip vector)
      double sy_u = clamp(-in.vel.dot(ey) / std::max(vref, o.slip_v_eps), -smax, smax);
      sy_u *= clamp(vmax / std::max(o.slip_v_ramp, 1e-9), 0.0, 1.0);
      slat_filt += a * (sy_u - slat_filt);
      const double sres = std::hypot(s_filt, slat_filt);
      if (sres > 1e-9) {
        const double Hres = Hmax * janosi_patch_ratio(sres, Lp, S.K);
        H = Hres * s_filt / sres; F_lat = geom.lat_grip * Hres * slat_filt / sres;
      } else H = 0.0;
      out.s_lat = slat_filt;
    }
    out.H = H;
    // --- rolling resistances. Rolling-velocity vector of the contact element: wheel rolling (rim speed)
    //     and roller rolling. The roller rolls at the speed of the RIGID contact point along the free
    //     direction, (v_hub - w r e_x) . e_f = v_hub . e_f + w r cos(gamma): the rim's own motion has a
    //     component cos(gamma) along e_f for a mecanum roller. Omni (gamma = 90 deg): e_x . e_f = 0, the
    //     roller speed is the hub speed along e_f. Mecanum driving straight (w r = v): the rollers stand
    //     still (v3: the hub speed alone was used, so the rollers spun -v cos(gamma) and loaded every
    //     direction alike; found on recorded youBot runs).
    const double u_w = in.omega * geom.r;
    const double exf = ex.dot(ef);                       // -cos(gamma); exactly 0 for omni (not cos(pi/2) = 6e-17)
    const double u_f = in.vel.dot(ef) - ((tyre || std::fabs(exf) < 1e-9) ? 0.0 : u_w * exf);
    const double un = std::sqrt(u_w * u_w + u_f * u_f + o.v_eps * o.v_eps);
    const double M_roll = -S.f_roll * N * geom.r * (u_w / un);            // about the axle
    const double mu_rb = geom.mu_rb >= 0 ? geom.mu_rb : S.mu_rb;
    const double F_free = tyre ? F_lat : -S.f_lat * N * (u_f / un) - mu_rb * N * std::tanh(u_f / o.v_eps);
    out.M_roll = M_roll; out.F_free = F_free;
    Vec3 Ft = ar * H + ef * F_free;                       // tangential force at the contact point
    Vec3 Fc(0, 0, 0);                                     // compaction resistance (acts at the hub, see below)
    if (soft && z_filt > 0) {
      // compaction resistance (energy to cut the rut per unit length), against the hub motion:
      // rolling along e_x cuts a rut of width b, sideways (rollers) a rut of width = contact chord
      const double zn1 = (1.0 - clamp(S.rebound, 0.0, 1.0)) * std::pow(z_filt, S.n_bek + 1.0) / (S.n_bek + 1.0);
      const double Rx = (S.kc + bw * S.kphi) * zn1;
      const double chord = 2.0 * std::sqrt(std::max(2.0 * geom.r * z_filt - z_filt * z_filt, 0.0));
      const double Ry = (S.kc + std::max(chord, 1e-4) * S.kphi) * zn1;
      const double vx = in.vel.dot(ex), vy = in.vel.dot(ey);
      const double vv = std::sqrt(vx * vx + vy * vy + o.v_eps * o.v_eps);
      Fc = ex * (-Rx * vx / vv) + ey * (-Ry * vy / vv);
      out.R_comp = Fc.norm();
    }
    // Compaction resistance is the horizontal part of the NORMAL stresses on the front of the rim; for a rigid
    // wheel they pass through the axle (Bekker, Wong ch. 2), so R_c acts on the hub without a moment about the axle.
    // The motor pays for it through the traction H that has to balance it (torque r H). Up to v7.6 R_c was applied at
    // the contact point: its moment r R_c then cancelled the extra torque, and the compaction work was not drawn
    // from the motor (currents on deformable surfaces too low; slips and trajectories were the same).
    out.force = n * N + Ft + Fc;
    const Vec3 rc = n * (-geom.r);                       // contact point relative to the centre
    out.torque = rc.cross(Ft) + ax * M_roll;
    return out;
  }
};

// ----------------------------------------------------------------------------------------- DC motor
// Robotino drive: DC motor + planetary gear, firmware speed controller. Electrical time constant
// (L/R ~ 1.7 ms) is neglected. Controller identified from the dataset: steady speed error
//   e = w_set - w = (R I + Vd)/(ff*Ce + kp)   (proportional controller with back-EMF feed-forward).
struct MotorParams {
  double R = 5.3351, Ce = 0.0501, kt = 0.0501;  // Ohm, V s/rad, N m/A (motor shaft)
  double gear = 16.0, eta = 0.8;                // ratio, gearbox efficiency
  double I0 = 0.17;                             // no-load current, A (friction of motor+gear)
  double w_eps = 2.0;                           // rad/s (motor), friction regularisation
  double Vmax = 24.0;                           // supply
  double Vd = 1.42;                             // driver / brush voltage drop (dead zone), V
  double kp = 0.75;                             // V per rad/s (motor)
  double ff = 1.0;                              // back-EMF feed-forward gain
  double ki = 0.0;                              // V per rad (motor), 0 = Robotino behaviour
};

inline MotorParams motor_from_section(const Config::Section *s, MotorParams m = MotorParams()) {
  if (!s) return m;
  m.R = Config::num(s, "R", m.R); m.Ce = Config::num(s, "Ce", m.Ce); m.kt = Config::num(s, "kt", m.kt);
  m.gear = Config::num(s, "gear", m.gear); m.eta = Config::num(s, "eta", m.eta); m.I0 = Config::num(s, "I0", m.I0);
  m.w_eps = Config::num(s, "w_eps", m.w_eps); m.Vmax = Config::num(s, "Vmax", m.Vmax); m.Vd = Config::num(s, "Vd", m.Vd);
  m.kp = Config::num(s, "kp", m.kp); m.ff = Config::num(s, "ff", m.ff); m.ki = Config::num(s, "ki", m.ki);
  return m;
}

struct MotorOut { double I = 0, V = 0, tau_wheel = 0, w_motor = 0; };

class DCMotor {
 public:
  double integ = 0;
  // Static motor + controller map: wheel speeds (rad/s) -> torque on the wheel (about its axle)
  MotorOut eval(const MotorParams &p, double w_set_wheel, double w_wheel) const {
    MotorOut o;
    const double ws = w_set_wheel * p.gear, w = w_wheel * p.gear;
    const double e = ws - w;
    double V = p.ff * p.Ce * ws + p.kp * e + p.ki * integ;
    V = clamp(V, -p.Vmax, p.Vmax);
    const double u = V - p.Ce * w;
    const double I = (u - p.Vd * clamp(u / std::max(p.Vd, 1e-9), -1.0, 1.0)) / p.R;   // dead zone of the driver
    const double tm = p.kt * I - p.kt * p.I0 * std::tanh(w / p.w_eps);                 // no-load friction
    o.I = I; o.V = V; o.w_motor = w;
    o.tau_wheel = (tm * w >= 0 ? p.eta : 1.0 / p.eta) * p.gear * tm;
    return o;
  }
  // Explicit step (torque from the current wheel speed).
  MotorOut step(const MotorParams &p, double w_set_wheel, double w_wheel, double dt) {
    integ += (w_set_wheel - w_wheel) * p.gear * dt;
    return eval(p, w_set_wheel, w_wheel);
  }
  // Semi-implicit step: the speed controller is stiff (time constant J/c ~ 1.3 ms for Robotino), so the
  // torque is evaluated at the wheel speed predicted at the end of the step,
  //   w+ = w + dt (tau_ext + tau)/J,  tau = f(w+)  (linearised; exact in steady state: tau = f(w)).
  // tau_ext: torque of the contact on the wheel about the axle in this step; J: wheel spin inertia.
  MotorOut step_implicit(const MotorParams &p, double w_set_wheel, double w_wheel, double dt, double J, double tau_ext) {
    integ += (w_set_wheel - w_wheel) * p.gear * dt;
    const MotorOut a = eval(p, w_set_wheel, w_wheel);
    const double dw = 1e-4;
    const double c = -(eval(p, w_set_wheel, w_wheel + dw).tau_wheel - a.tau_wheel) / dw;   // >= 0
    const double k = c * dt / std::max(J, 1e-9);
    const double tau = (a.tau_wheel - k * tau_ext) / (1.0 + k);
    return eval(p, w_set_wheel, w_wheel + dt * (tau_ext + tau) / std::max(J, 1e-9));
  }
};

}  // namespace terramech
