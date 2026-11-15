// Unit tests for terramech.hpp (run by scripts/build.sh). Reference values: Python prototype.
#include <cstdio>
#include <cstdlib>
#include "terramech/terramech.hpp"
#include "terramech/omni.hpp"
#include "terramech/vehicle_logic.hpp"
#include <fstream>
using namespace terramech;
static int fails = 0;
static void check(bool ok, const char *what, double got, double exp) {
  std::printf("%s %-58s got % .6g expected % .6g\n", ok ? "[ OK ]" : "[FAIL]", what, got, exp);
  if (!ok) ++fails;
}
static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol * std::max(1.0, std::fabs(b)); }

int main() {
  // 1) closed form, paper-literal settings (Table I, cp printed form), s = 0.18, z = 10.35 mm
  {
    WheelGeom w; w.r = 0.135; w.b = 0.165; w.h = 0.01; w.lambda = 0.5; w.Rj = 0.14;
    ModelOpts m; m.cp_form = 1; m.cp1 = -0.924; m.cp2 = 0.293; m.cp3 = -0.68; m.cM = -1.0; m.W_bar = 80; m.sN_lo = -1; m.sN_hi = 1;
    Forces f = closed_form(w, soil_marssim_table1(), m, 0.18, 0.01035, 0.0);
    check(near(f.FN, 62.437, 1e-3), "closed form literal FN (s=0.18, z=10.35mm)", f.FN, 62.437);
    check(near(f.MR, 2.5379, 1e-3), "closed form literal MR", f.MR, 2.5379);
    check(near(f.FDP, 0.6527, 2e-3), "closed form literal FDP", f.FDP, 0.6527);
  }
  // 2) calibrated single wheel: sinkage for W = 80 N at s = 0.2826 should be ~13.8 mm
  {
    WheelGeom w; w.r = 0.135; w.b = 0.165; w.h = 0.01; w.lambda = 0.0; w.Rj = 0.145;
    Soil s = soil_marssim_table1(); s.n0 = 0.8245; s.n1 = 1.0645;
    ModelOpts m;
    double z = solve_sinkage(w, s, m, 0.2826, 80.0);
    check(near(z * 1000, 13.8, 0.03), "calibrated sinkage at s=0.2826, W=80 N [mm]", z * 1000, 13.8);
    Forces f = closed_form(w, s, m, 0.2826, z, 0.0);
    check(near(f.FN, 80.0, 1e-4), "F_N at solved sinkage equals load", f.FN, 80.0);
    check(f.FDP > 18 && f.FDP < 22, "calibrated FDP at s=0.28 within measured band 18..22 N", f.FDP, 20.0);
  }
  // 3) slip ratio definition (MarsSim eq. 6)
  check(near(slip_ratio(0.08, 1.0, 0.1), 0.2, 1e-9), "slip ratio, slipping", slip_ratio(0.08, 1.0, 0.1), 0.2);
  check(near(slip_ratio(0.125, 1.0, 0.1), -0.2, 1e-9), "slip ratio, skidding", slip_ratio(0.125, 1.0, 0.1), -0.2);
  // 4) skid must not blow up the support force (sinkage exponent clamp)
  {
    WheelGeom w; ModelOpts m; Soil s = soil_marssim_table2();
    double a = closed_form(w, s, m, -0.9, 0.02, 0).FN, b = closed_form(w, s, m, 0.0, 0.02, 0).FN;
    check(std::fabs(a / b - 1.0) < 0.5, "F_N(s=-0.9)/F_N(s=0) bounded (slip clamp)", a / b, 1.0);
  }
  // 5) Wong-Reece integral sanity: F_N increases with sinkage, FDP increases with slip
  {
    WheelGeom w; ModelOpts m; m.model = WONG_REECE; Soil s = soil_marssim_table2();
    double f1 = wong_reece(w, s, m, 0.2, 0.01, 0).FN, f2 = wong_reece(w, s, m, 0.2, 0.02, 0).FN;
    check(f2 > f1, "Wong-Reece F_N monotone in z", f2 - f1, 1);
    double d1 = wong_reece(w, s, m, 0.1, 0.02, 0).FDP, d2 = wong_reece(w, s, m, 0.4, 0.02, 0).FDP;
    check(d2 > d1, "Wong-Reece F_DP monotone in s", d2 - d1, 1);
  }
  // 6) DTM bilinear soil mixing: rigid node must not dilute soil parameters
  {
    Soil soil = soil_marssim_table2(), rock = soil; rock.soil = 0; rock.kc = 0; rock.kphi = 0;
    Soil mix = DTM::mix({{soil, 0.5}, {rock, 0.5}});
    check(near(mix.kphi, soil.kphi, 1e-9), "DTM::mix keeps soil kphi", mix.kphi, soil.kphi);
    check(near(mix.soil, 0.5, 1e-9), "DTM::mix soil fraction", mix.soil, 0.5);
  }
  // 7) reverse driving: wheel spinning backwards faster than it moves must pull backwards (FDP<0),
  //    and the filtered slip must not flip sign when the driving direction flips (v2 bug)
  {
    DTM flat; flat.nx = 2; flat.ny = 2; flat.x0 = -10; flat.y0 = -10; flat.res = 20; flat.H.assign(4, 0.0); flat.T.assign(4, 0);
    flat.types = {soil_marssim_table2()}; flat.type_names = {"soil"};
    WheelContact wc; wc.geom = WheelGeom(); ModelOpts mo; ContactOpts co;
    WheelContactInput in; in.center = Vec3(0, 0, 0.14 - 0.02); in.axle_left = Vec3(0, 1, 0);
    in.vel = Vec3(-0.010, 0, 0); in.omega = -0.10;   // reverse, slipping (|w r| = 0.014 > |v|)
    WheelContactOutput o; for (int k = 0; k < 20000; ++k) o = wc.step(in, flat, mo, co, 1e-3);
    check(o.force.x < 0, "reverse driving with slip pulls backwards (Fx<0)", o.force.x, -1);
    check(o.s > 0.2 && o.s < 0.35, "reverse slip in driving-direction convention ~0.29", o.s, 0.286);
  }

  // 9) omni wheel / firm ground (Janosi patch law, rolling resistances, roller angle), DC motor
  {
    check(near(janosi_patch_ratio(0.001, 0.01, 0.002), 0.0025, 1e-3), "Janosi patch: small slip is linear sL/2K", janosi_patch_ratio(0.001, 0.01, 0.002), 0.0025);
    check(janosi_patch_ratio(1.0, 0.01, 0.0005) > 0.94, "Janosi patch: large slip saturates to mu N", janosi_patch_ratio(1.0, 0.01, 0.0005), 0.95);
    FirmSurface S; S.phi = std::atan(0.8); S.K = 0.002; S.f_roll = 0.03; S.f_lat = 0.2; S.kn = 2e5; S.cn = 0;
    OmniOpts o; OmniContact c; c.geom.r = 0.04;
    OmniIn in; in.center = Vec3(0, 0, 0.04 - 36.0 / 2e5); in.axle = Vec3(0, 1, 0);  // e_x = y x z = +x
    // free rolling along e_x: no traction, rolling moment opposes the spin
    in.vel = Vec3(0.1, 0, 0); in.omega = 0.1 / 0.04;
    OmniOut r; for (int k = 0; k < 400; ++k) r = c.step(in, S, o, 1e-3);
    check(near(r.N, 36.0, 1e-6), "omni: penalty normal force", r.N, 36.0);
    check(std::fabs(r.H) < 1e-6, "omni: free rolling -> no traction", r.H, 0.0);
    { double e = -0.03 * 36 * 0.04 * 0.1 / std::sqrt(0.01 + o.v_eps * o.v_eps);
    check(near(r.torque.y, e, 1e-3), "omni: rolling-resistance moment -f_roll N r", r.torque.y, e); }
    // pure motion along the axle: rollers roll, force = -f_lat N, no traction
    OmniContact c2; c2.geom.r = 0.04; in.vel = Vec3(0, 0.1, 0); in.omega = 0;
    for (int k = 0; k < 400; ++k) r = c2.step(in, S, o, 1e-3);
    { double e = -0.2 * 36 * 0.1 / std::sqrt(0.01 + o.v_eps * o.v_eps);
    check(near(r.force.y, e, 1e-3), "omni: lateral (roller) resistance -f_lat N", r.force.y, e); }
    check(std::fabs(r.force.x) < 1e-6, "omni: lateral motion gives no traction", r.force.x, 0.0);
    // driving with 20% slip, oblique motion: the slip that shears the patch is (Vd-Vg)/|v_c|
    OmniContact c3; c3.geom.r = 0.04; in.vel = Vec3(0.08, 0.06, 0); in.omega = 0.1 / 0.04;
    for (int k = 0; k < 2000; ++k) r = c3.step(in, S, o, 1e-3);
    check(near(r.s, 0.02 / 0.1, 1e-3), "omni: patch slip normalised by |v_contact|", r.s, 0.2);
    // ground-element normalisation: rim 0.1 m/s along x, hub 0.05 m/s along x -> s = 0.05/0.05 = 1
    { OmniOpts og; og.slip_norm = 1; OmniContact c5; c5.geom.r = 0.04; OmniIn g5 = in;
      g5.vel = Vec3(0.05, 0, 0); g5.omega = 0.1 / 0.04;
      for (int k = 0; k < 2000; ++k) r = c5.step(g5, S, og, 1e-3);
      check(near(r.s, 1.0, 1e-3), "omni: ground-element slip (Vd-Vg)/|v_g|", r.s, 1.0);
      OmniOpts ow; OmniContact c6; c6.geom.r = 0.04;
      for (int k = 0; k < 2000; ++k) r = c6.step(g5, S, ow, 1e-3);
      check(near(r.s, 0.5, 1e-3), "omni: wheel-element slip (Vd-Vg)/|Vd|", r.s, 0.5); }
    // mecanum (45 deg): pure lateral motion with locked wheel -> traction along the roller axis
    OmniContact c4; c4.geom.r = 0.04; c4.geom.gamma = deg2rad(45.0); in.vel = Vec3(0, 0.1, 0); in.omega = 0;
    for (int k = 0; k < 400; ++k) r = c4.step(in, S, o, 1e-3);
    check(r.force.y < -0.3 * 36, "mecanum: locked wheel resists lateral motion strongly", r.force.y, -0.3 * 36);
    // mecanum rolling straight (w r = v): the rollers stand still -> no roller resistance, and the contact
    // costs only the wheel rolling resistance (no torque from the free-direction force)
    { FirmSurface Sm = S; Sm.mu_rb = 0.05;
      OmniContact c7; c7.geom.r = 0.04; c7.geom.gamma = deg2rad(45.0); OmniIn g7 = in; g7.vel = Vec3(0.1, 0, 0); g7.omega = 0.1 / 0.04;
      for (int k = 0; k < 2000; ++k) r = c7.step(g7, Sm, o, 1e-3);
      check(std::fabs(r.F_free) < 1e-6, "mecanum: straight rolling, rollers still (no roller force)", r.F_free, 0.0);
      // sideways on a mecanum wheel without slip (hub 0.1 m/s along the axle, rim +0.1 m/s: the rigid contact point
      // moves along e_f = (-1, 1)/sqrt2): roller speed 0.1*sqrt(2), no traction
      OmniContact c8; c8.geom.r = 0.04; c8.geom.gamma = deg2rad(45.0); OmniIn g8 = in; g8.vel = Vec3(0, 0.1, 0); g8.omega = 0.1 / 0.04;
      for (int k = 0; k < 2000; ++k) r = c8.step(g8, Sm, o, 1e-3);
      const double uf = 0.1 * std::sqrt(2.0), un = std::sqrt(0.01 + uf * uf + o.v_eps * o.v_eps);
      const double e8 = -(Sm.f_lat * 36 * uf / un + Sm.mu_rb * 36 * std::tanh(uf / o.v_eps));
      check(near(r.F_free, e8, 2e-2), "mecanum: sideways, roller speed v*sqrt(2) (contact slip along e_f)", r.F_free, e8); }
    // tyre lateral grip ratio: pure side slip -> side force scales with lat_grip, traction untouched
    { FirmSurface St = S; OmniContact t1, t2; t1.geom.r = t2.geom.r = 0.1; t1.geom.type = t2.geom.type = 1; t2.geom.lat_grip = 0.5;
      OmniIn gi = in; gi.center = Vec3(0, 0, 0.1 - 36.0 / 2e5); gi.vel = Vec3(0.1, 0.05, 0); gi.omega = 0.1 / 0.1;
      OmniOut a1, a2; for (int k = 0; k < 2000; ++k) { a1 = t1.step(gi, St, o, 1e-3); a2 = t2.step(gi, St, o, 1e-3); }
      check(near(a2.F_free, 0.5 * a1.F_free, 1e-6) && std::fabs(a1.F_free) > 1.0, "tyre: lateral grip ratio scales the side force", a2.F_free / a1.F_free, 0.5);
      check(near(a2.H, a1.H, 1e-9), "tyre: lateral grip ratio leaves the traction", a2.H, a1.H); }
    // DC motor + P controller: steady speed error e = (R I + Vd)/(Ce + kp) at constant load
    MotorParams mp; DCMotor m; double w = 0, Jw = 2e-3, load = 0.5, I = 0;
    for (int k = 0; k < 4000; ++k) { MotorOut mo = m.step(mp, 2.5, w, 1e-3); w += 1e-3 * (mo.tau_wheel - load) / Jw; I = mo.I; }
    double e_exp = (mp.R * I + mp.Vd) / (mp.Ce + mp.kp), e_got = (2.5 - w) * mp.gear;
    check(near(e_got, e_exp, 2e-2), "motor: P-controller steady error (motor rad/s)", e_got, e_exp);
    double I_exp = load / (mp.eta * mp.gear * mp.kt) + mp.I0 * std::tanh(w * mp.gear / mp.w_eps);
    check(near(I, I_exp, 1e-2), "motor: current = load/(eta g kt) + I0", I, I_exp);
  }
  {
    // Bekker deformable layer: LETE sand (Wong), wheel r = 0.04, b = 0.03, N = 36 N
    FirmSurface S; S.kc = 102e3; S.kphi = 5301e3; S.n_bek = 0.793; S.b_contact = 0.03; S.phi = deg2rad(31.1); S.K = 0.012;
    const double ks = 102e3 + 0.03 * 5301e3;
    const double z_exp = std::pow(3 * 36.0 / ((3 - 0.793) * ks * std::sqrt(0.08)), 2.0 / (2 * 0.793 + 1));
    const double z = OmniContact::bekker_sinkage(S, 36.0, 0.04);
    check(near(z, z_exp, 1e-9) && z > 0.003 && z < 0.004, "Bekker: rigid-wheel sinkage (LETE sand, 36 N)", z, z_exp);
    S.layer = 0.002;
    check(near(OmniContact::bekker_sinkage(S, 36.0, 0.04), 0.002, 1e-12), "Bekker: sinkage limited by layer thickness", OmniContact::bekker_sinkage(S, 36.0, 0.04), 0.002);
    S.layer = 1e9;
    // rolling along +x at 0.1 m/s without slip: settles in the rut, compaction resistance b(kc/b+kphi) z^(n+1)/(n+1)
    OmniContact c; c.geom.r = 0.04; OmniOpts o; o.slip_norm = 0; S.kn = 2e5; S.cn = 300; S.f_roll = 0; S.f_lat = 0;
    OmniIn in; in.axle = Vec3(0, 1, 0); in.vel = Vec3(0.1, 0, 0); in.omega = 0.1 / 0.04;
    OmniOut r;
    for (int k = 0; k < 3000; ++k) { in.center = Vec3(0, 0, 0.04 - c.z_filt - 36.0 / 2e5); r = c.step(in, S, o, 1e-3); }
    const double Rc = ks * std::pow(r.z, 1.793) / 1.793;
    check(near(r.z, z_exp, 2e-5), "Bekker: wheel settles to the static sinkage", r.z, z_exp);
    check(near(r.R_comp, Rc, 0.02 * Rc), "Bekker: compaction resistance", r.R_comp, Rc);
    // R_c acts on the hub without an axle moment (normal stresses pass through the axle): a wheel rolling without
    // slip feels -R_c along x, and the contact moment about the axle is zero (no traction, no rolling resistance)
    check(near(r.force.x, -r.R_comp, 0.02 * Rc), "Bekker: compaction resistance acts on the hub", r.force.x, -r.R_comp);
    check(std::fabs(r.torque.dot(Vec3(0, 1, 0))) < 0.02 * 0.04 * Rc, "Bekker: no axle moment from compaction (motor pays via H)",
          r.torque.dot(Vec3(0, 1, 0)), 0.0);
  }
  {
    // surface random field: zero mean, unit variance, correlation exp(-1/2) at one correlation length
    RandomField2D g; g.init(3, 0.02);
    double s1 = 0, s2 = 0, c = 0; int n = 0;
    for (int i = 0; i < 400; ++i) for (int j = 0; j < 40; ++j) {
      double x = i * 0.05, y = j * 0.37, a = g(x, y), b = g(x + 0.02, y);
      s1 += a; s2 += a * a; c += a * b; ++n;
    }
    check(std::fabs(s1 / n) < 0.1, "field: zero mean", s1 / n, 0.0);
    check(near(s2 / n, 1.0, 0.15), "field: unit variance", s2 / n, 1.0);
    check(near(c / s2, std::exp(-0.5), 0.1), "field: correlation exp(-1/2) at one length", c / s2, std::exp(-0.5));
  }

  // ---------------- vehicle mode: kinematics, ordinary wheel side force, IMU statistics, reference path
  {
    auto make = [](const std::string &robot_sec, const std::string &wheels, VehicleLogic &L) {
      std::ofstream("/tmp/tm_test_path.csv") << "s,x,y,yaw,kappa\n0,0,0,0,0\n1,1,0,0,0\n2,2,0,0,0\n";
      std::ofstream f("/tmp/tm_test_vehicle.ini");
      f << "[robot]\ntype = vehicle\n" << robot_sec << wheels
        << "[surface gray]\nmu = 1.0\nK = 0.005\n[vehicle]\nsurface = gray\n[trajectory]\npath_file = /tmp/tm_test_path.csv\nspeed = 0.5\naccel = 1\n"
        << "[imu]\ngyro_sigma = 0.01\ngyro_bias = 0\ngyro_rw = 0\nacc_sigma = 0\nacc_bias = 0\nacc_rw = 0\n";
      f.close();
      Config c; c.load("/tmp/tm_test_vehicle.ini"); std::string err;
      const bool ok = L.configure(c, "/tmp/tm_test_vehicle_log.csv", "/tmp/tm_test_vehicle.done", err);
      if (!ok) std::printf("configure: %s\n", err.c_str());
      return ok;
    };
    // Robotino: rows = OmniLogic wheel_set (-sin th vx + cos th vy + R wz)/r
    {
      VehicleLogic L; std::string W;
      const double th[3] = {60, 180, 300}, R = 0.125, r = 0.04;
      for (int k = 0; k < 3; ++k) {
        const double a = deg2rad(th[k]); char b[256];
        std::snprintf(b, sizeof b, "[wheel %d]\nx = %.9f\ny = %.9f\nheading_deg = %.6f\nr = 0.04\ntype = omni\ngamma_deg = 90\n", k + 1, R * std::cos(a), R * std::sin(a), th[k] + 90);
        W += b;
      }
      make("drive = holonomic\n", W, L);
      double tw[3] = {0.13, -0.07, 0.4}; std::vector<double> w; L.twist_to_wheels(tw, w); double e = 0;
      for (int k = 0; k < 3; ++k) { const double a = deg2rad(th[k]); e = std::max(e, std::fabs(w[k] - (-std::sin(a) * tw[0] + std::cos(a) * tw[1] + R * tw[2]) / r)); }
      check(e < 1e-6, "vehicle: Robotino kinematics = OmniLogic wheel_set", e, 0.0);
    }
    // youBot mecanum (X configuration): w_FL = (vx - vy - (lx+ly) wz)/r, w_FR = (vx + vy + ..)/r, w_RL = (vx + vy - ..)/r, w_RR = (vx - vy + ..)/r
    {
      VehicleLogic L; const double lx = 0.235, ly = 0.15, r = 0.0475;
      const char *nm[4] = {"FL", "FR", "RL", "RR"}; const double px[4] = {lx, lx, -lx, -lx}, py[4] = {ly, -ly, ly, -ly}, g[4] = {-45, 45, 45, -45};
      std::string W;
      for (int k = 0; k < 4; ++k) { char b[256]; std::snprintf(b, sizeof b, "[wheel %s]\nx = %g\ny = %g\nheading_deg = 0\nr = %g\ntype = mecanum\ngamma_deg = %g\n", nm[k], px[k], py[k], r, g[k]); W += b; }
      make("drive = holonomic\n", W, L);
      double tw[3] = {0.3, 0.2, -0.5}; std::vector<double> w; L.twist_to_wheels(tw, w);
      const double l = lx + ly;
      const double ex[4] = {(tw[0] - tw[1] - l * tw[2]) / r, (tw[0] + tw[1] + l * tw[2]) / r, (tw[0] + tw[1] - l * tw[2]) / r, (tw[0] - tw[1] + l * tw[2]) / r};
      double e = 0; for (int k = 0; k < 4; ++k) e = std::max(e, std::fabs(w[k] - ex[k]));
      check(e < 1e-9, "vehicle: youBot mecanum inverse kinematics", e, 0.0);
      // odometry pseudo-inverse recovers the twist
      double rec[3] = {0, 0, 0}; for (int a = 0; a < 3; ++a) for (int k = 0; k < 4; ++k) rec[a] += L.Jp[a][k] * w[k];
      check(std::fabs(rec[0] - tw[0]) + std::fabs(rec[1] - tw[1]) + std::fabs(rec[2] - tw[2]) < 1e-6, "vehicle: odometry pseudo-inverse recovers the twist", rec[2], tw[2]);
    }
    // skid steer: yaw command scaled by 1/odom_yaw_scale, odometry by odom_yaw_scale
    {
      VehicleLogic L; std::string W = "[wheel L]\nx = 0\ny = 0.25\nr = 0.1\n[wheel R]\nx = 0\ny = -0.25\nr = 0.1\n";
      make("drive = diff\nodom_yaw_scale = 0.5\n", W, L);
      double tw[3] = {0.0, 0.0, 1.0}; std::vector<double> w; L.twist_to_wheels(tw, w);
      check(near(w[1], 0.25 * 2.0 / 0.1, 1e-9) && near(w[0], -w[1], 1e-9), "vehicle: skid-steer yaw command uses the effective track", w[1], 5.0);
    }
    // ordinary wheel: side slip gives a restoring side force; shares the friction limit with traction
    {
      OmniContact c; c.geom.r = 0.1; c.geom.type = 1; c.geom.b = 0.05; c.geom.patch = 0.05;
      FirmSurface S; S.phi = std::atan(1.0); S.K = 0.005; S.kn = 2e5; S.cn = 0;
      OmniOpts o; o.slip_relax_len = 0; o.slip_norm = 1;
      OmniIn in; in.center = Vec3(0, 0, 0.1 - 36.0 / 2e5); in.axle = Vec3(0, 1, 0); in.vel = Vec3(0.5, 0.05, 0); in.omega = 5.0;
      OmniOut r = c.step(in, S, o, 0.001);
      check(r.force.y < -1.0, "tyre: side slip (+y) gives a side force towards -y", r.force.y, -1.0);
      in.omega = 7.5; OmniOut r2 = c.step(in, S, o, 0.001);
      check(std::fabs(r2.force.y) < std::fabs(r.force.y), "tyre: traction reduces the side force (friction ellipse)", std::fabs(r2.force.y), std::fabs(r.force.y));
      check(std::hypot(r2.force.x, r2.force.y) <= 1.0 * r2.N * 1.0001, "tyre: resultant force within mu N", std::hypot(r2.force.x, r2.force.y), r2.N);
    }
    // IMU: white noise sigma per sample and zero mean for a resting body
    {
      Imu im; ImuParams p; p.rate = 100; p.g_sigma = 0.01; p.g_bias = 0; p.g_rw = 0; p.a_sigma = 0; p.a_bias = 0; p.a_rw = 0; im.init(p);
      double s1 = 0, s2 = 0; int n = 0;
      for (int k = 0; k < 200000; ++k) { im.accumulate(k * 0.001, 0.001, Vec3(0, 0, 0.2), Vec3(0, 0, 9.81)); if (im.fresh) { const double e = im.gyro.z - 0.2; s1 += e; s2 += e * e; ++n; } }
      check(n == 20000, "imu: sample count at 100 Hz", n, 20000);
      check(std::fabs(s1 / n) < 5e-4 && near(std::sqrt(s2 / n), 0.01, 0.03), "imu: gyro noise sigma per sample", std::sqrt(s2 / n), 0.01);
    }
    // caster (type 2): no traction, resistance f_roll N against the motion, no torque on the wheel
    {
      OmniContact c; c.geom.r = 0.035; c.geom.type = 2; c.geom.b = 0.02; c.geom.patch = 0.02;
      FirmSurface S; S.phi = std::atan(1.0); S.K = 0.005; S.kn = 2e5; S.cn = 0; S.f_roll = 0.05;
      OmniOpts o;
      OmniIn in; in.center = Vec3(0, 0, 0.035 - 20.0 / 2e5); in.axle = Vec3(0, 1, 0); in.vel = Vec3(0.3, 0.4, 0); in.omega = 30.0;
      OmniOut r = c.step(in, S, o, 0.001);
      const double Fh = std::hypot(r.force.x, r.force.y);
      check(near(r.N, 20.0, 1e-6), "caster: normal force from the penalty contact", r.N, 20.0);
      check(near(Fh, 0.05 * r.N, 1e-3), "caster: horizontal force = f_roll N (spin ignored)", Fh, 0.05 * r.N);
      check(near(r.force.x / Fh, -0.6, 1e-3) && near(r.force.y / Fh, -0.8, 1e-3), "caster: force opposite to the velocity", r.force.y / Fh, -0.8);
      check(r.torque.norm() < 1e-12, "caster: no torque on the wheel", r.torque.norm(), 0.0);
    }
    // caster with its own rotation: the floor spins it up to rolling; no force once it rolls; swivel aligns
    {
      OmniContact c; c.geom.r = 0.02; c.geom.type = 2; c.geom.b = 0.01; c.geom.Jc = 1e-5; c.geom.swivel = true;
      FirmSurface S; S.phi = std::atan(1.0); S.kn = 2e5; S.cn = 0; S.f_roll = 0.0;
      OmniOpts o; o.caster_tau = 0.01;
      OmniIn in; in.center = Vec3(0, 0, 0.02 - 10.0 / 2e5); in.axle = Vec3(0, 1, 0); in.vel = Vec3(0.2, 0, 0);
      double w = 0; for (int k = 0; k < 200; ++k) { in.omega = w; OmniOut r = c.step(in, S, o, 0.001); w += 0.001 * r.torque.dot(in.axle) / c.geom.Jc; }
      check(near(w, 0.2 / 0.02, 1e-3), "caster: floor spins it up to rolling (w = v / r)", w, 10.0);
      in.omega = w; OmniOut r = c.step(in, S, o, 0.001);
      check(std::fabs(r.force.x) < 1e-6, "caster: no drag once it rolls (f_roll = 0)", r.force.x, 0.0);
      in.axle = Vec3(-std::sin(0.3), std::cos(0.3), 0); in.omega = 10.0; r = c.step(in, S, o, 0.001);
      check(r.torque.z < 0, "caster: swivel torque turns the rolling direction towards the motion", r.torque.z, -1.0);
    }
    // odometer: pseudo-inverse twist, yaw scale, midpoint integration of a circle
    {
      Odometer od; std::vector<std::array<double, 3>> J = {{1 / 0.05, 0, -0.2 / 0.05}, {1 / 0.05, 0, 0.2 / 0.05}, {0, 0, 0}};   // diff + caster
      od.init(J, 0.5);
      const double v = 0.3, w = 0.6; std::vector<double> wh = {(v - 0.2 * w) / 0.05, (v + 0.2 * w) / 0.05, 0.0};
      double tw[3]; od.twist(wh, tw);
      check(near(tw[0], v, 1e-9) && near(tw[2], 0.5 * w, 1e-9), "odometer: twist from wheel speeds (caster row 0, yaw scale 0.5)", tw[2], 0.3);
      double p[3] = {0, 0, 0}; const double T = 2 * kPi / w; const int n = 2000;
      for (int k = 0; k < n; ++k) { double t2[3] = {v, 0, w}; Odometer::integrate(p, t2, w, T / n); }
      check(std::hypot(p[0], p[1]) < 1e-6, "odometer: full circle closes (midpoint rule)", std::hypot(p[0], p[1]), 0.0);
    }
    // IMU true values: mean of the inputs over the sample, without noise and bias
    {
      Imu im; ImuParams p; p.rate = 50; im.init(p);
      Vec3 last; bool got = false;
      for (int k = 0; k < 40; ++k) { im.accumulate(k * 0.001, 0.001, Vec3(0, 0, 0.001 * k), Vec3(1, 0, 9.81)); if (im.fresh) { last = im.gyro_true; got = true; } }
      check(got && near(last.z, 0.001 * 10.5, 1e-6) && near(im.acc_true.z, 9.81, 1e-12), "imu: true gyro = mean of the inputs since the last sample", last.z, 0.0105);
      check(kinematic_slip(0.8, 1.0) > 0.199 && kinematic_slip(0.8, 1.0) < 0.201 && kinematic_slip(0.0, 0.0) == 0.0, "full log: kinematic slip 1 - w_eff / w", kinematic_slip(0.8, 1.0), 0.2);
    }
    // reference path + speed profile
    {
      RefPath P; std::string err; P.load("/tmp/tm_test_path.csv", err);
      double x, y, yaw, k; P.at(1.5, x, y, yaw, k);
      check(near(x, 1.5, 1e-9) && near(P.length(), 2.0, 1e-12), "path: interpolation along s", x, 1.5);
      SpeedProfile sp; sp.init(2.0, 0.5, 1.0);
      check(near(sp.T, 2.0 / 0.5 + 0.5, 1e-9), "speed profile: T = L/v + v/a (trapezoid)", sp.T, 4.5);
    }
  }
  std::printf("%s (%d failures)\n", fails ? "TESTS FAILED" : "ALL TESTS PASSED", fails);
  return fails ? 1 : 0;
}
