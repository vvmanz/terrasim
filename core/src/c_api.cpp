// C API over terramech.hpp, used from Python via ctypes (tools/terramech.py).
#include "terramech/terramech.hpp"
using namespace terramech;

static WheelGeom mk_wheel(const double *w) { WheelGeom g; g.r = w[0]; g.b = w[1]; g.h = w[2]; g.lambda = w[3]; g.Rj = w[4]; return g; }
// soil: kc kphi n0 n1 c phi_rad K Ky a0 a1 soil
static Soil mk_soil(const double *s) {
  Soil o; o.kc = s[0]; o.kphi = s[1]; o.n0 = s[2]; o.n1 = s[3]; o.c = s[4]; o.phi = s[5]; o.K = s[6]; o.Ky = s[7];
  o.a0 = s[8]; o.a1 = s[9]; o.soil = s[10]; return o;
}
// opts: model cp_form cp1 cp2 cp3 cM W_bar n_integ sN_lo sN_hi
static ModelOpts mk_opts(const double *m) {
  ModelOpts o; o.model = (int)m[0]; o.cp_form = (int)m[1]; o.cp1 = m[2]; o.cp2 = m[3]; o.cp3 = m[4]; o.cM = m[5];
  o.W_bar = m[6]; o.n_integ = (int)m[7]; o.sN_lo = m[8]; o.sN_hi = m[9]; return o;
}

extern "C" {
// out: contact FN FDP FS MR MO MS theta1 thetam sigma_m tau_m jm N
int tm_forces(const double *wheel, const double *soil, const double *opts, double s, double z, double beta, double *out) {
  Forces f = wheel_forces(mk_wheel(wheel), mk_soil(soil), mk_opts(opts), s, z, beta);
  double v[13] = {(double)f.contact, f.FN, f.FDP, f.FS, f.MR, f.MO, f.MS, f.theta1, f.thetam, f.sigma_m, f.tau_m, f.jm, f.N};
  for (int i = 0; i < 13; ++i) out[i] = v[i];
  return 0;
}
double tm_sinkage(const double *wheel, const double *soil, const double *opts, double s, double W) {
  return solve_sinkage(mk_wheel(wheel), mk_soil(soil), mk_opts(opts), s, W);
}
double tm_slip(double vx, double omega, double rs) { return slip_ratio(vx, omega, rs); }
int tm_version() { return 1; }
}
