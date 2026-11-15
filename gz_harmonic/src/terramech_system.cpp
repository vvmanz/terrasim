// gz-sim 8 (Gazebo Harmonic) system plugin: MarsSim-style terramechanics for wheeled robots.
//   <plugin filename="terramech_gz_system" name="terramech::TerramechSystem">
//     <config>/abs/path/experiment.ini</config>
//   </plugin>
// Environment overrides: TM_CONFIG, TM_LOG, TM_DONE.
#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/ParentLinkName.hh>
#include <gz/common/Console.hh>

#include "terramech/sim_logic.hpp"

namespace terramech {

static Vec3 V(const gz::math::Vector3d &v) { return Vec3(v.X(), v.Y(), v.Z()); }
static gz::math::Vector3d G(const Vec3 &v) { return gz::math::Vector3d(v.x, v.y, v.z); }

class TerramechSystem : public gz::sim::System,
                        public gz::sim::ISystemConfigure,
                        public gz::sim::ISystemPreUpdate {
 public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override {
    model_ = gz::sim::Model(entity);
    std::string ini = sdf->HasElement("config") ? sdf->Get<std::string>("config") : "";
    std::string err;
    if (!logic_.configure(ini, err)) { gzerr << err << std::endl; return; }
    for (auto &w : logic_.wheels) {
      gz::sim::Entity le = model_.LinkByName(ecm, w.link);
      gz::sim::Entity je = model_.JointByName(ecm, w.joint);
      if (le == gz::sim::kNullEntity || je == gz::sim::kNullEntity) {
        gzerr << "terramech: wheel " << w.label << ": link '" << w.link << "' or joint '" << w.joint << "' not found" << std::endl; return; }
      gz::sim::Link l(le); l.EnableVelocityChecks(ecm, true);
      gz::sim::Joint j(je); j.EnableVelocityCheck(ecm, true); j.EnablePositionCheck(ecm, true);
      if (!ecm.Component<gz::sim::components::JointTransmittedWrench>(je))
        ecm.CreateComponent(je, gz::sim::components::JointTransmittedWrench());
      gz::sim::Entity pe = gz::sim::kNullEntity;
      auto pname = ecm.Component<gz::sim::components::ParentLinkName>(je);
      if (pname) pe = model_.LinkByName(ecm, pname->Data());
      if (pe != gz::sim::kNullEntity) gz::sim::Link(pe).EnableVelocityChecks(ecm, true);
      links_.push_back(le); joints_.push_back(je); parents_.push_back(pe);
    }
    chassis_ = model_.LinkByName(ecm, logic_.chassis_link);
    if (chassis_ != gz::sim::kNullEntity) gz::sim::Link(chassis_).EnableVelocityChecks(ecm, true);
    auto joint = [&](const std::string &n) {
      gz::sim::Entity e = n.empty() ? gz::sim::kNullEntity : model_.JointByName(ecm, n);
      if (e != gz::sim::kNullEntity) { gz::sim::Joint j(e); j.EnableVelocityCheck(ecm, true); j.EnablePositionCheck(ecm, true); }
      return e;
    };
    if (logic_.diff_on) { diffL_ = joint(logic_.diff_left); diffR_ = joint(logic_.diff_right); }
    if (logic_.bench_on) bench_ = joint(logic_.bench_joint);
    io_.resize(links_.size());
    ok_ = true;
    gzmsg << "terramech: loaded " << links_.size() << " wheels, DTM " << logic_.dtm.nx << "x" << logic_.dtm.ny
          << ", log -> " << logic_.log_file << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override {
    if (!ok_ || info.paused) return;
    const double t = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    for (size_t i = 0; i < links_.size(); ++i) {
      auto &w = io_[i]; auto &cfg = logic_.wheels[i];
      gz::sim::Link l(links_[i]);
      // worldPose() composes the Pose chain (valid from the very first step); the WorldPose component
      // is default (zero) until the physics system has run once.
      std::optional<gz::math::Pose3d> pose = gz::sim::worldPose(links_[i], ecm);
      auto v = l.WorldLinearVelocity(ecm);
      auto av = l.WorldAngularVelocity(ecm);
      if (!v || !av) continue;
      w.center = V(pose->Pos());
      w.vel = V(*v);
      w.angvel = V(*av);
      w.axle_left = V(pose->Rot().RotateVector(G(cfg.axle_local)));
      gz::sim::Joint j(joints_[i]);
      auto jv = j.Velocity(ecm);
      w.joint_vel = (jv && !jv->empty()) ? cfg.axle_sign * (*jv)[0] : 0.0;
      w.parent_x = Vec3(1, 0, 0);
      if (parents_[i] != gz::sim::kNullEntity) {
        gz::math::Pose3d pp = gz::sim::worldPose(parents_[i], ecm);
        w.parent_x = V(pp.Rot().RotateVector(gz::math::Vector3d(1, 0, 0)));
      }
      // JointTransmittedWrench: wrench applied by the parent on the child, joint (= wheel) frame.
      // The force the wheel exerts on its carrier is the negative of it.
      auto tw = ecm.Component<gz::sim::components::JointTransmittedWrench>(joints_[i]);
      if (tw) {
        const auto &f = tw->Data().force();
        w.hub_force = V(pose->Rot().RotateVector(gz::math::Vector3d(-f.x(), -f.y(), -f.z())));
        w.has_hub = hub_sign_ok_;
      }
      ready_ = true;
    }
    if (!ready_) return;
    double ch[6] = {0, 0, 0, 0, 0, 0};
    if (chassis_ != gz::sim::kNullEntity) {
      gz::math::Pose3d p = gz::sim::worldPose(chassis_, ecm);
      ch[0] = p.Pos().X(); ch[1] = p.Pos().Y(); ch[2] = p.Pos().Z();
      auto e = p.Rot().Euler(); ch[3] = e.X(); ch[4] = e.Y(); ch[5] = e.Z();
      logic_.chassis_q[0] = p.Rot().W(); logic_.chassis_q[1] = p.Rot().X(); logic_.chassis_q[2] = p.Rot().Y(); logic_.chassis_q[3] = p.Rot().Z();
      logic_.has_chassis_q = true;
      gz::sim::Link cl(chassis_);
      auto lv = cl.WorldLinearVelocity(ecm); auto av = cl.WorldAngularVelocity(ecm);
      if (lv && av) {
        logic_.chassis_vel[0] = lv->X(); logic_.chassis_vel[1] = lv->Y(); logic_.chassis_vel[2] = lv->Z();
        logic_.chassis_vel[3] = av->X(); logic_.chassis_vel[4] = av->Y(); logic_.chassis_vel[5] = av->Z(); logic_.has_chassis_vel = true;
      }
    }
    auto jio = [&](gz::sim::Entity e) {
      terramech::JointIO r; gz::sim::Joint j(e);
      auto p = j.Position(ecm); auto v = j.Velocity(ecm);
      if (p && !p->empty()) r.pos = (*p)[0];
      if (v && !v->empty()) r.vel = (*v)[0];
      return r;
    };
    terramech::JointIO jl, jr, jb;
    bool diff = diffL_ != gz::sim::kNullEntity && diffR_ != gz::sim::kNullEntity;
    if (diff) { jl = jio(diffL_); jr = jio(diffR_); }
    if (bench_ != gz::sim::kNullEntity) jb = jio(bench_);
    double diff_tau = 0, bench_f = 0;
    logic_.step(t, dt, io_, ch, diff ? &jl : nullptr, diff ? &jr : nullptr, diff_tau,
                bench_ != gz::sim::kNullEntity ? &jb : nullptr, bench_f);
    for (size_t i = 0; i < links_.size(); ++i) {
      auto &w = io_[i];
      if (w.force.norm() > 0 || w.torque.norm() > 0) gz::sim::Link(links_[i]).AddWorldWrench(ecm, G(w.force), G(w.torque));
      if (logic_.drive_on) gz::sim::Joint(joints_[i]).SetForce(ecm, {logic_.wheels[i].axle_sign * w.motor_torque});
    }
    // body-ground contact (vehicle mode): force at the chassis-link origin, torque about it (gz-sim 8 Link::AddWorldWrench)
    if (chassis_ != gz::sim::kNullEntity && logic_.body_on() && (logic_.body_force.norm() > 0 || logic_.body_torque.norm() > 0))
      gz::sim::Link(chassis_).AddWorldWrench(ecm, G(logic_.body_force), G(logic_.body_torque));
    if (diff) { gz::sim::Joint(diffL_).SetForce(ecm, {diff_tau}); gz::sim::Joint(diffR_).SetForce(ecm, {diff_tau}); }
    if (bench_ != gz::sim::kNullEntity) gz::sim::Joint(bench_).SetForce(ecm, {bench_f});
  }

 private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  std::vector<gz::sim::Entity> links_, joints_, parents_;
  gz::sim::Entity chassis_{gz::sim::kNullEntity}, diffL_{gz::sim::kNullEntity}, diffR_{gz::sim::kNullEntity}, bench_{gz::sim::kNullEntity};
  std::vector<terramech::WheelIO> io_;
  terramech::SimLogic logic_;
  bool ok_ = false, ready_ = false, hub_sign_ok_ = true;
};

}  // namespace terramech

GZ_ADD_PLUGIN(terramech::TerramechSystem, gz::sim::System, terramech::TerramechSystem::ISystemConfigure,
              terramech::TerramechSystem::ISystemPreUpdate)
