// Sole joint-force writer for the isolated Teacher simulation.
// No base wrench, pose servo, or post-startup position/velocity reset is used.
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <vector>
#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/SystemPluginInfo.hh>

namespace teacher_sim {
using A12 = std::array<double, 12>;
using Clock = std::chrono::steady_clock;

static std::string Quote(const std::string &s) {
  std::string out = "\"";
  for (unsigned char c : s) {
    if (c == '\\' || c == '"') { out += '\\'; out += c; }
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else if (c >= 0x20) out += c;
  }
  return out + "\"";
}

template <class T, size_t N>
static void Array(std::ostream &out, const std::array<T, N> &a) {
  out << '[';
  for (size_t i = 0; i < N; ++i) { if (i) out << ','; out << a[i]; }
  out << ']';
}

static void Vector(std::ostream &out, const gz::math::Vector3d &v) {
  out << '[' << v.X() << ',' << v.Y() << ',' << v.Z() << ']';
}
static void MessageVector(std::ostream &out, const gz::msgs::Vector3d &v) {
  out << '[' << v.x() << ',' << v.y() << ',' << v.z() << ']';
}

class TeacherActuator final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate {
 public:
  ~TeacherActuator() override { if (fd >= 0) ::close(fd); }

  void Configure(const gz::sim::Entity &entity,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &events) override {
    modelEntity = entity;
    eventManager = &events;
    gz::sim::Model model(entity);
    base = gz::sim::Link(model.LinkByName(ecm, "base_link"));
    if (!base.Valid(ecm)) throw std::runtime_error("Teacher base_link missing");
    base.EnableVelocityChecks(ecm);
    const auto inertia = ecm.Component<gz::sim::components::Inertial>(base.Entity());
    if (!inertia) throw std::runtime_error("Teacher base inertia missing");
    comOffset = inertia->Data().Pose().Pos();
    baseMass = inertia->Data().MassMatrix().Mass();
    kp = sdf->Get<double>("kp", 25.0).first;
    kd = sdf->Get<double>("kd", 0.5).first;
    effortLimit = sdf->Get<double>("effort_limit", 23.5).first;
    saturationEffort = sdf->Get<double>("saturation_effort", 23.5).first;
    velocityLimit = sdf->Get<double>("velocity_limit", 30.0).first;
    faultDamping = sdf->Get<double>("fault_damping", 2.0).first;
    expectedDt = sdf->Get<double>("expected_dt", 0.005).first;
    targetAbsBound = sdf->Get<double>("target_abs_bound", 10.0).first;
    decimation = sdf->Get<unsigned int>("decimation", 4).first;
    for (double x : {kp, kd, effortLimit, saturationEffort, velocityLimit,
                     faultDamping, expectedDt, targetAbsBound})
      if (!std::isfinite(x) || x <= 0) throw std::runtime_error("Invalid Teacher PD parameter");
    if (!decimation) throw std::runtime_error("Invalid Teacher decimation");
    if (sdf->HasElement("initial_q")) {
      std::istringstream values(sdf->Get<std::string>("initial_q"));
      for (double &q : target)
        if (!(values >> q) || !std::isfinite(q) || std::abs(q) > targetAbsBound)
          throw std::runtime_error("Teacher initial_q requires twelve finite angles");
      std::string extra;
      if (values >> extra) throw std::runtime_error("Teacher initial_q has extra values");
    }
    const char *socketEnv = std::getenv("TEACHER_SOCKET");
    const char *logEnv = std::getenv("TEACHER_ACTUATOR_LOG");
    if (!socketEnv || !*socketEnv || !logEnv || !*logEnv)
      throw std::runtime_error("TEACHER_SOCKET and TEACHER_ACTUATOR_LOG are required");
    socketPath = socketEnv;
    if (socketPath.size() >= sizeof(sockaddr_un::sun_path))
      throw std::runtime_error("Teacher UNIX socket path too long");
    log.open(logEnv, std::ios::out | std::ios::trunc);
    if (!log) throw std::runtime_error("Cannot open Teacher actuator evidence log");
    log << std::setprecision(17);
    for (size_t i = 0; i < 12; ++i) {
      joints[i] = gz::sim::Joint(model.JointByName(ecm, names[i]));
      if (!joints[i].Valid(ecm)) throw std::runtime_error("Teacher joint missing: " + names[i]);
      joints[i].EnablePositionCheck(ecm);
      joints[i].EnableVelocityCheck(ecm);
      const auto axes = joints[i].Axis(ecm);
      if (!axes || axes->size() != 1) throw std::runtime_error("Teacher revolute axis missing");
      hardLower[i] = (*axes)[0].Lower(); hardUpper[i] = (*axes)[0].Upper();
      // This reset occurs exactly once, before the first physics integration.
      joints[i].ResetPosition(ecm, {target[i]});
      joints[i].SetVelocityLimits(ecm, {{-velocityLimit, velocityLimit}});
      joints[i].SetEffortLimits(ecm, {{-effortLimit, effortLimit}});
    }
    const std::array<std::string, 5> linkNames = {"base_link", "rf_lower_leg_link",
        "lf_lower_leg_link", "rh_lower_leg_link", "lh_lower_leg_link"};
    for (size_t group = 0; group < linkNames.size(); ++group) {
      gz::sim::Link link(model.LinkByName(ecm, linkNames[group]));
      if (!link.Valid(ecm)) throw std::runtime_error("Teacher contact link missing");
      for (auto collision : link.Collisions(ecm)) {
        const auto name = ecm.Component<gz::sim::components::Name>(collision);
        if (group && (!name || name->Data().find("foot") == std::string::npos)) continue;
        contactEntities[group].push_back(collision);
        // Physics fills this component on each real integration, including
        // explicit empty contacts. Missing transport events are never inferred.
        if (!ecm.Component<gz::sim::components::ContactSensorData>(collision))
          ecm.CreateComponent(collision, gz::sim::components::ContactSensorData());
      }
      if (contactEntities[group].empty()) throw std::runtime_error("Teacher contact collision missing: " + linkNames[group]);
    }
    log << "{\"kind\":\"actuator_contract\",\"protocol\":\"native_le_float64_request64_response16_v1\","
        << "\"writer\":\"teacher_sim::TeacherActuator sole JointForceCmd writer\","
        << "\"state_source\":\"Gazebo native base link, base COM, joints and collision ContactSensorData\","
        << "\"torque_source\":\"clipped native JointForceCmd passed to physics, not hardware torque measurement\","
        << "\"base_mass_kg\":" << baseMass << ",\"base_com_offset_body_m\":";
    Vector(log, comOffset);
    log << ",\"kp\":" << kp << ",\"kd\":" << kd
        << ",\"effort_limit\":" << effortLimit << ",\"saturation_effort\":" << saturationEffort
        << ",\"velocity_limit\":" << velocityLimit << ",\"fault_damping\":" << faultDamping
        << ",\"expected_dt\":" << expectedDt << ",\"decimation\":" << decimation
        << ",\"initial_q\":";
    Array(log, target);
    log << ",\"joint_order\":[";
    for (size_t i = 0; i < names.size(); ++i) { if (i) log << ','; log << Quote(names[i]); }
    log << "],\"joint_position_reset_count\":12,\"body_pose_resets\":0}\n";
    log.flush();
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
      gz::sim::EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double time = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (!ownershipChecked) {
      ownershipChecked = true;
      if (!CheckOwnership(ecm)) {
        Latch(8, "multiple_joint_actuator_plugins", time);
        eventManager->Emit<gz::sim::events::Stop>();
        return;
      }
    }
    if (std::abs(dt - expectedDt) > 1e-9 || (lastTime >= 0 && time <= lastTime))
      Latch(7, "physics_timestep_or_clock_contract_violation", time);
    lastTime = time;
    // Allow the one-time reset and requested native measurement components to
    // reach Physics before constructing the first real observation.
    if (iteration++ == 0) {
      for (auto &joint : joints) joint.SetForce(ecm, {0.0});
      return;
    }
    A12 q{}, qd{};
    bool stateValid = true;
    for (size_t i = 0; i < 12; ++i) {
      auto p = joints[i].Position(ecm);
      auto v = joints[i].Velocity(ecm);
      if (!p || !v || p->size() != 1 || v->size() != 1 ||
          !std::isfinite((*p)[0]) || !std::isfinite((*v)[0])) { stateValid = false; continue; }
      q[i] = (*p)[0]; qd[i] = (*v)[0];
    }
    const auto pose = base.WorldPose(ecm);
    const auto originVelocity = base.WorldLinearVelocity(ecm);
    const auto comVelocity = base.WorldLinearVelocity(ecm, comOffset);
    const auto angularVelocity = base.WorldAngularVelocity(ecm);
    if (!pose || !originVelocity || !comVelocity || !angularVelocity) stateValid = false;
    else if (!pose->Pos().IsFinite() || !pose->Rot().IsFinite() || !originVelocity->IsFinite() ||
             !comVelocity->IsFinite() || !angularVelocity->IsFinite()) stateValid = false;
    if (!stateValid) Latch(6, "missing_or_invalid_native_physics_state", time);
    std::array<double, 5> contactCounts{};
    for (size_t group = 0; group < 5; ++group) {
      for (auto collision : contactEntities[group]) {
        const auto contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
        if (!contacts) { contactCounts[group] = -1; Latch(6, "missing_native_contact_component", time); break; }
        contactCounts[group] += contacts->Data().contact_size();
      }
    }
    gz::math::Vector3d originBody, comBody, omegaBody;
    std::array<double, 64> request{};
    request[0] = time;
    if (pose && originVelocity && comVelocity && angularVelocity) {
      originBody = pose->Rot().RotateVectorReverse(*originVelocity);
      comBody = pose->Rot().RotateVectorReverse(*comVelocity);
      omegaBody = pose->Rot().RotateVectorReverse(*angularVelocity);
      request[1] = pose->Pos().X(); request[2] = pose->Pos().Y(); request[3] = pose->Pos().Z();
      request[4] = pose->Rot().W(); request[5] = pose->Rot().X();
      request[6] = pose->Rot().Y(); request[7] = pose->Rot().Z();
      for (size_t i = 0; i < 3; ++i) { request[8+i] = comBody[i]; request[11+i] = omegaBody[i]; }
    }
    for (size_t i = 0; i < 12; ++i) {
      request[14+i] = q[i]; request[26+i] = qd[i]; request[38+i] = lastTorque[i];
    }
    for (size_t i = 0; i < 5; ++i) request[50+i] = contactCounts[i];
    request[55] = static_cast<double>(iteration-1); request[56] = fault;
    if (stateValid && (iteration-2) % decimation == 0) Exchange(request);
    A12 raw{}, lower{}, upper{};
    for (size_t i = 0; i < 12; ++i) {
      raw[i] = fault || terminating ? -faultDamping * qd[i] : kp * (target[i]-q[i]) - kd * qd[i];
      // Exact training DCMotor speed/torque envelope. Physics velocity limit is
      // also 30 rad/s, independently of the lower old-Demo calf limit.
      upper[i] = std::min(saturationEffort*(1.0-qd[i]/velocityLimit), effortLimit);
      lower[i] = std::max(saturationEffort*(-1.0-qd[i]/velocityLimit), -effortLimit);
      if (lower[i] > upper[i]) {
        Latch(6, "joint_velocity_outside_DCMotor_envelope", time);
        lastTorque[i] = std::clamp(-faultDamping*qd[i], -effortLimit, effortLimit);
      } else lastTorque[i] = std::clamp(raw[i], lower[i], upper[i]);
      joints[i].SetForce(ecm, {lastTorque[i]});
    }
    log << "{\"kind\":\"physics_step\",\"t\":" << time << ",\"dt\":" << dt
        << ",\"iteration\":" << iteration-1 << ",\"mode\":" << (fault || terminating ? 1 : 0)
        << ",\"fault\":" << fault << ",\"terminating\":" << (terminating ? "true" : "false")
        << ",\"q\":";
    Array(log, q); log << ",\"qd\":"; Array(log, qd);
    log << ",\"qtarget\":"; Array(log, target);
    std::array<int, 12> targetOutsideHard{};
    for (size_t i = 0; i < 12; ++i) targetOutsideHard[i] = target[i] < hardLower[i] || target[i] > hardUpper[i];
    log << ",\"target_outside_hard_limits\":"; Array(log, targetOutsideHard);
    log << ",\"pd_raw\":"; Array(log, raw);
    log << ",\"tau_lower\":"; Array(log, lower); log << ",\"tau_upper\":"; Array(log, upper);
    log << ",\"tau\":"; Array(log, lastTorque); log << ",\"contacts\":"; Array(log, contactCounts);
    log << ",\"body_lin_vel_origin\":"; Vector(log, originBody);
    log << ",\"body_lin_vel_com\":"; Vector(log, comBody);
    log << ",\"body_ang_vel\":"; Vector(log, omegaBody);
    log << ",\"position\":[" << request[1] << ',' << request[2] << ',' << request[3]
        << "],\"quaternion_wxyz\":[" << request[4] << ',' << request[5] << ',' << request[6] << ',' << request[7]
        << "],\"contact_pairs\":[";
    bool first = true;
    for (size_t group = 0; group < 5; ++group) for (auto collision : contactEntities[group]) {
      const auto contacts = ecm.Component<gz::sim::components::ContactSensorData>(collision);
      if (!contacts) continue;
      for (const auto &contact : contacts->Data().contact()) {
        if (!first) log << ',';
        first = false;
        double maxDepth = 0;
        for (double depth : contact.depth()) maxDepth = std::max(maxDepth, depth);
        log << "{\"group\":" << group << ",\"a\":" << Quote(contact.collision1().name())
            << ",\"b\":" << Quote(contact.collision2().name()) << ",\"points\":" << contact.position_size()
            << ",\"max_depth_m\":" << maxDepth
            << ",\"geometry_source\":\"actual gz.msgs.Contact fields; absent fields remain empty\""
            << ",\"positions_world\":[";
        for (int i=0; i<contact.position_size(); ++i) {
          if (i) log << ',';
          MessageVector(log, contact.position(i));
        }
        log << "],\"normals_world\":[";
        for (int i=0; i<contact.normal_size(); ++i) {
          if (i) log << ',';
          MessageVector(log, contact.normal(i));
        }
        log << "],\"depths_m\":[";
        for (int i=0; i<contact.depth_size(); ++i) { if (i) log << ','; log << contact.depth(i); }
        log << "],\"wrenches_raw\":[";
        for (int i=0; i<contact.wrench_size(); ++i) {
          if (i) log << ',';
          const auto &w=contact.wrench(i);
          log << "{\"body_1_name\":" << Quote(w.body_1_name()) << ",\"body_2_name\":" << Quote(w.body_2_name());
          auto rawWrench=[&](const auto &value) {
            log << "{\"force\":";
            if (value.has_force()) MessageVector(log,value.force()); else log << "null";
            log << ",\"torque\":";
            if (value.has_torque()) MessageVector(log,value.torque()); else log << "null";
            log << '}';
          };
          log << ",\"body_1_wrench\":";
          if (w.has_body_1_wrench()) rawWrench(w.body_1_wrench()); else log << "null";
          log << ",\"body_2_wrench\":";
          if (w.has_body_2_wrench()) rawWrench(w.body_2_wrench()); else log << "null";
          log << '}';
        }
        log << "],\"wrench_frame_scope\":\"raw message body wrench fields; no inferred vertical load measurement\"}";
      }
    }
    log << "]}\n";
    if (iteration % 200 == 0 || terminating || fault) log.flush();
    if (!log) Latch(9, "actuator_evidence_write_failed", time);
    if (terminating) eventManager->Emit<gz::sim::events::Stop>();
  }

 private:
  bool CheckOwnership(const gz::sim::EntityComponentManager &ecm) {
    const auto info = ecm.Component<gz::sim::components::SystemPluginInfo>(modelEntity);
    if (!info) return false;
    int teacherCount = 0;
    log << "{\"kind\":\"model_plugin_ownership\",\"plugins\":[";
    bool first = true, valid = true;
    for (const auto &plugin : info->Data().plugins()) {
      if (!first) log << ',';
      first = false;
      log << "{\"name\":" << Quote(plugin.name()) << ",\"filename\":" << Quote(plugin.filename()) << '}';
      std::string id = plugin.name() + " " + plugin.filename();
      std::transform(id.begin(), id.end(), id.begin(), [](unsigned char c) { return std::tolower(c); });
      if (id.find("teacher_actuator") != std::string::npos || id.find("teacheractuator") != std::string::npos) ++teacherCount;
      else if (id.find("control") != std::string::npos || id.find("velocity") != std::string::npos ||
               id.find("trajectory") != std::string::npos || id.find("actuator") != std::string::npos) valid = false;
    }
    valid = valid && teacherCount == 1;
    log << "],\"teacher_writers\":" << teacherCount << ",\"passed\":" << (valid ? "true" : "false") << "}\n";
    log.flush();
    return valid;
  }

  void Latch(int code, const std::string &reason, double time) {
    if (fault) return;
    fault = code;
    gzerr << "Teacher actuator damping latch: " << reason << " at " << time << '\n';
    if (log) { log << "{\"kind\":\"fault\",\"t\":" << time << ",\"code\":" << code
                  << ",\"reason\":" << Quote(reason) << "}\n"; log.flush(); }
  }

  bool Connect(double time) {
    if (connectionAttempted) return fd >= 0;
    connectionAttempted = true;
    fd = ::socket(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (fd < 0) { Latch(1, "socket_creation_failed", time); return false; }
    timeval timeout{0, 200000};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
    sockaddr_un address{}; address.sun_family = AF_UNIX;
    std::memcpy(address.sun_path, socketPath.c_str(), socketPath.size()+1);
    if (::connect(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
      Latch(1, std::string("socket_connect_failed: ") + std::strerror(errno), time);
      ::close(fd); fd = -1; return false;
    }
    return true;
  }

  bool Transfer(void *buffer, size_t bytes, bool sending) {
    auto *data = static_cast<unsigned char *>(buffer);
    const auto deadline = Clock::now() + std::chrono::milliseconds(200);
    size_t offset = 0;
    while (offset < bytes) {
      const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline-Clock::now()).count();
      if (remaining <= 0) return false;
      pollfd descriptor{fd, static_cast<short>(sending ? POLLOUT : POLLIN), 0};
      const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining));
      if (ready < 0 && errno == EINTR) continue;
      if (ready <= 0 || (descriptor.revents & (POLLERR | POLLNVAL))) return false;
      const ssize_t n = sending ? ::send(fd, data+offset, bytes-offset, MSG_NOSIGNAL) : ::recv(fd, data+offset, bytes-offset, 0);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) return false;
      offset += static_cast<size_t>(n);
    }
    return true;
  }

  void Exchange(std::array<double, 64> &request) {
    const double time = request[0];
    if (!Connect(time)) return;
    std::array<double, 16> response{};
    const auto started = Clock::now();
    if (!Transfer(request.data(), sizeof(request), true) || !Transfer(response.data(), sizeof(response), false)) {
      Latch(3, "socket_timeout_disconnect_or_partial_frame", time);
      ::close(fd); fd = -1; return;
    }
    const double duration = std::chrono::duration<double>(Clock::now()-started).count();
    bool valid = true;
    for (double value : response) valid = valid && std::isfinite(value);
    for (size_t i = 0; i < 12; ++i) valid = valid && std::abs(response[i]) <= targetAbsBound;
    valid = valid && (response[12] == 0 || response[12] == 1) && (response[13] == 0 || response[13] == 1);
    if (!valid) { Latch(4, "nonfinite_or_invalid_inference_response", time); return; }
    terminating = response[13] == 1;
    if (response[12] == 1 && !terminating) Latch(5, "inference_requested_fault_damping", time);
    if (!fault && !terminating) std::copy_n(response.begin(), 12, target.begin());
    log << "{\"kind\":\"policy_exchange\",\"t\":" << time << ",\"duration_wall_s\":" << duration
        << ",\"response_mode\":" << response[12] << ",\"terminate\":" << response[13] << "}\n";
  }

  const std::array<std::string, 12> names = {"rf_hip_joint", "rf_upper_leg_joint", "rf_lower_leg_joint",
      "lf_hip_joint", "lf_upper_leg_joint", "lf_lower_leg_joint", "rh_hip_joint", "rh_upper_leg_joint",
      "rh_lower_leg_joint", "lh_hip_joint", "lh_upper_leg_joint", "lh_lower_leg_joint"};
  std::array<gz::sim::Joint, 12> joints;
  std::array<std::vector<gz::sim::Entity>, 5> contactEntities;
  gz::sim::Link base;
  gz::sim::Entity modelEntity = gz::sim::kNullEntity;
  gz::sim::EventManager *eventManager = nullptr;
  gz::math::Vector3d comOffset;
  double baseMass = 0;
  A12 target = {-0.1,0.8,-1.5, 0.1,0.8,-1.5, -0.1,1.0,-1.5, 0.1,1.0,-1.5};
  A12 lastTorque{}, hardLower{}, hardUpper{};
  double kp=25, kd=0.5, effortLimit=23.5, saturationEffort=23.5, velocityLimit=30;
  double faultDamping=2, expectedDt=0.005, targetAbsBound=10, lastTime=-1;
  unsigned int decimation=4;
  uint64_t iteration=0;
  int fault=0, fd=-1;
  bool terminating=false, ownershipChecked=false, connectionAttempted=false;
  std::string socketPath;
  std::ofstream log;
};
}
GZ_ADD_PLUGIN(teacher_sim::TeacherActuator, gz::sim::System,
    teacher_sim::TeacherActuator::ISystemConfigure,
    teacher_sim::TeacherActuator::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(teacher_sim::TeacherActuator, "teacher_sim::TeacherActuator")
