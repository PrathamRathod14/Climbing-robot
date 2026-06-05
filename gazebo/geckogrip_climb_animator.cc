#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
struct LimbState
{
  std::string key;
  double side;
  double shoulderZ;
  std::vector<int> route;
};

class GeckoGripClimbAnimator : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    std::cout << "[GeckoGrip Plugin] Load called!" << std::endl;
    this->world = world;
    this->lastUpdate = world->SimTime().Double();
    this->startTime = this->lastUpdate;
    this->connection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GeckoGripClimbAnimator::OnUpdate, this));
    std::cout << "[GeckoGrip Plugin] Load finished successfully!" << std::endl;
  }

private:
  static constexpr double kMoveStart = 0.24;
  static constexpr double kMoveEnd = 0.90;
  static constexpr double kLiftY = 0.025;
  static constexpr double kLiftZ = 0.070;
  static constexpr double kStepDuration = 3.10;
  static constexpr int kPhaseCount = 4;
  static constexpr std::array<int, 4> kGaitOrder{{0, 3, 1, 2}};
  static constexpr double kBodySide = 0.065;
  static constexpr double kHipSide = 0.075;
  static constexpr double kKneeSideMoving = 0.012;
  static constexpr double kKneeSideRest = 0.008;
  static constexpr double kKneeDropMoving = 0.055;
  static constexpr double kKneeDropRest = 0.038;
  static constexpr double kKneeLiftMoving = 0.040;
  static constexpr double kKneeLiftRest = 0.012;
  static constexpr double kFootYaw = 0.025;
  static constexpr double kYawScale = 0.04;
  static constexpr double kYawLimit = 0.08;
  static constexpr double kBodyLerp = 0.20;
  static constexpr double kGravityBodySag = 0.045;
  static constexpr double kSwingBodySag = 0.028;
  static constexpr double kGravityAccelBase = 9.81;
  static constexpr double kGravityAccelMax = 16.0;
  static constexpr std::array<int, 3> kFallSteps{{4, 8, 22}};
  static constexpr int kFinalFallStep = 22;
  static constexpr double kFallClimbTime = kFinalFallStep * kStepDuration;
  static constexpr double kGripPeelY = 0.014;
  static constexpr double kFallDuration = 2.6;
  static constexpr double kWakePauseDuration = 1.2;
  static constexpr double kRecoverDuration = 4.4;
  static constexpr double kDemoCycleDuration =
    kFallClimbTime + 3.0 * (kFallDuration + kWakePauseDuration + kRecoverDuration);
  static constexpr double kFloorBodyY = -1.55;
  static constexpr double kFloorBodyZ = 0.34;

  static ignition::math::Vector3d HoldPoint(const int index)
  {
    static const std::array<ignition::math::Vector3d, 95> holds = {{
      {-0.72, -0.18, 0.45}, {-0.36, -0.18, 0.45}, {0.00, -0.18, 0.45}, {0.36, -0.18, 0.45}, {0.72, -0.18, 0.45},
      {-0.58, -0.18, 0.86}, {-0.18, -0.18, 0.86}, {0.22, -0.18, 0.86}, {0.62, -0.18, 0.86},
      {-0.72, -0.18, 1.27}, {-0.36, -0.18, 1.27}, {0.00, -0.18, 1.27}, {0.36, -0.18, 1.27}, {0.72, -0.18, 1.27},
      {-0.58, -0.18, 1.68}, {-0.18, -0.18, 1.68}, {0.22, -0.18, 1.68}, {0.62, -0.18, 1.68},
      {-0.72, -0.18, 2.09}, {-0.36, -0.18, 2.09}, {0.00, -0.18, 2.09}, {0.36, -0.18, 2.09}, {0.72, -0.18, 2.09},
      {-0.58, -0.18, 2.50}, {-0.18, -0.18, 2.50}, {0.22, -0.18, 2.50}, {0.62, -0.18, 2.50},
      {-0.72, -0.18, 2.91}, {-0.36, -0.18, 2.91}, {0.00, -0.18, 2.91}, {0.36, -0.18, 2.91}, {0.72, -0.18, 2.91},
      {-0.58, -0.18, 3.32}, {-0.18, -0.18, 3.32}, {0.22, -0.18, 3.32}, {0.62, -0.18, 3.32},
      {-0.72, -0.18, 3.73}, {-0.36, -0.18, 3.73}, {0.00, -0.18, 3.73}, {0.36, -0.18, 3.73}, {0.72, -0.18, 3.73},
      {-0.58, -0.18, 4.14}, {-0.18, -0.18, 4.14}, {0.22, -0.18, 4.14}, {0.62, -0.18, 4.14},
      {-0.72, -0.18, 4.55}, {-0.36, -0.18, 4.55}, {0.00, -0.18, 4.55}, {0.36, -0.18, 4.55}, {0.72, -0.18, 4.55},
      {-0.58, -0.18, 4.96}, {-0.18, -0.18, 4.96}, {0.22, -0.18, 4.96}, {0.62, -0.18, 4.96},
      {-0.72, -0.18, 5.37}, {-0.36, -0.18, 5.37}, {0.00, -0.18, 5.37}, {0.36, -0.18, 5.37}, {0.72, -0.18, 5.37},
      {-0.58, -0.18, 5.78}, {-0.18, -0.18, 5.78}, {0.22, -0.18, 5.78}, {0.62, -0.18, 5.78},
      {-0.72, -0.18, 6.19}, {-0.36, -0.18, 6.19}, {0.00, -0.18, 6.19}, {0.36, -0.18, 6.19}, {0.72, -0.18, 6.19},
      {-0.58, -0.18, 6.60}, {-0.18, -0.18, 6.60}, {0.22, -0.18, 6.60}, {0.62, -0.18, 6.60},
      {-0.72, -0.18, 7.01}, {-0.36, -0.18, 7.01}, {0.00, -0.18, 7.01}, {0.36, -0.18, 7.01}, {0.72, -0.18, 7.01},
      {-0.58, -0.18, 7.42}, {-0.18, -0.18, 7.42}, {0.22, -0.18, 7.42}, {0.62, -0.18, 7.42},
      {-0.72, -0.18, 7.83}, {-0.36, -0.18, 7.83}, {0.00, -0.18, 7.83}, {0.36, -0.18, 7.83}, {0.72, -0.18, 7.83},
      {-0.58, -0.18, 8.24}, {-0.18, -0.18, 8.24}, {0.22, -0.18, 8.24}, {0.62, -0.18, 8.24},
      {-0.72, -0.18, 8.65}, {-0.36, -0.18, 8.65}, {0.00, -0.18, 8.65}, {0.36, -0.18, 8.65}, {0.72, -0.18, 8.65},
    }};
    return holds.at(static_cast<size_t>(index));
  }

  static double SmoothStep(double t)
  {
    t = std::max(0.0, std::min(1.0, t));
    return t * t * (3.0 - 2.0 * t);
  }

  static ignition::math::Vector3d Lerp(
    const ignition::math::Vector3d &a,
    const ignition::math::Vector3d &b,
    const double t)
  {
    return a + (b - a) * t;
  }

  static double GravityAt(const double elapsed)
  {
    if (kFallClimbTime <= 0.0)
      return kGravityAccelBase;
    const double t = std::max(0.0, std::min(1.0, elapsed / kFallClimbTime));
    return kGravityAccelBase + (kGravityAccelMax - kGravityAccelBase) * t;
  }

  ignition::math::Vector3d NarrowWallFoot(const LimbState &limb, ignition::math::Vector3d foot) const
  {
    (void)limb;
    return foot;
  }

  ignition::math::Vector3d FloorFootFor(
    const LimbState &limb,
    const double bodyX,
    const double time = 0.0,
    const bool moving = false) const
  {
    const bool diagonal = limb.key == "FL" || limb.key == "RR";
    const double phase = diagonal ? 0.0 : M_PI;
    const double cycle = moving ? std::sin(time * 4.2 + phase) : 0.0;
    const double lift = std::max(0.0, cycle) * 0.055;
    const double stride = moving ? std::sin(time * 4.2 + phase) * 0.10 : 0.0;
    return ignition::math::Vector3d(
      bodyX + limb.side * 0.16,
      kFloorBodyY + limb.shoulderZ * 0.42 + stride,
      0.06 + lift);
  }

  ignition::math::Pose3d SegmentPose(
    const ignition::math::Vector3d &a,
    const ignition::math::Vector3d &b) const
  {
    const auto d = b - a;
    const double length = std::max(0.001, d.Length());
    const double yaw = std::atan2(d.Y(), d.X());
    const double pitch = std::acos(std::max(-1.0, std::min(1.0, d.Z() / length)));
    const auto mid = (a + b) * 0.5;
    return ignition::math::Pose3d(mid.X(), mid.Y(), mid.Z(), 0, pitch, yaw);
  }

  void SetModelPose(const std::string &name, const ignition::math::Pose3d &pose)
  {
    auto model = this->world->ModelByName(name);
    if (model)
      model->SetWorldPose(pose);
  }

  ignition::math::Vector3d FootFor(
    const LimbState &limb,
    const int step,
    const int activeOrder,
    const double progress) const
  {
    const int order = this->OrderIndex(limb.key);
    const int completed = this->CompletedSteps(order, step);
    const int fromIndex = std::min(completed, static_cast<int>(limb.route.size()) - 1);
    const bool active = order == activeOrder && fromIndex + 1 < static_cast<int>(limb.route.size());
    if (!active)
      return this->NarrowWallFoot(limb, HoldPoint(limb.route[fromIndex]));

    const auto a = this->NarrowWallFoot(limb, HoldPoint(limb.route[fromIndex]));
    const auto b = this->NarrowWallFoot(limb, HoldPoint(limb.route[fromIndex + 1]));
    if (progress <= kMoveStart)
      return a;
    if (progress >= kMoveEnd)
      return b;
    const double t = SmoothStep((progress - kMoveStart) / (kMoveEnd - kMoveStart));
    auto foot = Lerp(a, b, t);
    foot.Y() -= kLiftY * std::sin(t * M_PI);
    foot.Z() += kLiftZ * std::sin(t * M_PI);
    return foot;
  }

  int OrderIndex(const std::string &key) const
  {
    if (key == "FL") return 0;
    if (key == "FR") return 1;
    if (key == "RL") return 2;
    return 3;
  }

  int OrderCount(const int order) const
  {
    int count = 0;
    for (const int phase : kGaitOrder)
    {
      if (phase == order)
        ++count;
    }
    return count;
  }

  int CompletedSteps(const int order, const int step) const
  {
    if (step <= 0)
      return 0;
    const int perCycle = this->OrderCount(order);
    if (perCycle == 0)
      return 0;
    const int full = step / kPhaseCount;
    const int rem = step % kPhaseCount;
    int count = full * perCycle;
    for (int i = 0; i < rem; ++i)
    {
      if (kGaitOrder[static_cast<size_t>(i)] == order)
        ++count;
    }
    return count;
  }

  int LimbIndexForOrder(const int order) const
  {
    for (size_t i = 0; i < this->limbs.size(); ++i)
    {
      if (this->OrderIndex(this->limbs[i].key) == order)
        return static_cast<int>(i);
    }
    return 0;
  }

  int MaxSteps() const
  {
    int maxCycles = 0;
    for (const auto &limb : this->limbs)
    {
      const int moves = std::max(0, static_cast<int>(limb.route.size()) - 1);
      const int perCycle = this->OrderCount(this->OrderIndex(limb.key));
      if (perCycle == 0)
        continue;
      const int cycles = (moves + perCycle - 1) / perCycle;
      maxCycles = std::max(maxCycles, cycles);
    }
    if (maxCycles == 0)
      return 0;
    return maxCycles * kPhaseCount;
  }

  void OnUpdate()
  {
    static bool first = true;
    if (first)
    {
      std::cout << "[GeckoGrip Plugin] First OnUpdate call!" << std::endl;
      first = false;
    }
    const double simTime = this->world->SimTime().Double();
    if (simTime - this->lastUpdate < 1.0 / 30.0)
      return;
    this->lastUpdate = simTime;

    const double elapsed = std::max(0.0, simTime - this->startTime);
    const double cycleElapsed = kDemoCycleDuration > 0.0
      ? std::fmod(elapsed, kDemoCycleDuration)
      : elapsed;
    bool falling = false;
    bool fallen = false;
    bool recovering = false;
    double phaseTime = 0.0;
    double climbElapsed = cycleElapsed;
    double remaining = cycleElapsed;
    for (const int fallStep : kFallSteps)
    {
      const double segmentClimbTime = fallStep * kStepDuration;
      const double fallClimbTime = fallStep * kStepDuration;
      if (remaining < segmentClimbTime)
      {
        climbElapsed = remaining;
        break;
      }
      remaining -= segmentClimbTime;

      if (remaining < kFallDuration)
      {
        falling = true;
        phaseTime = remaining;
        climbElapsed = fallClimbTime;
        break;
      }
      remaining -= kFallDuration;

      if (remaining < kWakePauseDuration)
      {
        fallen = true;
        phaseTime = kFallDuration;
        climbElapsed = fallClimbTime;
        break;
      }
      remaining -= kWakePauseDuration;

      if (remaining < kRecoverDuration)
      {
        recovering = true;
        phaseTime = remaining;
        climbElapsed = 0.0;
        break;
      }
      remaining -= kRecoverDuration;
      climbElapsed = 0.0;
    }
    const double gravityAccel = GravityAt(climbElapsed);
    const double gravityScale = kGravityAccelBase > 0.0
      ? gravityAccel / kGravityAccelBase
      : 1.0;
    if (this->world)
      this->world->SetGravity(ignition::math::Vector3d(0, 0, -gravityAccel));
    const double stepDuration = kStepDuration;
    const int stepIndex = static_cast<int>(std::floor(climbElapsed / stepDuration));
    const int maxSteps = this->MaxSteps();
    const int step = std::min(stepIndex, maxSteps);
    const double progress = stepIndex >= maxSteps
      ? 1.0
      : std::fmod(climbElapsed, stepDuration) / stepDuration;
    const int activeOrder = kGaitOrder[static_cast<size_t>(step % kPhaseCount)];
    const bool swinging = step < maxSteps && progress > kMoveStart && progress < kMoveEnd;
    const double swingT = swinging
      ? SmoothStep((progress - kMoveStart) / (kMoveEnd - kMoveStart))
      : 0.0;

    std::array<ignition::math::Vector3d, 4> feet;
    for (size_t i = 0; i < this->limbs.size(); ++i)
      feet[i] = this->FootFor(this->limbs[i], step, activeOrder, progress);

    double poseBodyY = -0.34;
    double poseRoll = M_PI / 2;
    double posePitch = -M_PI / 2;
    if (falling || fallen || recovering)
    {
      const double fallT = falling ? SmoothStep(phaseTime / kFallDuration) : 1.0;
      const double recoverT = recovering ? SmoothStep(phaseTime / kRecoverDuration) : 0.0;
      const double floorT = recovering ? 1.0 - recoverT : fallT;
      poseBodyY = recovering
        ? kFloorBodyY + (-0.34 - kFloorBodyY) * recoverT
        : -0.34 + (kFloorBodyY + 0.34) * fallT;
      poseRoll = (1.0 - floorT) * (M_PI / 2);
      posePitch = (1.0 - floorT) * (-M_PI / 2);

      for (size_t i = 0; i < this->limbs.size(); ++i)
      {
        const auto floorFoot = this->FloorFootFor(this->limbs[i], 0.0, phaseTime, recovering);
        feet[i] = Lerp(feet[i], floorFoot, floorT);
      }
    }

    double targetBodyX = 0;
    double targetBodyZ = 0;
    for (size_t i = 0; i < this->limbs.size(); ++i)
    {
      targetBodyX += feet[i].X() - this->limbs[i].side * kBodySide;
      targetBodyZ += feet[i].Z() - this->limbs[i].shoulderZ;
    }
    const double gravityBodySag = kGravityBodySag * gravityScale;
    const double gravitySwingSag = kSwingBodySag * gravityScale;
    targetBodyX /= static_cast<double>(this->limbs.size());
    targetBodyZ = targetBodyZ / static_cast<double>(this->limbs.size()) + 0.14;
    targetBodyZ -= gravityBodySag + (swinging ? gravitySwingSag * std::sin(swingT * M_PI) : 0.0);
    poseBodyY -= swinging ? kGripPeelY * std::sin(swingT * M_PI) : kGripPeelY * 0.35;

    const double leftZ = (feet[0].Z() + feet[2].Z()) * 0.5;
    const double rightZ = (feet[1].Z() + feet[3].Z()) * 0.5;
    const double targetYaw = std::max(-kYawLimit, std::min(kYawLimit, (leftZ - rightZ) * kYawScale));
    if (!this->poseInitialized) {
      this->smoothBodyX = targetBodyX;
      this->smoothBodyZ = targetBodyZ;
      this->smoothYaw = targetYaw;
      this->poseInitialized = true;
    } else {
      this->smoothBodyX += (targetBodyX - this->smoothBodyX) * kBodyLerp;
      this->smoothBodyZ += (targetBodyZ - this->smoothBodyZ) * kBodyLerp;
      this->smoothYaw += (targetYaw - this->smoothYaw) * kBodyLerp;
    }

    auto model = this->world->ModelByName("geckogrip_quadruped");
    if (model)
    {
      model->SetWorldPose(ignition::math::Pose3d(
        this->smoothBodyX, poseBodyY, this->smoothBodyZ,
        poseRoll, posePitch, this->smoothYaw));
    }

    const auto activeIndex = static_cast<size_t>(this->LimbIndexForOrder(activeOrder));
    const auto &activeLimb = this->limbs[activeIndex];
    const int activeOrderIndex = this->OrderIndex(activeLimb.key);
    const int activeCompleted = this->CompletedSteps(activeOrderIndex, step);
    const int activeFrom = std::min(activeCompleted, static_cast<int>(activeLimb.route.size()) - 1);
    const int activeNext = std::min(activeFrom + 1, static_cast<int>(activeLimb.route.size()) - 1);
    const auto marker = HoldPoint(activeLimb.route[activeNext]);
    this->SetModelPose("predicted_hold_marker",
      ignition::math::Pose3d(marker.X(), -0.12, marker.Z(), 1.571, 0, 0));

    if (model)
    {
      for (size_t i = 0; i < this->limbs.size(); ++i)
      {
        const auto &limb = this->limbs[i];
        const bool moving = this->OrderIndex(limb.key) == activeOrder && progress < 1.0;
        const auto hip = ignition::math::Vector3d(
          this->smoothBodyX + limb.side * kHipSide, poseBodyY - 0.01, this->smoothBodyZ + limb.shoulderZ);
        const auto knee = ignition::math::Vector3d(
          hip.X() * 0.50 + feet[i].X() * 0.50 + limb.side * (moving ? kKneeSideMoving : kKneeSideRest),
          hip.Y() * 0.50 + feet[i].Y() * 0.50 - (moving ? kKneeDropMoving : kKneeDropRest),
          hip.Z() * 0.50 + feet[i].Z() * 0.50 + (moving ? kKneeLiftMoving : kKneeLiftRest));

        auto upper_link = model->GetLink(limb.key + "_upper");
        if (upper_link)
          upper_link->SetWorldPose(this->SegmentPose(hip, knee));

        auto lower_link = model->GetLink(limb.key + "_lower");
        if (lower_link)
          lower_link->SetWorldPose(this->SegmentPose(knee, feet[i]));

        auto foot_link = model->GetLink(limb.key + "_foot");
        if (foot_link)
          foot_link->SetWorldPose(ignition::math::Pose3d(feet[i].X(), feet[i].Y(), feet[i].Z(), 1.571, 0, limb.side * kFootYaw));
      }
    }
  }

  physics::WorldPtr world;
  event::ConnectionPtr connection;
  double lastUpdate{0};
  double startTime{0};
  double smoothBodyX{0};
  double smoothBodyZ{0};
  double smoothYaw{0};
  bool poseInitialized{false};
  const std::vector<LimbState> limbs{
    {"FL", -1.0, 0.32, {15, 24, 33, 42, 51, 60, 69, 78, 87}},
    {"FR", 1.0, 0.32, {16, 25, 34, 43, 52, 61, 70, 79, 88}},
    {"RL", -1.0, -0.32, {6, 15, 24, 33, 42, 51, 60, 69, 78}},
    {"RR", 1.0, -0.32, {7, 16, 25, 34, 43, 52, 61, 70, 79}},
  };
};

GZ_REGISTER_WORLD_PLUGIN(GeckoGripClimbAnimator)
}
