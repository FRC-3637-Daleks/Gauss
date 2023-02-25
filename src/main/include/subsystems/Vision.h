#pragma once

#include <optional>

// #include <frc/Joystick.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPoseEstimator.h>

#include "Constants.h"
#include <frc/Joystick.h>

class Vision : public frc2::SubsystemBase {
public:
  Vision(std::function<void(frc::Pose2d, units::second_t)> addVisionMeasurement,
         std::function<frc::Pose2d()> getRobotPose);
  // change pose2d to pose3d when using pose3d strategy
  void Periodic() override;

  bool HasTargets();

  void CalculateRobotPoseEstimate();

  void SwitchPipeline(frc::Joystick *joystick);

private:
  photonlib::PhotonCamera m_camera{VisionConstants::kPhotonCameraName};

  photonlib::PhotonPoseEstimator m_estimator;

  std::optional<photonlib::EstimatedRobotPose> m_apriltagEstimate{std::nullopt};

  std::function<void(frc::Pose2d, units::second_t)> m_addVisionMeasurement;
  std::function<frc::Pose2d()> m_getRobotPose2d;
  std::function<frc::Pose3d()> m_getRobotPose3d;
};
