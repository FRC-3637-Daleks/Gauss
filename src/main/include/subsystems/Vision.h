// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <photonlib/PhotonTargetSortMode.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/PhotonUtils.h>
#include <photonlib/RobotPoseEstimator.h>
#include <units/angle.h>
#include <units/length.h>

class Vision : public frc2::SubsystemBase {
public:
public:
  Vision();

  frc2::CommandPtr CommandTurnToTarget();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

private:
  // camera name
  photonlib::PhotonCamera camera{"limelight3637"};
  // camera height and target height
  const units::meter_t CAMERA_HEIGHT = 24_in; // tbd
  const units::meter_t TARGET_HEIGHT = 14_in;

  // Angle between horizontal and the camera.
  // units::radian_t CAMERA_PITCH = 0_deg;

  // PID constants should be tuned per robot --> all are tbd
  const double LINEAR_P = 0.1;
  const double LINEAR_D = 0.0;
  const double ANGULAR_P = 0.1;
  const double ANGULAR_D = 0.0;

  // boolean for hasTargets method in vision.cpp
  bool HasTargets();
  // vector which stores april tags
  std::vector<frc::AprilTag> tags = {
      {0, frc::Pose3d(units::meter_t(3), units::meter_t(3), units::meter_t(3), frc::Rotation3d())},
      {1, frc::Pose3d(units::meter_t(5), units::meter_t(5), units::meter_t(5), frc::Rotation3d())}
      };
  // allows april tags to be shared and has field dimensions LxW
  std::shared_ptr<frc::AprilTagFieldLayout> aprilTags = std::make_shared<frc::AprilTagFieldLayout>(tags, 54_ft, 27_ft);
};
