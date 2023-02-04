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
  photonlib::PhotonCamera camera{"limelight3637"};
  const units::meter_t CAMERA_HEIGHT = 24_in;
  const units::meter_t TARGET_HEIGHT = 5_ft;

  // Angle between horizontal and the camera.
  units::radian_t CAMERA_PITCH = 0_deg;

  // How far from the target we want to be
  const units::meter_t GOAL_RANGE_METERS = 3_ft;

  // PID constants should be tuned per robot
  const double LINEAR_P = 0.1;
  const double LINEAR_D = 0.0;
  // frc2::PIDController forwardController{LINEAR_P, 0.0, LINEAR_D};

  const double ANGULAR_P = 0.1;
  const double ANGULAR_D = 0.0;
  // frc2::PIDController turnController{ANGULAR_P, 0.0, ANGULAR_D};
private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
