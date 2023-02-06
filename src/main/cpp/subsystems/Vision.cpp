// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"
#include <frc/apriltag/AprilTagFieldLayout.h>

Vision::Vision() {
  // Implementation of subsystem constructor goes here.
}

// method which returns if targets are present or not
bool Vision::HasTargets() {
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();

  return result.HasTargets();
}

std::shared_ptr<photonlib::PhotonCamera> cameraOne = std::make_shared<photonlib::PhotonCamera>("limelight3637");
// Camera is mounted facing forward, half a meter forward of center, half a
// meter up from center.
frc::Transform3d robotToCam = frc::Transform3d(frc::Translation3d(0.5_m, 0_m, 0.5_m), frc::Rotation3d(0_rad, 0_rad, 0_rad));

// Assemble the list of cameras & mount locations
std::vector<std::pair<std::shared_ptr<photonlib::PhotonCamera>, frc::Transform3d>>cameras;
cameras.push_back(std::make_pair(cameraOne, robotToCam));

photonlib::RobotPoseEstimator
    estimator(aprilTags, photonlib::CLOSEST_TO_REFERENCE_POSE, cameras);
std::pair<frc::Pose2d, units::millisecond_t> getEstimatedGlobalPose(frc::Pose3d prevEstimatedRobotPose) {
  estimator.SetReferencePose(prevEstimatedRobotPose) units::millisecond_t currentTime = frc::Timer::GetFPGATimestamp();
  auto result = robotPoseEstimator.Update();
  if (result.second) {
    return std::make_pair<>(result.first.ToPose2d(), currentTime - result.second);
  }
   else {
    return std::make_pair(frc::Pose2d(), 0_ms);
  }
}

// frc2::CommandPtr Vision::CommandTurnToTarget() {
/*double rotationSpeed;

// Vision-alignment mode
// Query the latest result from PhotonVision
photonlib::PhotonPipelineResult result = camera.GetLatestResult();

if (result.HasTargets()) {
  // Rotation speed is the output of the PID controller
  rotationSpeed = -controller.Calculate(result.GetBestTarget().GetYaw(),
  0);

} else {
  // If we have no targets, stay still.
  rotationSpeed = 0;
}
*/

// Manual Driver Mode

// Use our forward/turn speeds to control the drivetrain
// drive.ArcadeDrive(rotationSpeed);
// }

void Vision::Periodic() {
  // Implementation of subsystem periodic method goes here.
  bool foundTarget;
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if (result.HasTargets()) {
    foundTarget = true;
  }
}

void Vision::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
