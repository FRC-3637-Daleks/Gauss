#include "subsystems/Vision.h"
#include "subsystems/DalekDrive.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonTargetSortMode.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/PhotonUtils.h>

Vision::Vision(
    std::function<void(frc::Pose2d, units::second_t)> addVisionMeasurement,
    std::function<frc::Pose2d()> getRobotPose)
    : m_estimator{
          frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp),
          photonlib::AVERAGE_BEST_TARGETS, std::move(m_camera),
          VisionConstants::kCameraToRobot} {
  m_addVisionMeasurement = addVisionMeasurement;
  m_getRobotPose = getRobotPose;
}

bool Vision::HasTargets() {
  photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();

  return result.HasTargets();
}

void Vision::CalculateRobotPoseEstimate() {
  // m_estimator.setReferencePose(m_getRobotPose());
  m_apriltagEstimate = m_estimator.Update();
}

// AddVisionPoseEstimate takes in Pose2d and a timestamp, thus the conversion of
// Pose3d to Pose2d is necessary
// Also, the timestamp should be calcuated by taking the timestamp of the
// current time and subtracting it from the result.second
// make sure to do above before calling AddVisionPoseEstimate

void Vision::Periodic() {
  CalculateRobotPoseEstimate();
  if (m_apriltagEstimate.has_value()) {
    m_addVisionMeasurement(m_apriltagEstimate.value().estimatedPose.ToPose2d(),
                           m_apriltagEstimate.value().timestamp);
  }
}
