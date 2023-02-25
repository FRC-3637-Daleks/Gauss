#include "subsystems/Vision.h"
#include "subsystems/DalekDrive.h"

#include <frc/Joystick.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <photonlib/PhotonPipelineResult.h>
#include <photonlib/PhotonTargetSortMode.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/PhotonUtils.h>

Vision::Vision(
    std::function<void(frc::Pose2d, units::second_t)> addVisionMeasurement,
    std::function<frc::Pose2d()> getRobotPose)
    : m_estimator{VisionConstants::kAprilTagFieldLayout,
                  photonlib::AVERAGE_BEST_TARGETS, std::move(m_camera),
                  VisionConstants::kRobotToCamera} {
  m_addVisionMeasurement = addVisionMeasurement;
  m_getRobotPose2d = m_getRobotPose2d;
}

//     std::function<frc::Pose3d()> getRobotPose)
//     : m_estimator{VisionConstants::kAprilTagFieldLayout,
//                   photonlib::CLOSEST_TO_LAST_POSE, std::move(m_camera),
//                   VisionConstants::kRobotToCamera} {
//   m_addVisionMeasurement = addVisionMeasurement;
//   m_getRobotPose = getRobotPose;
// }

//     std::function<frc::Pose3d()> getRobotPose)
//     : m_estimator{VisionConstants::kAprilTagFieldLayout,
//                   photonlib::CLOSEST_TO_REFERENCE_POSE, std::move(m_camera),
//                   VisionConstants::kRobotToCamera} {
//   m_addVisionMeasurement = addVisionMeasurement;
//   m_getRobotPose = getRobotPose;
// }

bool Vision::HasTargets() {
  photonlib::PhotonPipelineResult result = m_camera.GetLatestResult();

  return result.HasTargets();
}

void Vision::CalculateRobotPoseEstimate() {
  //
  m_apriltagEstimate = m_estimator.Update();
}

//   m_estimator.SetReferencePose(frc::Pose3d{m_getRobotPose()} +
//                                VisionConstants::kRobotToCamera);
//   m_apriltagEstimate = m_estimator.setReferencePose(;

//   m_estimator.SetReferencePose(frc::Pose3d{m_getRobotPose()} +
//                                VisionConstants::kRobotToCamera);
//   m_apriltagEstimate = m_estimator.Update();
// }

void Vision::SwitchPipeline(frc::Joystick *joystick) {
  enum PipelineState { AprilTags = 0, Tape = 1, DriverCamera = 2 };

  bool tapeButtonPress = joystick->GetRawButton(2);
  bool driverCameraButtonPress = joystick->GetRawButton(3);

  PipelineState pipelineState = PipelineState::AprilTags;

  if (tapeButtonPress) {
    pipelineState = PipelineState::Tape;
  } else if (driverCameraButtonPress) {
    pipelineState = PipelineState::DriverCamera;
  } else {
    pipelineState = PipelineState::AprilTags;
  }

  m_camera.SetPipelineIndex(static_cast<int>(pipelineState));
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

  } // add toPose2d to estimatedPose, when using a Pose3d strategy
}
