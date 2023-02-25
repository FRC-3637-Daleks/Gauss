#pragma once

#include <numbers>

#include <frc/apriltag/AprilTagFields.h>
#include <frc/geometry/Transform3d.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace DriveConstants {
constexpr int kLeftFrontMotorId = 1;
constexpr int kLeftRearMotorId = 2;
constexpr int kRightFrontMotorId = 3;
constexpr int kRightRearMotorId = 4;

constexpr int kEncoderCPR = 2048;
constexpr int kGearReduction = 6;
constexpr auto kWheelDiameter = 7.25_in;
constexpr auto kEncoderDistancePerPulse =
    std::numbers::pi * kWheelDiameter / (double)(kEncoderCPR * kGearReduction);

constexpr auto kTrackWidth = 24_in;

constexpr bool kGyroReversed = true;

constexpr double kTalonRampRate =
    0.5; // 0.5 seconds from neutral to full throttle.
constexpr int kTalonTimeoutMs = 30;

// NOTE: Temporary, for open loop drive command.
constexpr double kMaxOutput = 0.5;

constexpr auto kMaxSpeed = 10_fps;

// PID coefficients for closed-loop control of velocity.
constexpr double kFDriveSpeed = 0.0656;
constexpr double kPDriveSpeed = 0.1;
constexpr double kIDriveSpeed = 0.0001;
constexpr double kDDriveSpeed = 0;
constexpr double kIzDriveSpeed = 1000;

// NOTE: Guess value!
constexpr double kPTurn = 0.75;
constexpr double kPDistance = 0;

constexpr auto kTurnTolerance = 5_deg;
constexpr auto kTurnRateTolerance = 5_deg_per_s;

constexpr auto kMaxTurnRate = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = std::numbers::pi * 1_rad_per_s_sq;
} // namespace DriveConstants

namespace VisionConstants {
constexpr std::string_view kPhotonCameraName =
    "limelight3637"; // Should be the camera as named in PhotonVision GUI.
// 14 1/16 in x, 16 y
const frc::Transform3d kRobotToCamera{
    {14_in, 16_in, 30_in},
    frc::Rotation3d{
        90_deg, 0_deg,
        0_deg}}; // The camera location relative to the robot's center.
const frc::AprilTagFieldLayout kAprilTagFieldLayout{
    frc::LoadAprilTagLayoutField(frc::AprilTagField::k2023ChargedUp)};
// const frc::AprilTagFieldLayout kAprilTagFieldLayout{
//     std::vector<frc::AprilTag>{
//         {0, frc::Pose3d(units::meter_t(3), units::meter_t(3),
//         units::meter_t(3),
//                         frc::Rotation3d())},
//         {1, frc::Pose3d(units::meter_t(5), units::meter_t(5),
//         units::meter_t(5),
//                         frc::Rotation3d())}},
//     54_ft, 27_ft};
} // namespace VisionConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_fps;
constexpr auto kMaxAcceleration = units::feet_per_second_squared_t{10};
} // namespace AutoConstants

namespace OperatorConstants {
constexpr bool kTesting = true;
constexpr int kLeftJoystickPort = 1;
constexpr int kRightJoystickPort = 2;
constexpr int kXboxControllerPort = 3;
} // namespace OperatorConstants
