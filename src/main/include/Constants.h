#pragma once

#include <numbers>

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
constexpr auto kWheelDiameter = 8_in;
constexpr auto kEncoderDistancePerPulse =
    std::numbers::pi * kWheelDiameter / (double)(kEncoderCPR * kGearReduction);

constexpr auto kTrackWidth = 24_in;

constexpr bool kGyroReversed = true;

constexpr double kTalonRampRate =
    0.5; // 0.5 seconds from neutral to full throttle.
constexpr int kTalonTimeoutMs = 30;

constexpr auto kMaxSpeed = 10_fps;

// PID coefficients for closed-loop control of velocity.
constexpr double kFDriveSpeed = 0;
constexpr double kPDriveSpeed = 0;
constexpr double kIDriveSpeed = 0;
constexpr double kDDriveSpeed = 0;
constexpr double kIzDriveSpeed = 0;

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
const frc::Transform3d kCameraToRobot{
    {0_m, 36_in, 0_m},  // Camera is at the origin of the robot.
    frc::Rotation3d{}}; // The camera location relative to the robot's center.
const frc::AprilTagFieldLayout kAprilTagFieldLayout{
    std::vector<frc::AprilTag>{
        {0, frc::Pose3d(units::meter_t(3), units::meter_t(3), units::meter_t(3),
                        frc::Rotation3d())},
        {1, frc::Pose3d(units::meter_t(5), units::meter_t(5), units::meter_t(5),
                        frc::Rotation3d())}},
    54_ft, 27_ft};
} // namespace VisionConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_fps;
constexpr auto kMaxAcceleration = units::feet_per_second_squared_t{10};
} // namespace AutoConstants

namespace OperatorConstants {
constexpr int kLeftJoystickPort = 1;
constexpr int kRightJoystickPort = 2;
constexpr int kXboxControllerPort = 3;
} // namespace OperatorConstants
