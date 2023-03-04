#pragma once

#include <numbers>

#include <frc/apriltag/AprilTagFields.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

namespace ArmConstants {
constexpr int kPCMId = 5;
constexpr int kPistonChannel = 1;
constexpr int kClawChannel = 2;

constexpr int kMotorId = 8;

constexpr int kLimitSwitchChannel = 0;

// No pot for now.
// constexpr int kPotentiometerId = 0;
// constexpr double kVoltToLengthConversionFactor = 6.41;

// PID Loop
constexpr double kP = 10;
constexpr double kI = 1;
constexpr double kD = 0;
constexpr double kIz = 5;

constexpr auto kMaxTurnVelocity = 0.5 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = std::numbers::pi * 1_rad_per_s_sq;

constexpr bool kEncoderReversed = true;
constexpr int kEncoderCPR = 2048;
constexpr double kGearReduction = 16;
constexpr auto kEncoderRotationPerPulse =
    2_rad * std::numbers::pi / (kEncoderCPR * kGearReduction);

constexpr auto kLegExtensionPeriod = 1_s;

constexpr auto kOffset =
    units::degree_t{10}; // Angle of the arm when the limit switch is tripped.
constexpr auto kLegOffset =
    units::degree_t{17.1}; // Angle the arm moves when the legs are extended.

constexpr bool kDefaultPosition = false;
} // namespace ArmConstants

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

constexpr auto kMaxSpeed = 8_fps;
constexpr auto kPreciseSpeed = 2_fps;

// PID coefficients for closed-loop control of velocity.
constexpr double kFDriveSpeed = 0.0656;
constexpr double kPDriveSpeed = 0.1;
constexpr double kIDriveSpeed = 0.000;
constexpr double kDDriveSpeed = 0;
constexpr double kIzDriveSpeed = 1000;

constexpr double kIBrake = 0.0001;

// NOTE: Guess value!
constexpr double kPTurn = 0.75;
constexpr double kPDistance = 2;
constexpr auto kDistanceTolerance = 7_cm;

constexpr double kPLeftStraight = 0.2;
constexpr double kPRightStraight = 0.2;

constexpr auto kTurnTolerance = 5_deg;
constexpr auto kTurnRateTolerance = 5_deg_per_s;

constexpr auto kMaxTurnRate = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = std::numbers::pi * 1_rad_per_s_sq;
} // namespace DriveConstants

namespace ClawConstants {
constexpr int kPCMPort = 5;
constexpr int kPistonPort = 2;
constexpr int kLimitSwitchPort = 1;
} // namespace ClawConstants

namespace VisionConstants {
constexpr std::string_view kPhotonCameraName =
    "limelight3637"; // Should be the camera as named in PhotonVision GUI.
// 14 1/16 in x, 16 y
const frc::Transform3d kCameraToRobot{
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
constexpr auto kTargetAngle = frc::Rotation2d{105_deg};
constexpr auto kPlacementAngle = frc::Rotation2d{90_deg};

constexpr auto kMaxSpeed = 3_fps;
constexpr auto kMaxAcceleration = units::feet_per_second_squared_t{10};

constexpr double kPBalance = 0.025;
constexpr double kIBalance = 0.0;
constexpr double kDBalance = 0.04;

constexpr double kBalanceTolerance = 5;

constexpr auto kMidCone = 30_deg;
constexpr auto kHighCone = 100_deg;
constexpr auto kMidCube = 45_deg;
constexpr auto kHighCube = 90_deg;
constexpr auto kSubstationShelf = 100_deg;

} // namespace AutoConstants
namespace IntakeConstants {
constexpr int kPCMPort = 5;
constexpr int kPistonPort = 0;
constexpr int kRangefinderPort = 0;

constexpr int pickUpRangeCone = 0;
constexpr int pickUpRangeCube = 0;
} // namespace IntakeConstants

namespace OperatorConstants {
constexpr bool kTesting = true;
constexpr int kLeftJoystickPort = 1;
constexpr int kRightJoystickPort = 2;
constexpr int kXboxControllerPort = 0;
} // namespace OperatorConstants
