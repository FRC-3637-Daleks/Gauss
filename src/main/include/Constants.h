#pragma once

#include <numbers>

#include <frc/XboxController.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <photonlib/PhotonPoseEstimator.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

struct PIDCoefficients {
  double kP, kI, kD, kFF, kIz;
};

namespace ArmConstants {
constexpr int kPCMId = 13;
constexpr int kPistonChannel = 7;
constexpr int kClawChannel = 6;

constexpr int kMotorId = 12;

constexpr int kLimitSwitchChannel = 0;

constexpr auto kNeckPhysicalLowerBound = -5_deg;
constexpr auto kNeckPhysicalUpperBound = 120_deg;

// No pot for now.
// constexpr int kPotentiometerId = 0;
// constexpr double kVoltToLengthConversionFactor = 6.41;

// PID Loop
constexpr double kP = 10;
constexpr double kI = 2;
constexpr double kD = 0;
constexpr double kIz = 5;

constexpr auto kMaxTurnVelocity = 1 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 0.6 * std::numbers::pi * 1_rad_per_s_sq;

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

constexpr auto kMaxSpeed = 10_fps;
constexpr auto kArcadeMaxSpeed = 10_fps;
constexpr auto kPreciseSpeed = 2_fps;

// PID coefficients for closed-loop control of velocity.
constexpr double kFDriveSpeed = 0.0656;
constexpr double kPDriveSpeed = 0.1;
constexpr double kIDriveSpeed = 0.000;
constexpr double kDDriveSpeed = 0;
constexpr double kIzDriveSpeed = 1000;

constexpr double kIBrake = 0.0001;

// NOTE: Guess value!
constexpr double kPTurn = 2.2;
constexpr double kPDistance = 2;
constexpr auto kDistanceTolerance = 7_cm;

constexpr double kPLeftStraight = 0.2;
constexpr double kPRightStraight = 0.2;

constexpr auto kTurnTolerance = 3_deg;
constexpr auto kTurnRateTolerance = 1_deg_per_s;

constexpr auto kMaxTurnRate = 1 * std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = 1 * std::numbers::pi * 1_rad_per_s_sq;

// Swerve Constants (NEED TO INTEGRATE)

// left out as this variable are repeated above
// constexpr auto kTrackWidth =
//    20.25_in; // Distance between centers of right and left wheels.
constexpr auto kWheelBase =
    20_in; // Distance between centers of front and back wheels.

constexpr int kFrontLeftDriveMotorId = 1;
constexpr int kRearLeftDriveMotorId = 3;
constexpr int kFrontRightDriveMotorId = 5;
constexpr int kRearRightDriveMotorId = 7;

constexpr int kFrontLeftSteerMotorId = 2;
constexpr int kRearLeftSteerMotorId = 4;
constexpr int kFrontRightSteerMotorId = 6;
constexpr int kRearRightSteerMotorId = 8;

constexpr int kFrontLeftAbsoluteEncoderChannel = 0;
constexpr int kRearLeftAbsoluteEncoderChannel = 1;
constexpr int kFrontRightAbsoluteEncoderChannel = 2;
constexpr int kRearRightAbsoluteEncoderChannel = 3;

// Absolute encoder reading when modules are facing forward.
constexpr double kFrontLeftAbsoluteEncoderOffset = 3.15246;
constexpr double kRearLeftAbsoluteEncoderOffset = -2.25482; //3.9595;
constexpr double kFrontRightAbsoluteEncoderOffset = -2.03871;//4.28316;
constexpr double kRearRightAbsoluteEncoderOffset = 1.377484;
// constexpr double kFrontLeftAbsoluteEncoderOffset = -2.033;
// constexpr double kRearLeftAbsoluteEncoderOffset = -1.766;
// constexpr double kFrontRightAbsoluteEncoderOffset = -0.063;
// constexpr double kRearRightAbsoluteEncoderOffset = -2.246;

// XXX Roughly estimated values, needs to be properly tuned.
constexpr struct PIDCoefficients kFrontLeftDriveMotorPIDCoefficients {
  1e-4, 0, 0, 1.6e-4, 0 // 1e-5, 1e-6, 0, 1e-4, 0 ,0.25e-7, 1e-6, 1e-2, 1e-5, 0
};
constexpr struct PIDCoefficients kRearLeftDriveMotorPIDCoefficients {
  1e-4, 0, 0, 1.6e-4, 0
};
constexpr struct PIDCoefficients kFrontRightDriveMotorPIDCoefficients {
  1e-4, 0, 0, 1.6e-4, 0
};
constexpr struct PIDCoefficients kRearRightDriveMotorPIDCoefficients {
  1e-4, 0, 0, 1.6e-4, 0
};

constexpr struct PIDCoefficients kFrontLeftSteerMotorPIDCoefficients {
  3.3, 0, 0, 0, 0
};
constexpr struct PIDCoefficients kRearLeftSteerMotorPIDCoefficients {
  3, 0, 0, 0, 0
};
constexpr struct PIDCoefficients kFrontRightSteerMotorPIDCoefficients {
  3.2, 0, 0, 0, 0
};
constexpr struct PIDCoefficients kRearRightSteerMotorPIDCoefficients {
  3.5, 0, 0, 0, 0
};

constexpr auto kMaxTeleopSpeed = 20_fps;
// constexpr auto kPreciseSpeed = 2_fps; // left out because it already exists
// above

} // namespace DriveConstants

namespace ModuleConstants {
constexpr double kDriveMotorCurrentLimit = 50; // Up to 80 A is okay.
constexpr double kSteerMotorCurrentLimit = 20; // An educated guess.

constexpr double kMotorRampRate = 0.5; // Seconds from neutral to full output.

constexpr auto kWheelDiameter = 4_in;
constexpr double kDriveEncoderReduction =
    (double)20 / 3; // Reduction in the swerve module gearing.
constexpr double kDriveEncoderCPR = 42;
constexpr auto kDriveEncoderDistancePerRevolution =
    kWheelDiameter * std::numbers::pi / kDriveEncoderReduction;

constexpr double kSteerEncoderReduction =
    // Gearmotor reduction
    ((double)226233 / 3179) *
    // Module reduction
    ((double)40 / 48);
constexpr double kSteerEncoderCPR =
    kSteerEncoderReduction * 28; // CPR is 4 counts/cycle * 7 cycles/revolution.
constexpr auto kSteerEncoderDistancePerCount =
    2_rad * std::numbers::pi / kSteerEncoderCPR; // Radians per encoder count.

// Values measured with the drivetrain suspended.
constexpr auto kPhysicalMaxSpeed = 14_fps;
constexpr auto kPhysicalMaxAngularSpeed = 180_rpm;
} // namespace ModuleConstants

namespace ClawConstants {
constexpr int kPCMPort = 13;
constexpr int kPistonPort = 6;
constexpr int kLimitSwitchPort = 0;
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

constexpr auto kMaxSpeed = 1_mps;
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

// Swerve Constants (NEED TO BE INTEGRATED)
// constexpr auto kMaxSpeed = ModuleConstants::kPhysicalMaxSpeed / 3; // left
// out as these are repeat values constexpr auto kMaxAcceleration = 10_fps_sq;
constexpr auto kMaxAngularSpeed = 180_rpm;
constexpr auto kMaxAngularAcceleration = std::numbers::pi * 1_rad_per_s_sq;

// XXX Very untrustworthy placeholder values.
constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

// Trapezoidal motion profile for the robot heading.
const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints{kMaxAngularSpeed, kMaxAngularAcceleration};

} // namespace AutoConstants
namespace IntakeConstants {
constexpr int kPCMPort = 13;
constexpr int kPistonPort = 0;
constexpr int kLeftMotorPort = 10;
constexpr int kRightMotorPort = 11;
constexpr int kRangefinderPort = 0;

constexpr double kIntakeMotorSpeed = 0.5;
constexpr double kIntakeMotorSpeedReversed = -0.5;
constexpr int pickUpRangeCone = 0;
constexpr int pickUpRangeCube = 0;
} // namespace IntakeConstants

namespace OperatorConstants {
constexpr bool kTesting = true;
constexpr int kXboxControllerPort = 1;
constexpr int kDriverControllerPort = 0;

constexpr double kDeadband = 0.08;

constexpr int kStrafeAxis = frc::XboxController::Axis::kLeftX;
constexpr int kForwardAxis = frc::XboxController::Axis::kLeftY;
constexpr int kRotationAxis = frc::XboxController::Axis::kRightX;
constexpr int kFieldRelativeButton = frc::XboxController::Button::kRightBumper;

constexpr int kZeroHeadingButton = frc::XboxController::Button::kX;
constexpr int kResetModulesButton = frc::XboxController::Button::kY;
constexpr int kFreeModulesButton = frc::XboxController::Button::kA;
} // namespace OperatorConstants

// Define your robot's characteristics (TESTING)
// using for Trajectories
namespace TrajectoryConstants {

constexpr double kS = 1.0;
constexpr double kV = 0.8;
constexpr double kA = 0.2;
constexpr double kP = 0.05;
constexpr double kI = 0.0;
constexpr double kD = 0.0;

constexpr double kMaxSpeed = 3.0; // Maximum speed in meters per second
constexpr double kMaxAcceleration =
    2.0; // Maximum acceleration in meters per second squared
constexpr double kRamseteB = 2.0;    // Ramsete controller's B coefficient
constexpr double kRamseteZeta = 0.7; // Ramsete controller's Zeta coefficient

constexpr double kTrackWidth = 0.6; // Width of your robot's drivetrain
} // namespace TrajectoryConstants