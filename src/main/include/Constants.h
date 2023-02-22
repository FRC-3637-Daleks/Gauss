#pragma once

#include <numbers>

#include <frc/controller/ArmFeedforward.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>

namespace ArmConstants {
constexpr int kPCMId = 3;
constexpr int kPistonChannel = 0;

constexpr int kMotorId = 8;

constexpr int kLimitSwitchChannel = 0;

// No pot for now.
// constexpr int kPotentiometerId = 0;
// constexpr double kVoltToLengthConversionFactor = 6.41;

// NOTE Feedforward control may be unnecessary, considering the gas spring.
// Feedforward Loop
constexpr auto kS = 0_V;                     // Static gain
constexpr auto kG = 0_V;                     // Gravity gain
constexpr auto kV = 0_V * 0_s / 1_rad;       // Velocity gain
constexpr auto kA = 0_V * 0_s * 0_s / 1_rad; // Acceleration gain
const frc::ArmFeedforward kFeedForward = {kS, kG, kV, kA};

// PID Loop
constexpr double kP = 0;
constexpr double kI = 0;
constexpr double kD = 0;

constexpr auto kMaxTurnVelocity = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = std::numbers::pi * 1_rad_per_s_sq;

constexpr int kEncoderCPR = 2048;
constexpr double kGearReduction = 4;
constexpr auto kEncoderRotationPerPulse =
    2_rad * std::numbers::pi / (kEncoderCPR * kGearReduction);

constexpr auto kOffset = units::radian_t{0}; // Angle of the arm at the stop.

constexpr bool kDefaultPosition = false;
} // namespace ArmConstants

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

constexpr double kTalonRampRate =
    0.5; // 0.5 seconds from neutral to full throttle.
constexpr int kTalonTimeoutMs = 30;
} // namespace DriveConstants

namespace OperatorConstants {
constexpr int kLeftJoystickPort = 1;
constexpr int kRightJoystickPort = 2;
constexpr int kXboxControllerPort = 3;
} // namespace OperatorConstants
