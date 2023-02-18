#pragma once

#include <numbers>

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>

namespace ArmConstants {
constexpr int kPCM = 3;
constexpr int kPistonId = 0;
constexpr int kMotorId = 5;
constexpr int kPotentiometerId = 0;
constexpr int kLimitSwitchId = 0;
constexpr double kVoltToLengthConversionFactor = 6.41;

// Feedforward Loop
constexpr auto kS = 0_V;
constexpr auto kG = 0_V;
constexpr auto kV = 0_V * 0_s / 1_rad;
constexpr auto kA = 0_V * 0_s * 0_s / 1_rad;

// PID Loop
constexpr double kP = 0;
constexpr double kI = 0;
constexpr double kD = 0;

constexpr auto kMaxTurnVelocity = std::numbers::pi * 1_rad_per_s;
constexpr auto kMaxTurnAcceleration = std::numbers::pi * 1_rad_per_s_sq;

constexpr int kEncoderCPR = 2048;
constexpr double kMotorRadiansPerTick = (2 * std::numbers::pi) / kEncoderCPR;
constexpr double kArmGearRatio = 0.25;
constexpr auto kArmOffset = units::radian_t{0};
const auto kArmRadiansPerEncoderTick = kArmGearRatio * kMotorRadiansPerTick;

constexpr bool kDefaultPosition = false;
} // namespace ArmConstants

namespace OperatorConstants {

constexpr int kXboxController = 0;

} // namespace OperatorConstants
