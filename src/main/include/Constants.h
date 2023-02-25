#pragma once

#include <numbers>

#include <units/length.h>

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

namespace IntakeConstants {
constexpr int kPCMPort = 5;
constexpr int kPistonPort = 0;
constexpr int kRangefinderPort = 0;

constexpr int pickUpRangeCone = 0;
constexpr int pickUpRangeCube = 0;
} // namespace IntakeConstants

namespace OperatorConstants {
constexpr int kLeftJoystickPort = 1;
constexpr int kRightJoystickPort = 2;
constexpr int kXboxControllerPort = 3;
} // namespace OperatorConstants
