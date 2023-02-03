#pragma once

#include <numbers>

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
} // namespace DriveConstants

namespace OperatorConstants {
constexpr int kLeftJoystickPort = 1;
constexpr int kRightJoystickPort = 2;
constexpr int kXboxControllerPort = 3;
} // namespace OperatorConstants
