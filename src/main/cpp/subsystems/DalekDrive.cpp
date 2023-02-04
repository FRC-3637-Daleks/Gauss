#include "subsystems/DalekDrive.h"

#include <cmath>

#include <frc/SPI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>

using namespace DriveConstants;

DalekDrive::DalekDrive()
    : m_leftFront{kLeftFrontMotorId}, m_leftFollower{kLeftRearMotorId},
      m_rightFront{kRightFrontMotorId}, m_rightFollower{kRightRearMotorId},
      m_drive{m_leftFront, m_rightFront}, m_gyro{frc::SPI::Port::kMXP},
      m_odometry{
          m_gyro.GetRotation2d(),
          kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
          kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition(),
          frc::Pose2d{}},
      m_turnController{kPTurn, 0, 0, {kMaxTurnRate, kMaxTurnAcceleration}},
      m_distanceController{
          kPDistance,
          0,
          0,
          {AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration}} {
  m_turnController.EnableContinuousInput(-180_deg, 180_deg);
  m_turnController.SetTolerance(kTurnTolerance, kTurnRateTolerance);

  InitDriveMotors();
}

void DalekDrive::Log() {
  frc::SmartDashboard::PutNumber("Left Distance",
                                 m_leftFront.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Right Distance",
                                 m_rightFront.GetSelectedSensorPosition());
  frc::SmartDashboard::PutNumber("Left Speed",
                                 m_leftFront.GetSelectedSensorVelocity());
  frc::SmartDashboard::PutNumber("Right Speed",
                                 m_rightFront.GetSelectedSensorVelocity());

  frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutData("Field", &m_field);
}

void DalekDrive::InitDriveMotors() {
  // Wipe whatever was on the controllers beforehand.
  m_leftFront.ConfigFactoryDefault();
  m_leftFollower.ConfigFactoryDefault();
  m_rightFront.ConfigFactoryDefault();
  m_rightFollower.ConfigFactoryDefault();
  // Set up the followers.
  m_leftFollower.Follow(m_leftFront);
  m_rightFollower.Follow(m_rightFront);
  // Invert one side of the robot.
  m_leftFront.SetInverted(TalonFXInvertType::Clockwise);
  m_leftFollower.SetInverted(TalonFXInvertType::FollowMaster);
  m_rightFront.SetInverted(TalonFXInvertType::CounterClockwise);
  m_rightFollower.SetInverted(TalonFXInvertType::FollowMaster);
  // Brake when no voltage is applied to the motor.
  m_leftFront.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
  m_rightFront.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);
  // Set voltage ramp rate for motors.
  m_leftFront.ConfigOpenloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_leftFront.ConfigClosedloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_leftFollower.ConfigOpenloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_leftFollower.ConfigClosedloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_rightFront.ConfigOpenloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_rightFront.ConfigClosedloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_rightFollower.ConfigOpenloopRamp(kTalonRampRate, kTalonTimeoutMs);
  m_rightFollower.ConfigClosedloopRamp(kTalonRampRate, kTalonTimeoutMs);
  // Set PIDF values. PID slot 0 uses the Falcon 500's integrated encoder.
  m_leftFront.Config_kF(0, kFDriveSpeed, kTalonTimeoutMs);
  m_leftFront.Config_kP(0, kPDriveSpeed, kTalonTimeoutMs);
  m_leftFront.Config_kI(0, kIDriveSpeed, kTalonTimeoutMs);
  m_leftFront.Config_kD(0, kDDriveSpeed, kTalonTimeoutMs);
  m_leftFront.Config_IntegralZone(0, kIzDriveSpeed, kTalonTimeoutMs);
  m_rightFront.Config_kF(0, kFDriveSpeed, kTalonTimeoutMs);
  m_rightFront.Config_kP(0, kPDriveSpeed, kTalonTimeoutMs);
  m_rightFront.Config_kI(0, kIDriveSpeed, kTalonTimeoutMs);
  m_rightFront.Config_kD(0, kDDriveSpeed, kTalonTimeoutMs);
  m_rightFront.Config_IntegralZone(0, kIzDriveSpeed, kTalonTimeoutMs);
}

void DalekDrive::Drive(double left, double right, bool squareInputs) {
  m_drive.TankDrive(left, right, squareInputs);
}

void DalekDrive::TankDrive(double leftSpeed, double rightSpeed,
                           bool squareInputs) {
  auto [left, right] = m_drive.TankDriveIK(leftSpeed, rightSpeed, squareInputs);

  SetWheelSpeeds(left * kMaxSpeed, right * kMaxSpeed);
  m_drive.Feed();
}

void DalekDrive::ArcadeDrive(double forward, double rotation,
                             bool squareInputs) {
  auto [left, right] = m_drive.TankDriveIK(forward, rotation, squareInputs);

  SetWheelSpeeds(left * kMaxSpeed, right * kMaxSpeed);
  m_drive.Feed();
}

frc2::CommandPtr DalekDrive::TurnToAngleCommand(units::degree_t target) {
  return frc2::FunctionalCommand(
             // Set controller input to current heading.
             [this] {
               Reset();
               m_turnController.Reset(GetHeading());
             },
             // Use output from PID controller to turn robot.
             [this, &target] {
               double output = m_turnController.Calculate(GetHeading(), target);
               ArcadeDrive(0, output, false);
             },
             // Stop robot.
             [this](bool) -> void { ArcadeDrive(0, 0, false); },
             [this]() -> bool { return m_turnController.AtGoal(); }, {this})
      .ToPtr();
}

frc2::CommandPtr DalekDrive::DriveToDistance(units::meter_t target) {
  return frc2::FunctionalCommand(
             // Set controller input to current heading.
             [this] {
               Reset();
               m_distanceController.Reset(GetDistance());
             },
             // Use output from PID controller to turn robot.
             [this, &target] {
               double output =
                   m_distanceController.Calculate(GetDistance(), target);
               TankDrive(output, output, false);
             },
             // Stop robot.
             [this](bool) -> void { TankDrive(0, 0, false); },
             [this]() -> bool { return m_distanceController.AtGoal(); }, {this})
      .ToPtr();
}

units::meter_t DalekDrive::GetDistance() {
  // Average encoder readings to get distance driven.
  return kEncoderDistancePerPulse *
         (m_leftFront.GetSelectedSensorPosition() +
          m_rightFront.GetSelectedSensorPosition()) /
         2;
}

units::degree_t DalekDrive::GetHeading() const {
  return units::degree_t{std::remainder(m_gyro.GetAngle(), 360) *
                         (kGyroReversed ? -1.0 : 1.0)};
}

void DalekDrive::Reset() {
  m_gyro.Reset();
  m_rightFront.SetSelectedSensorPosition(0);
  m_leftFront.SetSelectedSensorPosition(0);
}

frc::Pose2d DalekDrive::GetPose() const { return m_odometry.GetPose(); }

void DalekDrive::ResetOdometry(const frc::Pose2d &pose) {
  m_odometry.ResetPosition(
      m_gyro.GetRotation2d(),
      kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
      kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition(),
      pose);
}

void DalekDrive::Periodic() {
  Log();

  m_odometry.Update(
      m_gyro.GetRotation2d(),
      kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
      kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition());

  m_field.SetRobotPose(GetPose());
}

void DalekDrive::SetWheelSpeeds(units::meters_per_second_t leftSpeed,
                                units::meters_per_second_t rightSpeed) {
  m_leftFront.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                  leftSpeed / kEncoderDistancePerPulse / (double)10 * 1_s);
  m_rightFront.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                   rightSpeed / kEncoderDistancePerPulse / (double)10 * 1_s);
}
