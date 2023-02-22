#include "subsystems/DalekDrive.h"

#include <frc/SPI.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace DriveConstants;

DalekDrive::DalekDrive()
    : m_leftFront{kLeftFrontMotorId}, m_leftFollower{kLeftRearMotorId},
      m_rightFront{kRightFrontMotorId}, m_rightFollower{kRightRearMotorId},
      m_drive{m_leftFront, m_rightFront}, m_gyro{frc::SPI::Port::kMXP},
      m_odometry{
          m_gyro.GetRotation2d(),
          kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
          kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition(),
          frc::Pose2d{}} {
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
}

void DalekDrive::Drive(double left, double right, bool squareInputs) {
  m_drive.TankDrive(left, right, squareInputs);
}

units::meter_t DalekDrive::GetDistance() {
  // Average encoder readings to get distance driven.
  return kEncoderDistancePerPulse *
         (m_leftFront.GetSelectedSensorPosition() +
          m_rightFront.GetSelectedSensorPosition()) /
         2;
}

units::degree_t DalekDrive::GetHeading() const {
  return -m_gyro.GetRotation2d().Degrees();
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
