#pragma once

#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/length.h>

#include "Constants.h"

class DalekDrive : public frc2::SubsystemBase {
public:
  DalekDrive();

  void Log();

  void InitDriveMotors();

  void Drive(double left, double right, bool squareInputs);

  void TankDrive(double leftSpeed, double rightSpeed, bool squareInputs);

  void ArcadeDrive(double forward, double rotation, bool squareInputs);

  units::meter_t GetDistance();

  units::degree_t GetHeading() const;

  void Reset();

  frc::Pose2d GetPose() const;

  void ResetOdometry(const frc::Pose2d &pose);

  void Periodic() override;

private:
  WPI_TalonFX m_leftFront;
  WPI_TalonFX m_leftFollower;
  WPI_TalonFX m_rightFront;
  WPI_TalonFX m_rightFollower;

  frc::DifferentialDrive m_drive;

  AHRS m_gyro;

  frc::DifferentialDriveOdometry m_odometry;
  frc::Field2d m_field;

  void SetWheelSpeeds(units::meters_per_second_t leftSpeed,
                      units::meters_per_second_t rightSpeed);
};
