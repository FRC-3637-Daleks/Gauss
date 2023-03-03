#pragma once

#include <AHRS.h>
#include <ctre/Phoenix.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/estimator/DifferentialDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
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

  void TankDrive(units::meters_per_second_t left,
                 units::meters_per_second_t right);

  void PreciseDrive(double leftSpeed, double rightSpeed, bool squareInputs);

  void TankDrive(double leftSpeed, double rightSpeed, bool squareInputs);

  void ArcadeDrive(double forward, double rotation, bool squareInputs);

  frc2::CommandPtr BrakeCommand();

  frc2::CommandPtr TurnToAngleCommand(units::degree_t target);

  frc2::CommandPtr TurnToPoseCommand(std::function<double()> getForward,
                                     std::function<frc::Pose2d()> getTarget);

  frc2::CommandPtr DriveToDistanceCommand(units::meter_t distance);

  frc2::CommandPtr DriveStraightCommand(units::meters_per_second_t forward);

  frc2::CommandPtr BalanceCommand();

  units::meter_t GetDistance();

  units::degree_t GetHeading() const;

  void AddVisionPoseEstimate(frc::Pose2d pose, units::second_t timestamp);

  units::degree_t GetPitch();

  void Reset();

  frc::Pose2d GetPose() const;

  void ResetOdometry(const frc::Pose2d &pose);

  void Periodic() override;

  void InitTest();

  void UpdatePIDValues();

private:
  WPI_TalonFX m_leftFront;
  WPI_TalonFX m_leftFollower;
  WPI_TalonFX m_rightFront;
  WPI_TalonFX m_rightFollower;

  frc::DifferentialDrive m_drive;

  AHRS m_gyro;

  frc::DifferentialDriveKinematics m_kinematics{DriveConstants::kTrackWidth};
  frc::DifferentialDrivePoseEstimator m_poseEstimator;
  frc::Field2d m_field;

  frc::ProfiledPIDController<units::radian> m_turnController;
  frc::PIDController m_distanceController;
  frc::PIDController m_balanceController;

  void SetWheelSpeeds(units::meters_per_second_t leftSpeed,
                      units::meters_per_second_t rightSpeed);
};
