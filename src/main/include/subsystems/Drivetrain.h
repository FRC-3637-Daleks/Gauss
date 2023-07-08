#pragma once

#include <AHRS.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "SwerveModule.h"

/**
 * The Drivetrain subsystem contains four swerve modules and a gyroscope. The
 * Drivetrain can be driven at a certain speed represented as a 2-dimensional
 * translation on the field. This translation can be commanded either relative
 * to the front of the robot, or relative to a certain absolute position. These
 * two modes are called "robot relative" and "field relative" respectively. The
 * drivetrain can also be commanded to rotate at a given speed independent of
 * its translation.
 */

class Drivetrain : public frc2::SubsystemBase {
public:
  // The ctor of the Drivetrain subsystem.
  Drivetrain();

  // Updates the odometer and SmartDashboard.
  void Periodic() override;

  // Drives the robot at the given translational and rotational speed. Forward
  // is front-back movement, strafe is left-right movement. If fieldRelative is
  // set, the translation is relative to the field instead of the robot heading.
  void Drive(units::meters_per_second_t forwardSpeed,
             units::meters_per_second_t strafeSpeed,
             units::radians_per_second_t angularSpeed, bool fieldRelative);

  // Sets the state of each swerve module.
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  // Resets the module encoders.
  void ResetModules();

  // Returns the heading of the robot.
  frc::Rotation2d GetHeading();

  // Zeroes the robot heading.
  void ZeroHeading();

  // Returns the rotational velocity of the robot in degrees per second.
  units::degrees_per_second_t GetTurnRate();

  // Returns the robot heading and translation as a Pose2d.
  frc::Pose2d GetPose();

  // Resets the odometry using the given a field-relative pose using current
  // gyro angle.
  void ResetOdometry(const frc::Pose2d &pose);

  // The kinematics model for a swerve drive. Once given the positions of each
  // module with the robot's center as the origin, the SwerveDriveKinematics
  // class will convert between chassis speeds (translation and rotation) and
  // module states (velocity and angle).
  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2),
      frc::Translation2d(DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2),
      frc::Translation2d(-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2)};

  // Display useful information on Shuffleboard.
  void UpdateDashboard();

  // Drive the robot with swerve controls.
  frc2::CommandPtr SwerveCommand(std::function<double()> forward,
                                 std::function<double()> strafe,
                                 std::function<double()> rot,
                                 std::function<bool()> fieldRelative);

  // Returns a command that zeroes the robot heading.
  frc2::CommandPtr ZeroHeadingCommand();

  // Returns a command that resets the module encoders.
  frc2::CommandPtr ResetModulesCommand();

  // Returns a command that stops the robot.
  frc2::CommandPtr BrakeCommand();

  // Add Vision Pose to SwerveDrivePoseEstimator.
  void AddVisionPoseEstimate(frc::Pose2d pose, units::second_t timestamp);

private:
  SwerveModule m_frontLeft;
  SwerveModule m_rearLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearRight;

  AHRS m_gyro;

  // Odometer for tracking the robot's position on the field.
  frc::SwerveDriveOdometry<4> m_odometry;

  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Field widget for Shuffleboard.
  frc::Field2d m_field;
};
