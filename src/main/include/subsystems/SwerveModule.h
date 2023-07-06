#pragma once

#include <ctre/Phoenix.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>

#include "Constants.h"

/**
 * The SwerveModule helper class consists of a steer motor (TalonSRX controller)
 * and a drive motor (SPARKMAX controller). Each module can be commanded to a
 * certain state, that is, its wheel will be driven at the specified velocity in
 * the specified direction. The Drivetrain subsystem makes use of SwerveModule
 * objects so that it doesn't need to deal with directly commanding each motor.
 */

class SwerveModule {
public:
  // The ctor of the SwerveModule class.
  SwerveModule(const std::string name, const int driveMotorId,
               const int steerMotorId, const int absoluteEncoderChannel,
               const double absoluteEncoderOffset,
               const PIDCoefficients driveMotorPIDCoefficients,
               const PIDCoefficients steerMotorPIDCoefficients);

  // Returns the meters driven based on encoder reading.
  units::meter_t GetModuleDistance();

  // Returns the velocity of the module in m/s.
  units::meters_per_second_t GetModuleVelocity();

  // Returns the module heading in the scope [-180,180] degrees.
  frc::Rotation2d GetModuleHeading();

  // Combines GetModuleDistance() and GetModuleHeading().
  frc::SwerveModulePosition GetPosition();

  // Combines GetModuleVelocity() and GetModuleHeading().
  frc::SwerveModuleState GetState();

  // Commands the module to accelerate to a certain velocity and take on a
  // certain heading.
  void SetDesiredState(const frc::SwerveModuleState &state);

  // Zeroes the drive and steer encoders.
  void ResetEncoders();

  // Sends the current swerve module state to the SmartDashboard.
  void UpdateDashboard();

private:
  // Converts m/s to rpm for the drive velocity setpoint.
  double ToSparkUnits(units::meters_per_second_t speed);

  // Converts an angle to Talon sensor units for the steer position setpoint.
  double ToTalonUnits(const frc::Rotation2d &rotation);

  // Returns the absolute position of the steer motor in Talon sensor units.
  units::radian_t GetAbsoluteEncoderPosition();

  const std::string m_name; // Useful to identify the module.

  rev::CANSparkMax m_driveMotor;
  rev::SparkMaxPIDController m_drivePIDController;
  rev::SparkMaxRelativeEncoder m_driveEncoder;

  ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_steerMotor;

  // Keeps track of the module heading between power cycles.
  frc::AnalogPotentiometer m_absoluteEncoder;

  PIDCoefficients m_drivePIDCoefficients;
  PIDCoefficients m_steerPIDCoefficients;
};
