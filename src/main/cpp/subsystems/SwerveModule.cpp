#include "subsystems/SwerveModule.h"

#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

using namespace ModuleConstants;

SwerveModule::SwerveModule(const std::string name, const int driveMotorId,
                           const int steerMotorId,
                           const int absoluteEncoderChannel,
                           const double absoluteEncoderOffset,
                           const PIDCoefficients driveMotorPIDCoefficients,
                           const PIDCoefficients steerMotorPIDCoefficients)
    : m_name{name},
      m_driveMotor(driveMotorId,
                   rev::CANSparkMaxLowLevel::MotorType::kBrushless),
      m_drivePIDController(m_driveMotor.GetPIDController()),
      m_driveEncoder(m_driveMotor.GetEncoder(
          rev::SparkMaxRelativeEncoder::Type::kHallSensor, kDriveEncoderCPR)),
      m_steerMotor(steerMotorId),
      // Have the absolute encoder return radian values.
      m_absoluteEncoder(absoluteEncoderChannel, 2 * std::numbers::pi,
                        -absoluteEncoderOffset),
      m_drivePIDCoefficients{driveMotorPIDCoefficients},
      m_steerPIDCoefficients{steerMotorPIDCoefficients} {
  // Reset the drive and steer motor controllers to their default settings,
  // then configure them for use.
  m_driveMotor.RestoreFactoryDefaults();
  m_steerMotor.ConfigFactoryDefault();
  m_driveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  m_steerMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
  m_driveMotor.SetClosedLoopRampRate(kMotorRampRate);
  m_driveMotor.SetOpenLoopRampRate(kMotorRampRate);
  m_steerMotor.ConfigClosedloopRamp(kMotorRampRate);
  m_steerMotor.ConfigOpenloopRamp(kMotorRampRate);
  // Hopefully prevents brownouts.
  m_driveMotor.SetSmartCurrentLimit(kDriveMotorCurrentLimit);
  m_steerMotor.ConfigContinuousCurrentLimit(kSteerMotorCurrentLimit);
  m_drivePIDController.SetP(m_drivePIDCoefficients.kP);
  m_drivePIDController.SetI(m_drivePIDCoefficients.kI);
  m_drivePIDController.SetD(m_drivePIDCoefficients.kD);
  m_drivePIDController.SetFF(m_drivePIDCoefficients.kFF);
  m_steerMotor.Config_kP(0, m_steerPIDCoefficients.kP, 0);
  m_steerMotor.Config_kI(0, m_steerPIDCoefficients.kI, 0);
  m_steerMotor.Config_kD(0, m_steerPIDCoefficients.kD, 0);
  m_steerMotor.ConfigSelectedFeedbackSensor(
      ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, 0);
  // Since the steer encoders are mounted on the back of their gearmotor, they
  // count opposite the axle's rotation.
  m_steerMotor.SetSensorPhase(true);
  // m_driveMotor.BurnFlash();
}

units::meter_t SwerveModule::GetModuleDistance() {
  return kDriveEncoderDistancePerRevolution * m_driveEncoder.GetPosition();
}

units::meters_per_second_t SwerveModule::GetModuleVelocity() {
  return kDriveEncoderDistancePerRevolution * m_driveEncoder.GetVelocity() /
         60_s;
}

frc::Rotation2d SwerveModule::GetModuleHeading() {
  return frc::AngleModulus(kSteerEncoderDistancePerCount *
                           m_steerMotor.GetSelectedSensorPosition());
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return {GetModuleDistance(), GetModuleHeading()};
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetModuleVelocity(), GetModuleHeading()};
}

void SwerveModule::SetDesiredState(
    const frc::SwerveModuleState &referenceState) {
  // Optimize the reference state to prevent the module turning >90 degrees.
  const auto state =
      frc::SwerveModuleState::Optimize(referenceState, GetModuleHeading());

  // Set the motor outputs.
  m_drivePIDController.SetReference(
      ToSparkUnits(state.speed), rev::CANSparkMax::ControlType::kVelocity, 0,
      // Drive feedforward is %output / native max speed.
      (state.speed / kPhysicalMaxSpeed * 3) / ToSparkUnits(kPhysicalMaxSpeed),
      rev::SparkMaxPIDController::ArbFFUnits::kPercentOut);
  m_steerMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position,
                   ToTalonUnits(state.angle));
  frc::SmartDashboard::PutNumber(fmt::format("{}/angle", m_name),
                                 state.angle.Degrees().value());
  frc::SmartDashboard::PutNumber(
      fmt::format("{}/velocity output (mps)", m_name), state.speed.value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive percent out", m_name),
                                 m_driveMotor.Get());
  frc::SmartDashboard::PutNumber(fmt::format("{}/steer percent out", m_name),
                                 m_steerMotor.Get());

  // fmt::print(
  //     fmt::format("{} steer percent out {}", m_name, m_steerMotor.Get()));
  // fmt::print(
  //     fmt::format("{} drive percent out {}", m_name, m_driveMotor.Get()));
}

void SwerveModule::ResetEncoders() {
  m_driveEncoder.SetPosition(0);
  m_steerMotor.SetSelectedSensorPosition(
      ToTalonUnits(GetAbsoluteEncoderPosition()), 0, 0);
}

// TODO Display things neater on the SmartDashboard.
void SwerveModule::UpdateDashboard() {
  const auto state = GetState();
  frc::SmartDashboard::PutString(fmt::format("{}/module state", m_name),
                                 fmt::format("{:4f}@{:4f}°",
                                             state.speed.value(),
                                             state.angle.Degrees().value()));
  frc::SmartDashboard::PutNumber(fmt::format("{}/MA3 angle", m_name),
                                 m_absoluteEncoder.Get());
  frc::SmartDashboard::PutNumber(fmt::format("{}/absolute position", m_name),
                                 GetAbsoluteEncoderPosition().value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/velocity state (mps)", m_name),
                                 state.speed.value());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive voltage", m_name),
                                 m_driveMotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber(fmt::format("{}/turn voltage", m_name),
                                 m_steerMotor.GetBusVoltage());
  frc::SmartDashboard::PutNumber(fmt::format("{}/drive current", m_name),
                                 m_driveMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber(fmt::format("{}/turn current", m_name),
                                 m_steerMotor.GetOutputCurrent());
  // frc::SmartDashboard::PutNumber(fmt::format("{} drive raw", m_name),
  //                                m_driveEncoder.GetVelocity());
  // frc::SmartDashboard::PutNumber(fmt::format("{} steer raw", m_name),
  //                                m_steerMotor.GetSelectedSensorPosition());
}

double SwerveModule::ToSparkUnits(units::meters_per_second_t speed) {
  return speed * 60_s / kDriveEncoderDistancePerRevolution;
}

double SwerveModule::ToTalonUnits(const frc::Rotation2d &rotation) {
  units::radian_t currentHeading =
      kSteerEncoderDistancePerCount * m_steerMotor.GetSelectedSensorPosition();
  // Puts the rotation in the correct scope for the incremental encoder.
  return (frc::AngleModulus(rotation.Radians() - currentHeading) +
          currentHeading) /
         kSteerEncoderDistancePerCount;
}

units::radian_t SwerveModule::GetAbsoluteEncoderPosition() {
  return frc::AngleModulus(m_absoluteEncoder.Get() * 1_rad);
}
