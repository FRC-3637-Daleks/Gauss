#include "subsystems/DalekDrive.h"

#include <cmath>

#include <frc/SPI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>

using namespace DriveConstants;

DalekDrive::DalekDrive()
    : m_leftFront{kLeftFrontMotorId}, m_leftFollower{kLeftRearMotorId},
      m_rightFront{kRightFrontMotorId}, m_rightFollower{kRightRearMotorId},
      m_drive{m_leftFront, m_rightFront}, m_gyro{frc::SPI::Port::kMXP},
      m_poseEstimator{
          m_kinematics, m_gyro.GetRotation2d(),
          kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
          kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition(),
          frc::Pose2d()},
      m_turnController{kPTurn, 0, 0, {kMaxTurnRate, kMaxTurnAcceleration}},
      m_distanceController{kPDistance, 0, 0}, m_balanceController{
                                                  AutoConstants::kPBalance,
                                                  AutoConstants::kIBalance,
                                                  AutoConstants::kDBalance,
                                              } {
  m_turnController.EnableContinuousInput(-180_deg, 180_deg);
  m_turnController.SetTolerance(kTurnTolerance, kTurnRateTolerance);

  m_drive.SetMaxOutput(kMaxOutput);

  InitDriveMotors();

  if (OperatorConstants::kTesting) {
    InitTest();
  }
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

  frc::SmartDashboard::PutNumber("Right Voltage",
                                 m_rightFront.GetMotorOutputVoltage());
  frc::SmartDashboard::PutNumber("Left Voltage",
                                 m_leftFront.GetMotorOutputVoltage());
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

// TODO: Refactor to use command factories for teleop driving (leaving just this
// method)
void DalekDrive::TankDrive(units::meters_per_second_t left,
                           units::meters_per_second_t right) {
  SetWheelSpeeds(left, right);
  m_drive.Feed();
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

void DalekDrive::PreciseDrive(double leftSpeed, double rightSpeed,
                              bool squareInputs) {
  auto [left, right] = m_drive.TankDriveIK(leftSpeed, rightSpeed, squareInputs);

  SetWheelSpeeds(left * kPreciseSpeed, right * kPreciseSpeed);
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

frc2::CommandPtr
DalekDrive::TurnToPoseCommand(std::function<double()> getForward,
                              std::function<frc::Pose2d()> getTarget) {
  return frc2::FunctionalCommand(
             // Set controller input to current heading.
             [this] { m_turnController.Reset(GetHeading()); },
             // Use output from PID controller to turn robot.
             [this, &getForward, &getTarget] {
               auto target = getTarget();
               auto relativePose = target.RelativeTo(GetPose());
               fmt::print("{} {}", relativePose.Rotation().Degrees().value(),
                          target.Rotation().Degrees().value());
               double output =
                   m_turnController.Calculate(relativePose.Rotation().Radians(),
                                              target.Rotation().Radians());
               ArcadeDrive(getForward(), output, false);
             },
             // Stop robot
             [this](bool) -> void { ArcadeDrive(0, 0, false); },
             [this]() -> bool { return m_turnController.AtGoal(); }, {this})
      .ToPtr();
}

frc2::CommandPtr DalekDrive::DriveToDistanceCommand(units::meter_t target) {
  return frc2::FunctionalCommand(
             // Set controller input to current distance.
             [this, &target] {
               Reset();
               m_distanceController.Reset();
               m_distanceController.SetTolerance(
                   units::meter_t{kDistanceTolerance}.value());
               m_distanceController.SetSetpoint(units::meter_t{1.5_ft}.value());
               fmt::print("Running DriveToDistance Command.\n");
             },
             // Use output from PID controller to drive robot.
             [this, &target] {
               units::meters_per_second_t output =
                   std::clamp<units::meters_per_second_t>(
                       units::meters_per_second_t{
                           m_distanceController.Calculate(
                               GetDistance().value())},
                       -AutoConstants::kMaxSpeed, AutoConstants::kMaxSpeed);
               frc::SmartDashboard::PutNumber("DriveToDistance output",
                                              output.value());
               frc::SmartDashboard::PutNumber(
                   "DriveToDistance setpoint",
                   m_distanceController.GetSetpoint());
               frc::SmartDashboard::PutNumber(
                   "DriveToDistance error",
                   m_distanceController.GetPositionError());

               frc::SmartDashboard::PutNumber("DriveToDistance Target {}\n",
                                              units::meter_t{target}.value());
               fmt::print("output: {}\n", output.value());

               TankDrive(output, output);
             },
             // Stop robot.
             [this](bool) -> void { TankDrive(0, 0, false); },
             [this]() -> bool { return m_distanceController.AtSetpoint(); },
             {this})
      .ToPtr();
}

frc2::CommandPtr DalekDrive::BalanceCommand() {
  return frc2::FunctionalCommand(
             // Set controller input to current heading.
             [this]() {
               m_balanceController.SetTolerance(
                   AutoConstants::kBalanceTolerance); // In degrees
               m_balanceController.SetSetpoint(0);
               m_balanceController.EnableContinuousInput(-180, 180);
             },
             // Use output from PID controller to keep the robot balanced.
             [this]() {
               auto output = std::clamp<units::meters_per_second_t>(
                   units::meters_per_second_t{
                       m_balanceController.Calculate(GetPitch().value())},
                   -AutoConstants::kMaxSpeed, AutoConstants::kMaxSpeed);
               frc::SmartDashboard::PutNumber("Balance output", output.value());
               frc::SmartDashboard::PutNumber(
                   "Balance setpoint", m_balanceController.GetSetpoint());
               TankDrive(-output, -output);
             },
             // Stop robot once balanced.
             [this](bool) -> void { TankDrive(0, 0, false); },
             [this]() -> bool { return m_balanceController.AtSetpoint(); },
             {this})
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

units::degree_t DalekDrive::GetPitch() {
  return units::degree_t{m_gyro.GetPitch()};
}

void DalekDrive::Reset() {
  m_gyro.Reset();
  m_rightFront.SetSelectedSensorPosition(0);
  m_leftFront.SetSelectedSensorPosition(0);
}

frc::Pose2d DalekDrive::GetPose() const {
  return m_poseEstimator.GetEstimatedPosition();
}

void DalekDrive::ResetOdometry(const frc::Pose2d &pose) {
  m_poseEstimator.ResetPosition(
      m_gyro.GetRotation2d(),
      kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
      kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition(),
      pose);
}

void DalekDrive::Periodic() {
  Log();

  if (OperatorConstants::kTesting) {
    // PeriodicTest();
    frc::SmartDashboard::PutNumber("Gyro Pitch", m_gyro.GetPitch());
  }

  m_poseEstimator.Update(
      m_gyro.GetRotation2d(),
      kEncoderDistancePerPulse * m_leftFront.GetSelectedSensorPosition(),
      kEncoderDistancePerPulse * m_rightFront.GetSelectedSensorPosition());

  m_field.SetRobotPose(GetPose());
}

void DalekDrive::InitTest() {
  frc::SmartDashboard::PutNumber("Velocity Kf", kFDriveSpeed);
  frc::SmartDashboard::PutNumber("Velocity Kp", kPDriveSpeed);
  frc::SmartDashboard::PutNumber("Velocity Ki", kIDriveSpeed);
  frc::SmartDashboard::PutNumber("Velocity Kd", kDDriveSpeed);
  frc::SmartDashboard::PutNumber("Velocity KIz", kIzDriveSpeed);

  // for auton.
  frc::SmartDashboard::PutNumber("Velocity kPBalance",
                                 AutoConstants::kPBalance);
  frc::SmartDashboard::PutNumber("Velocity kIBalance",
                                 AutoConstants::kIBalance);
  frc::SmartDashboard::PutNumber("Velocity kPBalance",
                                 AutoConstants::kDBalance);
  frc::SmartDashboard::PutNumber("Velocity kPDistance", kPDistance);

  frc::SmartDashboard::PutNumber("DriveToDistance output",
                                 m_distanceController.Calculate(0, 0));
  frc::SmartDashboard::PutNumber("DriveToDistance setpoint",
                                 m_distanceController.GetSetpoint());

  frc::SmartDashboard::PutNumber("Balance output", 0);
  frc::SmartDashboard::PutNumber("Balance setpoint",
                                 m_balanceController.GetSetpoint());

  frc::SmartDashboard::PutNumber("Right Voltage",
                                 m_rightFront.GetMotorOutputVoltage());
  frc::SmartDashboard::PutNumber("Left Voltage",
                                 m_leftFront.GetMotorOutputVoltage());
  frc::SmartDashboard::PutNumber("Left Current",
                                 m_leftFront.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Right Current",
                                 m_rightFront.GetOutputCurrent());
}

void DalekDrive::UpdatePIDValues() {
  m_leftFront.Config_kF(
      0, frc::SmartDashboard::GetNumber("Velocity Kf", kFDriveSpeed), 0);
  m_leftFront.Config_kP(
      0, frc::SmartDashboard::GetNumber("Velocity Kp", kPDriveSpeed), 0);
  m_leftFront.Config_kI(
      0, frc::SmartDashboard::GetNumber("Velocity Ki", kIDriveSpeed), 0);
  m_leftFront.Config_kD(
      0, frc::SmartDashboard::GetNumber("Velocity Kd", kDDriveSpeed), 0);
  m_leftFront.Config_IntegralZone(
      0, frc::SmartDashboard::GetNumber("Velocity KIz", kIzDriveSpeed), 0);

  m_rightFront.Config_kF(
      0, frc::SmartDashboard::GetNumber("Velocity Kf", kFDriveSpeed), 0);
  m_rightFront.Config_kP(
      0, frc::SmartDashboard::GetNumber("Velocity Kp", kFDriveSpeed), 0);
  m_rightFront.Config_kI(
      0, frc::SmartDashboard::GetNumber("Velocity Ki", kFDriveSpeed), 0);
  m_rightFront.Config_kD(
      0, frc::SmartDashboard::GetNumber("Velocity Kd", kFDriveSpeed), 0);
  m_rightFront.Config_IntegralZone(
      0, frc::SmartDashboard::GetNumber("Velocity KIz", kIzDriveSpeed), 0);
}

void DalekDrive::SetWheelSpeeds(units::meters_per_second_t leftSpeed,
                                units::meters_per_second_t rightSpeed) {
  if (OperatorConstants::kTesting) {
    frc::SmartDashboard::PutNumber("Left speed setpoint",
                                   leftSpeed / kEncoderDistancePerPulse /
                                       (double)10 * 1_s);
    frc::SmartDashboard::PutNumber("Right speed setpoint",
                                   rightSpeed / kEncoderDistancePerPulse /
                                       (double)10 * 1_s);
  }
  double leftOutput = leftSpeed / kEncoderDistancePerPulse / (double)10 * 1_s;
  double rightOutput = rightSpeed / kEncoderDistancePerPulse / (double)10 * 1_s;
  m_leftFront.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                  (-50 < leftOutput && leftOutput < 50) ? 0 : leftOutput);
  m_rightFront.Set(ctre::phoenix::motorcontrol::TalonFXControlMode::Velocity,
                   (-50 < rightOutput && rightOutput < 50) ? 0 : rightOutput);
}

void DalekDrive::AddVisionPoseEstimate(frc::Pose2d pose,
                                       units::second_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(pose, timestamp);
}
