#include "subsystems/Drivetrain.h"

#include <frc/SPI.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/array.h>

#include "Constants.h"

using namespace DriveConstants;

Drivetrain::Drivetrain()
    : m_frontLeft{"FL",
                  kFrontLeftDriveMotorId,
                  kFrontLeftSteerMotorId,
                  kFrontLeftAbsoluteEncoderChannel,
                  kFrontLeftAbsoluteEncoderOffset,
                  kFrontLeftDriveMotorPIDCoefficients,
                  kFrontLeftSteerMotorPIDCoefficients},
      m_rearLeft{"RL",
                 kRearLeftDriveMotorId,
                 kRearLeftSteerMotorId,
                 kRearLeftAbsoluteEncoderChannel,
                 kRearLeftAbsoluteEncoderOffset,
                 kRearLeftDriveMotorPIDCoefficients,
                 kRearLeftSteerMotorPIDCoefficients},
      m_frontRight{"FR",
                   kFrontRightDriveMotorId,
                   kFrontRightSteerMotorId,
                   kFrontRightAbsoluteEncoderChannel,
                   kFrontRightAbsoluteEncoderOffset,
                   kFrontRightDriveMotorPIDCoefficients,
                   kFrontRightSteerMotorPIDCoefficients},
      m_rearRight{"RR",
                  kRearRightDriveMotorId,
                  kRearRightSteerMotorId,
                  kRearRightAbsoluteEncoderChannel,
                  kRearRightAbsoluteEncoderOffset,
                  kRearRightDriveMotorPIDCoefficients,
                  kRearRightSteerMotorPIDCoefficients},
      m_gyro{frc::SPI::Port::kMXP}, m_odometry{kDriveKinematics,
                                               GetHeading(),
                                               {m_frontLeft.GetPosition(),
                                                m_frontRight.GetPosition(),
                                                m_rearLeft.GetPosition(),
                                                m_rearRight.GetPosition()},
                                               frc::Pose2d()},
      m_poseEstimator{kDriveKinematics, m_gyro.GetRotation2d(),
                      wpi::array<frc::SwerveModulePosition, 4U>{
                          m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                          m_frontRight.GetPosition(),
                          m_rearRight.GetPosition()},
                      frc::Pose2d()} {}

void Drivetrain::Periodic() {
  // Update the odometry with the current gyro angle and module states.
  m_odometry.Update(GetHeading(),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
  m_field.SetRobotPose(m_odometry.GetPose());

  UpdateDashboard();
}

void Drivetrain::Drive(units::meters_per_second_t forwardSpeed,
                       units::meters_per_second_t strafeSpeed,
                       units::radians_per_second_t angularSpeed,
                       bool fieldRelative) {
  // Use the kinematics model to get from the set of commanded speeds to a set
  // of states that can be commanded to each module.
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                forwardSpeed, strafeSpeed, angularSpeed, GetHeading())
          : frc::ChassisSpeeds{forwardSpeed, strafeSpeed, angularSpeed});

  // Occasionally a drive motor is commanded to go faster than its maximum
  // output can sustain. Desaturation lowers the module speeds so that no motor
  // is driven above its maximum speed, while preserving the intended motion.
  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);

  // Finally each of the desired states can be sent as commands to the modules.
  auto [fl, fr, rl, rr] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(rl);
  m_rearRight.SetDesiredState(rr);
}

void Drivetrain::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void Drivetrain::ResetModules() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

frc::Rotation2d Drivetrain::GetHeading() { return m_gyro.GetRotation2d(); }

void Drivetrain::ZeroHeading() { m_gyro.Reset(); }

units::degrees_per_second_t Drivetrain::GetTurnRate() {
  return m_gyro.GetRate() * 1_deg_per_s;
}

frc::Pose2d Drivetrain::GetPose() { return m_odometry.GetPose(); }

void Drivetrain::ResetOdometry(const frc::Pose2d &pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

void Drivetrain::UpdateDashboard() {
  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutBoolean("Gyro calibrating?", m_gyro.IsCalibrating());
  frc::SmartDashboard::PutNumber("Robot heading",
                                 GetHeading().Degrees().value());
  double swerveStates[] = {m_frontLeft.GetState().angle.Radians().value(),
                           m_frontLeft.GetState().speed.value(),
                           m_rearLeft.GetState().angle.Radians().value(),
                           m_rearLeft.GetState().speed.value(),
                           m_frontRight.GetState().angle.Radians().value(),
                           m_frontRight.GetState().speed.value(),
                           m_rearRight.GetState().angle.Radians().value(),
                           m_rearRight.GetState().speed.value()};
  frc::SmartDashboard::PutNumberArray(
      "Swerve Module States",
      swerveStates); // Have to initialize array separately due as an error
                     // occurs when an array attempts to initialize as a
                     // parameter.
  m_frontLeft.UpdateDashboard();
  m_rearLeft.UpdateDashboard();
  m_frontRight.UpdateDashboard();
  m_rearRight.UpdateDashboard();

  frc::SmartDashboard::PutNumber("Gyro", m_gyro.GetAngle());

  frc::SmartDashboard::PutData("Field", &m_field);
}

frc2::CommandPtr Drivetrain::SwerveCommand(std::function<double()> forward,
                                           std::function<double()> strafe,
                                           std::function<double()> rot) {
  return this->Run([&] {
    Drive(kMaxTeleopSpeed * forward(), kMaxTeleopSpeed * strafe(),
          AutoConstants::kMaxAngularSpeed * rot(), false);
  });
}

frc2::CommandPtr
Drivetrain::SwerveCommandFieldRelative(std::function<double()> forward,
                                       std::function<double()> strafe,
                                       std::function<double()> rot) {
  return this->Run([&] {
    Drive(kMaxTeleopSpeed * forward(), kMaxTeleopSpeed * strafe(),
          AutoConstants::kMaxAngularSpeed * rot(), true);
  });
}

// // needs work
// frc2::CommandPtr Drivetrain::DriveToPoseCommand(frc::Pose2d targetPose) {
//   // TODO
//   frc::Pose2d startPose = m_odometry.GetPose();
//   const double kDistanceTolerance =
//       0.1; // Tolerance for position error in meters

//   units::meter_t hypotenuse =
//       units::meter_t{std::hypot((targetPose.X() - startPose.X()).value(),
//                            (targetPose.Y() - startPose.Y()).value())};

//   while (std::hypot((targetPose.X() - startPose.X()).value(),
//                     (targetPose.Y() - startPose.Y()).value()) >
//          kDistanceTolerance) {
//     // todo
//     break;
//   }

//   return {nullptr};
// }

// Check if the robot has reached the target pose
// need to fix
bool Drivetrain::IsFinished(frc::Pose2d targetPose) {

  frc::Pose2d startPose = m_odometry.GetPose();

  auto distanceError =
      targetPose.Translation().Distance(startPose.Translation());
  auto angleError =
      targetPose.Rotation().Radians() - startPose.Rotation().Radians();
  return distanceError < kDistanceTolerance &&
         std::fabs((double)angleError) < 0.05;
}

frc2::CommandPtr Drivetrain::ZeroHeadingCommand() {
  return this->RunOnce([&] { ZeroHeading(); });
}

frc2::CommandPtr Drivetrain::ResetModulesCommand() {
  return this->RunOnce([&] { ResetModules(); });
}

void Drivetrain::AddVisionPoseEstimate(frc::Pose2d pose,
                                       units::second_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(pose, timestamp);
}