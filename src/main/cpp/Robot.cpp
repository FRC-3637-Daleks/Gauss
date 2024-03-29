// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {
  // Start recording to data log
  // frc::DataLogManager::Start();

  // Record DS control and joystick data.
  // Change to `false` to not record joystick data.
  // frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog(), true);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() { frc2::CommandScheduler::GetInstance().Run(); }

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  m_autonomousCommand = m_container.GetAutonomousCommand();
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  // m_container.ConfigureDashboard();
  // rev::CANSparkMax drivemotor{1,
  //                             rev::CANSparkMaxLowLevel::MotorType::kBrushless};
  // ctre::phoenix::motorcontrol::can::WPI_TalonSRX m_steerMotor{0};
  // drivemotor.Set(1.0);
  // // m_steerMotor.Set(1.0);
  // frc::SmartDashboard::PutNumber(
  //     "spped",
  //     drivemotor.Get() * DriveConstants::kEncoderDistancePerPulse.value());

  frc::SmartDashboard::PutNumber(
      "Forward Controller Output",
      m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis));
  frc::SmartDashboard::PutNumber(
      "Strafe Controller Output",
      m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis));
  frc::SmartDashboard::PutNumber(
      "Rotation Controller Output",
      m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis));
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
