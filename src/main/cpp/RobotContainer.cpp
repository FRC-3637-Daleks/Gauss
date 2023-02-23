#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutBoolean("Running SetNeckAngle", false);
  m_drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        // m_drivetrain.Drive(-m_leftJoystick.GetY(), -m_rightJoystick.GetY(),
        //                    true);
        m_drivetrain.Drive(0, 0, true);
      },
      {&m_drivetrain}));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_leftJoystick.Button(1).OnTrue(
      frc2::cmd::RunOnce([this] { m_drivetrain.Reset(); }, {&m_drivetrain}));

  m_leftJoystick.Button(2).OnTrue(
      frc2::cmd::RunOnce([this] { m_arm.SetArmZero(true); }, {&m_arm}));

  m_leftJoystick.Button(3).WhileTrue(m_arm.SetNeckAngle(-20_deg));

  m_leftJoystick.Button(4).WhileTrue(
      frc2::cmd::RunOnce([this] { m_arm.SetLegOut(true); }, {&m_arm}));

  m_leftJoystick.Button(5).WhileTrue(
      frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }, {&m_arm}));

  m_rightJoystick.Button(2).OnTrue(
      frc2::cmd::RunOnce([this] { m_claw.Toggle(); }, {&m_claw}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // No auton.
  return frc2::CommandPtr{nullptr};
}
