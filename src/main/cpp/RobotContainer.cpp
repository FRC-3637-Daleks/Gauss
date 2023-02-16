#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings() {
  m_drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_drivetrain.Drive(-m_leftJoystick.GetY(), -m_rightJoystick.GetY(),
                           true);
      },
      {&m_drivetrain}));

  m_leftJoystick.Button(1).OnTrue(
      frc2::cmd::RunOnce([this] { m_drivetrain.Reset(); }, {&m_drivetrain}));

  if (OperatorConstants::kTesting) {
    m_leftJoystick.Button(2).OnTrue(frc2::cmd::RunOnce(
        [this] { m_drivetrain.UpdatePIDValues(); }, {&m_drivetrain}));
  }

  m_rightJoystick.Button(1).WhileTrue(frc2::cmd::Run(
      [this] {
        m_drivetrain.TankDrive(
            m_leftRateLimiter.Calculate(-m_leftJoystick.GetY()),
            m_rightRateLimiter.Calculate(-m_rightJoystick.GetY()), true);
      },
      {&m_drivetrain}));

  m_rightJoystick.Button(2).WhileTrue(frc2::cmd::Run(
      [this] {
        m_drivetrain.TankDrive(
            m_leftRateLimiter.Calculate(-m_leftJoystick.GetY()),
            m_rightRateLimiter.Calculate(-m_rightJoystick.GetY()), true);
      },
      {&m_drivetrain}));

  m_rightJoystick.Button(3).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(0.15, 0.15, false); }, {&m_drivetrain}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // No auton.
  return frc2::CommandPtr{nullptr};
}
