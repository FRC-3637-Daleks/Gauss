#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() {
  m_drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_drivetrain.Drive(-m_leftJoystick.GetY(), -m_rightJoystick.GetY(),
                           true);
      },
      {&m_drivetrain}));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  m_leftJoystick.Button(1).OnTrue(
      frc2::cmd::RunOnce([this] { m_drivetrain.Reset(); }, {&m_drivetrain}));
  // m_driverController.A().OnTrue(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }, {&m_claw}));
  // m_driverController.B().OnTrue(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }, {&m_claw}));

  m_driverController.B().ToggleOnTrue(
      frc2::cmd::StartEnd([&] { m_claw.SetPosition(true); },
                          [&] { m_claw.SetPosition(false); }, {&m_claw}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // No auton.
  return frc2::CommandPtr{nullptr};
}
