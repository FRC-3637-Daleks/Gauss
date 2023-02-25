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
  // When the left bumper is clicked, it will open all the pistons
  m_driverController.LeftBumper().ToggleOnTrue(frc2::cmd::RunOnce(
      [this] { m_intake.SetIntake(true, true, true); }, {&m_intake}));
  // When the right bumper is clicked, it will open the two intakes and keep the
  // middle piston closed
  m_driverController.RightBumper().ToggleOnTrue(frc2::cmd::RunOnce(
      [this] { m_intake.SetIntake(true, true, false); }, {&m_intake}));
  // toggle for intake
  m_driverController.RightBumper().ToggleOnTrue(frc2::cmd::StartEnd(
      [&] { m_intake.SetIntake(true, true); },
      [&] { m_intake.SetIntake(false, false); }, {&m_intake}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // No auton.
  return frc2::CommandPtr{nullptr};
}
