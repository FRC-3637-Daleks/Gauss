#include "RobotContainer.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>

RobotContainer::RobotContainer() { ConfigureBindings(); }

void RobotContainer::ConfigureBindings() {
  // Schedule `ExampleMethodCommand` when the Xbox controller's B button is
  // pressed, cancelling on release.
  m_leftJoystick.Button(1).OnTrue(
      frc2::InstantCommand([this] { m_drivetrain.Reset(); }, {&m_drivetrain})
          .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // No auton.
  return frc2::CommandPtr{nullptr};
}
