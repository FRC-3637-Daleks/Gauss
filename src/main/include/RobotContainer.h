#pragma once

#include <frc/XboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/Arm.h"

class RobotContainer {
public:
  RobotContainer();
  Arm m_arm;

  void ConfigureDashboard();

private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_XboxController{
      OperatorConstants::kXboxController};

  // The robot's subsystems are defined here...

  // The robot's commands are defined here...

  void ConfigureBindings();
};
