#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/Trigger.h>
#include <iostream>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings and dashboard
  frc2::Trigger aButton = m_XboxController.A();
  ConfigureBindings();
  ConfigureDashboard();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here
}

void RobotContainer::ConfigureDashboard() {
  frc::SmartDashboard::PutBoolean("A Button", m_XboxController.GetAButton());
}
