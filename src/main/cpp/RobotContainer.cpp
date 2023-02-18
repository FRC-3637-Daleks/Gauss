// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
