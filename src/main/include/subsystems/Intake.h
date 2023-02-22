#pragma once

#include <frc/AnalogInput.h>
#include <frc/PWM.h>
#include <frc/PneumaticsControlModule.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Intake : public frc2::SubsystemBase {
public:
  Intake();

  void Log();

  void SetIntake(bool leftPiston, bool rightPiston, bool middlePiston);

  void SetIntake(bool leftPiston, bool rightPiston);

  bool ReadyToPickUp();

  double GetRangefidner();

  void Periodic() override;

private:
  frc::Solenoid m_left;
  frc::Solenoid m_right;
  frc::Solenoid m_middle;
  frc::AnalogInput m_rangefinder;
};
