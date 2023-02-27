#pragma once

#include <frc/DigitalInput.h>
#include <frc/PWM.h>
#include <frc/PneumaticsModuleType.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

class Claw : public frc2::SubsystemBase {
public:
  Claw();

  void Log();

  // void GetLimitSwtich();

  void SetPosition(bool position);

  void Toggle();

  void Periodic() override;

private:
  frc::Solenoid m_claw;
  // frc::DigitalInput m_limitSwitch;
};
