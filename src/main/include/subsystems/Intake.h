#pragma once

#include "cameraserver/CameraServer.h"
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

  void SetIntakeOn(bool SetPiston);

  bool ReadyToPickUp();

  double GetRangefinder();

  void Periodic() override;

private:
  frc::Solenoid m_intakePiston;
  frc::AnalogInput m_rangefinder;
};
