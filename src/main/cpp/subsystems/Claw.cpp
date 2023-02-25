#include "subsystems/Claw.h"

Claw::Claw()
    : m_claw{ClawConstants::kPCMPort, frc::PneumaticsModuleType::CTREPCM,
             ClawConstants::kPistonPort},
      m_limitSwitch{ClawConstants::kLimitSwitchPort} {
  m_claw.Set(true);
  this->Periodic();
}

void Claw::SetPosition(bool position) { m_claw.Set(position); }

void Claw::GetLimitSwtich() { m_limitSwitch.Get(); }

void Claw::Log() {
  frc::SmartDashboard::PutBoolean("Piston", m_claw.Get());
  frc::SmartDashboard::PutNumber("PCM Channel", m_claw.GetChannel());
  frc::SmartDashboard::PutBoolean("Pistion Disabled", m_claw.IsDisabled());
}

void Claw::Periodic() { Log(); }
