#include "subsystems/Intake.h"

Intake::Intake()
    : m_left{IntakeConstants::kPCMPort, frc::PneumaticsModuleType::CTREPCM,
             IntakeConstants::kLeftPistonPort},
      m_right{IntakeConstants::kPCMPort, frc::PneumaticsModuleType::CTREPCM,
              IntakeConstants::kRightPistonPort},
      m_middle{IntakeConstants::kPCMPort, frc::PneumaticsModuleType::CTREPCM,
               IntakeConstants::kMiddlePistonPort},
      m_rangefinder{IntakeConstants::kRangefinderPort} {
  m_left.Set(false);
  m_right.Set(false);
  m_middle.Set(false);
  this->Periodic();
}

void Intake::Log() {
  frc::SmartDashboard::PutBoolean("Intake Pistons",
                                  m_left.Get() && m_right.Get());
  frc::SmartDashboard::PutBoolean("Middle Piston", m_middle.Get());
  frc::SmartDashboard::PutNumber("RangeFinder", this->GetRangefidner());
  frc::SmartDashboard::PutBoolean("Pistion Disabled", m_left.IsDisabled());
}

void Intake::SetIntake(bool leftPiston, bool rightPiston, bool middlePiston) {
  m_left.Set(leftPiston);
  m_right.Set(rightPiston);
  m_middle.Set(middlePiston);
}

void Intake::SetIntake(bool leftPiston, bool rightPiston) {
  m_left.Set(leftPiston);
  m_right.Set(rightPiston);
}

bool Intake::ReadyToPickUp() {
  return true;
  // implications needed after testing
}
double Intake::GetRangefidner() { return m_rangefinder.GetVoltage(); }

void Intake::Periodic() { this->Log(); }
