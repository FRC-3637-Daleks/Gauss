#include "subsystems/Intake.h"

Intake::Intake()
    : m_intakePiston{IntakeConstants::kPCMPort,
                     frc::PneumaticsModuleType::CTREPCM,
                     IntakeConstants::kPistonPort},
      m_rangefinder{IntakeConstants::kRangefinderPort} {

  cs::UsbCamera intakeCamera = frc::CameraServer::StartAutomaticCapture();
  intakeCamera.SetResolution(315, 240);
  intakeCamera.SetFPS(20);

  m_intakePiston.Set(false);
  this->Periodic();
}

void Intake::Log() {
  frc::SmartDashboard::PutBoolean("Intake Pistons", m_intakePiston.Get());
  frc::SmartDashboard::PutNumber("RangeFinder", this->GetRangefinder());
  frc::SmartDashboard::PutBoolean("Pistion Disabled",
                                  m_intakePiston.IsDisabled());
}

void Intake::SetIntakeOn(bool SetPiston) { m_intakePiston.Set(SetPiston); }

bool Intake::ReadyToPickUp() {
  if ((this->GetRangefinder() <= IntakeConstants::pickUpRangeCone) ||
      (this->GetRangefinder() <= IntakeConstants::pickUpRangeCube)) {
    return true;
  }
  return false;
  // implications needed after testing
}
double Intake::GetRangefinder() { return m_rangefinder.GetVoltage(); }

void Intake::Periodic() { this->Log(); }