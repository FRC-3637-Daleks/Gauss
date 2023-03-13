#include "subsystems/Intake.h"

Intake::Intake()
    : m_intakePiston{IntakeConstants::kPCMPort,
                     frc::PneumaticsModuleType::CTREPCM,
                     IntakeConstants::kPistonPort},
      m_rangefinder{IntakeConstants::kRangefinderPort},
      m_leftIntakeMotor{IntakeConstants::kLeftMotorPort},
      m_rightIntakeMotor{IntakeConstants::kRightMotorPort} {

  cs::UsbCamera intakeCamera = frc::CameraServer::StartAutomaticCapture();
  intakeCamera.SetResolution(240, 160);
  intakeCamera.SetFPS(10);
  // intakeCamera.SetCompression(50);
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

void Intake::SetIntakeMotors() {
  m_leftIntakeMotor.Set(IntakeConstants::kIntakeMotorSpeed);
  m_rightIntakeMotor.Set(IntakeConstants::kIntakeMotorSpeed);
}

bool Intake::ReadyToPickUp() {
  if ((this->GetRangefinder() <= IntakeConstants::pickUpRangeCone) ||
      (this->GetRangefinder() <= IntakeConstants::pickUpRangeCube)) {
    return true;
  }
  return false;
  // implications needed after testing
}

void Intake::DetectionIntake() {
  if(ReadyToPickUp()) {
    SetIntakeMotors();
  }
}

double Intake::GetRangefinder() { return m_rangefinder.GetVoltage(); }

void Intake::Periodic() { this->Log(); }