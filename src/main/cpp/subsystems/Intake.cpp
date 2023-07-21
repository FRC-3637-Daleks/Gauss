#include "subsystems/Intake.h"

Intake::Intake()
    : m_leftIntakeMotor{IntakeConstants::kLeftMotorPort},
      m_rightIntakeMotor{IntakeConstants::kRightMotorPort} {

  // cs::UsbCamera intakeCamera = frc::CameraServer::StartAutomaticCapture();
  // intakeCamera.SetResolution(240, 160);
  // intakeCamera.SetFPS(10);
  // intakeCamera.SetCompression(50);
  //  m_intakePiston.Set(false);
  this->Periodic();
  // m_intakePiston{IntakeConstants::kPCMPort,
  //                    frc::PneumaticsModuleType::CTREPCM,
  //                    IntakeConstants::kPistonPort}
}

void Intake::Log() {
  // frc::SmartDashboard::PutBoolean("Intake Pistons", m_intakePiston.Get());
  // frc::SmartDashboard::PutBoolean("Piston Disabled",
  //                                 m_intakePiston.IsDisabled());
}

// void Intake::SetIntakeOn(bool SetPiston) { m_intakePiston.Set(SetPiston); }

void Intake::SetIntakeMotors() {
  m_leftIntakeMotor.Set(IntakeConstants::kIntakeMotorSpeedReversed);
  m_rightIntakeMotor.Set(IntakeConstants::kIntakeMotorSpeed);
}

void Intake::ReverseIntakeMotors() {
  m_leftIntakeMotor.Set(-IntakeConstants::kIntakeMotorSpeedReversed);
  m_rightIntakeMotor.Set(-IntakeConstants::kIntakeMotorSpeed);
}

void Intake::StopIntakeMotors() {
  m_leftIntakeMotor.Set(0);
  m_rightIntakeMotor.Set(0);
}

// bool Intake::ReadyToPickUp() {
//   if ((this->GetRangefinder() <= IntakeConstants::pickUpRangeCone) ||
//       (this->GetRangefinder() <= IntakeConstants::pickUpRangeCube)) {
//     return true;
//   }
//   return false;
//   // implications needed after testing
// }

// double Intake::GetRangefinder() { return m_rangefinder.GetVoltage(); }

void Intake::Periodic() { this->Log(); }