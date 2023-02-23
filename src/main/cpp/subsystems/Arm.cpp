#include "subsystems/Arm.h"

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>

using namespace ArmConstants;

Arm::Arm()
    : m_solenoid{kPCMId, frc::PneumaticsModuleType::CTREPCM, kPistonChannel},
      m_motor{kMotorId}, m_neckFeedforward{kS, kG, kV, kA},
      m_neckController{kP, 0.0, 0.0, {kMaxTurnVelocity, kMaxTurnAcceleration}}
{
  m_motor.Config_kP(0, kP);
  m_motor.Config_kI(0, kI);
  m_motor.Config_kD(0, kD);

  m_neckController.SetTolerance(5_deg, 5_deg_per_s);
}

units::radian_t Arm::GetNeckAngle() {
  return units::radian_t{m_motor.GetSelectedSensorPosition() *
                         kEncoderRotationPerPulse};
}

frc2::CommandPtr Arm::SetNeckAngle(units::degree_t target) {
  fmt::print("Calleď SetNeckAngle Command");
  frc::SmartDashboard::PutBoolean("Running SetNeckAngle", true);
  return frc2::FunctionalCommand(
             [this, &target]() { m_neckController.SetGoal(target); },
             [this, &target]() {
               auto output =
                   units::volt_t{m_neckController.Calculate(GetNeckAngle())};
               m_motor.SetVoltage(output);
             },
             [this](bool) -> void {
               frc::SmartDashboard::PutBoolean("Running SetNeckAngle", false);
               m_motor.SetVoltage(0_V);
             },
             [this]() -> bool { return m_neckController.AtGoal(); })
      .ToPtr();
}

// void Arm::SetNeckAngle(units::degree_t target) {
//   m_neckController.SetGoal(target);
//   // auto voltageOutput =
//   //     kFeedForward.Calculate(target, 0_rad_per_s, 0_rad_per_s_sq);
//   double PIDOutput = m_neckController.Calculate(GetNeckAngle(), target);
//   frc::SmartDashboard::PutNumber("PID output", PIDOutput);
//   // m_motor.SetVoltage(voltageOutput + units::volt_t{PIDOutput});
//   if (m_neckController.AtGoal()) {
//     m_motor.SetVoltage(units::volt_t{PIDOutput});
//   } else {
//     m_motor.SetVoltage(0_V);
//   }
// }

void Arm::Log() {
  frc::SmartDashboard::PutNumber(
      "PID goal", units::degree_t{m_neckController.GetGoal().position}.value());
  frc::SmartDashboard::PutNumber(
      "PID error",
      units::degree_t{m_neckController.GetPositionError()}.value());
  frc::SmartDashboard::PutNumber(
      "PID setpoint",
      units::degree_t{m_neckController.GetSetpoint().position()}.value());

  frc::SmartDashboard::PutBoolean("Legs Out", IsLegOut());
  frc::SmartDashboard::PutNumber("Angle",
                                 units::degree_t{GetNeckAngle()}.value());
}

void Arm::SetArmZero(bool limitswitch) {
  if (limitswitch) {
    Reset();
  }
}

void Arm::Periodic() {
  Log();
  // SetArmZero(m_limitSwitch.Get());
}

void Arm::Reset() { m_motor.SetSelectedSensorPosition(0); }