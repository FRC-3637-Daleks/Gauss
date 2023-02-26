#include "subsystems/Arm.h"

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>

using namespace ArmConstants;

Arm::Arm()
    : m_solenoid{kPCMId, frc::PneumaticsModuleType::CTREPCM, kPistonChannel},
      m_motor{kMotorId}, m_neckFeedforward{kS, kG, kV, kA},
      m_neckController{kP, 0.0, 0.0, {kMaxTurnVelocity, kMaxTurnAcceleration}},
      m_simpleNeckController{kP, 0, 0} {
  m_neckController.SetTolerance(3_deg, 3_deg_per_s);
  m_neckController.SetIntegratorRange(0, 1);
  m_motor.ConfigOpenloopRamp(0.5);
}

units::radian_t Arm::GetNeckAngle() {
  return units::radian_t{m_motor.GetSelectedSensorPosition() *
                         kEncoderRotationPerPulse};
}

void Arm::SetNeckVoltage(units::volt_t output) {
  m_motor.SetVoltage(output);
  frc::SmartDashboard::PutNumber("neck voltage", output.value());
}

frc2::CommandPtr Arm::SetNeckAngle(std::function<units::degree_t()> getTarget) {
  return frc2::Subsystem::RunOnce([this, &getTarget]() {
           fmt::print("Setting goal...");
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(20_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            fmt::print("Setting output...");
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            frc::SmartDashboard::PutNumber("PID output", output.value());
            fmt::print("{}", output.value());
            SetNeckVoltage(output);
          },
          // Set output to zero when done.
          [this]() {
            fmt::print("done!");
            SetNeckVoltage(0_V);
          }));
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
      units::degree_t{m_neckController.GetSetpoint().position}.value());

  frc::SmartDashboard::PutBoolean("Legs Out", IsLegOut());
  frc::SmartDashboard::PutNumber("Angle", GetNeckAngle().value());
  frc::SmartDashboard::PutBoolean("at goal?", m_neckController.AtGoal());
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