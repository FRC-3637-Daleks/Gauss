#include "subsystems/Arm.h"

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>

using namespace ArmConstants;

Arm::Arm()
    : m_solenoid{kPCMId, frc::PneumaticsModuleType::CTREPCM, kPistonChannel},
      m_motor{kMotorId},
      m_neckController{kP, 0.0, 0.0, {kMaxTurnVelocity, kMaxTurnAcceleration}},
      m_limitSwitch{kLimitSwitchChannel} {
  m_neckController.SetTolerance(1_deg, 3_deg_per_s);
  m_neckController.SetIntegratorRange(0, 1);
  m_motor.ConfigOpenloopRamp(0.5);
  // m_compressor.EnableDigital();
}

units::radian_t Arm::GetNeckAngle() {
  auto offset = kOffset;
  if (IsLegOut()) {
    offset += kLegOffset;
  }
  return (kEncoderReversed ? -1.0 : 1.0) *
             units::radian_t{m_motor.GetSelectedSensorPosition() *
                             kEncoderRotationPerPulse} +
         offset;
}

void Arm::SetNeckVoltage(units::volt_t output) {
  if (GetNeckAngle() < 5_deg) {
    output = std::clamp(output, 0_V, 12_V);
  }
  m_motor.SetVoltage(-output);
  frc::SmartDashboard::PutNumber("neck voltage", output.value());
}

frc2::CommandPtr Arm::ResetSwitchCommand() {
  return frc2::Subsystem::Run([this] { SetNeckVoltage(-1_V); })
      .Until([this]() -> bool { return m_limitSwitch.Get(); })
      .WithTimeout(1_s)
      .AndThen([this] { ZeroNeck(); });
}

frc2::CommandPtr Arm::LowAngleCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(50_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID output", output.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID setpoint",
                m_neckController.GetSetpoint().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID goal", m_neckController.GetGoal().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID error", m_neckController.GetPositionError().value());
          },
          // Set output to zero when done.
          [this]() { SetNeckVoltage(0_V); }));
}

frc2::CommandPtr Arm::MidAngleCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(101_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID output", output.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID setpoint",
                m_neckController.GetSetpoint().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID goal", m_neckController.GetGoal().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID error", m_neckController.GetPositionError().value());
          },
          // Set output to zero when done.
          [this]() { SetNeckVoltage(0_V); }));
}

frc2::CommandPtr Arm::HighAngleCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(95_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID output", output.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID setpoint",
                m_neckController.GetSetpoint().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID goal", m_neckController.GetGoal().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID error", m_neckController.GetPositionError().value());
          },
          // Set output to zero when done.
          [this]() { SetNeckVoltage(0_V); }));
}

frc2::CommandPtr Arm::IntakeCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(10_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID output", output.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID setpoint",
                m_neckController.GetSetpoint().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID goal", m_neckController.GetGoal().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID error", m_neckController.GetPositionError().value());
          },
          // Set output to zero when done.
          [this]() { SetNeckVoltage(0_V); }));
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
  frc::SmartDashboard::PutBoolean("arm safe?", GetNeckAngle() > 5_deg);
  frc::SmartDashboard::PutNumber(
      "PID goal", units::degree_t{m_neckController.GetGoal().position}.value());
  frc::SmartDashboard::PutNumber(
      "PID error",
      units::degree_t{m_neckController.GetPositionError()}.value());
  frc::SmartDashboard::PutNumber(
      "PID setpoint",
      units::degree_t{m_neckController.GetSetpoint().position}.value());

  frc::SmartDashboard::PutBoolean("Switch tripped?", m_limitSwitch.Get());
  frc::SmartDashboard::PutBoolean("Legs Out?", IsLegOut());
  frc::SmartDashboard::PutNumber("Angle",
                                 units::degree_t{GetNeckAngle()}.value());
  frc::SmartDashboard::PutBoolean("at goal?", m_neckController.AtGoal());
}

void Arm::ZeroNeck() { m_motor.SetSelectedSensorPosition(0); }

void Arm::Periodic() {
  Log();

  if (!m_stopped && m_limitSwitch.Get()) {
    ZeroNeck();
    m_stopped = true;
  } else if (!m_limitSwitch.Get()) {
    // m_stopped = false;
  }
}

void Arm::Reset() { m_motor.SetSelectedSensorPosition(0); }