#include "subsystems/Arm.h"

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>

using namespace ArmConstants;

Arm::Arm()
    : m_solenoid{kPCMId, frc::PneumaticsModuleType::CTREPCM, kPistonChannel},
      m_motor{kMotorId},
      m_neckController{kP, kI, kD, {kMaxTurnVelocity, kMaxTurnAcceleration}},
      m_limitSwitch{kLimitSwitchChannel} {
  m_neckController.SetTolerance(1_deg, 10_deg_per_s);
  m_neckController.SetIntegratorRange(0, 3);
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
  return frc2::cmd::Sequence(
      frc2::cmd::Run(
          [this] {
            m_motor.SetVoltage(1.5_V);
            frc::SmartDashboard::PutNumber("neck voltage", -69);
          },
          {this})
          .Until([this]() -> bool { return m_limitSwitch.Get(); })
          .WithTimeout(3_s),
      frc2::cmd::RunOnce([this] { SetNeckVoltage(0_V); }, {this}),
      frc2::cmd::RunOnce([this] { ZeroNeck(); }, {this}));
}

frc2::CommandPtr Arm::LowAngleCommand() {
  return this
      ->RunOnce([this]() {
        m_neckController.Reset(GetNeckAngle());
        m_neckController.SetGoal(50_deg);
      })
      .AndThen(this->RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID output", output.value());
            frc::SmartDashboard::PutNumber("Arm PID measurement",
                                           GetNeckAngle().value());
            frc::SmartDashboard::PutNumber(
                "Arm PID setpoint",
                m_neckController.GetSetpoint().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID goal", m_neckController.GetGoal().position.value());
            frc::SmartDashboard::PutNumber(
                "Arm PID error", m_neckController.GetPositionError().value());
          },
          // Set output to zero when done.
          [this]() {
            fmt::print("hii");
            SetNeckVoltage(0_V);
          }));
}

frc2::CommandPtr Arm::MidAngleCommand() {
  return this
      ->RunOnce([this]() {
        m_neckController.Reset(GetNeckAngle());
        m_neckController.SetGoal(65_deg);
      })
      .AndThen(this->RunEnd(
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
  return this
      ->RunOnce([this]() {
        m_neckController.Reset(GetNeckAngle());
        m_neckController.SetGoal(120_deg);
      })
      .AndThen(this->RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID measurement",
                                           GetNeckAngle().value());
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

frc2::CommandPtr Arm::AlternateHighAngleCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(120_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            auto rawAngle =
                (kEncoderReversed ? -1.0 : 1.0) *
                    units::radian_t{m_motor.GetSelectedSensorPosition() *
                                    kEncoderRotationPerPulse} +
                kOffset + kLegOffset;
            auto output = 1_V * m_neckController.Calculate(rawAngle);
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID measurement",
                                           GetNeckAngle().value());
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

frc2::CommandPtr Arm::AlternateHighCubeAngleCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(110_deg);
         })
      .AndThen(frc2::Subsystem::RunEnd(
          // Sets motor output.
          [this]() {
            auto rawAngle =
                (kEncoderReversed ? -1.0 : 1.0) *
                    units::radian_t{m_motor.GetSelectedSensorPosition() *
                                    kEncoderRotationPerPulse} +
                kOffset + kLegOffset;
            auto output = 1_V * m_neckController.Calculate(rawAngle);
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID measurement",
                                           GetNeckAngle().value());
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

frc2::CommandPtr Arm::HigherAngleCommand() {
  return frc2::Subsystem::RunOnce([this]() {
           m_neckController.Reset(GetNeckAngle());
           m_neckController.SetGoal(65_deg);
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
  return this
      ->RunOnce([this]() {
        m_neckController.Reset(GetNeckAngle());
        m_neckController.SetGoal(10_deg);
      })
      .AndThen(this->RunEnd(
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

frc2::CommandPtr Arm::SubstationCommand() {
  return this
      ->RunOnce([this]() {
        m_neckController.Reset(GetNeckAngle());
        m_neckController.SetGoal(90_deg);
      })
      .AndThen(this->RunEnd(
          // Sets motor output.
          [this]() {
            auto output = 1_V * m_neckController.Calculate(GetNeckAngle());
            SetNeckVoltage(output);
            frc::SmartDashboard::PutNumber("Arm PID measurement",
                                           GetNeckAngle().value());
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
  frc::SmartDashboard::PutBoolean("ARM ZEROED?!", m_stopped);
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