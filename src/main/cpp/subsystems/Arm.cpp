#include "subsystems/Arm.h"

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>

using namespace ArmConstants;

Arm::Arm()
    : m_solenoid{kPCMId, frc::PneumaticsModuleType::CTREPCM, kPistonChannel},
      m_motor{kMotorId}, m_neckFeedforward{kS, kG, kV, kA},
      m_neckController{kP, 0.0, 0.0, {kMaxTurnVelocity, kMaxTurnAcceleration}}
//, m_limitSwitch{kLimitSwitchChannel}
{
  m_motor.Config_kP(0, kP);
  m_motor.Config_kI(0, kI);
  m_motor.Config_kD(0, kD);
}

units::radian_t Arm::GetNeckAngle() {
  return units::radian_t{m_motor.GetSelectedSensorPosition() *
                         kEncoderRotationPerPulse};
}

void Arm::SetNeckAngle(units::degree_t target) {
  m_neckController.SetGoal(target);
  auto voltageOutput =
      kFeedForward.Calculate(target, 0_rad_per_s, 0_rad_per_s_sq);
  double PIDOutput = m_neckController.Calculate(GetNeckAngle(), target);
  m_motor.SetVoltage(voltageOutput + units::volt_t{PIDOutput});
}

void Arm::Log() { frc::SmartDashboard::PutBoolean("Legs Out", IsLegOut()); }

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