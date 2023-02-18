#include "subsystems/Arm.h"

#include <frc/AnalogInput.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/FunctionalCommand.h>

using namespace ArmConstants;

Arm::Arm()
    : m_solenoid{kPCM, frc::PneumaticsModuleType::CTREPCM, kPistonId},
      m_motor{kMotorId}, m_analogPotentiometer{kPotentiometerId},
      m_neckController{kP, 0.0, 0.0, {kMaxTurnVelocity, kMaxTurnAcceleration}},
      m_feedforward{kS, kG, kV, kA}, m_limitSwitch{kLimitSwitchId} {
  m_motor.Config_kP(0, kP);
  m_motor.Config_kI(0, kI);
  m_motor.Config_kD(0, kD);
}

void Arm::SetLegPosition(bool isOut) { m_solenoid.Set(isOut); }

bool Arm::IsLegOut() { return m_solenoid.Get(); }

void Arm::SwitchLegPosition() { m_solenoid.Set(IsLegOut()); }

double Arm::GetPotentiometer() { return m_analogPotentiometer.GetVoltage(); }

units::radian_t Arm::GetNeckAngle() {
  return units::radian_t{m_motor.GetSelectedSensorPosition() *
                         kArmRadiansPerEncoderTick};
}

void Arm::SetNeckAngle(units::degree_t target) {
  m_neckController.SetGoal(target);
  auto voltageOutput =
      m_feedforward.Calculate(target, 0_rad_per_s, 0_rad_per_s_sq);
  double PIDOutput = m_neckController.Calculate(GetNeckAngle(), target);
  m_motor.SetVoltage(voltageOutput + units::volt_t{PIDOutput});
}

void Arm::Log() {
  frc::SmartDashboard::PutNumber("Potentiometer Output", GetPotentiometer());
  frc::SmartDashboard::PutBoolean("Legs Out", IsLegOut());
}

void Arm::SetArmGoal(int goal) {
  if (goal == 1) {
    SetNeckAngle(20_deg);
  } else if (goal == 2) {
    SetNeckAngle(50_deg);
  } else if (goal == 3) {
    SetNeckAngle(90_deg);
  }
}

void Arm::SetArmZero(bool limitswitch) {
  if (limitswitch) {
    Reset();
  }
}

void Arm::Periodic() {
  Log();
  SetArmZero(m_limitSwitch.Get());
}

void Arm::Reset() { m_motor.SetSelectedSensorPosition(0); }