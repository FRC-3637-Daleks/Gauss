#pragma once

#include <ctre/Phoenix.h>
#include <fmt/ostream.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANAnalog.h>
#include <units/angle.h>
#include <units/voltage.h>

#include "Constants.h"

class Arm : public frc2::SubsystemBase {

public:
  Arm();

  void SetLegOut(bool isOut) { m_solenoid.Set(isOut); }

  bool IsLegOut() { return m_solenoid.Get(); }

  void SwitchLegPosition() { m_solenoid.Set(IsLegOut()); }

  frc2::CommandPtr SetNeckAngle(std::function<units::degree_t()> getTarget);

  void Log();

  void Periodic();

  void Reset();

  units::radian_t GetNeckAngle();

  void SetNeckVoltage(units::volt_t output);

  void SetArmZero(bool limitswitch);

  void SetArmGoal(units::degree_t output);

private:
  frc::Solenoid m_solenoid;
  WPI_TalonFX m_motor;

  frc::ArmFeedforward m_neckFeedforward;
  frc::ProfiledPIDController<units::radian> m_neckController;
  frc::PIDController m_simpleNeckController;

  // frc::DigitalInput m_limitSwitch;
};