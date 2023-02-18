#pragma once

#include <ctre/Phoenix.h>
#include <frc/AnalogInput.h>
#include <frc/AnalogPotentiometer.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/Solenoid.h>
#include <frc/controller/ArmFeedforward.h>
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
  void SetLegPosition(bool isOut);

  bool IsLegOut();

  void SwitchLegPosition();

  double GetPotentiometer();

  void SetNeckAngle(units::degree_t target);

  // Puts potentiometer output on the SmartDashboard
  void Log();

  void Periodic();

  void Reset();

  units::radian_t GetNeckAngle();

  void SetArmZero(bool limitswitch);

  void SetArmGoal(int goal);

private:
  frc::Solenoid m_solenoid;
  WPI_TalonFX m_motor;
  frc::AnalogInput m_analogPotentiometer;
  frc::ProfiledPIDController<units::radian> m_neckController;
  frc::ArmFeedforward m_feedforward;
  frc::DigitalInput m_limitSwitch;
};
