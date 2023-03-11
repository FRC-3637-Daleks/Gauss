#pragma once

#include <ctre/Phoenix.h>
#include <fmt/ostream.h>
#include <frc/Compressor.h>
#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
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

  frc2::CommandPtr ResetSwitchCommand();

  frc2::CommandPtr LowAngleCommand(frc::Rotation2d target);

  frc2::CommandPtr HighAngleCommand(frc::Rotation2d target);

  frc2::CommandPtr IntakeCommand(frc::Rotation2d target);

  void Log();

  void Periodic();

  void Reset();

  units::radian_t GetNeckAngle();

  void SetNeckVoltage(units::volt_t output);

  void ZeroNeck();

  void SetArmGoal(units::degree_t output);

private:
  frc::Solenoid m_solenoid;
  WPI_TalonFX m_motor;

  frc::ProfiledPIDController<units::radian> m_neckController;

  frc::DigitalInput m_limitSwitch;
  bool m_stopped{false};

  // frc::Compressor m_compressor{ArmConstants::kPCMId,
  //                              frc::PneumaticsModuleType::CTREPCM};
};
