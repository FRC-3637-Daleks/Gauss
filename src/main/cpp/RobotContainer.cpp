#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/Trigger.h>
#include <units/voltage.h>

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutBoolean("Running SetNeckAngle", false);
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Drive forward, closed loop with squared inputs.
  m_drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_drivetrain.TankDrive(-m_leftJoystick.GetY(), -m_rightJoystick.GetY(),
                               true);
      },
      {&m_drivetrain}));

  // Move neck with xbox joystick.
  m_arm.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        m_arm.SetNeckVoltage(-2 * m_driverController.GetLeftY() * 1_V);
      },
      {&m_arm}));

  // Arcade controls.
  m_leftJoystick.Button(1).OnTrue(frc2::cmd::Run(
      [this] {
        m_drivetrain.ArcadeDrive(-m_leftJoystick.GetY(),
                                 -m_rightJoystick.GetX(), true);
      },
      {&m_drivetrain}));

  // Reset neck.
  m_leftJoystick.Button(2).OnTrue(
      frc2::cmd::RunOnce([this] { m_arm.ZeroNeck(); }, {&m_arm}));

  frc::SmartDashboard::PutNumber("Turn goal input", 0);

  // Set neck angle.
  m_leftJoystick.Button(3).WhileTrue(
      m_arm.SetNeckAngleCommand([this]() -> units::degree_t {
        return 1_deg * frc::SmartDashboard::GetNumber("Turn goal input", 40);
      }));

  // Brake.
  m_leftJoystick.Button(4).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(0, 0, true); }, {&m_drivetrain}));

  m_leftJoystick.Button(5).OnTrue(frc2::cmd::RunOnce(
      [this] { m_drivetrain.UpdatePIDValues(); }, {&m_drivetrain}));

  m_leftJoystick.Button(6).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(1_mps, 1_mps); }, {&m_drivetrain}));

  // Slow drive
  m_rightJoystick.Button(1).WhileTrue(frc2::cmd::Run(
      [this] {
        m_drivetrain.PreciseDrive(-m_leftJoystick.GetY(),
                                  -m_rightJoystick.GetY(), true);
      },
      {&m_drivetrain}));

  m_rightJoystick.Button(2).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(0.15, 0.15, false); }, {&m_drivetrain}));

  m_rightJoystick.Button(3).WhileTrue(
      m_drivetrain.DriveToDistanceCommand(3_ft));

  // When the left bumper is clicked, it will open all the pistons
  // toggle for intake
  m_driverController.A().ToggleOnTrue(frc2::cmd::StartEnd(
      [this] { m_intake.SetIntakeOn(true); },
      [this] { m_intake.SetIntakeOn(false); }, {&m_intake}));
  // toggle claw
  m_driverController.RightBumper().ToggleOnTrue(
      frc2::cmd::StartEnd([&] { m_claw.SetPosition(true); },
                          [&] { m_claw.SetPosition(false); }, {&m_claw}));
  // toggle arm piston
  m_driverController.LeftBumper().ToggleOnTrue(frc2::cmd::Either(
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(false); }, {&m_arm}),
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(true); }, {&m_arm}),
      [&]() -> bool { return m_arm.IsLegOut(); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Drive and balance on the charge station.
  // return std::move(m_chargeStationAuto);

  // No auton.
  return frc2::CommandPtr{nullptr};
}
