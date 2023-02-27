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
  m_drivetrain.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        // m_drivetrain.Drive(-m_leftJoystick.GetY(), -m_rightJoystick.GetY(),
        //                    true);
        m_drivetrain.TankDrive(-m_leftJoystick.GetY(), -m_rightJoystick.GetY(),
                               true);
      },
      {&m_drivetrain}));

  m_arm.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        double powerMultiplier = 0;
        if (m_arm.IsLegOut()) {
          powerMultiplier = 2;
        } else {
          powerMultiplier = 2;
        }
        m_arm.SetNeckVoltage(powerMultiplier * -m_driverController.GetLeftY() *
                             1_V);
      },
      {&m_arm}));

  m_leftJoystick.Button(1).OnTrue(
      frc2::cmd::RunOnce([this] { m_drivetrain.Reset(); }, {&m_drivetrain}));

  m_leftJoystick.Button(2).OnTrue(
      frc2::cmd::RunOnce([this] { m_arm.ZeroNeck(); }, {&m_arm}));

  frc::SmartDashboard::PutNumber("Turn goal input", 0);

  m_leftJoystick.Button(3).WhileTrue(
      m_arm.SetNeckAngleCommand([this]() -> units::degree_t {
        return 1_deg * frc::SmartDashboard::GetNumber("Turn goal input", 0);
      }));

  // m_leftJoystick.Button(4).WhileTrue(
  //     frc2::cmd::RunOnce([this] { m_arm.SetLegOut(true); }, {&m_arm}));

  // m_leftJoystick.Button(5).WhileTrue(
  //     frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }, {&m_arm}));

  m_driverController.X().ToggleOnTrue(
      frc2::cmd::StartEnd([&] { m_claw.SetPosition(true); },
                          [&] { m_claw.SetPosition(false); }, {&m_claw}));
  m_driverController.B().ToggleOnTrue(frc2::cmd::Either(
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(false); }, {&m_arm}),
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(true); }, {&m_arm}),
      [&]() -> bool { return m_arm.IsLegOut(); }));

  // Brake.
  m_leftJoystick.Button(4).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(0, 0, true); }, {&m_drivetrain}));

  m_leftJoystick.Button(5).OnTrue(frc2::cmd::RunOnce(
      [this] { m_drivetrain.UpdatePIDValues(); }, {&m_drivetrain}));

  // Slow drive
  m_rightJoystick.Button(1).WhileTrue(frc2::cmd::Run(
      [this] {
        m_drivetrain.TankDrive(
            m_leftRateLimiter.Calculate(-m_leftJoystick.GetY()),
            m_rightRateLimiter.Calculate(-m_rightJoystick.GetY()), true);
      },
      {&m_drivetrain}));

  m_rightJoystick.Button(3).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(0.15, 0.15, false); }, {&m_drivetrain}));
  // When the left bumper is clicked, it will open all the pistons
  // toggle for intake
  m_driverController.RightBumper().ToggleOnTrue(frc2::cmd::StartEnd(
      [this] { m_intake.SetIntakeOn(true); },
      [this] { m_intake.SetIntakeOn(false); }, {&m_intake}));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // Drive and balance on the charge station.
  return std::move(m_chargeStationAuto);

  // No auton.
  // return frc2::CommandPtr{nullptr};
}
