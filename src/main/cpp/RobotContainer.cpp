#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/voltage.h>

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutBoolean("Running SetNeckAngle", false);
  frc::SmartDashboard::PutData(&m_chooser);
  frc::SmartDashboard::PutBoolean("CONE HIGH LEFT", false);
  frc::SmartDashboard::PutBoolean("CONE HIGH RIGHT", false);
  frc::SmartDashboard::PutBoolean("CUBE HIGH", false);
  frc::SmartDashboard::PutBoolean("CUBE LOW", false);
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
        double output = m_driverController.GetLeftY();
        m_arm.SetNeckVoltage(-2.0 * std::copysign(output * output, output) *
                             1_V);
      },
      {&m_arm}));

  // Arcade controls.
  (m_rightJoystick.Button(1) && !m_leftJoystick.Button(1))
      .WhileTrue(frc2::cmd::Run(
          [this] {
            m_drivetrain.ArcadeDrive(-m_leftJoystick.GetY(),
                                     -m_rightJoystick.GetX(), true);
          },
          {&m_drivetrain}));

  // Precise arcade controls when both triggers held.
  (m_rightJoystick.Button(1) && m_leftJoystick.Button(1))
      .WhileTrue(frc2::cmd::Run(
          [this] {
            m_drivetrain.PreciseArcadeDrive(-m_leftJoystick.GetY(),
                                            -m_rightJoystick.GetX(), true);
          },
          {&m_drivetrain}));

  // Reset neck.
  //   m_leftJoystick.Button(2).OnTrue(
  //       frc2::cmd::RunOnce([this] { m_arm.ZeroNeck(); }, {&m_arm}));

  //   frc::SmartDashboard::PutNumber("Turn goal input", 0);

  // Set neck angle.
  //   m_leftJoystick.Button(3).WhileTrue(
  //       m_arm.SetNeckAngleCommand([this]() -> units::degree_t {
  //         return 1_deg * frc::SmartDashboard::GetNumber("Turn goal input",
  //         40);
  //       }));

  // Brake.
  m_leftJoystick.Button(4).WhileTrue(m_drivetrain.BrakeCommand());

  m_leftJoystick.Button(5).OnTrue(frc2::cmd::RunOnce(
      [this] { m_drivetrain.UpdatePIDValues(); }, {&m_drivetrain}));

  m_leftJoystick.Button(6).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(1_mps, 1_mps); }, {&m_drivetrain}));

  // Slow drive
  (m_leftJoystick.Button(1) && !m_rightJoystick.Button(1))
      .WhileTrue(frc2::cmd::Run(
          [this] {
            m_drivetrain.PreciseDrive(-m_leftJoystick.GetY(),
                                      -m_rightJoystick.GetY(), false);
          },
          {&m_drivetrain}));

  m_rightJoystick.Button(2).WhileTrue(frc2::cmd::Run(
      [this] { m_drivetrain.TankDrive(0.15, 0.15, false); }, {&m_drivetrain}));

  m_rightJoystick.Button(3).WhileTrue(
      m_drivetrain.DriveToDistanceCommand(3_ft));

  //   m_rightJoystick.Button(4).WhileTrue(
  //       m_drivetrain.TestTurnToAngleCommand(180_deg).WithTimeout(5_s));

  m_rightJoystick.Button(5).OnTrue(m_arm.ResetSwitchCommand());

  m_driverController.B().WhileTrue(m_arm.LowAngleCommand());
  m_driverController.Y().WhileTrue(m_arm.HighAngleCommand());
  // D-pad up
  frc2::Trigger{[&]() { return m_driverController.GetPOV() == 0; }}.WhileTrue(
      m_arm.SubstationCommand());
  // D-pad down
  frc2::Trigger{[&]() { return m_driverController.GetPOV() == 180; }}.WhileTrue(
      m_arm.IntakeCommand());
  // When the left bumper is clicked, it will open all the pistons
  // toggle for intake
  m_driverController.A().WhileTrue(
      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }, {&m_intake}));
  m_driverController.A().OnFalse(
      frc2::cmd::Run([this] { m_intake.StopIntakeMotors(); }, {&m_intake}));

  m_driverController.X().WhileTrue(
      frc2::cmd::Run([this] { m_intake.ReverseIntakeMotors(); }, {&m_intake}));
  m_driverController.X().OnFalse(
      frc2::cmd::Run([this] { m_intake.StopIntakeMotors(); }, {&m_intake}));
  // toggle claw
  m_driverController.RightBumper().ToggleOnTrue(
      frc2::cmd::StartEnd([&] { m_claw.SetPosition(true); },
                          [&] { m_claw.SetPosition(false); }, {&m_claw}));
  // toggle arm piston
  m_driverController.LeftBumper().ToggleOnTrue(frc2::cmd::Either(
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(false); }, {&m_arm}),
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(true); }, {&m_arm}),
      [&]() -> bool { return m_arm.IsLegOut(); }));

  // Reset the arm if the limit switch gets accidentally tripped. (or if Arm
  // angle returns less than Physical Lower Bound or greater than Physical Upper
  // Bound)
  //   m_armResetTrigger.Debounce(100_ms).WhileTrue(m_arm.ResetSwitchCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // XXX: Not tested!
  frc2::CommandPtr placeLowCubeCommand = frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      m_arm.LowAngleCommand().WithTimeout(2_s),
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
      // Get arm back into intake.
      m_arm.IntakeCommand().WithTimeout(1_s), frc2::cmd::Run([this] {
                                                m_arm.SetNeckVoltage(-0.2_V);
                                              }).WithTimeout(0.3_s),
      frc2::cmd::RunOnce([this] {
        m_intake.SetIntakeOn(true);
      }).WithTimeout(0.1_s),
      frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps); },
                     {&m_drivetrain})
          .WithTimeout(1.75_s));

  frc2::CommandPtr placeHighCubeCommand = frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      frc2::cmd::Race(
          m_arm.AlternateHighCubeAngleCommand(),
          frc2::cmd::Sequence(
              frc2::cmd::Run([this] {
                m_intake.ReverseIntakeMotors();
              }).WithTimeout(1_s),
              frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s),
              frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps); },
                             {&m_drivetrain})
                  .WithTimeout(0.8_s),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s))),
      frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
                              m_claw.SetPosition(true);
                            }).WithTimeout(0.1_s),
      frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
      frc2::cmd::Race(
          m_arm.IntakeCommand(),
          frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps); },
                         {&m_drivetrain})
              .WithTimeout(1.75_s)),
      m_drivetrain.TurnTo180CCWCommand().WithTimeout(2_s),
      frc2::cmd::Race(m_arm.LowAngleCommand(),
                      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }),
                      frc2::cmd::Run([this] {
                        m_drivetrain.TankDrive(3_fps, 3_fps);
                      }).WithTimeout(1_s)),
      m_arm.IntakeCommand().WithTimeout(1_s),
      frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      frc2::cmd::Race(m_drivetrain.TurnTo180CCWCommand().WithTimeout(2_s),
                      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); })),
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
      frc2::cmd::Race(m_arm.LowAngleCommand().WithTimeout(0.75_s),
                      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); })),
      frc2::cmd::Run([this] {
        m_intake.ReverseIntakeMotors();
      }).WithTimeout(0.5_s),
      frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }));

  frc2::CommandPtr placeHighConeCommandLeft = frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      frc2::cmd::Race(
          m_arm.AlternateHighAngleCommand(),
          frc2::cmd::Sequence(
              frc2::cmd::Run([this] {
                m_intake.ReverseIntakeMotors();
              }).WithTimeout(1_s),
              frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s),
              frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps); },
                             {&m_drivetrain})
                  .WithTimeout(0.8_s),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s))),
      frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
                              m_claw.SetPosition(true);
                            }).WithTimeout(0.1_s),
      frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
      frc2::cmd::Race(
          m_arm.IntakeCommand(),
          frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps); },
                         {&m_drivetrain})
              .WithTimeout(1.75_s)),
      m_drivetrain.TurnTo170CCWCommand().WithTimeout(2_s),
      frc2::cmd::Race(m_arm.LowAngleCommand(),
                      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }),
                      frc2::cmd::Run([this] {
                        m_drivetrain.TankDrive(3_fps, 3_fps);
                      }).WithTimeout(1.5_s)),
      m_arm.IntakeCommand().WithTimeout(1_s),
      //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      //   frc2::cmd::Race(m_drivetrain.TurnTo185Command().WithTimeout(2_s),
      //                   frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
      //                   })),
      m_drivetrain.TurnTo185CCWCommand().WithTimeout(2_s),
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
      frc2::cmd::Wait(0.5_s),
      frc2::cmd::Race(m_arm.LowAngleCommand().WithTimeout(0.75_s),
                      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); })),
      frc2::cmd::Run([this] {
        m_intake.ReverseIntakeMotors();
      }).WithTimeout(0.5_s),
      frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }));

  frc2::CommandPtr placeHighConeCommandRight = frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      frc2::cmd::Race(
          m_arm.AlternateHighAngleCommand(),
          frc2::cmd::Sequence(
              frc2::cmd::Run([this] {
                m_intake.ReverseIntakeMotors();
              }).WithTimeout(1_s),
              frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s),
              frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps); },
                             {&m_drivetrain})
                  .WithTimeout(0.8_s),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s))),
      frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
                              m_claw.SetPosition(true);
                            }).WithTimeout(0.1_s),
      frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
      frc2::cmd::Race(
          m_arm.IntakeCommand(),
          frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps); },
                         {&m_drivetrain})
              .WithTimeout(2_s))
      //   m_drivetrain.TurnTo175CWCommand().WithTimeout(2_s),
      //   frc2::cmd::Race(m_arm.LowAngleCommand(),
      //                   frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
      //                   }), frc2::cmd::Run([this] {
      //                     m_drivetrain.TankDrive(3_fps, 3_fps);
      //                   }).WithTimeout(1.5_s)),
      //   m_arm.IntakeCommand().WithTimeout(1_s),
      //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
      //   frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      //   frc2::cmd::Race(m_drivetrain.TurnTo185CWCommand().WithTimeout(2_s),
      //                   frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
      //                   })),
      //   frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
      //   frc2::cmd::Wait(0.5_s),
      //   frc2::cmd::Race(m_arm.LowAngleCommand().WithTimeout(0.75_s),
      //                   frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
      //                   })),
      //   frc2::cmd::Run([this] {
      //     m_intake.ReverseIntakeMotors();
      //   }).WithTimeout(0.5_s),
      //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); })
  );
  if (frc::SmartDashboard::GetBoolean("CONE HIGH LEFT", false)) {
    return placeHighConeCommandLeft;
  } else if (frc::SmartDashboard::GetBoolean("CONE HIGH RIGHT", false)) {
    return placeHighConeCommandRight;
  } else if (frc::SmartDashboard::GetBoolean("CUBE HIGH", false)) {
    return placeHighCubeCommand;
  } else if (frc::SmartDashboard::GetBoolean("CUBE LOW", false)) {
    return placeLowCubeCommand;
  }

  return frc2::CommandPtr{nullptr};
}
