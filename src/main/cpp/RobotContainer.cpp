#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/ConditionalCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/voltage.h>

#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/PIDConstants.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

RobotContainer::RobotContainer()
    : m_pathPlannerTest{
          Autos::PathPlannerAuto(&m_swerve, &m_arm, &m_claw, &m_intake)} {
  frc::SmartDashboard::PutBoolean("Running SetNeckAngle", false);
  // frc::SmartDashboard::PutData(&m_chooser);
  frc::SmartDashboard::PutBoolean("CONE HIGH LEFT", false);
  frc::SmartDashboard::PutBoolean("CONE HIGH RIGHT", false);
  frc::SmartDashboard::PutBoolean("CUBE HIGH", false);
  frc::SmartDashboard::PutBoolean("CUBE LOW", false);

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  // Log Match Info
  std::string matchType =
      frc::DriverStation::GetMatchType() == frc::DriverStation::MatchType::kNone
          ? ""
          : (frc::DriverStation::GetMatchType() ==
                     frc::DriverStation::MatchType::kElimination
                 ? "Elimination"
                 : (frc::DriverStation::GetMatchType() ==
                            frc::DriverStation::MatchType::kQualification
                        ? "Qualification"
                        : "Practice"));

  std::string alliance = frc::DriverStation::GetAlliance() ==
                                 frc::DriverStation::Alliance::kInvalid
                             ? "No"
                             : (frc::DriverStation::GetAlliance() ==
                                        frc::DriverStation::Alliance::kRed
                                    ? "Red"
                                    : "Blue");

  frc::DataLogManager::Log(
      fmt::format("Playing {} Match {} at {} as {} alliance\n", matchType,
                  frc::DriverStation::GetMatchNumber(),
                  frc::DriverStation::GetEventName(), alliance));

  // Print out Git Information
  std::getline(infoFile, gitInfo);
  frc::DataLogManager::Log(fmt::format("Git Branch: {}\n", gitInfo));

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  // Move neck with xbox joystick.
  m_arm.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        double output = frc::ApplyDeadband(m_driverController.GetLeftY(),
                                           OperatorConstants::kDeadband);
        m_arm.SetNeckVoltage(-2.0 * std::copysign(output * output, output) *
                             1_V);
      },
      {&m_arm}));

  // Set up the default drive command.

  auto fwd = [this]() -> units::meters_per_second_t {
    return (DriveConstants::kMaxTeleopSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis),
                OperatorConstants::kDeadband));
  };
  auto strafe = [this]() -> units::meters_per_second_t {
    return (DriveConstants::kMaxTeleopSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis),
                OperatorConstants::kDeadband));
  };

  auto rot = [this]() -> units::revolutions_per_minute_t {
    return (AutoConstants::kMaxAngularSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kRotationAxis),
                OperatorConstants::kDeadband));
  };

  // m_swerve.SetDefaultCommand(frc2::ConditionalCommand(
  //     m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot).Unwrap(),
  //     m_swerve.SwerveCommand(fwd, strafe, rot).Unwrap(), [this]() -> bool {
  //       return m_swerveController.GetRawButton(
  //           OperatorConstants::kFieldRelativeButton);
  //     }));
  m_swerve.SetDefaultCommand(m_swerve.SwerveCommand(fwd, strafe, rot));
  m_swerveController.Button(OperatorConstants::kFieldRelativeButton)
      .WhileTrue(m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot));

  // Swerve Button Configs
  // Configure button bindings.
  m_swerveController.Button(OperatorConstants::kZeroHeadingButton)
      .OnTrue(m_swerve.ZeroHeadingCommand());
  m_swerveController.Button(OperatorConstants::kResetModulesButton)
      .OnTrue(m_swerve.ResetModulesCommand());
  // Runs a command that does nothing on the Drivetrain subsystem.
  m_swerveController.Button(OperatorConstants::kFreeModulesButton)
      .ToggleOnTrue(frc2::cmd::Run([this] {}, {&m_swerve}));

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

  m_driverController.Start().OnTrue(m_arm.ResetSwitchCommand());

  m_driverController.B().WhileTrue(m_arm.LowAngleCommand());  // B
  m_driverController.Y().WhileTrue(m_arm.HighAngleCommand()); // Y
  // D-pad up
  frc2::Trigger{[&]() { return m_driverController.GetPOV() == 0; }}.WhileTrue(
      m_arm.SubstationCommand());
  // D-pad down
  frc2::Trigger{[&]() { return m_driverController.GetPOV() == 180; }}.WhileTrue(
      m_arm.IntakeCommand());
  // When the left bumper is clicked, it will open all the pistons
  // toggle for intake
  m_driverController.A().WhileTrue( // A
      frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }, {&m_intake}));
  m_driverController.A().OnFalse(
      frc2::cmd::Run([this] { m_intake.StopIntakeMotors(); }, {&m_intake}));

  m_driverController.X().WhileTrue( // X
      frc2::cmd::Run([this] { m_intake.ReverseIntakeMotors(); }, {&m_intake}));
  m_driverController.X().OnFalse(
      frc2::cmd::Run([this] { m_intake.StopIntakeMotors(); }, {&m_intake}));
  // toggle claw
  m_driverController.LeftBumper().ToggleOnTrue( // Right
      frc2::cmd::StartEnd([&] { m_claw.SetPosition(true); },
                          [&] { m_claw.SetPosition(false); }, {&m_claw}));
  // toggle arm piston
  m_driverController.RightBumper().ToggleOnTrue(frc2::cmd::Either( // Left
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(false); }, {&m_arm}),
      frc2::cmd::RunOnce([&] { m_arm.SetLegOut(true); }, {&m_arm}),
      [&]() -> bool { return m_arm.IsLegOut(); }));
  // End of big comment
  // Reset the arm if the limit switch gets accidentally tripped. (or if Arm
  // angle returns less than Physical Lower Bound or greater than Physical Upper
  // Bound)
  m_armResetTrigger.Debounce(100_ms).WhileTrue(m_arm.ResetSwitchCommand());
  m_odometryResetTrigger.WhileTrue(
      frc2::cmd::Run([this] { m_vision.CalculateRobotPoseEstimate(); }));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  frc2::CommandPtr placeConeHighBack = frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      frc2::cmd::Race(
          m_arm.HighAngleCommand(),
          frc2::cmd::Sequence(
              frc2::cmd::Run([this] {
                m_intake.ReverseIntakeMotors();
              }).WithTimeout(1_s),
              frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s),
              frc2::cmd::Run(
                  [this] { m_swerve.Drive(-1_fps, 0_fps, 0_rad_per_s, false); },
                  {&m_swerve})
                  .WithTimeout(1.5_s),
              frc2::cmd::Run([this] {
                m_arm.SetLegOut(true);
              }).WithTimeout(0.5_s))),
      frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
                              m_claw.SetPosition(true);
                            }).WithTimeout(0.2_s),
      frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
      frc2::cmd::Run(
          [this] { m_swerve.Drive(10_fps, 0_fps, 0_rad_per_s, false); },
          {&m_swerve})
          .WithTimeout(4.5_s),
      frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }));

  frc2::CommandPtr backUp =
      frc2::cmd::Run(
          [this] { m_swerve.Drive(1_fps, 0_fps, 0_rad_per_s, false); },
          {&m_swerve})
          .WithTimeout(6_s);

  frc2::CommandPtr placeConeBack = frc2::cmd::Sequence(
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
      m_arm.HighAngleCommand().WithTimeout(3_s),
      frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
      // Get arm back into intake.
      m_arm.IntakeCommand().WithTimeout(1_s), frc2::cmd::Run([this] {
                                                m_arm.SetNeckVoltage(-0.2_V);
                                              }).WithTimeout(0.3_s),
      frc2::cmd::RunOnce([this] {
        m_intake.SetIntakeMotors();
      }).WithTimeout(0.1_s),
      frc2::cmd::Run(
          [this] { m_swerve.Drive(10_fps, 0_fps, 0_rad_per_s, false); },
          {&m_swerve})
          .WithTimeout(4.5_s),
      frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }));

  return placeConeHighBack;

  // // XXX: Not tested!
  //   frc2::CommandPtr placeLowCubeCommand = frc2::cmd::Sequence(
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       m_arm.LowAngleCommand().WithTimeout(2_s),
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
  //       // Get arm back into intake.
  //       m_arm.IntakeCommand().WithTimeout(1_s), frc2::cmd::Run([this] {
  //                                                 m_arm.SetNeckVoltage(-0.2_V);
  //                                               }).WithTimeout(0.3_s),
  //       frc2::cmd::RunOnce([this] {
  //         m_intake.SetIntakeMotors();
  //       }).WithTimeout(0.1_s),
  //       frc2::cmd::Run([this] { m_swerve.Drive(-6.4_fps, 0_fps, 0_rad_per_s);
  //       },
  //                      {&m_swerve})
  //           .WithTimeout(1.75_s));

  //   frc2::CommandPtr placeHighCubeCommand = frc2::cmd::Sequence(
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       frc2::cmd::Race(
  //           m_arm.AlternateHighCubeAngleCommand(),
  //           frc2::cmd::Sequence(
  //               frc2::cmd::Run([this] {
  //                 m_intake.ReverseIntakeMotors();
  //               }).WithTimeout(1_s),
  //               frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(0.5_s),
  //               frc2::cmd::Run([this] { m_swerve.TankDrive(1_fps, 0_fps,
  //               0_rad_per_s); },
  //                              {&m_swerve})
  //                   .WithTimeout(0.8_s),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(0.5_s))),
  //       frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                               m_claw.SetPosition(true);
  //                             }).WithTimeout(0.1_s),
  //       frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //       frc2::cmd::Race(
  //           m_arm.IntakeCommand(),
  //           frc2::cmd::Run([this] { m_swerve.TankDrive(-6.4_fps, 0_fps,
  //           0_rad_per_s); },
  //                          {&m_swerve})
  //               .WithTimeout(1.75_s)),
  //       m_drivetrain.TurnTo180CCWCommand().WithTimeout(2_s),
  //       frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       }), frc2::cmd::Run([this] {
  //                         m_drivetrain.TankDrive(3_fps, 3_fps);
  //                       }).WithTimeout(1_s)),
  //       m_arm.IntakeCommand().WithTimeout(1_s),
  //       frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       frc2::cmd::Race(m_drivetrain.TurnTo180CCWCommand().WithTimeout(2_s),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       })));

  //   frc2::CommandPtr placeBlueLeftHighConeCommand = frc2::cmd::Sequence(
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       frc2::cmd::Race(
  //           m_arm.AlternateHighAngleCommand(),
  //           frc2::cmd::Sequence(
  //               frc2::cmd::Run([this] {
  //                 m_intake.ReverseIntakeMotors();
  //               }).WithTimeout(1_s),
  //               frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(0.5_s),
  //               frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps);
  //               },
  //                              {&m_drivetrain})
  //                   .WithTimeout(0.8_s),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(1.5_s))),
  //       frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                               m_claw.SetPosition(true);
  //                             }).WithTimeout(0.1_s),
  //       frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //       frc2::cmd::Race(
  //           m_arm.IntakeCommand(),
  //           frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps,
  //           -6.4_fps); },
  //                          {&m_drivetrain})
  //               .WithTimeout(1.75_s)),
  //       m_drivetrain.TurnTo155CCWCommand().WithTimeout(2_s),
  //       frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       }), frc2::cmd::Run([this] {
  //                         m_drivetrain.TankDrive(3_fps, 3_fps);
  //                       }).WithTimeout(1.5_s)),
  //       m_arm.IntakeCommand().WithTimeout(1_s),
  //       //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       // frc2::cmd::Race(m_drivetrain.TurnTo185Command().WithTimeout(2_s),
  //       //                   frc2::cmd::Run([this] {
  //       m_intake.SetIntakeMotors();
  //       //                   })),
  //       m_drivetrain.TurnTo185CCWCommand().WithTimeout(2_s));

  //   frc2::CommandPtr placeHighConeCommandNoSpin = frc2::cmd::Sequence(
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       frc2::cmd::Race(
  //           m_arm.AlternateHighAngleCommand(),
  //           frc2::cmd::Sequence(
  //               frc2::cmd::Run([this] {
  //                 m_intake.ReverseIntakeMotors();
  //               }).WithTimeout(1_s),
  //               frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(0.5_s),
  //               frc2::cmd::Run(
  //                   [this] { m_swerve.Drive(1_fps, 0_fps, 0.5_rad_per_s); },
  //                   {&m_swerve})
  //                   .WithTimeout(0.8_s),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(1.5_s))),
  //       frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                               m_claw.SetPosition(true);
  //                             }).WithTimeout(0.1_s),
  //       frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //       frc2::cmd::Race(
  //           m_arm.IntakeCommand(),
  //           frc2::cmd::Run(
  //               [this] { m_swerve.Drive(-6.4_fps, 0_fps, 0.5_rad_per_s); },
  //               {&m_swerve})
  //               .WithTimeout(2_s)) // m_swerve.TurnTo175CWCommand()
  //           .WithTimeout(2_s),
  //       frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       }), frc2::cmd::Run([this] {
  //                         m_swerve.Drive(3_fps, 0_fps, 0_fps);
  //                       }).WithTimeout(1.5_s)),
  //       m_arm.IntakeCommand().WithTimeout(1_s),
  //       frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       frc2::cmd::Race(m_swerve.TurnTo185CWCommand().WithTimeout(2_s),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       })),
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
  //       frc2::cmd::Wait(0.5_s),
  //       frc2::cmd::Race(m_arm.LowAngleCommand().WithTimeout(0.75_s),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       })),
  //       frc2::cmd::Run([this] {
  //         m_intake.ReverseIntakeMotors();
  //       }).WithTimeout(0.5_s),
  //       frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }));

  //   frc2::CommandPtr placeRedRightHighConeCommand = frc2::cmd::Sequence(
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       frc2::cmd::Race(
  //           m_arm.AlternateHighAngleCommand(),
  //           frc2::cmd::Sequence(
  //               frc2::cmd::Run([this] {
  //                 m_intake.ReverseIntakeMotors();
  //               }).WithTimeout(1_s),
  //               frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(0.5_s),
  //               frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps);
  //               },
  //                              {&m_drivetrain})
  //                   .WithTimeout(0.8_s),
  //               frc2::cmd::Run([this] {
  //                 m_arm.SetLegOut(true);
  //               }).WithTimeout(1.5_s))),
  //       frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                               m_claw.SetPosition(true);
  //                             }).WithTimeout(0.1_s),
  //       frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //       frc2::cmd::Race(
  //           m_arm.IntakeCommand(),
  //           frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps,
  //           -6.4_fps); },
  //                          {&m_drivetrain})
  //               .WithTimeout(1.75_s)),
  //       m_drivetrain.TurnTo155CWCommand().WithTimeout(2_s),
  //       frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                       frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                       }), frc2::cmd::Run([this] {
  //                         m_drivetrain.TankDrive(3_fps, 3_fps);
  //                       }).WithTimeout(1.5_s)),
  //       m_arm.IntakeCommand().WithTimeout(1_s),
  //       //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //       frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //       // frc2::cmd::Race(m_drivetrain.TurnTo185Command().WithTimeout(2_s),
  //       //                   frc2::cmd::Run([this] {
  //       m_intake.SetIntakeMotors();
  //       //                   })),
  //       m_drivetrain.TurnTo185CWCommand().WithTimeout(2_s));

  //   return placeBlueLeftHighConeCommand;

  //   if (frc::SmartDashboard::GetBoolean("CONE HIGH LEFT", false)) {
  //     fmt::print("CONE HIGH LEFT");
  //     // return placeBlueLeftHighConeCommand;
  //   } else if (frc::SmartDashboard::GetBoolean("CONE HIGH RIGHT", false)) {
  //     fmt::print("CONE HIGH RIGHT");
  //     // return placeHighConeCommandNoSpin;
  //   } else if (frc::SmartDashboard::GetBoolean("CUBE HIGH", false)) {
  //     fmt::print("CUBE HIGH");
  //     // return placeHighCubeCommand;
  //   } else if (frc::SmartDashboard::GetBoolean("CUBE LOW", false)) {
  //     fmt::print("CUBE LOW");
  //     // return placeLowCubeCommand;
  //   }
  //   m_pathPlannerTest =
  //       std::move(Autos::PathPlannerAuto(&m_swerve, &m_arm, &m_claw,
  //       &m_intake));

  //   std::vector<pathplanner::PathPlannerTrajectory> pathGroup =
  //       pathplanner::PathPlanner::loadPathGroup("Test Path",
  //                                               AutoConstants::kMaxSpeed,
  //                                               AutoConstants::kMaxAcceleration);

  //   std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  //   eventMap.emplace(
  //       "placeCone",
  //       frc2::cmd::Sequence(
  //           frc2::PrintCommand("placeCone"),
  //           frc2::cmd::RunOnce([&] { m_claw.SetPosition(false); }),
  //           frc2::cmd::Race(m_arm.AlternateHighAngleCommand(),
  //                           frc2::cmd::Sequence(frc2::cmd::Run([&] {
  //                                                 m_intake.ReverseIntakeMotors();
  //                                               }).WithTimeout(1_s),
  //                                               frc2::cmd::RunOnce([&] {
  //                                                 m_intake.StopIntakeMotors();
  //                                               }),
  //                                               frc2::cmd::Run([&] {
  //                                                 m_arm.SetLegOut(true);
  //                                               }).WithTimeout(0.5_s),
  //                                               frc2::cmd::Run([&] {
  //                                                 m_arm.SetLegOut(true);
  //                                               }).WithTimeout(1.5_s))),
  //           frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([&] {
  //                                   m_claw.SetPosition(true);
  //                                 }).WithTimeout(0.1_s),
  //           frc2::cmd::RunOnce([&] { m_arm.SetLegOut(false); }),
  //           m_arm.LowAngleCommand())
  //           .Unwrap());

  //   eventMap.emplace("pickupCube",
  //                    frc2::cmd::Sequence(
  //                        frc2::cmd::Run([&] { m_intake.SetIntakeMotors(); }),
  //                        m_arm.IntakeCommand().WithTimeout(1_s),
  //                        frc2::cmd::RunOnce([&] {
  //                        m_intake.StopIntakeMotors(); }),
  //                        frc2::cmd::RunOnce([&] { m_claw.SetPosition(false);
  //                        })) .Unwrap());

  //   eventMap.emplace("placeCube",
  //                    frc2::cmd::Sequence(
  //                        frc2::cmd::RunOnce([&] { m_claw.SetPosition(false);
  //                        }), m_arm.LowAngleCommand().WithTimeout(1_s),
  //                        frc2::cmd::RunOnce([&] { m_claw.SetPosition(true);
  //                        })) .Unwrap());

  //   pathplanner::SwerveAutoBuilder autoBuilder(
  //       [this]() -> frc::Pose2d {
  //         return m_swerve.GetPose();
  //       }, // Function to supply current robot pose
  //       [this](frc::Pose2d initPose) -> void {
  //         m_swerve.ResetOdometry(initPose);
  //       }, // Function used to reset odometry at the beginning of auto
  //       pathplanner::PIDConstants{
  //           AutoConstants::kPXController, 0.0,
  //           0.0}, // PID constants to correct for translation error (used
  //                 // to create the X and Y PID controllers)
  //       pathplanner::PIDConstants{
  //           AutoConstants::kPThetaController, 0.0,
  //           0.0}, // PID constants to correct for rotation error (used to
  //                 // create the rotation controller)
  //       [this](frc::ChassisSpeeds speeds) -> void {
  //         m_swerve.Drive(speeds.vx, speeds.vy, speeds.omega, true);
  //       },                     // Output function that accepts field relative
  //       eventMap, {&m_swerve}, // Our event map {m_swerve}, // Drive
  //                              //   requirements,
  //                              //   usually just a single drive
  //       // subsystem
  //       true // Should the path be automatically mirrored depending on
  //            // alliance
  //            // color. Optional, defaults to true
  //   );

  //   frc2::CommandPtr fullAuto = autoBuilder.fullAuto(pathGroup);

  //   return fullAuto;

  // return std::move(m_pathPlannerTest);

  // Configures the parameters needed for generating the robot's trajectory.
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed,
                               AutoConstants::kMaxAcceleration);
  // Add kinematics to the config to ensure max speed is actually obeyed.
  config.SetKinematics(m_swerve.kDriveKinematics);

  // NOTE This command hasn't been tested at all!
  // An example trajectory to follow.
  auto trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +x direction
      frc::Pose2d(0_ft, 0_ft, frc::Rotation2d(0_deg)),
      // Pass through these two inter waypoints, making an 's' curve path
      {frc::Translation2d(-2_ft, 0_ft)},
      // End 6 feet straight ahead of where we started, facing the other way
      frc::Pose2d(-4_ft, 0_ft, frc::Rotation2d(180_deg)), config);

  // PID controller for the robot heading.
  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  // Wrap angle when crossing from -pi to pi.
  thetaController.EnableContinuousInput(units::radian_t(-std::numbers::pi),
                                        units::radian_t(std::numbers::pi));

  // Generates the trajectory. Runs PID controllers on the translational speed
  // and heading of the robot.
  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      trajectory, [this]() { return m_swerve.GetPose(); },

      m_swerve.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

      [this](auto moduleStates) { m_swerve.SetModuleStates(moduleStates); },

      {&m_swerve});

  return frc2::SequentialCommandGroup(
             // Reset odometry to the starting pose of the trajectory.
             frc2::InstantCommand(
                 [this, &trajectory]() {
                   m_swerve.ResetOdometry(trajectory.InitialPose());
                 },
                 {}),
             // Execute the trajectory routine.
             std::move(swerveControllerCommand),
             // Have the robot come to a stop.
             frc2::InstantCommand(
                 [this]() {
                   m_swerve.Drive(units::meters_per_second_t(0),
                                  units::meters_per_second_t(0),
                                  units::radians_per_second_t(0), false);
                 },
                 {}))
      .ToPtr();

  //   return frc2::CommandPtr{nullptr};
  //   return placeHighConeCommandNoSpin;
}

void GenerateAndRunTrajectoryCommand() {
  // Load trajectory from file
  // can use PathWeaver/Pathplanner to generate/export json files
  // https://docs.wpilib.org/en/stable/docs/software/pathplanning/pathweaver/introduction.html

  std::string trajectoryPath = "trajectory.json";
  frc::Trajectory trajectory =
      frc::TrajectoryUtil::FromPathweaverJson(trajectoryPath);

  // use our own controller once created
  // frc::RamseteController ramseteController{kRamseteB, kRamseteZeta};
}