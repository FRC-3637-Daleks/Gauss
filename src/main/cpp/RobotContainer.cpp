#include "RobotContainer.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Translation2d.h>
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

RobotContainer::RobotContainer() {
  frc::SmartDashboard::PutBoolean("Running SetNeckAngle", false);
  // frc::SmartDashboard::PutData(&m_chooser);
  frc::SmartDashboard::PutBoolean("CONE HIGH LEFT", false);
  frc::SmartDashboard::PutBoolean("CONE HIGH RIGHT", false);
  frc::SmartDashboard::PutBoolean("CUBE HIGH", false);
  frc::SmartDashboard::PutBoolean("CUBE LOW", false);

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  frc::DataLogManager::LogNetworkTables(true);

  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {

  // Move neck with xbox joystick.
  m_arm.SetDefaultCommand(frc2::cmd::Run(
      [this] {
        double output = m_driverController.GetLeftY();
        m_arm.SetNeckVoltage(-2.0 * std::copysign(output * output, output) *
                             1_V);
      },
      {&m_arm}));

  // Set up the default drive command.

  auto fwd = [this]() -> double {
    return (AutoConstants::kMaxSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kForwardAxis),
                OperatorConstants::kDeadband))
        .value();
  };
  auto strafe = [this]() -> double {
    return (AutoConstants::kMaxSpeed *
            frc::ApplyDeadband(
                m_swerveController.GetRawAxis(OperatorConstants::kStrafeAxis),
                OperatorConstants::kDeadband))
        .value();
  };

  auto rot = [this]() -> double {
    return (AutoConstants::kMaxAngularSpeed *
            frc::ApplyDeadband(-m_swerveController.GetRawAxis(
                                   OperatorConstants::kRotationAxis),
                               OperatorConstants::kDeadband))
        .value();
  };

  m_swerve.SetDefaultCommand(frc2::ConditionalCommand(
      m_swerve.SwerveCommandFieldRelative(fwd, strafe, rot).Unwrap(),
      m_swerve.SwerveCommand(fwd, strafe, rot).Unwrap(), [this]() -> bool {
        return m_swerveController.GetRawButton(
            OperatorConstants::kFieldRelativeButton);
      }));

  // Swerve Button Configs
  // Configure button bindings.
  m_swerveController.Button(OperatorConstants::kZeroHeadingButton)
      .OnTrue(m_swerve.ZeroHeadingCommand());
  m_swerveController.Button(OperatorConstants::kResetModulesButton)
      .OnTrue(m_swerve.ResetModulesCommand());
  // Runs a command that does nothing on the Drivetrain subsytem.
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
  // // XXX: Not tested!
  // frc2::CommandPtr placeLowCubeCommand = frc2::cmd::Sequence(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     m_arm.LowAngleCommand().WithTimeout(2_s),
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
  //     // Get arm back into intake.
  //     m_arm.IntakeCommand().WithTimeout(1_s), frc2::cmd::Run([this] {
  //                                               m_arm.SetNeckVoltage(-0.2_V);
  //                                             }).WithTimeout(0.3_s),
  //     frc2::cmd::RunOnce([this] {
  //       m_intake.SetIntakeOn(true);
  //     }).WithTimeout(0.1_s),
  //     frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps); },
  //                    {&m_drivetrain})
  //         .WithTimeout(1.75_s));

  // frc2::CommandPtr placeHighCubeCommand = frc2::cmd::Sequence(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     frc2::cmd::Race(
  //         m_arm.AlternateHighCubeAngleCommand(),
  //         frc2::cmd::Sequence(
  //             frc2::cmd::Run([this] {
  //               m_intake.ReverseIntakeMotors();
  //             }).WithTimeout(1_s),
  //             frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(0.5_s),
  //             frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps);
  //             },
  //                            {&m_drivetrain})
  //                 .WithTimeout(0.8_s),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(0.5_s))),
  //     frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                             m_claw.SetPosition(true);
  //                           }).WithTimeout(0.1_s),
  //     frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //     frc2::cmd::Race(
  //         m_arm.IntakeCommand(),
  //         frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps);
  //         },
  //                        {&m_drivetrain})
  //             .WithTimeout(1.75_s)),
  //     m_drivetrain.TurnTo180CCWCommand().WithTimeout(2_s),
  //     frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                     frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }),
  //                     frc2::cmd::Run([this] {
  //                       m_drivetrain.TankDrive(3_fps, 3_fps);
  //                     }).WithTimeout(1_s)),
  //     m_arm.IntakeCommand().WithTimeout(1_s),
  //     frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     frc2::cmd::Race(m_drivetrain.TurnTo180CCWCommand().WithTimeout(2_s),
  //                     frc2::cmd::Run([this] { m_intake.SetIntakeMotors();
  //                     })));

  // frc2::CommandPtr placeBlueLeftHighConeCommand = frc2::cmd::Sequence(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     frc2::cmd::Race(
  //         m_arm.AlternateHighAngleCommand(),
  //         frc2::cmd::Sequence(
  //             frc2::cmd::Run([this] {
  //               m_intake.ReverseIntakeMotors();
  //             }).WithTimeout(1_s),
  //             frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(0.5_s),
  //             frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps);
  //             },
  //                            {&m_drivetrain})
  //                 .WithTimeout(0.8_s),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(1.5_s))),
  //     frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                             m_claw.SetPosition(true);
  //                           }).WithTimeout(0.1_s),
  //     frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //     frc2::cmd::Race(
  //         m_arm.IntakeCommand(),
  //         frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps);
  //         },
  //                        {&m_drivetrain})
  //             .WithTimeout(1.75_s)),
  //     m_drivetrain.TurnTo155CCWCommand().WithTimeout(2_s),
  //     frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                     frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }),
  //                     frc2::cmd::Run([this] {
  //                       m_drivetrain.TankDrive(3_fps, 3_fps);
  //                     }).WithTimeout(1.5_s)),
  //     m_arm.IntakeCommand().WithTimeout(1_s),
  //     //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     //   frc2::cmd::Race(m_drivetrain.TurnTo185Command().WithTimeout(2_s),
  //     //                   frc2::cmd::Run([this] {
  //     m_intake.SetIntakeMotors();
  //     //                   })),
  //     m_drivetrain.TurnTo185CCWCommand().WithTimeout(2_s));

  // frc2::CommandPtr placeHighConeCommandNoSpin = frc2::cmd::Sequence(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     frc2::cmd::Race(
  //         m_arm.AlternateHighAngleCommand(),
  //         frc2::cmd::Sequence(
  //             frc2::cmd::Run([this] {
  //               m_intake.ReverseIntakeMotors();
  //             }).WithTimeout(1_s),
  //             frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(0.5_s),
  //             frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps);
  //             },
  //                            {&m_drivetrain})
  //                 .WithTimeout(0.8_s),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(1.5_s))),
  //     frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                             m_claw.SetPosition(true);
  //                           }).WithTimeout(0.1_s),
  //     frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //     frc2::cmd::Race(
  //         m_arm.IntakeCommand(),
  //         frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps);
  //         },
  //                        {&m_drivetrain})
  //             .WithTimeout(2_s))
  //     //   m_drivetrain.TurnTo175CWCommand().WithTimeout(2_s),
  //     //   frc2::cmd::Race(m_arm.LowAngleCommand(),
  //     //                   frc2::cmd::Run([this] {
  //     m_intake.SetIntakeMotors();
  //     //                   }), frc2::cmd::Run([this] {
  //     //                     m_drivetrain.TankDrive(3_fps, 3_fps);
  //     //                   }).WithTimeout(1.5_s)),
  //     //   m_arm.IntakeCommand().WithTimeout(1_s),
  //     //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //     //   frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     // frc2::cmd::Race(m_drivetrain.TurnTo185CWCommand().WithTimeout(2_s),
  //     //                   frc2::cmd::Run([this] {
  //     m_intake.SetIntakeMotors();
  //     //                   })),
  //     //   frc2::cmd::RunOnce([this] { m_claw.SetPosition(true); }),
  //     //   frc2::cmd::Wait(0.5_s),
  //     //   frc2::cmd::Race(m_arm.LowAngleCommand().WithTimeout(0.75_s),
  //     //                   frc2::cmd::Run([this] {
  //     m_intake.SetIntakeMotors();
  //     //                   })),
  //     //   frc2::cmd::Run([this] {
  //     //     m_intake.ReverseIntakeMotors();
  //     //   }).WithTimeout(0.5_s),
  //     //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); })
  // );

  // frc2::CommandPtr placeRedRightHighConeCommand = frc2::cmd::Sequence(
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     frc2::cmd::Race(
  //         m_arm.AlternateHighAngleCommand(),
  //         frc2::cmd::Sequence(
  //             frc2::cmd::Run([this] {
  //               m_intake.ReverseIntakeMotors();
  //             }).WithTimeout(1_s),
  //             frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(0.5_s),
  //             frc2::cmd::Run([this] { m_drivetrain.TankDrive(1_fps, 1_fps);
  //             },
  //                            {&m_drivetrain})
  //                 .WithTimeout(0.8_s),
  //             frc2::cmd::Run([this] {
  //               m_arm.SetLegOut(true);
  //             }).WithTimeout(1.5_s))),
  //     frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([this] {
  //                             m_claw.SetPosition(true);
  //                           }).WithTimeout(0.1_s),
  //     frc2::cmd::RunOnce([this] { m_arm.SetLegOut(false); }),
  //     frc2::cmd::Race(
  //         m_arm.IntakeCommand(),
  //         frc2::cmd::Run([this] { m_drivetrain.TankDrive(-6.4_fps, -6.4_fps);
  //         },
  //                        {&m_drivetrain})
  //             .WithTimeout(1.75_s)),
  //     m_drivetrain.TurnTo155CWCommand().WithTimeout(2_s),
  //     frc2::cmd::Race(m_arm.LowAngleCommand(),
  //                     frc2::cmd::Run([this] { m_intake.SetIntakeMotors(); }),
  //                     frc2::cmd::Run([this] {
  //                       m_drivetrain.TankDrive(3_fps, 3_fps);
  //                     }).WithTimeout(1.5_s)),
  //     m_arm.IntakeCommand().WithTimeout(1_s),
  //     //   frc2::cmd::RunOnce([this] { m_intake.StopIntakeMotors(); }),
  //     frc2::cmd::RunOnce([this] { m_claw.SetPosition(false); }),
  //     //   frc2::cmd::Race(m_drivetrain.TurnTo185Command().WithTimeout(2_s),
  //     //                   frc2::cmd::Run([this] {
  //     m_intake.SetIntakeMotors();
  //     //                   })),
  //     m_drivetrain.TurnTo185CWCommand().WithTimeout(2_s));

  // // return placeBlueLeftHighConeCommand;

  // if (frc::SmartDashboard::GetBoolean("CONE HIGH LEFT", false)) {
  //   fmt::print("CONE HIGH LEFT");
  //   // return placeBlueLeftHighConeCommand;
  // } else if (frc::SmartDashboard::GetBoolean("CONE HIGH RIGHT", false)) {
  //   fmt::print("CONE HIGH RIGHT");
  //   // return placeHighConeCommandNoSpin;
  // } else if (frc::SmartDashboard::GetBoolean("CUBE HIGH", false)) {
  //   fmt::print("CUBE HIGH");
  //   // return placeHighCubeCommand;
  // } else if (frc::SmartDashboard::GetBoolean("CUBE LOW", false)) {
  //   fmt::print("CUBE LOW");
  //   // return placeLowCubeCommand;
  // }

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
      {frc::Translation2d(2_ft, 2_ft), frc::Translation2d(4_ft, -2_ft)},
      // End 6 feet straight ahead of where we started, facing the other way
      frc::Pose2d(6_ft, 0_ft, frc::Rotation2d(180_deg)), config);

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

  // return frc2::CommandPtr{nullptr};
  // return placeHighConeCommandNoSpin;
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