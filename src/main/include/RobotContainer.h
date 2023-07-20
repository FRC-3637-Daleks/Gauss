#pragma once

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "commands/Autos.h"
#include "subsystems/Arm.h"
#include "subsystems/Claw.h"
#include "subsystems/DalekDrive.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Vision.h"

// testing for Trajectories (need to organize after)

#include <frc/controller/PIDController.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc2/command/RamseteCommand.h>

class RobotContainer {
public:
  RobotContainer();

  // frc2::Command *GetAutonomousCommand();
  frc2::CommandPtr GetAutonomousCommand();

  // frc::SendableChooser<frc2::Command *> m_chooser;

  void GenerateAndRunTrajectoryCommand();

private:
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kXboxControllerPort};
  frc2::CommandXboxController m_swerveController{
      OperatorConstants::kDriverControllerPort};

  // frc2::Trigger m_armResetTrigger{[this]() -> bool {
  //   return m_arm.GetNeckAngle() < ArmConstants::kNeckPhysicalLowerBound ||
  //          m_arm.GetNeckAngle() > ArmConstants::kNeckPhysicalUpperBound;
  // }};

  // Arm m_arm;
  // Claw m_claw;
  // Vision m_vision{[this](frc::Pose2d pose, units::second_t timestamp) {
  //                   m_swerve.AddVisionPoseEstimate(pose, timestamp);
  //                 },
  //                 [this] { return m_swerve.GetPose(); }};

  // Intake m_intake;

  Drivetrain m_swerve;

  void ConfigureBindings();

  // const double kMaxSpeed = 3.0; // Maximum speed in meters per second
  // const double kMaxAcceleration =
  //     2.0; // Maximum acceleration in meters per second squared
  // const double kRamseteB = 2.0;    // Ramsete controller's B coefficient
  // const double kRamseteZeta = 0.7; // Ramsete controller's Zeta coefficient
  // const double kTrackWidth = 0.6;  // Width of your robot's drivetrain
};
