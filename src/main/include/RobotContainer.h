#pragma once

#include <frc/XboxController.h>
#include <frc/filesystem.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <fstream>

#include "Constants.h"
#include "commands/Autos.h"
#include "subsystems/Arm.h"
#include "subsystems/Claw.h"
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

  frc2::Trigger m_armResetTrigger{[this]() -> bool {
    return m_arm.GetNeckAngle() < ArmConstants::kNeckPhysicalLowerBound ||
           m_arm.GetNeckAngle() > ArmConstants::kNeckPhysicalUpperBound;
  }};

  frc2::Trigger m_odometryResetTrigger {
    [this]() -> bool {
      return m_swerve.GetPose().X() == 0_m && m_swerve.GetPose().Y() == 0_m;
    }
  };

  Arm m_arm;
  Claw m_claw;

  Intake m_intake;

  Drivetrain m_swerve;

  Vision m_vision{[this](frc::Pose2d pose, units::second_t timestamp) -> void {
                    m_swerve.AddVisionPoseEstimate(pose, timestamp);
                  },
                  [this]() -> frc::Pose2d { return m_swerve.GetPose(); }};

  void ConfigureBindings();

  // const double kMaxSpeed = 3.0; // Maximum speed in meters per second
  // const double kMaxAcceleration =
  //     2.0; // Maximum acceleration in meters per second squared
  // const double kRamseteB = 2.0;    // Ramsete controller's B coefficient
  // const double kRamseteZeta = 0.7; // Ramsete controller's Zeta coefficient
  // const double kTrackWidth = 0.6;  // Width of your robot's drivetrain

  std::string deployDir = frc::filesystem::GetDeployDirectory();
  std::ifstream infoFile{deployDir + "/gitInfo.txt"};
  std::string gitInfo;
};
