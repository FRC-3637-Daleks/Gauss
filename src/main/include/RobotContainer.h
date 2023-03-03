#pragma once

#include <frc/XboxController.h>
#include <frc/filter/SlewRateLimiter.h>
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
#include "subsystems/Intake.h"
#include "subsystems/Vision.h"

class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  frc2::CommandJoystick m_leftJoystick{OperatorConstants::kLeftJoystickPort};
  frc2::CommandJoystick m_rightJoystick{OperatorConstants::kRightJoystickPort};
  frc2::CommandXboxController m_driverController{
      OperatorConstants::kXboxControllerPort};

  Arm m_arm;
  DalekDrive m_drivetrain;
  Claw m_claw;
  Vision m_vision{[this](frc::Pose2d pose, units::second_t timestamp) {
                    m_drivetrain.AddVisionPoseEstimate(pose, timestamp);
                  },
                  [this] { return m_drivetrain.GetPose(); }};

  Intake m_intake;

  frc2::CommandPtr m_chargeStationAuto{Autos::ChargeStationAuto(&m_drivetrain)};

  frc2::CommandPtr m_communityRunAuto{Autos::CommunityRunAuto(&m_drivetrain)};

  void ConfigureBindings();
};
