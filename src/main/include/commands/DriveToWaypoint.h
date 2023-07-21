#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"

class DriveToWaypoint : public frc2::CommandHelper<frc2::CommandBase, DriveToWaypoint> {
public:
  DriveToWaypoint(Drivetrain *drivetrain, frc::Pose2d targetPose);

  // Code to run on initialization of command.
  void Initialize() override;

  // Code to run until IsFinished condition is met.
  void Execute() override;

  // Condition to check for finishing command.
  bool IsFinished() override;

  // Code to run on end of command.
  void End(bool interrupted) override;

private:
  Drivetrain *m_drivetrain;

  frc::Pose2d m_targetPose;
  frc::Pose2d m_startPose;

  double kDistanceTolerance = 0.1;

  std::function<units::meter_t()> getDistance = [this]() -> units::meter_t {
    return units::math::hypot((m_targetPose.X() - m_startPose.X()),
                              (m_targetPose.Y() - m_startPose.Y()));
  };
};