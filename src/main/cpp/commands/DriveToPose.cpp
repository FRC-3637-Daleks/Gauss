#include "commands/DriveToPose.h"

DriveToPose::DriveToPose(Drivetrain *drivetrain, frc::Pose2d targetPose)
    : m_drivetrain{drivetrain}, m_targetPose{targetPose} {
  SetName("DriveToPose");
  AddRequirements(m_drivetrain);
}

void DriveToPose::Initialize() { m_startPose = {m_drivetrain->GetPose()}; }

void DriveToPose::Execute() {
  units::meter_t x = (m_targetPose.X() - m_startPose.X());
  units::meter_t y = (m_targetPose.Y() - m_startPose.Y());

  auto y_comp =
      (y / units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y)));
  auto x_comp =
      (x / units::math::sqrt(units::math::pow<2>(x) + units::math::pow<2>(y)));

  auto fwd = y_comp * AutoConstants::kMaxSpeed;
  auto strafe = x_comp * AutoConstants::kMaxSpeed;

  m_drivetrain->Drive(fwd, strafe, 0_rad_per_s, true);

  m_startPose = {m_drivetrain->GetPose()};
}

bool DriveToPose::IsFinished() {
  return getDistance().value() < kDistanceTolerance;
}

void DriveToPose::End(bool interrupted) {
  m_drivetrain->Drive(0_mps, 0_mps, 0_rad_per_s, true);
  /*wooooooooo!
  Flowers are so nice and pretty
  la la la la la*/
  /* alakazam your code is now fixed! --lily */
}