#include "commands/Autos.h"

frc2::CommandPtr Autos::ChargeStationAuto(DalekDrive *m_drivetrain) {
  // Move up to the charge station and balance.
  // return std::move(m_drivetrain->DriveToDistanceCommand(60.69_in))
  //     .AndThen(std::move(m_drivetrain->BalanceCommand()));
  return std::move(m_drivetrain->BalanceCommand());
}

frc2::CommandPtr Autos::CommunityRunAuto(DalekDrive *m_drivetrain) {
  // In the event of an emergency, use this auton.
  return std::move(m_drivetrain->DriveToDistanceCommand(2_m));
}