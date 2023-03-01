#include "commands/Autos.h"

frc2::CommandPtr Autos::ChargeStationAuto(DalekDrive *m_drivetrain) {
  // Move up to the charge station and balance.
  // return std::move(m_drivetrain->DriveToDistanceCommand(60.69_in))
  //     .AndThen(std::move(m_drivetrain->BalanceCommand()));
  return std::move(m_drivetrain->BalanceCommand());
}