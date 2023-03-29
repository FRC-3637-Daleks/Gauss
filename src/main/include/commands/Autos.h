#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "subsystems/DalekDrive.h"

namespace Autos {
frc2::CommandPtr ChargeStationAuto(DalekDrive *m_drivetrain);

frc2::CommandPtr CommunityRunAuto(DalekDrive *m_drivetrain);
} // namespace Autos