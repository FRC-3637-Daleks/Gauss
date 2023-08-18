#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandPtr.h>

#include "Constants.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Arm.h"
#include "subsystems/Claw.h"
#include "subsystems/Intake.h"

namespace Autos {
// frc2::CommandPtr ChargeStationAuto(Drivetrain *m_drivetrain);

// frc2::CommandPtr CommunityRunAuto(Drivetrain *m_drivetrain);

frc2::CommandPtr PathPlannerAuto(Drivetrain *m_drivetrain, Arm *m_arm, Claw *m_claw, Intake *m_intake);
} // namespace Autos