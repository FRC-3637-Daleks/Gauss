#include "commands/Autos.h"

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/PathConstraints.h>
#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/auto/PIDConstants.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/commands/FollowPathWithEvents.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>
#include <vector>

// frc2::CommandPtr Autos::ChargeStationAuto(DalekDrive *m_drivetrain) {
//   // Move up to the charge station and balance.
//   // return std::move(m_drivetrain->DriveToDistanceCommand(60.69_in))
//   //     .AndThen(std::move(m_drivetrain->BalanceCommand()));
//   return std::move(m_drivetrain->BalanceCommand());
// }

// frc2::CommandPtr Autos::CommunityRunAuto(DalekDrive *m_drivetrain) {
//   // In the event of an emergency, use & auton.
//   return std::move(m_drivetrain->DriveToDistanceCommand(2_m));
// }

frc2::CommandPtr Autos::PathPlannerAuto(Drivetrain *m_drivetrain, Arm *m_arm,
                                        Claw *m_claw, Intake *m_intake) {
  std::vector<pathplanner::PathPlannerTrajectory> pathGroup =
      pathplanner::PathPlanner::loadPathGroup("Test Path",
                                              AutoConstants::kMaxSpeed,
                                              AutoConstants::kMaxAcceleration);

  std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;

  eventMap.emplace(
      "placeCone",
      frc2::cmd::Sequence(
          frc2::PrintCommand("placeCone"),
          frc2::cmd::RunOnce([&] { m_claw->SetPosition(false); }),
          frc2::cmd::Race(m_arm->AlternateHighAngleCommand(),
                          frc2::cmd::Sequence(frc2::cmd::Run([&] {
                                                m_intake->ReverseIntakeMotors();
                                              }).WithTimeout(1_s),
                                              frc2::cmd::RunOnce([&] {
                                                m_intake->StopIntakeMotors();
                                              }),
                                              frc2::cmd::Run([&] {
                                                m_arm->SetLegOut(true);
                                              }).WithTimeout(0.5_s),
                                              frc2::cmd::Run([&] {
                                                m_arm->SetLegOut(true);
                                              }).WithTimeout(1.5_s))),
          frc2::cmd::Wait(1_s), frc2::cmd::RunOnce([&] {
                                  m_claw->SetPosition(true);
                                }).WithTimeout(0.1_s),
          frc2::cmd::RunOnce([&] { m_arm->SetLegOut(false); }),
          m_arm->LowAngleCommand())
          .Unwrap());

  eventMap.emplace(
      "pickupCube",
      frc2::cmd::Sequence(
          frc2::cmd::Run([&] { m_intake->SetIntakeMotors(); }),
          m_arm->IntakeCommand().WithTimeout(1_s),
          frc2::cmd::RunOnce([&] { m_intake->StopIntakeMotors(); }),
          frc2::cmd::RunOnce([&] { m_claw->SetPosition(false); }))
          .Unwrap());

  eventMap.emplace("placeCube",
                   frc2::cmd::Sequence(
                       frc2::cmd::RunOnce([&] { m_claw->SetPosition(false); }),
                       m_arm->LowAngleCommand().WithTimeout(1_s),
                       frc2::cmd::RunOnce([&] { m_claw->SetPosition(true); }))
                       .Unwrap());

  pathplanner::SwerveAutoBuilder autoBuilder(
      [&]() -> frc::Pose2d {
        return m_drivetrain->GetPose();
      }, // Function to supply current robot pose
      [&](frc::Pose2d initPose) -> void {
        m_drivetrain->ResetOdometry(initPose);
      }, // Function used to reset odometry at the beginning of auto
      pathplanner::PIDConstants{
          AutoConstants::kPXController, 0.0,
          0.0}, // PID constants to correct for translation error (used
                // to create the X and Y PID controllers)
      pathplanner::PIDConstants{
          AutoConstants::kPThetaController, 0.0,
          0.0}, // PID constants to correct for rotation error (used to
                // create the rotation controller)
      [&](frc::ChassisSpeeds speeds) -> void {
        m_drivetrain->Drive(speeds.vx, speeds.vy, speeds.omega, true);
      },        // Output function that accepts field relative ChassisSpeeds
      eventMap, // Our event map
      {m_drivetrain}, // Drive requirements, usually just a single drive
                      // subsystem
      true // Should the path be automatically mirrored depending on alliance
           // color. Optional, defaults to true
  );

  frc2::CommandPtr fullAuto = autoBuilder.fullAuto(pathGroup);
  
  return fullAuto;
}