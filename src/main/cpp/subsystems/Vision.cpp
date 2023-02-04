// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Vision.h"
#include <frc/apriltag/AprilTagFieldLayout.h>



Vision::Vision() {
  // Implementation of subsystem constructor goes here.
}

frc2::CommandPtr Vision::CommandTurnToTarget() {
  /*double rotationSpeed;

  // Vision-alignment mode
  // Query the latest result from PhotonVision
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();

  if (result.HasTargets()) {
    // Rotation speed is the output of the PID controller
    rotationSpeed = -controller.Calculate(result.GetBestTarget().GetYaw(),
    0);
    
  } else {
    // If we have no targets, stay still.
    rotationSpeed = 0;
  }
  */

  // Manual Driver Mode

  // Use our forward/turn speeds to control the drivetrain
  //drive.ArcadeDrive(rotationSpeed);
}

void Vision::Periodic() {
  // Implementation of subsystem periodic method goes here.
  bool foundTarget;
  photonlib::PhotonPipelineResult result = camera.GetLatestResult();
  if(result.HasTargets()){
    foundTarget = true;
  }
}

void Vision::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}
