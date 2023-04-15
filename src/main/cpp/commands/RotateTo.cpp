// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTo.h"

RotateTo::RotateTo(SwerveDrive *swerve, double targetAngle)
:
m_swerve(swerve),
m_targetAngle(targetAngle),
m_rotationPIDController(SwerveDriveConstants::rotP, SwerveDriveConstants::rotI, SwerveDriveConstants::rotD),
m_withinThresholdLoops(0),
m_lastError(99999) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve);
  m_rotationPIDController.SetTolerance(1.0);
  m_rotationPIDController.EnableContinuousInput(0, 360);
}

// Called when the command is initially scheduled.
void RotateTo::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void RotateTo::Execute() {
  // Uses PID to calculate the robot's next output based on the current yaw and desired yaw
  // Multiplied by max rotational velocity
  units::angular_velocity::radians_per_second_t rot = 
    -units::radians_per_second_t{m_rotationPIDController.Calculate(m_swerve->GetNormalizedYaw(), m_targetAngle)
    * SwerveDriveConstants::maxAngularVelocity};
  
  // Tolerance is 1 degree
  if (abs(m_lastError) < 1.0) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }

  m_lastError = m_swerve->GetNormalizedYaw() - m_targetAngle;
  // Invert rot because CCW is positive
  m_swerve->PercentDrive(0.0_mps, 0.0_mps, -rot, true);
}

// Called once the command ends or is interrupted.
void RotateTo::End(bool interrupted) {
  m_timer.Stop();
  m_timer.Reset();
}

// Returns true when the command should end.
bool RotateTo::IsFinished() {
  // Ends when within 1 degree for a while
  return (m_withinThresholdLoops >= 10 || m_timer.HasElapsed(1.25_s));
}
