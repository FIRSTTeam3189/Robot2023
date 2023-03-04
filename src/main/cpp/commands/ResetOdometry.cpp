// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ResetOdometry.h"

ResetOdometry::ResetOdometry(SwerveDrive *swerve_drive, frc::Pose2d pose) 
: m_swerve_drive(swerve_drive), m_pose(pose) {
  AddRequirements(swerve_drive);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ResetOdometry::Initialize() {
  m_swerve_drive->ResetGyro();
  m_swerve_drive->ResetOdometry(m_pose);
  m_isFinished = true;
}

// Called repeatedly when this Command is scheduled to run
void ResetOdometry::Execute() {}

// Called once the command ends or is interrupted.
void ResetOdometry::End(bool interrupted) {}

// Returns true when the command should end.
bool ResetOdometry::IsFinished() {
  return m_isFinished;
}
