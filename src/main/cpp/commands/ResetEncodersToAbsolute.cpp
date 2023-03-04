// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ResetEncodersToAbsolute.h"

ResetEncodersToAbsolute::ResetEncodersToAbsolute(SwerveDrive *swerve_drive) : m_swerve_drive(swerve_drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_drive);
}

// Called when the command is initially scheduled.
void ResetEncodersToAbsolute::Initialize() {
  m_swerve_drive->ResetEncodersToAbsolute();
}

// Called repeatedly when this Command is scheduled to run
void ResetEncodersToAbsolute::Execute() {}

// Called once the command ends or is interrupted.
void ResetEncodersToAbsolute::End(bool interrupted) {}

// Returns true when the command should end.
bool ResetEncodersToAbsolute::IsFinished() {
  return false;
}
