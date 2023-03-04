// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/UpdatePIDValues.h"

UpdatePIDValues::UpdatePIDValues(SwerveDrive *swerve_drive) : m_swerve_drive(swerve_drive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_drive);
}

// Called when the command is initially scheduled.
void UpdatePIDValues::Initialize() {
  m_swerve_drive->UpdatePIDValues();
}

// Called repeatedly when this Command is scheduled to run
void UpdatePIDValues::Execute() {}

// Called once the command ends or is interrupted.
void UpdatePIDValues::End(bool interrupted) {}

// Returns true when the command should end.
bool UpdatePIDValues::IsFinished() {
  return false;
}
