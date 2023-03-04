// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootFromCarriage.h"

ShootFromCarriage::ShootFromCarriage(Shooter *shooter, double power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(shooter);
}

// Called when the command is initially scheduled.
void ShootFromCarriage::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootFromCarriage::Execute() {
  m_shooter->SetSpeed(m_power);
}

// Called once the command ends or is interrupted.
void ShootFromCarriage::End(bool interrupted) {}

// Returns true when the command should end.
bool ShootFromCarriage::IsFinished() {
  return false;
}
