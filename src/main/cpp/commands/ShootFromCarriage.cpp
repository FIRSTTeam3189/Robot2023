// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootFromCarriage.h"

ShootFromCarriage::ShootFromCarriage(Grabber *grabber, double power)
: m_grabber(grabber), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(grabber);
}

// Called when the command is initially scheduled.
void ShootFromCarriage::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootFromCarriage::Execute() {
  m_grabber->SetSpeed(m_power);
}

// Called once the command ends or is interrupted.
void ShootFromCarriage::End(bool interrupted) {
  m_grabber->SetSpeed(0);
}

// Returns true when the command should end.
bool ShootFromCarriage::IsFinished() {
  return false;
}
