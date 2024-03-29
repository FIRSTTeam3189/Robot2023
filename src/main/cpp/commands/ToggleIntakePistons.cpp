// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ToggleIntakePistons.h"

ToggleIntakePistons::ToggleIntakePistons(Intake *intake)
: m_intake(intake) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ToggleIntakePistons::Initialize() {
  // Uses intake motors to assist pistons (allows for faster and more consistent toggling)
  if (m_intake->GetPistonExtentionState())
  {
    m_intake->SetPower(-INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER);
  } else {
    m_intake->SetPower(INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER);
  }
  m_intake->ToggleIntake();
  m_isFinished = true;
}

// Called repeatedly when this Command is scheduled to run
void ToggleIntakePistons::Execute() {}

// Called once the command ends or is interrupted.
void ToggleIntakePistons::End(bool interrupted) {}

// Returns true when the command should end.
bool ToggleIntakePistons::IsFinished() {
  // End command instantly after running once
  return m_isFinished;
}
