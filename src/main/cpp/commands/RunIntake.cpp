// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunIntake.h"

RunIntake::RunIntake(Intake *intake, double rollerPower, double conveyorPower, double coneCorrectPower)
: m_intake(intake), m_rollerPower(rollerPower), m_conveyorPower(conveyorPower), m_coneCorrectPower(coneCorrectPower)  {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void RunIntake::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunIntake::Execute() {
  m_intake->SetPower(m_rollerPower, m_conveyorPower, m_coneCorrectPower);
}

// Called once the command ends or is interrupted.
void RunIntake::End(bool interrupted) {}

// Returns true when the command should end.
bool RunIntake::IsFinished() {
  return false;
}
