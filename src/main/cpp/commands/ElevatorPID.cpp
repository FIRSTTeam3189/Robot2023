// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorPID.h"

ElevatorPID::ElevatorPID(Elevator *elevator, Grabber *grabber, Intake *intake, double target, bool interrupt) 
: m_elevator(elevator), m_grabber(grabber), m_intake(intake), m_target(target), m_interrupt(interrupt) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
  AddRequirements(grabber);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ElevatorPID::Initialize() {
    m_intake->SetPistonExtension(true);
    m_elevator->GoToPosition(m_target);
}

// Called repeatedly when this Command is scheduled to run
void ElevatorPID::Execute() {
  // Maybe run grabber inwards during PID so the piece doesn't fall out
  // m_grabber->SetSpeed(GRABBER_CARRY_SPEED);
}

// Called once the command ends or is interrupted.
void ElevatorPID::End(bool interrupted) {}

// Returns true when the command should end.
// Ends command when elevator is close to its target
bool ElevatorPID::IsFinished() {
  // return m_elevator->AtSetpoint();
  return m_interrupt;
}
