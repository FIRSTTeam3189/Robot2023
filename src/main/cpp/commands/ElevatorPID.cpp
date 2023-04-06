// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorPID.h"

ElevatorPID::ElevatorPID(Elevator *elevator, Intake *intake, double target, bool shouldFinish) 
: m_elevator(elevator), m_intake(intake), m_target(target), m_shouldFinish(shouldFinish) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ElevatorPID::Initialize() {
  if (m_target > 300.0) {
    m_intake->SetPistonExtension(true);
  }
  if (m_target == 0.0) {
    m_elevator->SetPID(50.0, 0.0, 0.0);
  } else {
    m_elevator->SetPID(ELEVATOR_P + 200.0, ELEVATOR_I, ELEVATOR_D);
  }
}

// Called repeatedly when this Command is scheduled to run
void ElevatorPID::Execute() {
  // Continuously tracks if elevator is close to target
  // If it is, elevator will start counting up loops where its within tolerance
  // After a certain amount of loops within tolerance, the command ends
  m_elevator->GoToPosition(m_target);
  if (m_elevator->AtSetpoint()) {
    m_shouldFinish = true;
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
    m_shouldFinish = false;
  }
}

// Called once the command ends or is interrupted.
void ElevatorPID::End(bool interrupted) {}

// Returns true when the command should end.
// Ends command when elevator is close to its target for a certain amount of time
bool ElevatorPID::IsFinished() {
  if (m_withinThresholdLoops >= ELEVATOR_SETTLE_LOOPS && m_shouldFinish) {
    return true;
  }
  return false;
}
