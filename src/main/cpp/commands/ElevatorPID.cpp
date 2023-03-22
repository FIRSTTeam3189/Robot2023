// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorPID.h"

ElevatorPID::ElevatorPID(Elevator *elevator, Grabber *grabber, Intake *intake, double target, bool shouldFinish) 
: m_elevator(elevator), m_grabber(grabber), m_intake(intake), m_target(target), m_shouldFinish(shouldFinish) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
  AddRequirements(grabber);
  AddRequirements(intake);
  std::cout << "PID\n";
}

// Called when the command is initially scheduled.
void ElevatorPID::Initialize() {
    m_intake->SetPistonExtension(true);
}

// Called repeatedly when this Command is scheduled to run
void ElevatorPID::Execute() {
  // Maybe run grabber inwards during PID so the piece doesn't fall out
  // m_grabber->SetSpeed(GRABBER_CARRY_SPEED);
  std::cout << "PID running\n";
  m_elevator->GoToPosition(m_target);
  if (m_elevator->AtSetpoint()) {
    m_shouldFinish = true;
    m_withinThresholdLoops++;
    // Maybe retract intake when PID command is done -- test first
    // m_intake->SetPistonExtension(false);
  } else {
    m_withinThresholdLoops = 0;
    m_shouldFinish = false;
  }

  if(m_elevator->GetPosition() == 0) {
    // m_intake->SetPistonExtension(false);
  }
}

// Called once the command ends or is interrupted.
void ElevatorPID::End(bool interrupted) {}

// Returns true when the command should end.
// Ends command when elevator is close to its target
bool ElevatorPID::IsFinished() {
  // return m_elevator->AtSetpoint();
  // return m_shouldFinish;
  std::cout << "Threshold loops " << m_withinThresholdLoops << " should finish " << m_shouldFinish << "\n";
  if (m_withinThresholdLoops >= ELEVATOR_SETTLE_LOOPS && m_shouldFinish) {
    m_elevator->SetRunningState(false);
    return true;
  }
  return false;
}
