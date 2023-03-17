// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/UltraShoot.h"

UltraShoot::UltraShoot(Elevator *elevator, Grabber *grabber)
: m_elevator(elevator), m_grabber(grabber) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
  AddRequirements(grabber);
}

// Called when the command is initially scheduled.
void UltraShoot::Initialize() {
  // Start driving elevator up quickly
  m_elevator->GoToPosition(ELEVATOR_ULTRA_SHOOT_TARGET);
  m_elevator->SetPID(ELEVATOR_ULTRA_SHOOT_P, ELEVATOR_I, ELEVATOR_D);
}

// Called repeatedly when this Command is scheduled to run
void UltraShoot::Execute() {
  if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
    m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
  }
}

// Called once the command ends or is interrupted.
void UltraShoot::End(bool interrupted) {}

// Returns true when the command should end.
bool UltraShoot::IsFinished() {
  if (m_elevator->AtSetpoint()) {
    m_grabber->SetSpeed(0);
    return true;
  }
  return false;
}
