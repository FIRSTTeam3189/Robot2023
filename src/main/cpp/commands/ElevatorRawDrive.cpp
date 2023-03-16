// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorRawDrive.h"

ElevatorRawDrive::ElevatorRawDrive(Elevator *elevator, Grabber *grabber, frc::Joystick *ted)
: m_elevator(elevator), m_grabber(grabber), m_ted(ted) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
}

// Called when the command is initially scheduled.
void ElevatorRawDrive::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ElevatorRawDrive::Execute() {
  if (-m_ted->GetRawAxis(PS5_AXIS_LSTICK_Y) > 0.05) {
      m_elevator->Drive((-m_ted->GetRawAxis(PS5_AXIS_LSTICK_Y)) / 5.0);
    } else if (-m_ted->GetRawAxis(PS5_AXIS_LSTICK_Y) < -0.05) {
      m_elevator->Drive((-m_ted->GetRawAxis(PS5_AXIS_LSTICK_Y)) / 10.0);
  }
  // m_elevator->Drive(m_power);
  // m_grabber->SetSpeed(GRABBER_CARRY_SPEED);
}

// Called once the command ends or is interrupted.
void ElevatorRawDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool ElevatorRawDrive::IsFinished() {
  return false;
}
