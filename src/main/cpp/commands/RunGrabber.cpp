// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunGrabber.h"

RunGrabber::RunGrabber(Grabber *grabber, GrabberAction action) : m_grabber(grabber) {
  AddRequirements(grabber);

  // Checks if it is in cone mode from the SmartDashboard
  bool isConeMode = frc::SmartDashboard::GetBoolean("Is Cone Mode?", false);

  // Runs the grabber at differnet speeds depending on if mode is cone or cube 
  switch (action)
  {
  case GrabberAction::Grab:
    m_power = isConeMode ? GRABBER_CONE_GRAB_SPEED : GRABBER_CUBE_GRAB_SPEED;
    break;
  case GrabberAction::Shoot:
    m_power = isConeMode ? GRABBER_CONE_SHOOT_SPEED : GRABBER_CUBE_SHOOT_SPEED;
    break;
  default:
    break;
  }
}

RunGrabber::RunGrabber(Grabber *grabber, double power)
: m_grabber(grabber), m_power(power) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(grabber);
}

// Called when the command is initially scheduled.
void RunGrabber::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void RunGrabber::Execute() {
  m_grabber->SetSpeed(m_power);
}

// Called once the command ends or is interrupted.
void RunGrabber::End(bool interrupted) {
  m_grabber->SetSpeed(0);
}

// Returns true when the command should end.
bool RunGrabber::IsFinished() {
  return false;
}
