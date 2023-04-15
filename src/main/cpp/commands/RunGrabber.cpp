// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RunGrabber.h"

RunGrabber::RunGrabber(Grabber *grabber, GrabberAction action) : m_grabber(grabber), m_action(action) {
  AddRequirements(grabber);
}

RunGrabber::RunGrabber(Grabber *grabber, double power)
: m_grabber(grabber), m_power(power), m_action(GrabberAction::None) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(grabber);
}

// Called when the command is initially scheduled.
void RunGrabber::Initialize() {
  m_timer.Start();
  // Checks if it is in cone mode from the SmartDashboard
  m_isConeMode = frc::SmartDashboard::GetBoolean("Is Cone Mode?", false);

  // Runs the grabber at differnet speeds depending on if mode is cone or cube 
  switch (m_action)
  {
  case GrabberAction::Grab:
    m_power = m_isConeMode ? GRABBER_CONE_GRAB_SPEED : GRABBER_CUBE_GRAB_SPEED;
    break;
  case GrabberAction::Shoot:
    m_power = m_isConeMode ? GRABBER_CONE_SHOOT_SPEED : GRABBER_CUBE_SHOOT_SPEED;
    break;
  case GrabberAction::None:
    break;
  default:
    break;
  }
}

// Called repeatedly when this Command is scheduled to run
void RunGrabber::Execute() {
  m_grabber->SetSpeed(m_power);
}

// Called once the command ends or is interrupted.
void RunGrabber::End(bool interrupted) {
  m_grabber->SetSpeed(0);
  m_timer.Stop();
  m_timer.Reset();
}

// Returns true when the command should end.
bool RunGrabber::IsFinished() {
  if (!m_isConeMode && m_grabber->IsPieceGrabbed() && m_power < 0 && m_timer.HasElapsed(GRABBER_MINIMUM_SPIN_TIME)) {
    return true;
  }
  else if (m_isConeMode && m_grabber->IsPieceGrabbed() && m_power > 0 && m_timer.HasElapsed(GRABBER_MINIMUM_SPIN_TIME)) {
    return true;
  }
  else {
    return false;
  }
}
