// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ElevatorPID.h"

ElevatorPID::ElevatorPID(Elevator *elevator, ElevatorLevel level, bool shouldFinish) 
: m_elevator(elevator), m_shouldFinish(shouldFinish), m_level(level) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
}

ElevatorPID::ElevatorPID(Elevator *elevator, double target, bool shouldFinish) 
: m_elevator(elevator), m_target(target), m_shouldFinish(shouldFinish), m_level(ElevatorLevel::None) {
  AddRequirements(elevator);
}

// Called when the command is initially scheduled.
void ElevatorPID::Initialize() {
  // Read the current game piece mode from SmartDashboard
  bool isConeMode = frc::SmartDashboard::GetBoolean("Is Cone Mode?", false);

  // Check the mode and target height and choose target height based on that
  // Cones are scored at a higher level, for example
  // So if the co-driver sets the robot to cone mode and scores, the elevator will raise slightly higher
  switch (m_level)
  {
  case ElevatorLevel::Low:
    m_target = isConeMode ? ELEVATOR_LOW_CONE_TARGET : ELEVATOR_LOW_CUBE_TARGET;
    break;
  case ElevatorLevel::Mid:
    m_target = isConeMode ? ELEVATOR_MID_CONE_TARGET : ELEVATOR_MID_CUBE_TARGET;
    break;
  case ElevatorLevel::High:
    m_target = isConeMode ? ELEVATOR_HIGH_CONE_TARGET : ELEVATOR_HIGH_CUBE_TARGET;
    break;
  case ElevatorLevel::DoubleSubstation:
    m_target = isConeMode ? ELEVATOR_DOUBLE_SUBSTATION_CONE_TARGET : ELEVATOR_DOUBLE_SUBSTATION_CUBE_TARGET;
    break;
  case ElevatorLevel::None:
    break;
  default:
    break;
  }

  // Go slower on the way down, or else go normal speed
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
  // Ends if elevator is close to target for a while
  if (m_withinThresholdLoops >= ELEVATOR_SETTLE_LOOPS && m_shouldFinish) {
    return true;
  }
  return false;
}
