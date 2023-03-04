// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SlowTranslate.h"

SlowTranslate::SlowTranslate(SwerveDrive *swerveDrive, 
                             units::meters_per_second_t xPower, 
                             units::meters_per_second_t yPower) 
: m_swerve(swerveDrive), m_xPower(xPower), m_yPower(yPower) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void SlowTranslate::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SlowTranslate::Execute() {
  m_swerve->Drive(m_xPower, m_yPower, units::radians_per_second_t{0.0}, true);
}

// Called once the command ends or is interrupted.
void SlowTranslate::End(bool interrupted) {}

// Returns true when the command should end.
bool SlowTranslate::IsFinished() {
  return false;
}
