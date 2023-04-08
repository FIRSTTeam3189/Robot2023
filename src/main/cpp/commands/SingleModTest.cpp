// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SingleModTest.h"

SingleModTest::SingleModTest(SwerveDrive *swerve_drive, SwerveModuleLocation module, double power, ManualModuleDriveType test_type) 
: m_swerve_drive(swerve_drive), m_module(module), m_power(power), m_test_type(test_type) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_drive);
}

// Called when the command is initially scheduled.
void SingleModTest::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SingleModTest::Execute() {
  // Sets constant speed or rotational velocity of one module
  if(m_test_type == ManualModuleDriveType::forward) {
    m_swerve_drive->ManualModuleSpeed(m_module, m_power);
  } else if (m_test_type == ManualModuleDriveType::turn) {
    m_swerve_drive->ManualModuleTurn(m_module, m_power);
  } else {
    m_swerve_drive->Stop();
  }
}

// Called once the command ends or is interrupted.
void SingleModTest::End(bool interrupted) {}

// Returns true when the command should end.
bool SingleModTest::IsFinished() {
  return false;
}
