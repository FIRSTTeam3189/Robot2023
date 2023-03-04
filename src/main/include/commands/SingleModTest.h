// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/SwerveDrive.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class SingleModTest
    : public frc2::CommandHelper<frc2::CommandBase, SingleModTest> {
 public:
  SingleModTest(SwerveDrive *swerve_drive, SwerveModuleLocation module, double power, ManualModuleDriveType test_type);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve_drive;
  SwerveModuleLocation m_module;
  double m_power;
  ManualModuleDriveType m_test_type;
};
