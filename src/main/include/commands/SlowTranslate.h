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
class SlowTranslate
    : public frc2::CommandHelper<frc2::CommandBase, SlowTranslate> {
 public:
  SlowTranslate(SwerveDrive *swerveDrive, units::meters_per_second_t xPower, units::meters_per_second_t yPower);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve;
  units::meters_per_second_t m_xPower;
  units::meters_per_second_t m_yPower;
};
