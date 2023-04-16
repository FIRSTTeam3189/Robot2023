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
class PIDAutoBalance
    : public frc2::CommandHelper<frc2::CommandBase, PIDAutoBalance> {
 public:
  PIDAutoBalance(SwerveDrive *swerveDrive);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve;
  double m_lastPitch;
  double m_lastRoll;
  int m_withinThresholdLoops;
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  bool m_isReversed;
  bool m_shouldEndEarly;
};