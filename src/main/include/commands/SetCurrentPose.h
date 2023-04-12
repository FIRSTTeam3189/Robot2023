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
class SetCurrentPose
    : public frc2::CommandHelper<frc2::CommandBase, SetCurrentPose> {
 public:
  SetCurrentPose(SwerveDrive *swerve_drive, frc::Pose2d pose);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve_drive;
  frc::Pose2d m_pose;
  bool m_isFinished = false;
};
