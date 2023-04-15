// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>
#include <pathplanner/lib/PathPoint.h>
#include <pathplanner/lib/commands/PPSwerveControllerCommand.h>

#include "subsystems/SwerveDrive.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class PathPlannerAutoBalance
    : public frc2::CommandHelper<frc2::CommandBase, PathPlannerAutoBalance> {
 public:
  PathPlannerAutoBalance(SwerveDrive *swerveDrive, frc::Pose2d targetPose);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve;
  frc2::PIDController m_xController;
  frc2::PIDController m_yController;
  frc2::PIDController m_rotController;
  int m_withinThresholdLoops;
  frc::Pose2d m_targetPose;
};
