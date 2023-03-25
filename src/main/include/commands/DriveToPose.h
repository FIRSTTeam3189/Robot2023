// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/SwerveDrive.h"
#include "Constants.h"
#include <frc/controller/PIDController.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveToPose
    : public frc2::CommandHelper<frc2::CommandBase, DriveToPose> {
 public:
  DriveToPose(SwerveDrive *swerve, frc::Transform2d targetTransform, double targetAngle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve;
  frc::Transform2d m_transform;
  double m_targetAngle;
  frc::Pose2d m_currentPose;
  frc::Pose2d m_targetPose;
  frc2::PIDController m_xPIDController;
  frc2::PIDController m_yPIDController;
  frc2::PIDController m_rotPIDController;
};
