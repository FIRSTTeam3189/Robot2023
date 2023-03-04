// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/controller/PIDController.h>
#include "subsystems/Vision.h"
#include "subsystems/SwerveDrive.h"
#include "Constants.h"

#include <iostream>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */

class AimAssist
    : public frc2::CommandHelper<frc2::CommandBase, AimAssist> {
 public:
  AimAssist(Vision *vision, SwerveDrive *swerve, double targetXDistance, double targetYDistance, double targetRotAngle);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  Vision *m_vision;
  SwerveDrive *m_swerve;
  frc::PIDController m_xController;
  frc::PIDController m_yController;
  frc::PIDController m_rotationController;
  VisionData m_visionData;
  double xOutput{0.0};
  double yOutput{0.0};
  double rotOutput{0.0};
  double m_targetXDistance;
  double m_targetYDistance;
  double m_targetRotAngle;
};
