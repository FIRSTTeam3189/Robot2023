// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include "subsystems/Vision.h"
#include "subsystems/SwerveDrive.h"
#include "Constants.h"

class TrajectoryAimAssist
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TrajectoryAimAssist> {
 public:
  TrajectoryAimAssist(Vision *vision, SwerveDrive *swerve, double targetXDistance, double targetYDistance, double targetRotAngle);

 private:
  Vision *m_vision;
  SwerveDrive *m_swerve;
  VisionData m_visionData;
  double m_targetXDistance;
  double m_targetYDistance;
  double m_targetRotAngle;
};
