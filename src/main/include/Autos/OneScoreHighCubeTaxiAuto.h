// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include "subsystems/SwerveDrive.h"

#include <string>
#include <vector>

#include "Constants.h"

class OneScoreHighCubeTaxiAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 OneScoreHighCubeTaxiAuto> {
 public:
  OneScoreHighCubeTaxiAuto(SwerveDrive *swerve, pathplanner::SwerveAutoBuilder *builder, std::string filePath);

 private:
    SwerveDrive *m_swerve;
};
