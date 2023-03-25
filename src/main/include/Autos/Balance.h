// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>

#include "subsystems/SwerveDrive.h"
#include "commands/AutoBalance.h"
#include "commands/ResetOdometry.h"
#include "commands/DriveToPose.h"
#include "Constants.h"

class Balance
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 Balance> {
 public:
  Balance(SwerveDrive *swerve);

 private:
  SwerveDrive *m_swerve;
};
