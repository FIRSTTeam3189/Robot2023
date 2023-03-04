// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "subsystems/SwerveDrive.h"
#include "Constants.h"
#include "commands/ResetOdometry.h"
#include "commands/RotateTo.h"
#include "commands/AutoBalance.h"

#include <iostream>

class TestAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TestAuto> {
 public:
  TestAuto(SwerveDrive *swerveDrive);

 private:
  SwerveDrive *m_swerve;
};
