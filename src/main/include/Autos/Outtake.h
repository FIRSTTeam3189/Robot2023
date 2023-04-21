// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Intake.h"
#include "subsystems/SwerveDrive.h"
#include "commands/RunIntake.h"
#include "Constants.h"

class Outtake
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 Outtake> {
 public:
  Outtake(SwerveDrive *swerve, Intake *intake);

 private:
  SwerveDrive *m_swerve;
  Intake *m_intake;
};
