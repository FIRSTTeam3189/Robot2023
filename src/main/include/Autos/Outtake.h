// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>

#include "subsystems/Intake.h"
#include "subsystems/SwerveDrive.h"

class Outtake
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 Outtake> {
 public:
  Outtake(Intake *intake, SwerveDrive *swerve);

 private:
  Intake *m_intake;
  SwerveDrive *m_swerve;
};
