// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/RunCommand.h>

#include "subsystems/Elevator.h"
#include "subsystems/Grabber.h"
#include "subsystems/Intake.h"
#include "commands/ElevatorPID.h"
#include "commands/RunIntake.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class UltraShoot
    : public frc2::CommandHelper<frc2::SequentialCommandGroup, UltraShoot> {
 public:
  UltraShoot(Elevator *elevator, Intake *intake, Grabber *grabber);

 private:
  Elevator *m_elevator;
  Intake *m_intake;
  Grabber *m_grabber;
};
