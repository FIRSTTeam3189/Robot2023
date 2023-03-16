// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/WaitCommand.h>

#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Grabber.h"
#include "commands/ShootFromCarriage.h"
#include "commands/ElevatorPID.h"
#include "Constants.h"

#include <iostream>

class OneCargo
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 OneCargo> {
 public:
  OneCargo(Elevator *elevator, Grabber *grabber, Intake *intake);

 private:
  Elevator *m_elevator;
  Grabber *m_grabber;
  Intake *m_intake;
};
