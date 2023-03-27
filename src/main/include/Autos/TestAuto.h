// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/controller/PIDController.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/RepeatCommand.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/ParallelDeadlineGroup.h>

#include "subsystems/SwerveDrive.h"
#include "subsystems/Elevator.h"
#include "subsystems/Grabber.h"
#include "subsystems/Intake.h"
#include "Constants.h"
#include "commands/ResetOdometry.h"
#include "commands/RunIntake.h"
#include "commands/RotateTo.h"
#include "commands/DriveToPose.h"
#include "commands/AutoBalance.h"
#include "commands/ShootFromCarriage.h"
#include "Autos/Balance.h"

#include <iostream>

class TestAuto
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TestAuto> {
 public:
  TestAuto(SwerveDrive *swerveDrive, Elevator *elevator, Intake *intake, Grabber *grabber, int testNum);

 private:
  SwerveDrive *m_swerve;
  Elevator *m_elevator;
  Intake *m_intake;
  Grabber *m_grabber;
};
