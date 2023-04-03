// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Grabber.h"
#include "subsystems/SwerveDrive.h"
#include "commands/ResetOdometry.h"
#include "commands/ToggleIntakePistons.h"
#include "commands/RunIntake.h"
#include "commands/RotateTo.h"
#include "Autos/OneCargo.h"
#include "Constants.h"
#include "RobotParameters.h"

#include <iostream>


class OneCargoPickupOne
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 OneCargoPickupOne> {
 public:
  OneCargoPickupOne(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake, int elevatorTarget);

 private:
  SwerveDrive *m_swerve;
  Elevator *m_elevator;
  Grabber *m_grabber;
  Intake *m_intake;
};
