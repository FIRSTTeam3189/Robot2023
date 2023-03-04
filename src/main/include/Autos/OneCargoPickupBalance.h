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
#include "subsystems/Shooter.h"
#include "subsystems/SwerveDrive.h"
#include "Autos/OneCargoPickupOne.h"
#include "commands/AutoBalance.h"
#include "commands/RotateTo.h"
#include "Constants.h"

#include <iostream>

class OneCargoPickupBalance
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 OneCargoPickupBalance> {
 public:
  OneCargoPickupBalance(SwerveDrive *swerveDrive, Elevator *elevator, Shooter *shooter, Intake *intake, bool isRedSide);
  
 private:
  SwerveDrive *m_swerve;
  Elevator *m_elevator;
  Shooter *m_shooter;
  Intake *m_intake;
};