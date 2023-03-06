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
#include "commands/ShootFromCarriage.h"
#include "commands/RotateTo.h"
#include "commands/ElevatorPID.h"
#include "commands/AimAssist.h"
#include "Autos/OneCargoPickupOne.h"
#include "Constants.h"

#include <iostream>

class TwoPieceWithVision
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 TwoPieceWithVision> {
 public:
  TwoPieceWithVision(SwerveDrive *swerveDrive, Elevator *elevator, Shooter *shooter, Intake *intake, Vision *vision);
  
 private:
  SwerveDrive *m_swerve;
  Elevator *m_elevator;
  Shooter *m_shooter;
  Intake *m_intake;
  Vision *m_vision;
};
