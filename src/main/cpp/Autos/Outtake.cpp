// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/Outtake.h"
#include <frc2/command/WaitCommand.h>
#include "commands/RunIntake.h"
#include "commands/ResetOdometry.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Outtake::Outtake(Intake *intake, SwerveDrive *swerve) : m_intake(intake), m_swerve(swerve) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  AddCommands(
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    frc2::WaitCommand(0.25_s),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
    frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER / 2, -INTAKE_CONVEYOR_POWER / 2, 0)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(5.0_s),
      RunIntake(m_intake, -0.50, -0.50, -INTAKE_CONE_CORRECT_POWER)
    ),
      // ShootFromCarriage(m_grabber, 0.25)
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_intake->SetPistonExtension(false);
    },{m_intake})
  );
}
