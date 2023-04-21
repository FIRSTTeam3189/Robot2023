// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/Outtake.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Outtake::Outtake(SwerveDrive *swerve, Intake *intake) : m_swerve(swerve), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    frc2::WaitCommand(0.25_s),
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    frc2::WaitCommand(0.25_s),
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    frc2::WaitCommand(0.25_s),
    frc2::InstantCommand([this]{m_intake->SetPistonExtension(true);},{m_intake}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(0.5_s),
      RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(10.0_s),
      RunIntake(m_intake, OUTTAKE_ROLLER_POWER, OUTTAKE_CONVEYOR_POWER)
    ),
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0);
      m_intake->SetPistonExtension(false);
    },{m_intake})
  );
}
