// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TwoCargoUltrashootBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoCargoUltrashootBalance::TwoCargoUltrashootBalance(SwerveDrive *swerve, Elevator *elevator, Grabber *grabber, Intake *intake)
: m_swerve(swerve), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    OneCargoPickupBalance(m_swerve, m_elevator, m_grabber, m_intake, true),
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
    frc2::ParallelDeadlineGroup(
      ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_ULTRA_SHOOT_TARGET, false, true)),
      frc2::RunCommand([this]{
        if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
          m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
        }},{m_elevator, m_grabber})
    ),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_grabber, m_intake, 0, false, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  );
}
