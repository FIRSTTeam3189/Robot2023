// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/UltraShoot.h"

UltraShoot::UltraShoot(Elevator *elevator, Intake *intake, Grabber *grabber)
: m_elevator(elevator), m_intake(intake), m_grabber(grabber) {
  // Use addRequirements() here to declare subsystem dependencies.
  m_intake->SetPistonExtension(true);
  // m_elevator->SetPID(ELEVATOR_ULTRA_SHOOT_P, ELEVATOR_I, ELEVATOR_D);
  AddRequirements(elevator);
  AddRequirements(intake);
  AddRequirements(grabber);

  AddCommands(
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(0.5_s),
      RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
    ),
    frc2::ParallelDeadlineGroup(
      ElevatorPID(m_elevator, m_intake, ELEVATOR_ULTRA_SHOOT_TARGET, false),
      frc2::RunCommand([this]{
        if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
          m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
        }
      },{m_elevator, m_elevator})
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s),
      ElevatorPID(m_elevator, m_intake, 0, false)),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  );
}
