// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/UltraShoot.h"

UltraShoot::UltraShoot(Elevator *elevator, Intake *intake, Grabber *grabber)
: m_elevator(elevator), m_intake(intake), m_grabber(grabber) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(elevator);
  AddRequirements(intake);
  AddRequirements(grabber);

  AddCommands(
    // Extend pistons
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(0.5_s),
      RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
    ),
    // Sends elevator to max height
    // During the trip up, checks if the grabber is above the release point
    // If it is, the grabber will shoot the piece while the elevator is rising to use momentum
    frc2::ParallelDeadlineGroup(
      ElevatorPID(m_elevator, ELEVATOR_ULTRA_SHOOT_TARGET, false),
      frc2::RunCommand([this]{
        if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
          m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
        }
      },{m_grabber})
    ),
    // Send the elevator back to the base and retract pistons
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s),
      ElevatorPID(m_elevator, 0, false)),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  );
}
