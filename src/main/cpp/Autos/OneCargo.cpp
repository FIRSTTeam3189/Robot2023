// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargo.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargo::OneCargo(Elevator *elevator, Shooter *shooter)
: m_elevator(elevator), m_shooter(shooter) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  // Sends elevator to target, then runs shooter for 5 seconds
  // std::vector<std::unique_ptr<Command>> commands;

  // commands.emplace_back(std::make_unique<ElevatorPID>(m_elevator, ELEVATOR_MID_TARGET));
  // commands.emplace_back(ShootFromCarriage(m_shooter, SHOOTER_MID_SPEED).WithTimeout(5.0_s).Unwrap());

  // auto group = SequentialCommandGroup(std::move(commands));

  // AddCommands(
  //   group
  // );

  // Sends elevator to target, then runs shooter for 5 seconds
  AddCommands(
    ElevatorPID(m_elevator, ELEVATOR_MID_CUBE_TARGET),
    frc2::ParallelDeadlineGroup(frc2::WaitCommand(5.0_s), ShootFromCarriage(m_shooter, SHOOTER_MID_CUBE_SPEED))
  );
}
