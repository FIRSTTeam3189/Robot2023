// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargo.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargo::OneCargo(SwerveDrive *swerve, Elevator *elevator, Grabber *grabber, Intake *intake)
: m_swerve(swerve), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  // Sends elevator to target, then runs grabber for 5 seconds
  // std::vector<std::unique_ptr<Command>> commands;

  // commands.emplace_back(std::make_unique<ElevatorPID>(m_elevator, ELEVATOR_MID_TARGET));
  // commands.emplace_back(ShootFromCarriage(m_grabber, GRABBER_MID_SPEED).WithTimeout(5.0_s).Unwrap());

  // auto group = SequentialCommandGroup(std::move(commands));

  // AddCommands(
  //   group
  // );

  // Sends elevator to target, then runs grabber for 5 seconds
  AddCommands(
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::WaitCommand(0.5_s), 
        ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_MID_TARGET, false, true)),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_grabber, m_intake, 0, false, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  );
}
