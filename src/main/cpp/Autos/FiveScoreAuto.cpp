// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/FiveScoreAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
FiveScoreAuto::FiveScoreAuto(SwerveDrive *swerve, pathplanner::SwerveAutoBuilder *builder, std::string filePath) : m_swerve(swerve) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  
  /* Loads PathPlanner path from project file
   * Pass in constraints to be used in path generation
   * Uses the autoBuilder made in RobotContainer to create a full autonomous routine
   * Includes event markers, stop events, and robot trajectories
  */

std::vector<pathplanner::PathPlannerTrajectory> fiveScoreGroup = pathplanner::PathPlanner::loadPathGroup(filePath, {pathplanner::PathConstraints(SwerveDriveConstants::kMaxSpeed * 0.75, SwerveDriveConstants::kMaxAcceleration * 0.75)});
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(builder->fullAuto(fiveScoreGroup).Unwrap());
  auto group = SequentialCommandGroup(std::move(commands));

  AddCommands(
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
    std::move(group)
  );
}
