// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneScoreTaxiCone.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneScoreTaxiCone::OneScoreTaxiCone(pathplanner::SwerveAutoBuilder *builder, std::string filePath) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  std::vector<pathplanner::PathPlannerTrajectory> oneScoreGroup = pathplanner::PathPlanner::loadPathGroup(filePath, {pathplanner::PathConstraints(SwerveDriveConstants::kMaxSpeed, SwerveDriveConstants::kMaxAcceleration)});
  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(builder->fullAuto(oneScoreGroup).Unwrap());
  auto group = SequentialCommandGroup(std::move(commands));

  AddCommands(
    std::move(group)
  );
}