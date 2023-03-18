// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/Balance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
Balance::Balance(SwerveDrive *swerve) : m_swerve(swerve) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed, SwerveDriveConstants::kMaxAcceleration};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  auto scoringToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{1.75_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{3.5_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  // scoringToChargeStationTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  frc2::SwerveControllerCommand<4> swerveScoringToChargeCommand = m_swerve->CreateSwerveCommand(scoringToChargeStationTrajectory);

  AddCommands(
    swerveScoringToChargeCommand,
    AutoBalance(m_swerve)
  );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveScoringToChargeCommand,
  //   AutoBalance(m_swerve)
  // );
}
