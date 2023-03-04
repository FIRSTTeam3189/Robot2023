// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargoBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargoBalance::OneCargoBalance(SwerveDrive *swerveDrive, Elevator *elevator, Shooter *shooter) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  // Use 179.9 degrees because if it's perfectly 180->0
  // Then it can't figure out which way to turn and throws an error
  // Since clockwise and counterclockwise are equally efficient
  // std::cout << "Scoring to charge\n";
  auto scoringToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{1.75_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{3.5_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  frc2::SwerveControllerCommand<4> swerveScoringToChargeCommand = m_swerve->CreateSwerveCommand(scoringToChargeStationTrajectory);

  AddCommands(
    OneCargo(m_elevator, m_shooter),
    ResetOdometry(m_swerve, scoringToChargeStationTrajectory.InitialPose()),
    RotateTo(m_swerve, 180.0),
    swerveScoringToChargeCommand,
    AutoBalance(m_swerve)
  );
}