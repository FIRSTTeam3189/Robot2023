// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargoBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargoBalance::OneCargoBalance(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake) 
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  // std::cout << "Scoring to charge\n";
  auto scoringToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{1.75_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{3.5_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  // scoringToChargeStationTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  frc2::SwerveControllerCommand<4> swerveScoringToChargeCommand = m_swerve->CreateSwerveCommand(scoringToChargeStationTrajectory);

  AddCommands(
    OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
    RotateTo(m_swerve, 0.0),
    swerveScoringToChargeCommand,
    AutoBalance(m_swerve)
  );

  // AddCommands(
  //   OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   RotateTo(m_swerve, 180.0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveScoringToChargeCommand,
  //   AutoBalance(m_swerve)
  // );
}
