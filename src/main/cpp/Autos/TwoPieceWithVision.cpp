// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TwoPieceWithVision.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoPieceWithVision::TwoPieceWithVision(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake, Vision *vision)
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake), m_vision(vision) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);
  config.SetReversed(false);

  std::cout << "Cargo to scoring\n";
  auto cargoToAutoAimTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{4.0_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{2.5_m * AutoConstants::TrajectoryScale, 0.0_m, 180_deg},
    config);

  auto autoAimToScoringTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m, 180_deg},
    {frc::Translation2d{0.5_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m, 180_deg},
    config);

  frc2::SwerveControllerCommand<4> swerveCargoToAutoAimCommand = m_swerve->CreateSwerveCommand(cargoToAutoAimTrajectory);
  frc2::SwerveControllerCommand<4> swerveAimToScoringCommand = m_swerve->CreateSwerveCommand(autoAimToScoringTrajectory);

  AddCommands(
    OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
    RotateTo(m_swerve, 180.0),
    swerveCargoToAutoAimCommand,
    AimAssist(m_vision, m_swerve, 1.0, 0.0, 0),
    swerveAimToScoringCommand,
    ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET, false, true),
    ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
  );

  // AddCommands(
  //   OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
  //   RotateTo(m_swerve, 180.0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveCargoToAutoAimCommand,
  //   AimAssist(m_vision, m_swerve, 1.0, 1.0, 0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveAimToScoringCommand,
  //   ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET, false),
  //   ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
  // );
}
