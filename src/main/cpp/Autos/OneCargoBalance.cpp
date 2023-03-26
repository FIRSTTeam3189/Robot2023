// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargoBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargoBalance::OneCargoBalance(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake, int elevatorTarget) 
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed, SwerveDriveConstants::kMaxAcceleration};
  config.SetKinematics(SwerveDriveConstants::kinematics);
  config.SetReversed(true);

  // std::cout << "Scoring to charge\n";
  auto scoringToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{-1.5_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{-3.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  // // scoringToChargeStationTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  frc2::SwerveControllerCommand<4> swerveScoringToChargeCommand = m_swerve->CreateSwerveCommand(scoringToChargeStationTrajectory);

  AddCommands(
    // OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);
    std::cout << "Starting one cargo\n";},{m_swerve}),
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      ElevatorPID(m_elevator, m_grabber, m_intake, elevatorTarget, false, true)),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s),
      ElevatorPID(m_elevator, m_grabber, m_intake, 0, false, false)),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake}),
    RotateTo(m_swerve, 170.0),
    swerveScoringToChargeCommand,
    AutoBalance(m_swerve)
  );

  // AddCommands(
  //   OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
  //   DriveToPose(m_swerve, frc::Transform2d{frc::Translation2d{3.5_m, 0.0_m}, frc::Rotation2d{0.0_deg}}, 0.0),
  //   AutoBalance(m_swerve)
  // );

  // AddCommands(
  //   OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
  //   RotateTo(m_swerve, 0.0),
  //   swerveScoringToChargeCommand,
  //   AutoBalance(m_swerve)
  // );

  // AddCommands(
  //   OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   RotateTo(m_swerve, 180.0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveScoringToChargeCommand,
  //   AutoBalance(m_swerve)
  // );
}
