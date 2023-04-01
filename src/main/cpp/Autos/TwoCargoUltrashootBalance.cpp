// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TwoCargoUltrashootBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoCargoUltrashootBalance::TwoCargoUltrashootBalance(SwerveDrive *swerve, Elevator *elevator, Grabber *grabber, Intake *intake, bool isRedSide)
: m_swerve(swerve), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed, SwerveDriveConstants::kMaxAcceleration};
  config.SetKinematics(SwerveDriveConstants::kinematics);
  config.SetReversed(true);

  auto scoringToCargoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{-2.845_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{-5.50_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  frc::TrajectoryConfig slowConfig{SwerveDriveConstants::kMaxSpeed / 2.5, SwerveDriveConstants::kMaxAcceleration / 2.5};
  slowConfig.SetKinematics(SwerveDriveConstants::kinematics);
  slowConfig.SetReversed(true);

  auto cargoCreepForwardTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{-5.50_m, 0.0_m, 0_deg},
    {frc::Translation2d{-5.60_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{-5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    slowConfig
  );

  config.SetReversed(false);
  frc::Trajectory cargoToChargeStationTrajectory{};
  if (isRedSide) {
    // std::cout << "Red\n";
    cargoToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{-5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{-5.49_m * AutoConstants::TrajectoryScale, -1.25_m * AutoConstants::TrajectoryScale},
     frc::Translation2d{-5.29_m * AutoConstants::TrajectoryScale, -1.5_m * AutoConstants::TrajectoryScale}}, 
    frc::Pose2d{-4.44_m * AutoConstants::TrajectoryScale, -1.829_m * AutoConstants::TrajectoryScale, 0_deg},
    config);
  } 
  else {
    // std::cout << "Blue\n";
    cargoToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{-5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{-5.49_m * AutoConstants::TrajectoryScale, 1.25_m * AutoConstants::TrajectoryScale},
     frc::Translation2d{-5.29_m * AutoConstants::TrajectoryScale, 1.5_m * AutoConstants::TrajectoryScale}}, 
    frc::Pose2d{-4.44_m * AutoConstants::TrajectoryScale, 1.829_m * AutoConstants::TrajectoryScale, 0_deg},
    config);
  }

  frc2::SwerveControllerCommand<4> swerveCargoToChargeCommand = m_swerve->CreateSwerveCommand(cargoToChargeStationTrajectory);
  frc2::SwerveControllerCommand<4> swerveScoringToCargoCommand = m_swerve->CreateSwerveCommand(scoringToCargoTrajectory);
  frc2::SwerveControllerCommand<4> swerveCargoCreepForwardCommand = m_swerve->CreateSwerveCommand(cargoCreepForwardTrajectory);

  AddCommands(
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
    frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);
    std::cout << "Starting one cargo\n";},{m_swerve}),
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      ElevatorPID(m_elevator, m_intake, ELEVATOR_HIGH_TARGET, false)),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s),
      ElevatorPID(m_elevator, m_intake, 0, false)),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake}),
    RotateTo(m_swerve, 0.0),
    swerveScoringToCargoCommand,
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{
        m_intake->SetPistonExtension(true);
      },{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      )
    ),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.5_s),
      RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER),
      swerveCargoCreepForwardCommand
    ),
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_swerve->Drive(0_mps, 0_mps, 0_rad / 1_s, true);
      m_intake->SetPistonExtension(false);
    },{m_intake}),
    frc2::WaitCommand(0.5_s),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s), 
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER, 0),
      ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)
    ),
    frc2::InstantCommand([this]{m_intake->SetPower(0, 0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber}),
    swerveCargoToChargeCommand,
    AutoBalance(m_swerve),
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
    frc2::ParallelDeadlineGroup(
      ElevatorPID(m_elevator, m_intake, ELEVATOR_ULTRA_SHOOT_TARGET, false)),
      frc2::RunCommand([this]{
        if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
          m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
        }},{m_elevator, m_grabber})
    ),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  );
}
