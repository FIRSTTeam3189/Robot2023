// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TwoCargo.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoCargo::TwoCargo(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake) 
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
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

  frc::TrajectoryConfig slowConfig{SwerveDriveConstants::kMaxSpeed / 1.5, SwerveDriveConstants::kMaxAcceleration / 1.5};
  slowConfig.SetKinematics(SwerveDriveConstants::kinematics);
  slowConfig.SetReversed(true);

  auto cargoCreepForwardTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{-5.50_m, 0.0_m, 0_deg},
    {frc::Translation2d{-5.60_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{-5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    slowConfig
  );

  config.SetReversed(false);
  std::cout << "Cargo to scoring\n";
  auto cargoToScoringTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{-5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{-2.845_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{-0.1_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  // cargoToScoringTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  frc2::SwerveControllerCommand<4> swerveCargoToScoringCommand = m_swerve->CreateSwerveCommand(cargoToScoringTrajectory);
  frc2::SwerveControllerCommand<4> swerveScoringToCargoCommand = m_swerve->CreateSwerveCommand(scoringToCargoTrajectory);
  frc2::SwerveControllerCommand<4> swerveCargoCreepForwardCommand = m_swerve->CreateSwerveCommand(cargoCreepForwardTrajectory);

  // frc::Pose2d targetPose{};

  // AddCommands(
  //   OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
  //   frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([&]{
  //       auto currentPose = m_swerve->GetPose();
  //       targetPose = currentPose.TransformBy(frc::Transform2d{frc::Translation2d{-5.69_m, 0.05_m}, frc::Rotation2d{0.0_deg}});
  //     },{m_swerve}),
  //     DriveToPose(m_swerve, targetPose, 180.0)
  //   ),
  //   frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
  //     frc2::WaitCommand(0.5_s), 
  //     ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_MID_TARGET, false, true)),
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(.25_s), 
  //     ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
  //   frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
  //   ElevatorPID(m_elevator, m_grabber, m_intake, 0, false, false),
  //   frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  // );

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
      ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET, false, true)),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(2.0_s),
      ElevatorPID(m_elevator, m_grabber, m_intake, 0, false, false)),
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
      frc2::WaitCommand(2.5_s),
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
    swerveCargoToScoringCommand,
    RotateTo(m_swerve, 180.0),
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_MID_TARGET, false, true)),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED + 0.25)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_grabber, m_intake, 0, false, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  );

  // AddCommands(
  //   OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
  //   RotateTo(m_swerve, 180.0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveCargoToScoringCommand,
  //   ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET, false),
  //   ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
  // );
}
