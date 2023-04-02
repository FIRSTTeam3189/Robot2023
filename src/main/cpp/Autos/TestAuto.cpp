// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TestAuto.h"
#include <iostream>

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TestAuto::TestAuto(SwerveDrive *swerveDrive, Elevator *elevator, Intake *intake, Grabber *grabber, int testNum) 
: m_swerve(swerveDrive), m_elevator(elevator), m_intake(intake), m_grabber(grabber) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});

  switch(testNum) {

    // Straight line
    case 1:
      {
      frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 1.0, SwerveDriveConstants::kMaxAcceleration / 1.0};
      config.SetKinematics(SwerveDriveConstants::kinematics);
      config.SetReversed(false);

      // // std::cout << "Straight Line\n";
      // // Manually creates trajectory on RoboRIO
      // // Alternatively, import "paths" from PathWeaver as JSON files
      auto straightLineTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{0.0_m, 0.0_m, 0_deg},
        {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m}},
        frc::Pose2d{2.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
        config);

      frc2::SwerveControllerCommand<4> swerveLineCommand = m_swerve->CreateSwerveCommand(straightLineTrajectory);

      AddCommands(
        swerveLineCommand
      );
      }
      break;

    // S Shape with auto rotation
    case 2:
      {
      frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 1.5, SwerveDriveConstants::kMaxAcceleration / 1.5};
      config.SetKinematics(SwerveDriveConstants::kinematics);
      config.SetReversed(false);

      std::cout << "Test +rotate\n";
      auto straightLineTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{0.0_m, 0.0_m, 0_deg},
        {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m},
         frc::Translation2d{1.0_m, -1.0_m}}, 
        frc::Pose2d{2.0_m * AutoConstants::TrajectoryScale, -1.0_m, 180_deg},
        config);
      std::cout << "Test +rotate\n";

      auto swerveLineCommand = m_swerve->CreateSwerveCommand(straightLineTrajectory);

      AddCommands(
        swerveLineCommand
      );
      }
      break;

    // Figure eight
    case 3:
      {
      frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 1.5, SwerveDriveConstants::kMaxAcceleration / 1.5};
      config.SetKinematics(SwerveDriveConstants::kinematics);
      config.SetReversed(false);

      auto figureEightTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{0.0_m, 0.0_m, 0_deg},
        {frc::Translation2d{0.15_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{0.5_m * AutoConstants::TrajectoryScale, -0.5_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{0.85_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{0.85_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{0.5_m * AutoConstants::TrajectoryScale, 0.5_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{0.15_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-0.15_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-0.5_m * AutoConstants::TrajectoryScale, -0.5_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-0.85_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-1.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-0.85_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-0.5_m * AutoConstants::TrajectoryScale, 0.5_m * AutoConstants::TrajectoryScale},
        frc::Translation2d{-0.15_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},},
        frc::Pose2d{0.0_m, 0.0_m, 0_deg},
        config);

      auto figureEightCommand = m_swerve->CreateSwerveCommand(figureEightTrajectory);
      AddCommands(
        frc2::RepeatCommand(figureEightCommand)
      );
      }
      break;

    // Move forward while intaking
    case 4:
      {
        frc::TrajectoryConfig slowConfig{SwerveDriveConstants::kMaxSpeed / 1.5, SwerveDriveConstants::kMaxAcceleration / 1.5};
        slowConfig.SetKinematics(SwerveDriveConstants::kinematics);
        slowConfig.SetReversed(false);

        auto cargoCreepForwardTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d{0.0_m, 0.0_m, 0_deg},
          {frc::Translation2d{0.2_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
          frc::Pose2d{0.4_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
          slowConfig
        );

        frc2::SwerveControllerCommand<4> swerveCargoCreepForwardCommand = m_swerve->CreateSwerveCommand(cargoCreepForwardTrajectory);

        AddCommands(
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
        frc2::InstantCommand([this]{m_intake->SetPower(0, 0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber})
        );
      }
      break;
    
    // Rotate, move forward, rotate
    case 5:
      {
      frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 1.5, SwerveDriveConstants::kMaxAcceleration / 1.5};
      config.SetKinematics(SwerveDriveConstants::kinematics);
      config.SetReversed(false);

      // // std::cout << "Straight Line\n";
      // // Manually creates trajectory on RoboRIO
      // // Alternatively, import "paths" from PathWeaver as JSON files
      auto forwardTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{0.0_m, 0.0_m, 0_deg},
        {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
        frc::Pose2d{2.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
        config);

      
      auto backwardsTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
        frc::Pose2d{2.0_m, 0.0_m, 180_deg},
        {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
        frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m, 180_deg},
        config);


      frc2::SwerveControllerCommand<4> swerveForwardCommand = m_swerve->CreateSwerveCommand(forwardTrajectory);
      frc2::SwerveControllerCommand<4> swerveBackwardCommand = m_swerve->CreateSwerveCommand(backwardsTrajectory);

      AddCommands(
        ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
        ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
        ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
        frc2::WaitCommand(0.25_s),
        RotateTo(m_swerve, 0.0),
        swerveForwardCommand,
        RotateTo(m_swerve, 180.0),
        swerveBackwardCommand
      );
      }
      break;

    // Move forward and left (around charge station), rotate 45 deg counter-clockwise, intake
    case 6:
      {
        frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 1.0, SwerveDriveConstants::kMaxAcceleration / 1.0};
        config.SetKinematics(SwerveDriveConstants::kinematics);
        config.SetReversed(false);

        auto scoringToSecondCargoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d{0.0_m, 0.0_m, 0_deg},
          {frc::Translation2d{2.5_m, 0.0_m},
          frc::Translation2d{3.5_m, 0.0_m}}, 
          frc::Pose2d{4.5_m, 0.0_m, 0_deg},
          config
        );

        frc::TrajectoryConfig slowConfig{SwerveDriveConstants::kMaxSpeed / 3.0, SwerveDriveConstants::kMaxAcceleration / 3.0};
        slowConfig.SetKinematics(SwerveDriveConstants::kinematics);
        slowConfig.SetReversed(false);

        auto cargoCreepForwardTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d{0.0_m, 0.0_m, 0_deg},
          {frc::Translation2d{0.5_m, 0.0_m}}, 
          frc::Pose2d{1.5_m, 0.0_m, 0_deg},
          slowConfig
        );

        m_swerve->SetActiveTrajectory(scoringToSecondCargoTrajectory);
        frc2::SwerveControllerCommand<4> swerveCargoCreepForwardCommand = m_swerve->CreateSwerveCommand(cargoCreepForwardTrajectory);
        frc2::SwerveControllerCommand<4> swerveScoringToCargo2Command = m_swerve->CreateSwerveCommand(scoringToSecondCargoTrajectory);

        AddCommands(
          ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
          ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
          ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
          frc2::WaitCommand(0.125_s),
          swerveScoringToCargo2Command,
          RotateTo(m_swerve, -60.0),
          ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
          ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
          ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}), 
          frc2::WaitCommand(0.125_s),
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
        frc2::InstantCommand([this]{m_intake->SetPower(0, 0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber})
        );
      }
      break;

    // S-shape without rotation
    case 7:
      {
        frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2.0, SwerveDriveConstants::kMaxAcceleration / 2.0};
        config.SetKinematics(SwerveDriveConstants::kinematics);
        config.SetReversed(false);

        auto sTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d{0.0_m, 0.0_m, 0_deg},
          {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m},
           frc::Translation2d{1.0_m, -1.0_m}},
          frc::Pose2d{2.0_m * AutoConstants::TrajectoryScale, -1.0_m, 0_deg},
          config);

        frc2::SwerveControllerCommand<4> swerveSCommand = m_swerve->CreateSwerveCommand(sTrajectory);

        AddCommands(
          swerveSCommand
        );
      }
      break;

    // S-shape with rotation
    case 8:
      {
        frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2.0, SwerveDriveConstants::kMaxAcceleration / 2.0};
        config.SetKinematics(SwerveDriveConstants::kinematics);
        config.SetReversed(false);

        auto sTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
          frc::Pose2d{0.0_m, 0.0_m, 0_deg},
          {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m},
           frc::Translation2d{1.0_m, -1.0_m}},
          frc::Pose2d{2.0_m * AutoConstants::TrajectoryScale, -1.0_m, 180_deg},
          config);

        frc2::SwerveControllerCommand<4> swerveCommand = m_swerve->CreateSwerveCommand(sTrajectory);

        AddCommands(
          swerveCommand
        );
      }

    default:
      break;
  }

  // config.SetReversed(true);
  // auto straightLineTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {frc::Translation2d{-1.0_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
  //   frc::Pose2d{-2.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
  //   config);
    
  // // std::cout << "S Shape\n";
  // auto sShapeTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   // "Clamped cubic" trajectory -- robot poses at endpoints, only positions for interior waypoints
  //   // Start position at (0, 0) facing positive X-axis
  //   frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
  //   // Pass through 2 interior points to make sideways "S"
  //   {frc::Translation2d{-1.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale}, 
  //   frc::Translation2d{-1.0_m * AutoConstants::TrajectoryScale, -1.0_m * AutoConstants::TrajectoryScale}},
  //   frc::Pose2d{-2.0_m * AutoConstants::TrajectoryScale, -1.0_m * AutoConstants::TrajectoryScale, 180_deg},
  //   config);

  // // std::cout << "Special\n";
  // auto specialTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {
  //     frc::Translation2d{-0.4_m * AutoConstants::TrajectoryScale, 0.1_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{-0.6_m * AutoConstants::TrajectoryScale, 0.3_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{-0.6_m * AutoConstants::TrajectoryScale, 0.6_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{-0.4_m * AutoConstants::TrajectoryScale, 0.8_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{0.0_m * AutoConstants::TrajectoryScale, 0.8_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{0.4_m * AutoConstants::TrajectoryScale, 0.4_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{1.8_m * AutoConstants::TrajectoryScale, 0.25_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{2.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{1.8_m * AutoConstants::TrajectoryScale, -0.25_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{0.4_m * AutoConstants::TrajectoryScale, -0.4_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{0.0_m * AutoConstants::TrajectoryScale, -0.8_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{-0.3_m * AutoConstants::TrajectoryScale, -0.8_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{-0.6_m * AutoConstants::TrajectoryScale, -0.6_m * AutoConstants::TrajectoryScale}, 
  //     frc::Translation2d{-0.6_m * AutoConstants::TrajectoryScale, -0.3_m * AutoConstants::TrajectoryScale},
  //     frc::Translation2d{-0.4_m * AutoConstants::TrajectoryScale, -0.1_m * AutoConstants::TrajectoryScale}
  //   },
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   config);

  // auto squareTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m},
  //    frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 1.0_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.0_m, 1.0_m * AutoConstants::TrajectoryScale}}, 
  //   frc::Pose2d{0.0_m, 0.0_m, 45_deg},
  //   config);

  // // config.SetEndVelocity(0.1_mps);
  // // config.SetStartVelocity(0.1_mps);

  // auto figureEightTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {frc::Translation2d{-0.15_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{-0.5_m * AutoConstants::TrajectoryScale, -0.5_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{-0.85_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{-1.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{-0.85_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{-0.5_m * AutoConstants::TrajectoryScale, 0.5_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{-0.15_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.15_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.5_m * AutoConstants::TrajectoryScale, -0.5_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.85_m * AutoConstants::TrajectoryScale, -0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.85_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.5_m * AutoConstants::TrajectoryScale, 0.5_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{0.15_m * AutoConstants::TrajectoryScale, 0.35_m * AutoConstants::TrajectoryScale},},
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   config);

  // auto line2Trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {frc::Translation2d{1.0_m * AutoConstants::TrajectoryScale, 0.0_m}},
  //   frc::Pose2d{2.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
  //   config);

  // frc2::SwerveControllerCommand<4> swerveSCommand = m_swerve->CreateSwerveCommand(sShapeTrajectory);
  // frc2::SwerveControllerCommand<4> specialCommand = m_swerve->CreateSwerveCommand(specialTrajectory);
  // frc2::SwerveControllerCommand<4> squareCommand = m_swerve->CreateSwerveCommand(squareTrajectory);
  // frc2::SwerveControllerCommand<4> figureEightCommand = m_swerve->CreateSwerveCommand(figureEightTrajectory);
  // frc2::SwerveControllerCommand<4> line2Command = m_swerve->CreateSwerveCommand(line2Trajectory);

  // frc::Pose2d targetPose{};

  // AddCommands(
  //   frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([&]{
  //       m_swerve->SetRobotYaw(0.0);
  //       auto currentPose = m_swerve->GetPose();
  //       targetPose = currentPose.TransformBy(frc::Transform2d{frc::Translation2d{1.0_m, 0.0_m}, frc::Rotation2d{0.0_deg}});
  //     },{m_swerve}),
  //     DriveToPose(m_swerve, targetPose, 0.0)
  //   )
  // );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d(0.0_m, 0.0_m, 0_deg)),
  //   frc2::ParallelDeadlineGroup(frc2::WaitCommand(2.5_s), RunIntake(m_intake, INTAKE_ROLLER_POWER - 0.25, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)),
  //   RotateTo(m_swerve, 180),
  //   Balance(m_swerve)
  // );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d(0.0_m, 0.0_m, 0_deg)),
  //   frc2::ParallelDeadlineGroup(frc2::WaitCommand(10.0_s), RunIntake(m_intake, INTAKE_ROLLER_POWER - 0.25, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER))
    // RotateTo(m_swerve, 180),
    // ResetOdometry(m_swerve, frc::Pose2d(0.0_m, 0.0_m, 0_deg))
  //   // ResetOdometry(m_swerve, frc::Pose2d(0.0_m, 0.0_m, 0_deg))
    // swerveLineCommand
  // //   AutoBalance(m_swerve)
  // );

  // AddCommands(
  //   Balance(m_swerve)
  // );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), 
  //   specialCommand,
  //   ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg})
  // );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //   frc2::RepeatCommand(squareCommand)
  // );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //   frc2::RepeatCommand(figureEightCommand)
  // );

  // AddCommands(
  //   ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //   RotateTo(m_swerve, 180.0),
  //   line2Command,
  //   RotateTo(m_swerve, 0.0)
  // );
}
