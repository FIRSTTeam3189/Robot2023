// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TestAuto.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TestAuto::TestAuto(SwerveDrive *swerveDrive, Intake *intake) 
: m_swerve(swerveDrive), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 1.5, SwerveDriveConstants::kMaxAcceleration / 1.5};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  // // std::cout << "Straight Line\n";
  // // Manually creates trajectory on RoboRIO
  // // Alternatively, import "paths" from PathWeaver as JSON files
  auto straightLineTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{0.0_m * AutoConstants::TrajectoryScale, 1.0_m}}, 
    frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 2.0_m, 0_deg},
    config);
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

  frc2::SwerveControllerCommand<4> swerveLineCommand = m_swerve->CreateSwerveCommand(straightLineTrajectory);
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
  
  AddCommands(
    swerveLineCommand
  );

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
