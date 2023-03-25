// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargoPickupBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargoPickupBalance::OneCargoPickupBalance(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake, bool isRedSide) 
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake), m_isRedSide(isRedSide) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  // frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  // config.SetKinematics(SwerveDriveConstants::kinematics);

  // // Strafe to center of charge station and then drive onto charge station
  // frc::Trajectory cargoToChargeStationTrajectory{};
  // if (isRedSide) {
  //   // std::cout << "Red\n";
  //   cargoToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 180_deg},
  //   {frc::Translation2d{5.49_m * AutoConstants::TrajectoryScale, -1.25_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{5.29_m * AutoConstants::TrajectoryScale, -1.5_m * AutoConstants::TrajectoryScale}}, 
  //   frc::Pose2d{4.44_m * AutoConstants::TrajectoryScale, -1.829_m * AutoConstants::TrajectoryScale, 180_deg},
  //   config);
  // } 
  // else {
  //   // std::cout << "Blue\n";
  //   cargoToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 180_deg},
  //   {frc::Translation2d{5.49_m * AutoConstants::TrajectoryScale, 1.25_m * AutoConstants::TrajectoryScale},
  //    frc::Translation2d{5.29_m * AutoConstants::TrajectoryScale, 1.5_m * AutoConstants::TrajectoryScale}}, 
  //   frc::Pose2d{4.44_m * AutoConstants::TrajectoryScale, 1.829_m * AutoConstants::TrajectoryScale, 180_deg},
  //   config);
  // }

  // // cargoToChargeStationTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  // frc2::SwerveControllerCommand<4> swerveCargoToChargeCommand = m_swerve->CreateSwerveCommand(cargoToChargeStationTrajectory);

  // AddCommands(
  //   OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
  //   *DriveToPose(m_swerve, frc::Transform2d{frc::Translation2d{0.0_m, -1.829_m}, frc::Rotation2d{0.0_deg}}, 180.0).Unless([this]{return !m_isRedSide;}).Unwrap().release(),
  //   *DriveToPose(m_swerve, frc::Transform2d{frc::Translation2d{0.0_m, 1.829_m}, frc::Rotation2d{0.0_deg}}, 180.0).Unless([this]{return m_isRedSide;}).Unwrap().release(),
  //   DriveToPose(m_swerve, frc::Transform2d{frc::Translation2d{-1.25_m, 0.0_m}, frc::Rotation2d{0.0_deg}}, 0.0),
  //   AutoBalance(m_swerve)
  // );

  // AddCommands(
  //   OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
  //   RotateTo(m_swerve, 180.0),
  //   swerveCargoToChargeCommand,
  //   AutoBalance(m_swerve)
  // );

  // AddCommands(
  //   OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
  //   RotateTo(m_swerve, 180.0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
  //   swerveCargoToChargeCommand,
  //   AutoBalance(m_swerve)
  // );
}
