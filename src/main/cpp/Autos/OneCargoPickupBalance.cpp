// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargoPickupBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargoPickupBalance::OneCargoPickupBalance(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake, bool isRedSide) 
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  // Strafe to center of charge station and then drive onto charge station
  frc::Trajectory cargoToChargeStationTrajectory{};
  if (isRedSide) {
    // std::cout << "Red\n";
    cargoToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{0.2_m * AutoConstants::TrajectoryScale, 1.25_m * AutoConstants::TrajectoryScale},
     frc::Translation2d{0.4_m * AutoConstants::TrajectoryScale, 1.5_m * AutoConstants::TrajectoryScale}}, 
    frc::Pose2d{1.25_m * AutoConstants::TrajectoryScale, 1.829_m * AutoConstants::TrajectoryScale, 0_deg},
    config);
  } 
  else {
    // std::cout << "Blue\n";
    cargoToChargeStationTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{0.2_m * AutoConstants::TrajectoryScale, -1.25_m * AutoConstants::TrajectoryScale},
     frc::Translation2d{0.4_m * AutoConstants::TrajectoryScale, -1.5_m * AutoConstants::TrajectoryScale}}, 
    frc::Pose2d{1.25_m * AutoConstants::TrajectoryScale, -1.829_m * AutoConstants::TrajectoryScale, 0_deg},
    config);
  }

  frc2::SwerveControllerCommand<4> swerveCargoToChargeCommand = m_swerve->CreateSwerveCommand(cargoToChargeStationTrajectory);

  AddCommands(
    OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
    RotateTo(m_swerve, 180.0),
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
    swerveCargoToChargeCommand,
    AutoBalance(m_swerve)
  );
}
