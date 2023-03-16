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
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);
  config.SetReversed(true);

  // std::cout << "Cargo to scoring\n";
  auto cargoToScoringTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    {frc::Translation2d{2.845_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  frc2::SwerveControllerCommand<4> swerveCargoToScoringCommand = m_swerve->CreateSwerveCommand(cargoToScoringTrajectory);

  AddCommands(
    OneCargoPickupOne(m_swerve, m_elevator, m_grabber, m_intake),
    RotateTo(m_swerve, 180.0),
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
    swerveCargoToScoringCommand,
    ElevatorPID(m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET, false),
    ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
  );
}
