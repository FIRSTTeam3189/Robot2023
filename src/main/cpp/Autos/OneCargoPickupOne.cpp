// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/OneCargoPickupOne.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
OneCargoPickupOne::OneCargoPickupOne(SwerveDrive *swerveDrive, Elevator *elevator, Grabber *grabber, Intake *intake) 
: m_swerve(swerveDrive), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  // std::cout << "Scoring to cargo\n";
  auto scoringToCargoTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{2.845_m * AutoConstants::TrajectoryScale, 0.0_m}}, 
    frc::Pose2d{5.69_m * AutoConstants::TrajectoryScale, 0.0_m, 0_deg},
    config);

  // scoringToCargoTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  frc2::SwerveControllerCommand<4> swerveScoringToCargoCommand = m_swerve->CreateSwerveCommand(scoringToCargoTrajectory);

  // AddCommands(
  //   OneCargo(m_elevator, m_grabber, m_intake),
  //   ElevatorPID(m_elevator, m_grabber, m_intake, 0, false),
  //   RotateTo(m_swerve, 180.0),
  //   swerveScoringToCargoCommand,
  //   ToggleIntakePistons(m_intake),
  //   frc2::ParallelDeadlineGroup(frc2::WaitCommand(5.0_s), RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)),
  //   ToggleIntakePistons(m_intake)
  // );

  AddCommands(
    // Consider parallelizimationing the elevator pid and swerve driving
    OneCargo(m_swerve, m_elevator, m_grabber, m_intake),
    ElevatorPID(m_elevator, m_grabber, m_intake, 0, false),
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
    RotateTo(m_swerve, 180.0),
    ResetOdometry(m_swerve, frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}}),
    // Consider paralleliziamationsingiasd the intake and drivetrain
    swerveScoringToCargoCommand,
    ToggleIntakePistons(m_intake),
    frc2::ParallelDeadlineGroup(frc2::WaitCommand(5.0_s), RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)),
    ToggleIntakePistons(m_intake)
  );
}
