// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TrajectoryAimAssist.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TrajectoryAimAssist::TrajectoryAimAssist(Vision *vision, SwerveDrive *swerve, double targetXDistance, double targetYDistance, double targetRotAngle)
: m_vision(vision), 
  m_swerve(swerve), 
  m_visionData(m_vision->GetData()),
  m_targetXDistance(targetXDistance),
  m_targetYDistance(targetYDistance),
  m_targetRotAngle(targetRotAngle) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddRequirements(m_swerve);
  AddRequirements(m_vision);

  // if (m_visionData.detectionID != DetectionType::None) {
  //   auto m_pose = m_swerve->GetPose();
  //   frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed, SwerveDriveConstants::kMaxAcceleration};
  //   config.SetKinematics(SwerveDriveParameters::kinematics);

  //   units::meter_t xDistance = units::meter_t{m_visionData.translationMatrix[0] - m_targetXDistance};
  //   units::meter_t yDistance = units::meter_t{m_visionData.translationMatrix[1] - m_targetYDistance};
  //   frc::Trajectory visionTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     m_pose,
  //     {frc::Translation2d{(m_pose.X() + xDistance) / 2, (m_pose.Y() + yDistance) / 2}}, 
  //     frc::Pose2d{m_pose.X() + xDistance, m_pose.Y() + yDistance, frc::Rotation2d{units::degree_t{m_targetRotAngle}}},
  //     config);

  //   // visionTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  //   frc2::SwerveControllerCommand<4> visionCommand = m_swerve->CreateSwerveCommand(visionTrajectory);

  //   frc::Pose2d targetPose{};

  //   AddCommands(frc2::SequentialCommandGroup(
  //     frc2::InstantCommand([&]{
  //       targetPose = m_pose.TransformBy(frc::Transform2d{frc::Translation2d{xDistance, yDistance}, frc::Rotation2d{0.0_deg}});
  //     },{m_swerve}),
  //     DriveToPose(m_swerve, targetPose, targetRotAngle)
  //     )
  //   );

    // AddCommands(visionCommand);
  // }
}
