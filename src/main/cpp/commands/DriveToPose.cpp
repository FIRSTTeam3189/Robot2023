// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveToPose.h"
#include <iostream>

DriveToPose::DriveToPose(SwerveDrive *swerve, frc::Transform2d targetTransform, double targetAngle)
: m_swerve(swerve), 
  m_transform(targetTransform),
  m_targetAngle(targetAngle),
  m_currentPose(m_swerve->GetPose()),
  m_targetPose(m_currentPose.TransformBy(m_transform)),
  m_xPIDController(AutoConstants::kPXController, 0.0, 0.0),
  m_yPIDController(AutoConstants::kPYController, 0.0, 0.0),
  m_rotPIDController(0.15, 0.0, 0.0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve);
  std::cout << "Constructing drive to pose\n";
  m_rotPIDController.EnableContinuousInput(0, 360);
}

// Called when the command is initially scheduled.
void DriveToPose::Initialize() {
  std::cout << "Initializing drive to pose\n";
  m_currentPose = m_swerve->GetPose();
  m_targetPose = m_currentPose.TransformBy(m_transform);
}

// Called repeatedly when this Command is scheduled to run
void DriveToPose::Execute() {
  std::cout << "Executing drive to pose\n";
  m_currentPose = m_swerve->GetPose();

  auto xOutput = units::meters_per_second_t{m_xPIDController.Calculate(m_currentPose.X().value(), m_targetPose.X().value()) * (double)SwerveDriveConstants::kMaxSpeed};
  auto yOutput = units::meters_per_second_t{m_yPIDController.Calculate(m_currentPose.Y().value(), m_targetPose.Y().value()) * (double)SwerveDriveConstants::kMaxSpeed};
  auto rotOutput = units::radians_per_second_t{m_rotPIDController.Calculate(m_swerve->GetNormalizedYaw(), m_targetAngle) * (double)SwerveDriveConstants::maxAngularVelocity};

  m_swerve->Drive(xOutput, yOutput, rotOutput, true);
  std::cout << "Drive to pose execute loop done\n";
}

// Called once the command ends or is interrupted.
void DriveToPose::End(bool interrupted) {}

// Returns true when the command should end.
bool DriveToPose::IsFinished() {
  if (abs(m_currentPose.X().value() - m_targetPose.X().value()) < 0.05 &&
      abs(m_currentPose.Y().value() - m_targetPose.Y().value()) < 0.05 &&
      abs(m_swerve->GetNormalizedYaw() - m_targetAngle) < 5.0) {
      m_swerve->Drive(0_mps, 0_mps, 0_rad / 1_s, true);
      return true;
  }
  return false;
}
