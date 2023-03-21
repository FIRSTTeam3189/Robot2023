// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalance.h"

AutoBalance::AutoBalance(SwerveDrive *swerveDrive)
: m_swerve(swerveDrive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void AutoBalance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute() {
  // Auto balance command for charge station -- simple bang bang control
  // Keep driving forward/backward until balance is about level
  // If balance tips rapidly, reverse slightly and rotate bot to lock wheels
  double pitch = m_swerve->GetPitch();
  auto speed = 0.0_mps;
  auto rot = units::angular_velocity::radians_per_second_t{0.0};

  // Signage depends on Pigeon mount orientation
  if (pitch > 4.0) {
    speed = 3.5_mps;
  } else if (pitch < -4.0) {
    speed = -3.0_mps;
  }
 
  if (pitch - m_lastPitch < -0.1) {
    speed = -.75_mps;
    rot = units::angular_velocity::radians_per_second_t{0.25};
  } else if (pitch - m_lastPitch > 0.1) {
    speed = .75_mps;
    rot = units::angular_velocity::radians_per_second_t{0.25};
  }

  if (abs(m_lastPitch) < 2.5 && abs((double)m_swerve->m_odometry.GetPose().X() - (double)m_lastXPosition) < 0.001) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }

  m_lastPitch = pitch;
  m_lastXPosition = m_swerve->m_odometry.GetPose().X();
  m_swerve->Drive(speed, 0.0_mps, rot, true);
}

// Called once the command ends or is interrupted.
void AutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  // Ends balance command if robot is level and didn't move much since last command schedule
  return (m_withinThresholdLoops >= 25);
}
