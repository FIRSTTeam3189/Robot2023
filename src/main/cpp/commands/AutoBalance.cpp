// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoBalance.h"

AutoBalance::AutoBalance(SwerveDrive *swerveDrive)
: m_swerve(swerveDrive),
  m_isGoingUpFirstTime(true),
  m_xController(0.1, 0.0, 0.0),
  m_yController(0.1, 0.0, 0.0),
  m_isReversed(false) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void AutoBalance::Initialize() {
  // Robot is reversed if is pointing within 180 degree backward range
  if (abs(m_swerve->GetNormalizedYaw()) > 90) {
    m_isReversed = true;
  }
}

// Called repeatedly when this Command is scheduled to run
void AutoBalance::Execute() {
  // Auto balance command for charge station -- simple bang bang control
  // Keep driving forward/backward until balance is about level
  // If balance tips rapidly, reverse slightly and rotate bot to lock wheels
  auto speed = 0.0_mps;
  auto rot = units::angular_velocity::radians_per_second_t{0.0};

  // Uses field relative PID and checking for whether robot is forward or backward instead
  double pitch = m_isReversed ? m_swerve->GetPitch() : -m_swerve->GetPitch();
  double roll = m_isReversed ? m_swerve->GetRoll() : -m_swerve->GetRoll();
  auto xOutput = m_xController.Calculate(pitch, 0.0);
  auto yOutput = m_yController.Calculate(roll, 0.0);

  // if (m_isGoingUpFirstTime)
  //   speed = 6.0_mps;
  // // Signage depends on Pigeon mount orientation
  // if (pitch > 4.0) {
  //   speed = 2.5_mps;
  // } else if (pitch < -4.0) {
  //   speed = -1.5_mps;
  //   m_isGoingUpFirstTime = false;
  // }
 
  // if (pitch - m_lastPitch < -0.1) {
  //   speed = -.25_mps;
  //   rot = units::angular_velocity::radians_per_second_t{0.25};
  // } else if (pitch - m_lastPitch > 0.1) {
  //   speed = .25_mps;
  //   rot = units::angular_velocity::radians_per_second_t{0.25};
  // }

  if (abs(m_lastPitch) < 2.5 && abs((double)m_swerve->m_odometry.GetPose().X() - (double)m_lastXPosition) < 0.1) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }

  m_lastPitch = pitch;
  m_lastXPosition = m_swerve->m_odometry.GetPose().X();
  // m_swerve->PercentDrive(speed, 0.0_mps, rot, true);
  m_swerve->PercentDrive(units::meters_per_second_t{xOutput}, units::meters_per_second_t{yOutput}, 0.0 * 1_rad / 1_s, false);
}

// Called once the command ends or is interrupted.
void AutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  // Ends balance command if robot is level and didn't move much since last command schedule
  if (m_withinThresholdLoops >= AutoConstants::autoBalanceSettleLoops) {
    m_swerve->LockWheels();
    return true;
  }
  return false;
}
