// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include "commands/SimpleAutoBalance.h"

SimpleAutoBalance::SimpleAutoBalance(SwerveDrive *swerveDrive)
: m_swerve(swerveDrive) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void SimpleAutoBalance::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void SimpleAutoBalance::Execute() {
  // Auto balance command for charge station -- simple bang bang control
  // Keep driving forward/backward until balance is about level
  // If balance tips rapidly, reverse slightly and rotate bot to lock wheels
  double pitch = m_swerve->GetPitch();
  auto speed = 0.0_mps;
  auto rot = units::angular_velocity::radians_per_second_t{0.0};

  if (m_isGoingUpFirstTime)
    speed = 4.0_mps;
  // Signage depends on Pigeon mount orientation
  if (pitch > 4.0 && abs(pitch - m_lastPitch) < 0.1) {
    speed = 2.5_mps;
  } else if (pitch < -4.0 && abs(pitch - m_lastPitch) < 0.1) {
    speed = -1.5_mps;
    m_isGoingUpFirstTime = false;
  } else if (pitch - m_lastPitch < -0.1) {
    speed = -.25_mps;
    rot = units::angular_velocity::radians_per_second_t{0.25};
  } else if (pitch - m_lastPitch > 0.1) {
    speed = .25_mps;
    rot = units::angular_velocity::radians_per_second_t{0.25};
  }

  if (abs(m_lastPitch) < 2.5 && abs((double)m_swerve->GetEstimatedPose().X() - (double)m_lastXPosition) < 0.1) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }

  m_lastPitch = pitch;
  m_lastXPosition = m_swerve->GetEstimatedPose().X();
  m_swerve->PercentDrive(speed, 0.0_mps, rot, true);
}

// Called once the command ends or is interrupted.
void SimpleAutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool SimpleAutoBalance::IsFinished() {
  // Ends balance command if robot is level and didn't move much since last command schedule
  if (m_withinThresholdLoops >= AutoConstants::autoBalanceSettleLoops) {
    m_swerve->LockWheels();
    return true;
  }
  return false;
}