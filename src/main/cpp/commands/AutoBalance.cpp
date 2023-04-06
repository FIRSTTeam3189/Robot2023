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
  // Uses robot relative PID and checking for whether robot is forward or backward
  double pitch = m_isReversed ? m_swerve->GetPitch() : -m_swerve->GetPitch();
  double roll = m_isReversed ? m_swerve->GetRoll() : -m_swerve->GetRoll();
  auto xOutput = m_xController.Calculate(pitch, 0.0);
  auto yOutput = m_yController.Calculate(roll, 0.0);

  // If bridge is balanced, start counting up loops where it's balanced
  // End command after certain amount of time balanced
  if (abs(m_lastPitch) < 2.5 && abs(m_lastRoll) < 2.5) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }

  m_lastPitch = pitch;
  m_lastRoll = roll;
  m_swerve->PercentDrive(units::meters_per_second_t{xOutput}, units::meters_per_second_t{yOutput}, 0.0 * 1_rad / 1_s, false); // Note it drives ROBOT relative, not field relative
}

// Called once the command ends or is interrupted.
void AutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool AutoBalance::IsFinished() {
  // Ends balance command if robot is level and didn't move much since last command schedule
  if (m_withinThresholdLoops >= AutoConstants::autoBalanceSettleLoops) {
    // Turn wheels in x shape to lock on charge station
    m_swerve->LockWheels();
    return true;
  }
  return false;
}
