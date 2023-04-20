// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PIDAutoBalance.h"

PIDAutoBalance::PIDAutoBalance(SwerveDrive *swerveDrive)
: m_swerve(swerveDrive),
  m_xController(AutoConstants::gyroBalanceKP, 0.0, 0.0),
  m_yController(AutoConstants::gyroBalanceKP, 0.0, 0.0),
  m_rotController(SwerveDriveConstants::rotP, SwerveDriveConstants::rotI, SwerveDriveConstants::rotD),
  m_isReversed(false) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void PIDAutoBalance::Initialize() {
  // Robot is reversed if it is pointing within 180 degree backward range
  if (abs(m_swerve->GetNormalizedYaw()) > 90) {
    m_isReversed = true;
  } else {
    m_isReversed = false;
  }
}

// Called repeatedly when this Command is scheduled to run
void PIDAutoBalance::Execute() {
  // Uses robot relative PID and checking for whether robot is facing forward or backward
  // Pitch is negative when robot tilts up by default, so the values are inverted
  // This makes an upwards tilt drive the robot forwards with positive PID calculated values
  double xOutput = 0, yOutput = 0;
  units::radians_per_second_t rotOutput;
  // double pitch = m_isReversed ? m_swerve->GetPitch() : -m_swerve->GetPitch();
  double pitch = -m_swerve->GetPitch();
  double roll = m_isReversed ? m_swerve->GetRoll() : -m_swerve->GetRoll();
  if (m_withinThresholdLoops == 0) { // If robot is not currently balanced, drive to balance
    xOutput = m_xController.Calculate(pitch, 0.0);
    // No y output right now
    // yOutput = m_yController.Calculate(roll, 0.0);
    if (!m_isReversed) {
      rotOutput = units::radians_per_second_t{m_rotController.Calculate(m_swerve->GetNormalizedYaw(), 0.0)
                  * SwerveDriveConstants::maxAngularVelocity};
    } else {
      rotOutput = units::radians_per_second_t{m_rotController.Calculate(m_swerve->GetNormalizedYaw(), 180.0)
                  * SwerveDriveConstants::maxAngularVelocity};
    }
  } 
  // else if (m_withinThresholdLoops > 0) { // Otherwise, robot is balanced, so wait to see if it tips again (don't drive while it's balanced)
  //   xOutput = 0;
  //   yOutput = 0;
  //   rotOutput = 0.0_rad / 1.0_s;
  // }
  
  // If bridge is balanced, start counting up loops where it's balanced
  // End command after certain amount of time balanced
  if (abs(m_lastPitch) < 2.5 && abs(m_lastRoll) < 2.5) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }

  m_lastPitch = pitch;
  m_lastRoll = roll;
  m_swerve->PercentDrive(units::meters_per_second_t{xOutput}, units::meters_per_second_t{yOutput}, rotOutput, false); // Note it drives ROBOT relative, not field relative
}

// Called once the command ends or is interrupted.
void PIDAutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool PIDAutoBalance::IsFinished() {
  // Ends balance command if robot is level for a while
  if ((m_withinThresholdLoops >= AutoConstants::gyroAutoBalanceSettleLoops)) {
    // Turn wheels in x shape to lock on charge station
    m_swerve->LockWheels();
    return true;
  }
  return false;
}