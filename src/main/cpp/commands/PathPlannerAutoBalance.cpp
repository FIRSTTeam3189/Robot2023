// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PathPlannerAutoBalance.h"

PathPlannerAutoBalance::PathPlannerAutoBalance(SwerveDrive *swerveDrive)
: m_swerve(swerveDrive),
  m_xController(AutoConstants::balanceKP, AutoConstants::balanceKI, AutoConstants::balanceKD),
  m_yController(AutoConstants::balanceKP, AutoConstants::balanceKI, AutoConstants::balanceKD),
  m_rotController(AutoConstants::balanceRotKP, AutoConstants::balanceRotKI, AutoConstants::balanceRotKD),
  m_withinThresholdLoops(0) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerveDrive);
}

// Called when the command is initially scheduled.
void PathPlannerAutoBalance::Initialize() {
  // Read current estimated pose and create pathpoint from it
  frc::Pose2d currentPose = m_swerve->GetEstimatedPose();
  pathplanner::PathPoint startpoint{currentPose.Translation(), frc::Rotation2d{}};
  // Set control lengths to 0 to create a guaranteed linear path -- pro tip to avoid random curves
  startpoint = startpoint.withControlLengths(0.000001_m, 0.000001_m);

  // Get the center of the charge station (accounting for robot center of balance)
  pathplanner::PathPoint endpoint{FieldCoordinates::chargeStationCenter, frc::Rotation2d{}};
  endpoint = endpoint.withControlLengths(0.000001_m, 0.000001_m);
  
  // Generate trajectory with slow constraints and path points
  pathplanner::PathPlannerTrajectory balanceTrajectory = pathplanner::PathPlanner::generatePath(
    pathplanner::PathConstraints(1_mps, 1_mps_sq),
    startpoint, endpoint
  );
  
  // Create swerve command
  pathplanner::PPSwerveControllerCommand command{
    balanceTrajectory, // the trajectory to run
    [this] { return m_swerve->GetEstimatedPose(); }, // a function supplying the robot's current pose
    m_xController, // PID controllers to correct for robot position during the trajectory
    m_yController,
    m_rotController,
    [this](auto speeds) { m_swerve->PercentDrive(speeds); }, // a function to output chassis speeds
    {m_swerve}, // the subsystem requirements
    true // boolean -- whether or not the trajectory should be reflected for red/blue alliance (trajectory is created blue-relative)
  };
  
  command.Schedule();
}

// Called repeatedly when this Command is scheduled to run
void PathPlannerAutoBalance::Execute() {
  // Continually check for pitch and roll from pigeon
  // Consider robot balanced if both are close to level
  double pitch = m_swerve->GetPitch();
  double roll = m_swerve->GetRoll();

  if (abs(pitch) < 2.0 && abs(roll) < 2.0) {
    m_withinThresholdLoops++;
  } else {
    m_withinThresholdLoops = 0;
  }
}

// Called once the command ends or is interrupted.
void PathPlannerAutoBalance::End(bool interrupted) {}

// Returns true when the command should end.
bool PathPlannerAutoBalance::IsFinished() {
  // Ends balance command if robot is level and didn't move much since last command schedule
  if (m_withinThresholdLoops >= AutoConstants::autoBalanceSettleLoops) {
    m_swerve->LockWheels();
    return true;
  }
  return false;
}
