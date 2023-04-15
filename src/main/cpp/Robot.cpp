// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/Filesystem.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <wpi/fs.h>

void Robot::RobotInit() {
  m_container.ResetGyroscope();
  // cs::UsbCamera frontCamera = frc::CameraServer::StartAutomaticCapture(0);
  // cs::UsbCamera interiorCamera = frc::CameraServer::StartAutomaticCapture(1);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 * wgrated updating.
 */
void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
  m_wheelBrakeTimer.Reset();
  m_wheelBrakeTimer.Start();
}

void Robot::DisabledPeriodic() {
  m_container.Sync();
  // Set swerve to coast mode 15 seconds after disabled for easier moving
  if (m_wheelBrakeTimer.HasElapsed(15.0_s)) {
    m_container.SetSwerveCoast();
    m_wheelBrakeTimer.Stop();
  }
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
  // Set swerve wheels to brake again once enabled
  m_container.SetSwerveBrake();
  m_autonomousCommand = m_container.GetAutonomousCommand();
  m_container.ResetGyroscope();

  // Make sure auto exists
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {
  m_container.Sync();
}

void Robot::TeleopInit() {
  // Set swerve wheels to brake again once enabled
  m_container.SetSwerveBrake();
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  // m_container.SetSwerveYaw(m_container.GetStoredYaw());
  if (m_autonomousCommand != nullptr) {
    m_autonomousCommand->Cancel();
    m_autonomousCommand = nullptr;
  }
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
  m_container.Sync();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
}

/**
 * This function is called once when the robot is first started up.
 */
void Robot::SimulationInit() {}

/**
 * This function is called periodically whilst in simulation.
 */
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  // std::cout << "About to start robot\n";
  return frc::StartRobot<Robot>();
}
#endif
