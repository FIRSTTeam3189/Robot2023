// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/RamseteCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <iostream>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // std::cout << "Robot container constructing\n";
  // Consider resetting Pigeon 2.0 gyroscope values - look up API
  // Configure the button bindings
  ConfigureButtonBindings();
  // Joystick operated - real control scheme
  m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, false));

  m_chooser.SetDefaultOption("Default Test Auto", &m_TestAuto);
  m_chooser.AddOption("Test Auto", &m_TestAuto);
  m_chooser.AddOption("One Cargo", &m_oneCargo);
  m_chooser.AddOption("One Cargo Balance", &m_oneCargoBalance);
  m_chooser.AddOption("One Cargo Pickup + Balance Red Side", &m_oneCargoPickupBalanceRed);
  m_chooser.AddOption("One Cargo Pickup + Balance Blue Side", &m_oneCargoPickupBalanceBlue);
  m_chooser.AddOption("One Cargo Pickup One", &m_oneCargoPickupOne);
  m_chooser.AddOption("Two Cargo", &m_twoCargo);
  m_chooser.AddOption("Two Cargo With Vision", &m_twoPieceWithVision);

  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  AutoConstants::thetaPIDController.EnableContinuousInput(units::radian_t{-PI}, units::radian_t{PI});
  AutoConstants::thetaPIDController.SetTolerance(units::radian_t{1.0 / 30.0});
  // std::cout << "Robot container constructed\n";
}

void RobotContainer::ConfigureButtonBindings() {
  // Individually spin each of 8 motors - most basic test
  // We didn't have enough electrical connections
  
  // m_frontLeftSpeedTestButton = m_bill.Button(PS5_BUTTON_SQR);
  // m_frontLeftSpeedTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::fl, .1, ManualModuleDriveType::forward).ToPtr());

  // m_frontRightSpeedTestButton = m_bill.Button(PS5_BUTTON_TRI);
  // m_frontRightSpeedTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::fr, .1, ManualModuleDriveType::forward).ToPtr());

  // m_backLeftSpeedTestButton = m_bill.Button(PS5_BUTTON_X);     
  // m_backLeftSpeedTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::bl, .1, ManualModuleDriveType::forward).ToPtr());

  // m_backRightSpeedTestButton = m_bill.Button(PS5_BUTTON_O);
  // m_backRightSpeedTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::br, .1, ManualModuleDriveType::forward).ToPtr());

  m_translateLeftButton = m_bill.Button(PS5_BUTTON_SQR);
  m_translateLeftButton.WhileTrue(SlowTranslate(m_swerve, 0.0_mps, -0.5_mps).ToPtr());

  m_translateRightButton = m_bill.Button(PS5_BUTTON_O);
  m_translateRightButton.WhileTrue(SlowTranslate(m_swerve, 0.0_mps, 0.5_mps).ToPtr());

  m_rotateTo0Button = m_bill.Button(PS5_BUTTON_TRI);
  m_rotateTo0Button.OnTrue(RotateTo(m_swerve, 0.0).ToPtr());

  m_rotateTo180Button = m_bill.Button(PS5_BUTTON_X);
  m_rotateTo180Button.OnTrue(RotateTo(m_swerve, 180.0).ToPtr());

  m_frontLeftRotTestButton = m_bill.Button(PS5_BUTTON_LBUMPER);
  m_frontLeftRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::fl, .1, ManualModuleDriveType::turn).ToPtr());

  m_frontRightRotTestButton = m_bill.Button(PS5_BUTTON_RBUMPER);
  m_frontRightRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::fr, .1, ManualModuleDriveType::turn).ToPtr());

  m_backLeftRotTestButton = m_bill.Button(PS5_BUTTON_LTRIGGER);
  m_backLeftRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::bl, .1, ManualModuleDriveType::turn).ToPtr());

  m_backRightRotTestButton = m_bill.Button(PS5_BUTTON_RTRIGGER);
  m_backRightRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::br, .1, ManualModuleDriveType::turn).ToPtr());

  m_resetOdometryButton = m_bill.Button(PS5_BUTTON_PS);
  m_resetOdometryButton.OnTrue(ResetOdometry(
    m_swerve, frc::Pose2d{0.0_m, 0.0_m, {0.0_deg}}).ToPtr());

  // m_toggleATan2RotButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_toggleATan2RotButton.ToggleOnTrue(OISwerveDrive(&m_bill, m_swerve, true).ToPtr());
  
  // m_updatePIDButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_updatePIDButton.OnTrue(UpdatePIDValues(m_swerve).ToPtr());

  m_autoBalanceButton = m_bill.Button(PS5_BUTTON_RSTICK);
  m_autoBalanceButton.WhileTrue(AutoBalance(m_swerve).ToPtr());
  
  // m_resetEncodersToAbsoluteButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_resetEncodersToAbsoluteButton.OnTrue(ResetEncodersToAbsolute(m_swerve).ToPtr());

  m_toggleIntakePistonsButton = m_bill.Button(PS5_BUTTON_LSTICK);
  m_toggleIntakePistonsButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());
  
  frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  config.SetKinematics(SwerveDriveConstants::kinematics);

  auto leftTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{0.0_m, -0.20_m}},
    frc::Pose2d{0.0_m, -0.40_m, 0_deg},
    config 
  );
  auto rightTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0.0_m, 0.0_m, 0_deg},
    {frc::Translation2d{0.0_m, 0.20_m}},
    frc::Pose2d{0.0_m, 0.40_m, 0_deg},
    config 
  );

  auto leftTranslateCommand = m_swerve->CreateSwerveCommand(leftTrajectory);
  auto rightTranslateCommand = m_swerve->CreateSwerveCommand(rightTrajectory);

  m_leftAimAssistButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_leftAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)), 
  //     ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), leftTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());
  
  m_leftAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
      RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
      leftTranslateCommand,
      RotateTo(m_swerve, 0)).ToPtr());

  m_centerAimAssistButton = m_bill.Button(PS5_BUTTON_TOUCHPAD);
  m_centerAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
    RotateTo(m_swerve, 0),
    ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
    frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)),
    RotateTo(m_swerve, 0)).ToPtr());

  m_rightAimAssistButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_rightAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)),
  //     ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), rightTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());

  m_rightAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
      RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
      rightTranslateCommand,
      RotateTo(m_swerve, 0)).ToPtr());

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Auto command will be user-selected auto from smartdashboard
  return m_chooser.GetSelected();
}

void RobotContainer::Sync() {
  m_swerve->SyncSmartdashBoardValues();
  // std::cout << "Syncing robot container\n";
}

void RobotContainer::ResetGyroscope() {
  m_swerve->ResetGyro();
}
