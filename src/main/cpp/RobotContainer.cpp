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
  // --------------------Driver controls-----------------------
  // Both driver and co-driver have intake control
  m_spinIntakeInButton = m_bill.Button(PS5_BUTTON_RBUMPER);
  m_spinIntakeInButton.WhileTrue(RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER).ToPtr());

  m_spinIntakeOutButton = m_bill.Button(PS5_BUTTON_LBUMPER);
  m_spinIntakeOutButton.WhileTrue(RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, -INTAKE_CONE_CORRECT_POWER).ToPtr());

  m_toggleIntakePistonsDriver = m_bill.Button(PS5_BUTTON_LTRIGGER);
  m_toggleIntakePistonsDriver.OnTrue(ToggleIntakePistons(m_intake).ToPtr());

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

  // m_frontLeftRotTestButton = m_bill.Button(PS5_BUTTON_LBUMPER);
  // m_frontLeftRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::fl, .1, ManualModuleDriveType::turn).ToPtr());

  // m_frontRightRotTestButton = m_bill.Button(PS5_BUTTON_RBUMPER);
  // m_frontRightRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::fr, .1, ManualModuleDriveType::turn).ToPtr());

  // m_backLeftRotTestButton = m_bill.Button(PS5_BUTTON_LTRIGGER);
  // m_backLeftRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::bl, .1, ManualModuleDriveType::turn).ToPtr());

  // m_backRightRotTestButton = m_bill.Button(PS5_BUTTON_RTRIGGER);
  // m_backRightRotTestButton.WhileTrue(SingleModTest(m_swerve, SwerveModuleLocation::br, .1, ManualModuleDriveType::turn).ToPtr());

  m_resetOdometryButton = m_bill.Button(PS5_BUTTON_TOUCHPAD);
  m_resetOdometryButton.OnTrue(ResetOdometry(
    m_swerve, frc::Pose2d{0.0_m, 0.0_m, {0.0_deg}}).ToPtr());

  // m_toggleATan2RotButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_toggleATan2RotButton.ToggleOnTrue(OISwerveDrive(&m_bill, m_swerve, true).ToPtr());
  
  // m_updatePIDButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_updatePIDButton.OnTrue(UpdatePIDValues(m_swerve).ToPtr());

  // Change to bumper or trigger
  m_autoBalanceButton = m_bill.Button(PS5_BUTTON_RTRIGGER);
  m_autoBalanceButton.WhileTrue(AutoBalance(m_swerve).ToPtr());
  
  // m_resetEncodersToAbsoluteButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_resetEncodersToAbsoluteButton.OnTrue(ResetEncodersToAbsolute(m_swerve).ToPtr());
  
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

  // REMEMBER TO TEST OUT REDUCING THE AMOUNT OF TIMES
  // WE CALL RESETODOMETRY WITH 0 AND INSTEAD CALL IT WITH 
  // THE CURRENT ROBOT ANGLE SO IT RESETS ENCODERS AND NOT THE ROBOT'S ANGLE
  // THIS WAY THE DRIVER DOESNT HAVE TO TURN AROUND AND PRESS THE BUTTON AS OFTEN
  // NOW MOST TRAJECTORIES WOULD BE FIELD RELATIVE INSTEAD OF ROBOT RELATIVE
  // OR COULD RESET GYRO ANGLE BACK TO HOW IT WAS BEFORE EVERY RESET AFTER EVERY RESET

  // m_leftAimAssistButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_leftAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)), 
  //     ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), leftTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());

  m_leftTranslateTrajectoryButton = m_bill.Button(PS5_BUTTON_CREATE);  
  m_leftTranslateTrajectoryButton.OnTrue(frc2::SequentialCommandGroup(
      RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
      leftTranslateCommand,
      RotateTo(m_swerve, 0)).ToPtr());

  m_centerAimAssistButton = m_bill.Button(PS5_BUTTON_PS);
  m_centerAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
    RotateTo(m_swerve, 0),
    ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
    frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)),
    RotateTo(m_swerve, 0)).ToPtr());

  // m_rightTranslateTrajectoryButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_rightAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)),
  //     ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), rightTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());

  m_rightTranslateTrajectoryButton = m_bill.Button(PS5_BUTTON_MENU);  
  m_rightTranslateTrajectoryButton.OnTrue(frc2::SequentialCommandGroup(
      RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
      rightTranslateCommand,
      RotateTo(m_swerve, 0)).ToPtr());

  // ---------------------Ted's controls----------------------
  // Co-driver drives the elevator manually and continuously pulls in by default
  // Can also use PID buttons instead
  m_elevator->SetDefaultCommand(ElevatorRawDrive(m_elevator, m_grabber, m_ted.GetRawAxis(PS5_AXIS_LSTICK_Y)));

  m_elevatorLowLevelButton = m_ted.Button(PS5_BUTTON_X);
  m_elevatorLowLevelButton.OnTrue(ElevatorPID(m_elevator, m_grabber, ELEVATOR_LOW_TARGET, false).ToPtr());

  m_elevatorMidLevelButton = m_ted.Button(PS5_BUTTON_SQR);
  m_elevatorMidLevelButton.OnTrue(ElevatorPID(m_elevator, m_grabber, ELEVATOR_MID_TARGET, false).ToPtr());

  m_elevatorHighLevelButton = m_ted.Button(PS5_BUTTON_TRI);
  m_elevatorHighLevelButton.OnTrue(ElevatorPID(m_elevator, m_grabber, ELEVATOR_HIGH_TARGET, false).ToPtr());

  m_cancelElevatorPIDControl = m_ted.Button(PS5_BUTTON_LSTICK);
  m_cancelElevatorPIDControl.OnTrue(ElevatorPID(m_elevator, m_grabber, ELEVATOR_LOW_TARGET, true).ToPtr());

  m_toggleIntakePistonsButton = m_ted.Button(PS5_BUTTON_LBUMPER);
  m_toggleIntakePistonsButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());

  m_grabButton = m_ted.Button(PS5_BUTTON_LTRIGGER);
  m_grabButton.WhileTrue(ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED).ToPtr());

  m_shootButton = m_ted.Button(PS5_BUTTON_RTRIGGER);
  m_shootButton.WhileTrue(ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED).ToPtr());

  m_codriverStopButton = m_ted.Button(PS5_BUTTON_O);
  m_codriverStopButton.OnTrue((frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    m_elevator->Drive(0);
    m_grabber->SetSpeed(0);
  },{m_intake, m_grabber, m_elevator}).ToPtr()));

  // If co-driver pushes right stick forward or backward, intake will run accordingly to reject or pick up piece
  m_intake->SetDefaultCommand((frc2::RunCommand([this]{
    if (m_ted.GetRawAxis(PS5_AXIS_RSTICK_Y) > 0.25) {
      m_intake->SetPower(-INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, -INTAKE_CONE_CORRECT_POWER);
    } else if (m_ted.GetRawAxis(PS5_AXIS_RSTICK_Y) < -0.25) {
      m_intake->SetPower(INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER);
    } else {
      m_intake->SetPower(0, 0, 0);
    }
  },{m_intake})));
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
