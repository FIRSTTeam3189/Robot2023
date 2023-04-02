// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // std::cout << "Robot container constructing\n";
  // Consider resetting Pigeon 2.0 gyroscope values - look up API
  // Configure the button bindings
  ConfigureButtonBindings();
  frc::PowerDistribution pdh{1, frc::PowerDistribution::ModuleType::kRev};
  frc::SmartDashboard::PutNumber("PDH Current", pdh.GetTotalCurrent());

  // Joystick operated - real control scheme
  m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, false));
  m_grabber->SetDefaultCommand(frc2::RunCommand([this]{m_grabber->SetSpeed(-0.15);},{m_grabber}));

  m_chooser.SetDefaultOption("Default Test Auto", &m_outtake);
  m_chooser.AddOption("Test Auto: Straight Line", &m_testAuto1);
  m_chooser.AddOption("Test Auto: S With Rotation", &m_testAuto2);
  m_chooser.AddOption("Test Auto: Straight Line W/ -Rotate", &m_testAuto3);
  m_chooser.AddOption("Test Auto: Drive Forward W/ Intake", &m_testAuto4);
  m_chooser.AddOption("Test Auto: Drive Forward and Backard W/ Rotate n Place", &m_testAuto5);
  m_chooser.AddOption("Test Auto: Intake Around Charge Station", &m_testAuto6);
  m_chooser.AddOption("Test Auto: S Without Rotation", &m_testAuto7);
  m_chooser.AddOption("Test Auto: N/A", &m_testAuto8);
  m_chooser.AddOption("Outtake", &m_outtake);
  m_chooser.AddOption("Balance", &m_balance);
  m_chooser.AddOption("One Cargo High", &m_oneCargoHigh);
  m_chooser.AddOption("One Cargo Mid", &m_oneCargoMid);
  m_chooser.AddOption("One Cargo High + Balance", &m_oneCargoHighBalance);
  m_chooser.AddOption("One Cargo Mid + Balance", &m_oneCargoMidBalance);
  m_chooser.AddOption("One Cargo High + Pickup + Balance Red Side", &m_oneCargoHighPickupBalanceRed);
  m_chooser.AddOption("One Cargo High + Pickup + Balance Blue Side", &m_oneCargoHighPickupBalanceBlue);
  m_chooser.AddOption("One Cargo Mid + Pickup + Balance Red Side", &m_oneCargoMidPickupBalanceRed);
  m_chooser.AddOption("One Cargo Mid + Pickup + Balance Blue Side", &m_oneCargoMidPickupBalanceBlue);
  m_chooser.AddOption("One Cargo High + Pickup One", &m_oneCargoHighPickupOne);
  m_chooser.AddOption("One Cargo Mid + Pickup One", &m_oneCargoMidPickupOne);
  m_chooser.AddOption("Two Cargo", &m_twoCargo);
  // m_chooser.AddOption("Two Cargo With Vision", &m_twoPieceWithVision);
  m_chooser.AddOption("Two Cargo Red + Balance + Ultrashoot", &m_twoCargoRedUltrashoot);
  m_chooser.AddOption("Two Cargo Blue + Balance + Ultrashoot", &m_twoCargoBlueUltrashoot);

  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  AutoConstants::thetaPIDController.EnableContinuousInput(units::radian_t{-PI}, units::radian_t{PI});
  AutoConstants::thetaPIDController.SetTolerance(units::radian_t{1.0 / 30.0});
  // std::cout << "Robot container constructed\n";
}

void RobotContainer::ConfigureButtonBindings() {
  // --------------------Driver controls-----------------------
  m_spinIntakeInButton = m_bill.Button(PS5_BUTTON_RBUMPER);
  m_spinIntakeInButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  m_spinIntakeInButton.WhileTrue(
    frc2::SequentialCommandGroup(
      RunIntake(m_intake, -0.50, -0.50, -INTAKE_CONE_CORRECT_POWER),
      ShootFromCarriage(m_grabber, 0.25)
    )
    .ToPtr());
  m_spinIntakeInButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    frc2::WaitCommand(1.0_s).Schedule();
    m_intake->SetPistonExtension(false);
  },{m_intake}).ToPtr());
  // m_spinIntakeInButton.WhileTrue(RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER).ToPtr());

  m_spinIntakeOutButton = m_bill.Button(PS5_BUTTON_LBUMPER);
  m_spinIntakeOutButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  m_spinIntakeOutButton.WhileTrue(
    frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)
    ).ToPtr());
  m_spinIntakeOutButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_intake->SetPistonExtension(false);
    },{m_intake}),
    frc2::WaitCommand(0.5_s),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s), 
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER, 0)
      // ShootFromCarriage(m_grabber, 0.15)
    ),
    frc2::InstantCommand([this]{m_intake->SetPower(0, 0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber})
  ).ToPtr());

  // m_spinIntakeOutButton.WhileTrue(RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, -INTAKE_CONE_CORRECT_POWER).ToPtr());

  // m_toggleIntakePistonsDriver = m_bill.Button(PS5_BUTTON_LTRIGGER);
  // m_toggleIntakePistonsDriver.OnTrue(ToggleIntakePistons(m_intake).ToPtr());

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

  // m_driveFastButton = m_bill.Button(PS5_BUTTON_LSTICK);
  // m_driveFastButton.WhileTrue((frc2::RunCommand([this]{
  //   m_swerve->DriveFast();
  // },{m_swerve})).ToPtr());
  // m_driveFastButton.OnFalse((frc2::InstantCommand([this]{
  //   m_swerve->Drive(0.0_mps, 0.0_mps, units::radians_per_second_t{0.0}, false);
  // },{m_swerve})).ToPtr());

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

  m_toggleATan2RotButton = m_bill.Button(PS5_BUTTON_RSTICK);
  m_toggleATan2RotButton.OnTrue(
    frc2::InstantCommand([this]{
      m_isMagnitudeRot = !m_isMagnitudeRot;
      m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot));
    },{m_swerve}).ToPtr()
  );
  
  // m_updatePIDButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_updatePIDButton.OnTrue(UpdatePIDValues(m_swerve).ToPtr());

  // Change to bumper or trigger
  m_autoBalanceButton = m_bill.Button(PS5_BUTTON_LTRIGGER);
  m_autoBalanceButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());
  // m_autoBalanceButton.WhileTrue(AutoBalance(m_swerve).ToPtr());
  
  m_resetEncodersToAbsoluteButton = m_bill.Button(PS5_BUTTON_CREATE);
  m_resetEncodersToAbsoluteButton.WhileTrue(AutoBalance(m_swerve).ToPtr());
  
  // frc::TrajectoryConfig config{SwerveDriveConstants::kMaxSpeed / 2, SwerveDriveConstants::kMaxAcceleration / 2};
  // config.SetKinematics(SwerveDriveConstants::kinematics);

  // frc::Trajectory leftTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {frc::Translation2d{0.0_m, -0.20_m}},
  //   frc::Pose2d{0.0_m, -0.40_m, 0_deg},
  //   config 
  // );
  // frc::Trajectory rightTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //   frc::Pose2d{0.0_m, 0.0_m, 0_deg},
  //   {frc::Translation2d{0.0_m, 0.20_m}},
  //   frc::Pose2d{0.0_m, 0.40_m, 0_deg},
  //   config 
  // );

  // frc::Trajectory leftTrajectory, rightTrajectory;
  // auto leftTranslateCommand = m_swerve->CreateSwerveCommand(frc::Trajectory());
  // auto rightTranslateCommand = m_swerve->CreateSwerveCommand(frc::Trajectory());
  // frc2::SequentialCommandGroup leftTranslateCommandGroup = frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([&]{
  //     auto m_pose = m_swerve->GetPose();
  //     leftTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     m_pose,
  //     {frc::Translation2d{m_pose.X(), m_pose.Y() - 0.20_m}},
  //     frc::Pose2d{m_pose.X(), m_pose.Y() - 0.40_m, m_pose.Rotation().Degrees()},
  //     config 
  //     );
  //     // leftTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  //     leftTranslateCommand = m_swerve->CreateSwerveCommand(leftTrajectory);
  //   },{m_swerve}),
  //   leftTranslateCommand
  // );
  // frc2::SequentialCommandGroup rightTranslateCommandGroup = frc2::SequentialCommandGroup(
  //   frc2::InstantCommand([&]{
  //     auto m_pose = m_swerve->GetPose();
  //     rightTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
  //     m_pose,
  //     {frc::Translation2d{m_pose.X(), m_pose.Y() + 0.20_m}},
  //     frc::Pose2d{m_pose.X(), m_pose.Y() + 0.40_m, m_pose.Rotation().Degrees()},
  //     config 
  //     );
  //     rightTranslateCommand = m_swerve->CreateSwerveCommand(rightTrajectory);
  //     // rightTrajectory.TransformBy(frc::Transform2d{frc::Pose2d{0_m, 0_m, 0_deg}, m_swerve->GetPose()});
  //   },{m_swerve}),
  //   rightTranslateCommand
  // );
  
  // auto leftTranslateCommand = m_swerve->CreateSwerveCommand(leftTrajectory);
  // auto rightTranslateCommand = m_swerve->CreateSwerveCommand(rightTrajectory);

  m_leftAimAssistButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_leftAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)), 
  //     ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), leftTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());
  // m_leftAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, -0.4, 0.0)
  // ).ToPtr());

  // m_leftTranslateTrajectoryButton = m_bill.Button(PS5_BUTTON_LBUMPER);
  // m_leftTranslateTrajectoryButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     leftTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());
  // m_leftTranslateTrajectoryButton.OnTrue(&leftTranslateCommandGroup);

  m_rightAimAssistButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_rightAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)),
  //     ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}), rightTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());
  // m_rightAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, 0.4, 0.0)
  // ).ToPtr());

  // m_rightTranslateTrajectoryButton = m_bill.Button(PS5_BUTTON_MENU);  
  // m_rightTranslateTrajectoryButton = m_bill.Button(PS5_BUTTON_RBUMPER);
  // m_rightTranslateTrajectoryButton.OnTrue(frc2::SequentialCommandGroup(
  //     RotateTo(m_swerve, 0), ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //     rightTranslateCommand,
  //     RotateTo(m_swerve, 0)).ToPtr());
  // m_rightTranslateTrajectoryButton.OnTrue(&rightTranslateCommandGroup);

  m_centerAimAssistButton = m_bill.Button(PS5_BUTTON_PS);
  // m_centerAimAssistButton.OnTrue(frc2::SequentialCommandGroup(
  //   RotateTo(m_swerve, 0),
  //   ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, 0_deg}),
  //   frc2::ParallelRaceGroup(frc2::WaitCommand(3.0_s), AimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)),
  //   RotateTo(m_swerve, 0)).ToPtr());
  // m_centerAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)
  // ).ToPtr());

  // m_slideStationAimAssistButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_slideStationAimAssistButton.OnTrue(frc2::InstantCommand([this] {
  //   auto data = m_vision->GetData();
  //   if (data.detectionID != DetectionType::None) {
  //     if (data.ID == 4) {
  //       AimAssist(m_vision, m_swerve, 2.03835, 1.143, 90.0).Schedule();
  //     } else if (data.ID == 5) {
  //       AimAssist(m_vision, m_swerve, 2.03835, -1.143, -90.0).Schedule();
  //     }
  //   }
  // }).ToPtr());
 
  m_lockWheelsButton = m_bill.Button(PS5_BUTTON_RTRIGGER);
  m_lockWheelsButton.WhileTrue(frc2::InstantCommand([this]{m_swerve->LockWheels();},{m_swerve}).ToPtr().Repeatedly());

  // ---------------------Ted's controls----------------------
  // Co-driver drives the elevator manually and continuously pulls in by default
  // Can also use PID buttons instead
  // m_elevator->SetDefaultCommand(frc2::RunCommand([this]{
  //   if (-m_ted.GetRawAxis(PS5_AXIS_LSTICK_Y) > 0.05) {
  //     m_elevator->Drive((-m_ted.GetRawAxis(PS5_AXIS_LSTICK_Y)) / 5.0);
  //   } else if (-m_ted.GetRawAxis(PS5_AXIS_LSTICK_Y) < -0.05) {
  //     m_elevator->Drive((-m_ted.GetRawAxis(PS5_AXIS_LSTICK_Y)) / 10.0);
  //   }
  //   },
  //   {m_elevator}).ToPtr());
  m_elevator->SetDefaultCommand(ElevatorRawDrive(m_elevator, m_grabber, m_intake, &m_ted));

  // When codriver buttons are pressed, elevator will go to corresponding position
  // When codriver releases button (i.e. they should hold it down until they want this to happen),
  // the shooter will shoot for a specified amount of time

  // Should replace the one above once tested
  // auto timedShoot = frc2::SequentialCommandGroup( 
  //   frc2::ParallelDeadlineGroup(
  //     frc2::WaitCommand(3.0_s), 
  //     ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
  //   ElevatorPID(m_elevator, m_grabber, m_intake, 0, false),
  //   frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false); },{m_intake})
  // );

  // When pressing codriver elevator buttons, moves elevator up to target
  // On release, shoot the piece
  m_elevatorLowLevelButton = m_ted.Button(PS5_BUTTON_X);  
  m_elevatorLowLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      ElevatorPID(m_elevator, m_intake, ELEVATOR_LOW_TARGET, false)).ToPtr());
  m_elevatorLowLevelButton.OnFalse(frc2::SequentialCommandGroup( 
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  ).ToPtr());
  // m_elevatorLowLevelButton.OnFalse(ElevatorPID(m_elevator, m_grabber, m_intake, 0, false).ToPtr());
  
  m_elevatorMidLevelButton = m_ted.Button(PS5_BUTTON_SQR);
  m_elevatorMidLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
        ElevatorPID(m_elevator, m_intake, ELEVATOR_MID_TARGET, false)).ToPtr());
  m_elevatorMidLevelButton.OnFalse(frc2::SequentialCommandGroup( 
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  ).ToPtr());
  // m_elevatorMidLevelButton.OnFalse(ElevatorPID(m_elevator, m_grabber, m_intake, 0, false).ToPtr());

  m_elevatorHighLevelButton = m_ted.Button(PS5_BUTTON_TRI);
  m_elevatorHighLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
        ElevatorPID(m_elevator, m_intake, ELEVATOR_HIGH_TARGET, false)).ToPtr());
  m_elevatorHighLevelButton.OnFalse(frc2::SequentialCommandGroup( 
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  ).ToPtr());
  // m_elevatorHighLevelButton.OnFalse(ElevatorPID(m_elevator, m_grabber, m_intake, 0, false).ToPtr());

  m_cancelElevatorPIDControl = m_ted.Button(PS5_BUTTON_LSTICK);
  m_cancelElevatorPIDControl.OnTrue(ElevatorPID(m_elevator, m_intake, 0, true).ToPtr());

  m_toggleIntakePistonsButton = m_ted.Button(PS5_BUTTON_TOUCHPAD);
  m_toggleIntakePistonsButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());

  m_runConveyorButton = m_ted.Button(PS5_BUTTON_CREATE);
  m_runConveyorButton.OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{
        m_intake->SetPower(0.0, INTAKE_CONVEYOR_POWER, 0);
      // m_grabber->SetSpeed(GRABBER_GRAB_SPEED);
      },{m_intake, m_grabber}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.35_s),
        ElevatorPID(m_elevator, m_intake, 200, false)
        // ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)
      )).ToPtr()
  );
  m_runConveyorButton.OnFalse(frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_grabber->SetSpeed(0);
    },{m_intake, m_grabber}).ToPtr()
  );

  m_coneCorrectButton = m_ted.Button(PS5_BUTTON_TOUCHPAD);
  // m_coneCorrectButton.OnTrue(frc2::InstantCommand([this]{m_intake->SetPistonExtension(false);},{m_intake}).ToPtr());
  // m_coneCorrectButton.WhileTrue(
  //   frc2::ParallelRaceGroup(RunIntake(m_intake, INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER),
  //                           ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)).ToPtr());
  m_coneCorrectButton.WhileTrue(frc2::InstantCommand([this]{m_intake->SetPower((INTAKE_ROLLER_POWER / 2), -INTAKE_CONVEYOR_POWER, 0);},{m_intake}).ToPtr());
  m_coneCorrectButton.OnFalse(RunIntake(m_intake, 0, 0, 0).ToPtr());

  // Hold down intake buttons to extend pistons and run; retracts when you let go
  m_runIntakeInButton = m_ted.Button(PS5_BUTTON_LBUMPER);
  m_runIntakeInButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  m_runIntakeInButton.WhileTrue(
    frc2::SequentialCommandGroup(
      frc2::WaitCommand(0.5_s),
      RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)
    ).ToPtr());
  m_runIntakeInButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_intake->SetPistonExtension(false);
    },{m_intake}),
    frc2::WaitCommand(0.5_s),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s), 
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER, 0),
      ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)
    ),
    frc2::InstantCommand([this]{m_intake->SetPower(0, 0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber})
  ).ToPtr());

  m_runIntakeOutButton = m_ted.Button(PS5_BUTTON_RBUMPER);
  m_runIntakeOutButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  m_runIntakeOutButton.WhileTrue(RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, -INTAKE_CONE_CORRECT_POWER).ToPtr());
  m_runIntakeOutButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    frc2::WaitCommand(1.0_s).Schedule();
    m_intake->SetPistonExtension(false);
  },{m_intake}).ToPtr());

  m_grabButton = m_ted.Button(PS5_BUTTON_LTRIGGER);
  m_grabButton.WhileTrue(ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED).ToPtr());
  m_grabButton.OnFalse(ShootFromCarriage(m_grabber, 0).ToPtr());

  m_shootButton = m_ted.Button(PS5_BUTTON_RTRIGGER);
  m_shootButton.WhileTrue(ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED).ToPtr());
  m_shootButton.OnFalse(ShootFromCarriage(m_grabber, 0).ToPtr());

  m_codriverStopButton = m_ted.Button(PS5_BUTTON_PS);
  m_codriverStopButton.OnTrue((frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    m_elevator->Drive(0);
    m_grabber->SetSpeed(0);
  },{m_intake, m_grabber, m_elevator}).ToPtr()));

  m_ultraShootButton = m_ted.Button(PS5_BUTTON_LSTICK);
  m_ultraShootButton.OnTrue(UltraShoot(m_elevator, m_intake, m_grabber).ToPtr());

  // If co-driver pushes right stick forward or backward, intake will run accordingly to reject or pick up piece
  // m_intake->SetDefaultCommand((frc2::RunCommand([this]{
  //   if (m_ted.GetRawAxis(PS5_AXIS_RSTICK_Y) > 0.25) {
  //     m_intake->SetPower(-INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, -INTAKE_CONE_CORRECT_POWER);
  //   } else if (m_ted.GetRawAxis(PS5_AXIS_RSTICK_Y) < -0.25) {
  //     m_intake->SetPower(INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER);
  //   } else {
  //     m_intake->SetPower(0, 0, 0);
  //   }
  // },{m_intake})));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Auto command will be user-selected auto from smartdashboard
  return m_chooser.GetSelected();
}

void RobotContainer::Sync() {
  // std::cout << m_elevator->ElevatorTicksToMeters(1) << "\n";
  m_swerve->SyncSmartdashBoardValues();
  // std::cout << "Syncing robot container\n";
}

void RobotContainer::ResetGyroscope() {
  m_swerve->ResetGyro();
}

void RobotContainer::CheckPOV() {
  // Checks POV constantly on controllers
  // USES ONLY 4 CARDINAL DIRECTIONS -- MUST BE PRESSED PRECISELY
  if (m_bill.GetPOV() != -1) {
    int driverPOV = m_bill.GetPOV();

    switch (driverPOV) {
      case 0: 
        break;
      case 90:
        AimAssist(m_vision, m_swerve, 0.05, 0.75, 0.0).Schedule();
        break;
      case 180:
        break;
      case 270:
        AimAssist(m_vision, m_swerve, 0.05, -0.75, 0.0).Schedule();
        break;
      default:
        break;
    }
  }

  if (m_ted.GetPOV() != -1) {
    int codriverPOV = m_ted.GetPOV();

    switch (codriverPOV) {
      case 0:
        if (!m_elevator->GetRunningState()) {
          std::cout << "About to run PID\n";
          frc2::CommandScheduler::GetInstance().Schedule(ElevatorPID(m_elevator, m_intake, ELEVATOR_HIGH_TARGET, false).ToPtr());
          m_elevator->SetRunningState(true);
        }
        break;
      case 90:
        if (!m_elevator->GetRunningState()) {
          std::cout << "About to run PID\n";
          frc2::CommandScheduler::GetInstance().Schedule(ElevatorPID(m_elevator, m_intake, ELEVATOR_MID_TARGET, false).ToPtr());
          m_elevator->SetRunningState(true);
        }
        break;
      case 180:
        if (!m_elevator->GetRunningState()) {
          std::cout << "About to run PID\n";
          frc2::CommandScheduler::GetInstance().Schedule(ElevatorPID(m_elevator, m_intake, ELEVATOR_LOW_TARGET, false).ToPtr());
          m_elevator->SetRunningState(true);
        }
        break;
      case 270:
        break;
      default:
        break;
    }
  }
}