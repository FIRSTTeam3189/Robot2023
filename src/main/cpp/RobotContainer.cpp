// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here
  // Configure the button bindings
  ConfigureButtonBindings();
  // Adds auto events to map of possible events
  BuildEventMap();
  CreateAutoPaths();
  frc::PowerDistribution pdh{1, frc::PowerDistribution::ModuleType::kRev};
  frc::SmartDashboard::PutNumber("PDH Current", pdh.GetTotalCurrent());

  // Joystick operated by DEFAULT
  m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, false));

  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  AutoParameters::thetaPIDController.EnableContinuousInput(units::radian_t{-pi}, units::radian_t{pi});
  AutoParameters::thetaPIDController.SetTolerance(units::radian_t{1.0 / 30.0});
}

void RobotContainer::ConfigureButtonBindings() {
  // --------------------Driver controls-----------------------
  frc2::Trigger outtakeButton{m_bill.Button(PS5_BUTTON_RBUMPER)};
  outtakeButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  outtakeButton.WhileTrue(
    frc2::SequentialCommandGroup(
      RunIntake(m_intake, OUTTAKE_ROLLER_POWER, OUTTAKE_CONVEYOR_POWER, 0),
      ShootFromCarriage(m_grabber, GRABBER_OUTTAKE_SPEED)
    )
    .ToPtr());
  outtakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    m_intake->SetPistonExtension(false);
  },{m_intake}).ToPtr());

  frc2::Trigger intakeButton{m_bill.Button(PS5_BUTTON_LBUMPER)};
  intakeButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  intakeButton.WhileTrue(
    frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)
    ).ToPtr());
  intakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_intake->SetPistonExtension(false);
    },{m_intake}),
    frc2::WaitCommand(0.5_s),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s), 
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER, 0)
    ),
    frc2::InstantCommand([this]{m_intake->SetPower(0, 0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber})
  ).ToPtr());

  // ----------------------------------------------- POV Button Example ----------------------------------------------------------------------------
  // m_Trigger = frc2::Trigger(m_controller.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()));
  // m_Trigger.OnTrue/WhileTrue(frc2::Command);
  // ------------------------------------------------------------------------------------------------------------------------------------------------

  frc2::Trigger translateLeftButton{m_bill.Button(PS5_BUTTON_SQR)};
  translateLeftButton.WhileTrue(SlowTranslate(m_swerve, 0.0_mps, SwerveDriveConstants::leftTranslateSpeed).ToPtr());

  frc2::Trigger translateRightButton{m_bill.Button(PS5_BUTTON_O)};
  translateRightButton.WhileTrue(SlowTranslate(m_swerve, 0.0_mps, SwerveDriveConstants::rightTranslateSpeed).ToPtr());

  frc2::Trigger rotateTo0Button{m_bill.Button(PS5_BUTTON_TRI)};
  rotateTo0Button.OnTrue(RotateTo(m_swerve, 0.0).ToPtr());

  frc2::Trigger rotateTo180Button{m_bill.Button(PS5_BUTTON_X)};
  rotateTo180Button.OnTrue(RotateTo(m_swerve, 180.0).ToPtr());

  frc2::Trigger resetOdometryButton{m_bill.Button(PS5_BUTTON_TOUCHPAD)};
  resetOdometryButton.OnTrue(
    frc2::SequentialCommandGroup(
      ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, {0.0_deg}}),
      frc2::InstantCommand([this]{m_swerve->SetRobotYaw(0.0);},{m_swerve})
    ).ToPtr());

  frc2::Trigger toggleATan2RotButton{m_bill.Button(PS5_BUTTON_RSTICK)};
  toggleATan2RotButton.OnTrue(
    frc2::InstantCommand([this]{
      m_isMagnitudeRot = !m_isMagnitudeRot;
      m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot));
    },{m_swerve}).ToPtr()
  );
  
  frc2::Trigger autoBalanceButton{m_bill.Button(PS5_BUTTON_LTRIGGER)};
  autoBalanceButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());
  
  frc2::Trigger resetEncodersToAbsoluteButton{m_bill.Button(PS5_BUTTON_CREATE)};
  resetEncodersToAbsoluteButton.WhileTrue(AutoBalance(m_swerve).ToPtr());
  
  frc2::Trigger leftAimAssistButton{m_bill.Button(PS5_BUTTON_CREATE)};
  // leftAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, -0.4, 0.0)
  // ).ToPtr());

  frc2::Trigger rightAimAssistButton{m_bill.Button(PS5_BUTTON_MENU)};
  // rightAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, 0.4, 0.0)
  // ).ToPtr());

  frc2::Trigger centerAimAssistButton{m_bill.Button(PS5_BUTTON_PS)};
  // centerAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)
  // ).ToPtr());
 
  frc2::Trigger lockWheelsButton{m_bill.Button(PS5_BUTTON_RTRIGGER)};
  lockWheelsButton.WhileTrue(frc2::InstantCommand([this]{m_swerve->LockWheels();},{m_swerve}).ToPtr().Repeatedly());

  // ---------------------Ted's controls----------------------
  m_elevator->SetDefaultCommand(ElevatorRawDrive(m_elevator, m_grabber, m_intake, &m_ted));

  // When codriver buttons are pressed, elevator will go to corresponding position
  // When codriver releases button (i.e. they should hold it down until they want this to happen),
  // the shooter will shoot for a specified amount of time

  // When pressing codriver elevator buttons, moves elevator up to target
  // On release, shoot the piece
  frc2::Trigger elevatorLowLevelButton{m_ted.Button(PS5_BUTTON_X)};  
  elevatorLowLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      ElevatorPID(m_elevator, m_intake, ELEVATOR_LOW_TARGET, false)).ToPtr());
  elevatorLowLevelButton.OnFalse(frc2::SequentialCommandGroup( 
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  ).ToPtr());
  
  frc2::Trigger elevatorMidLevelButton{m_ted.Button(PS5_BUTTON_O)};
  elevatorMidLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
        ElevatorPID(m_elevator, m_intake, ELEVATOR_MID_TARGET, false)).ToPtr());
  elevatorMidLevelButton.OnFalse(frc2::SequentialCommandGroup( 
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  ).ToPtr());

  frc2::Trigger elevatorHighLevelButton{m_ted.Button(PS5_BUTTON_TRI)};
  elevatorHighLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
        ElevatorPID(m_elevator, m_intake, ELEVATOR_HIGH_TARGET, false)).ToPtr());
  elevatorHighLevelButton.OnFalse(frc2::SequentialCommandGroup( 
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(.25_s), 
      ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)),
    frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
    ElevatorPID(m_elevator, m_intake, 0, false),
    frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
  ).ToPtr());

  frc2::Trigger grabDoubleStationButton{m_ted.Button(PS5_BUTTON_SQR)};
  grabDoubleStationButton.OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      frc2::ParallelRaceGroup(
        ElevatorPID(m_elevator, m_intake, ELEVATOR_DOUBLE_SUBSTATION_TARGET, false),
        ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)
      )
    ).ToPtr()
  );
  grabDoubleStationButton.OnFalse(
    frc2::SequentialCommandGroup(
      ElevatorPID(m_elevator, m_intake, 0, false),
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false); m_grabber->SetSpeed(GRABBER_CARRY_SPEED); },{m_intake})
    ).ToPtr()
  );

  frc2::Trigger cancelElevatorPIDControl{m_ted.Button(PS5_BUTTON_LSTICK)};
  cancelElevatorPIDControl.OnTrue(ElevatorPID(m_elevator, m_intake, 0, true).ToPtr());

  frc2::Trigger runConveyorButton{m_ted.Button(PS5_BUTTON_CREATE)};
  runConveyorButton.OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{
        m_intake->SetPower(0.0, INTAKE_CONVEYOR_POWER, 0);
      },{m_intake, m_grabber}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.35_s),
        ElevatorPID(m_elevator, m_intake, ELEVATOR_INTERIOR_GRAB_TARGET, false),
        ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)
      )).ToPtr()
  );
  runConveyorButton.OnFalse(frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_grabber->SetSpeed(0);
    },{m_intake, m_grabber}).ToPtr()
  );

  frc2::Trigger coneCorrectButton{m_ted.Button(PS5_BUTTON_MENU)};
  coneCorrectButton.WhileTrue(frc2::InstantCommand([this]{m_intake->SetPower((INTAKE_ROLLER_POWER / 2), -INTAKE_CONVEYOR_POWER, 0);},{m_intake}).ToPtr());
  coneCorrectButton.OnFalse(RunIntake(m_intake, 0, 0, 0).ToPtr());

  frc2::Trigger grabButton{m_ted.Button(PS5_BUTTON_LTRIGGER)};
  grabButton.WhileTrue(ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED).ToPtr());
  grabButton.OnFalse(ShootFromCarriage(m_grabber, 0).ToPtr());

  frc2::Trigger shootButton{m_ted.Button(PS5_BUTTON_RTRIGGER)};
  shootButton.WhileTrue(ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED).ToPtr());
  shootButton.OnFalse(ShootFromCarriage(m_grabber, 0).ToPtr());

  frc2::Trigger codriverStopButton{m_ted.Button(PS5_BUTTON_PS)};
  codriverStopButton.OnTrue((frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    m_elevator->Drive(0);
    m_grabber->SetSpeed(0);
  },{m_intake, m_grabber, m_elevator}).ToPtr()));

  frc2::Trigger ultraShootButton{m_ted.Button(PS5_BUTTON_LSTICK)};
  ultraShootButton.OnTrue(UltraShoot(m_elevator, m_intake, m_grabber).ToPtr());

  frc::SmartDashboard::PutBoolean("Piece Mode", m_isConeMode);
  frc2::Trigger pieceModeToggleButton{m_ted.Button(PS5_BUTTON_TOUCHPAD)};
  pieceModeToggleButton.OnTrue(frc2::InstantCommand([this]{ m_isConeMode = !m_isConeMode; frc::SmartDashboard::PutBoolean("Piece Mode", m_isConeMode); }, {}).ToPtr());
}

void RobotContainer::BuildEventMap() {
    /*
  * Emplace all the possible auto commands into auto event map on RoboRIO startup
  * The key is the string associated with the event, and this string
  * should match the string entered into the PathPlanner event marker
  * These events will then be called during PathPlanner paths
  * The value is a std::sharedptr<frc2::Command>,
  * which is a pointer to the command that should be executed when its
  * associated key is called
  */ 

  // --------------------------------START OF AUTO EVENTS---------------------------------

  AutoParameters::eventMap.emplace(
    "intake_pistons_out", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.25_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "intake_pistons_in", 
    std::make_shared<frc2::InstantCommand>(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
    )
  );

  AutoParameters::eventMap.emplace(
    "score_low", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        ElevatorPID(m_elevator, m_intake, ELEVATOR_LOW_TARGET, false),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(.25_s), 
          ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "score_mid", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        ElevatorPID(m_elevator, m_intake, ELEVATOR_MID_TARGET, false),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(.25_s), 
          ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "score_high", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        ElevatorPID(m_elevator, m_intake, ELEVATOR_HIGH_TARGET, false),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(.25_s), 
          ShootFromCarriage(m_grabber, GRABBER_DROP_SPEED)
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );
  
  AutoParameters::eventMap.emplace(
    "elevator_to_0", 
    std::make_shared<frc2::ParallelDeadlineGroup>(
      frc2::WaitCommand(2.0_s),
      ElevatorPID(m_elevator, m_intake, 0, false)
    )
  );

  AutoParameters::eventMap.emplace(
    "grab", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{
          m_intake->SetPower(0.0, INTAKE_CONVEYOR_POWER, 0);
          m_grabber->SetSpeed(GRABBER_GRAB_SPEED);
        },{m_intake, m_grabber}),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(0.35_s),
          ElevatorPID(m_elevator, m_intake, ELEVATOR_INTERIOR_GRAB_TARGET, false)
        ),
        frc2::WaitCommand(0.125_s),
        frc2::InstantCommand([this]{
          m_intake->SetPower(0, 0, 0);
          m_grabber->SetSpeed(0);
        },{m_intake, m_grabber})
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "run_intake_in", 
    std::make_shared<frc2::ParallelDeadlineGroup>(
      frc2::ParallelDeadlineGroup(
        RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, 0),
        ShootFromCarriage(m_grabber, GRABBER_GRAB_SPEED)
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "run_intake_out", 
    std::make_shared<frc2::ParallelDeadlineGroup>(
      frc2::ParallelDeadlineGroup(
        RunIntake(m_intake, OUTTAKE_ROLLER_POWER, OUTTAKE_CONVEYOR_POWER, -INTAKE_CONE_CORRECT_POWER),
        ShootFromCarriage(m_grabber, 0.25)
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "ultra_shoot", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        frc2::ParallelDeadlineGroup(
          ElevatorPID(m_elevator, m_intake, ELEVATOR_ULTRA_SHOOT_TARGET, false),
          frc2::RunCommand([this]{
            if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
              m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
            }
          },{m_grabber})
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );

  // AutoParameters::eventMap.emplace(
  //   "reset_odometry", 
  //   std::make_shared<frc2::SequentialCommandGroup>(
  //     frc2::SequentialCommandGroup(
  //       ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}),
  //       frc2::WaitCommand(0.05_s),
  //       ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}),
  //       frc2::WaitCommand(0.05_s),
  //       ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}}),
  //       frc2::WaitCommand(0.05_s)
  //     )
  //   )
  // );

  // AutoParameters::eventMap.emplace(
  //   "set_yaw_0", 
  //   std::make_shared<frc2::SequentialCommandGroup>(
  //     frc2::SequentialCommandGroup(
  //       frc2::InstantCommand([this]{m_swerve->SetRobotYaw(0.0);},{m_swerve}),
  //       frc2::WaitCommand(0.05_s),
  //       frc2::InstantCommand([this]{m_swerve->SetRobotYaw(0.0);},{m_swerve}),
  //       frc2::WaitCommand(0.05_s),
  //       frc2::InstantCommand([this]{m_swerve->SetRobotYaw(0.0);},{m_swerve}),
  //       frc2::WaitCommand(0.05_s)
  //     )
  //   )
  // );

  // AutoParameters::eventMap.emplace(
  //   "set_yaw_180", 
  //   std::make_shared<frc2::SequentialCommandGroup>(
  //     frc2::SequentialCommandGroup(
  //       frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
  //       frc2::WaitCommand(0.05_s),
  //       frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
  //       frc2::WaitCommand(0.05_s),
  //       frc2::InstantCommand([this]{m_swerve->SetRobotYaw(180.0);},{m_swerve}),
  //       frc2::WaitCommand(0.05_s)
  //     )
  //   )
  // );

  AutoParameters::eventMap.emplace(
    "stop_intake", 
    std::make_shared<RunIntake>(
      m_intake, 0, 0, 0
    )
  );

  AutoParameters::eventMap.emplace(
    "print", 
    std::make_shared<frc2::InstantCommand>(
      frc2::InstantCommand([this]{std::cout << "Event triggered\n";},{})
    )
  );

  // --------------------------------END OF AUTO EVENTS-----------------------------------
}

void RobotContainer::CreateAutoPaths() {
  // Make auto builder
  m_autoBuilder = new pathplanner::SwerveAutoBuilder(
    [this]() { return m_swerve->GetCorrectedPose(); }, // Function to supply current robot pose
    [this](auto initPose) { m_swerve->ResetOdometry(initPose); }, // Function used to reset odometry at the beginning of auto
    pathplanner::PIDConstants(AutoConstants::kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    pathplanner::PIDConstants(AutoConstants::autoRotP, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    [this](auto speeds) { m_swerve->PercentDrive(speeds); }, // Output function that accepts field relative ChassisSpeeds
    AutoParameters::eventMap, // Our event map
    { m_swerve }, // Drive requirements, usually just a single drive subsystem
    true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );

  // Add auto commands to auto chooser
  m_chooser.SetDefaultOption("N/A", nullptr);
  m_chooser.AddOption("Test: Line", new TestLineAuto(m_autoBuilder, "Test - Line"));
  m_chooser.AddOption("Test - S", new TestSAuto(m_autoBuilder, "Test - S"));
  m_chooser.AddOption("Test - Line + Rotate", new TestLineRotateAuto(m_autoBuilder, "Test - Line + Rotate"));
  m_chooser.AddOption("Test - Line + Intake", new TestLineIntakeAuto(m_autoBuilder, "Test - Line + Intake"));
  m_chooser.AddOption("Test - S + Rotate", new TestSRotateAuto(m_autoBuilder, "Test - S + Rotate"));
  m_chooser.AddOption("Special", new SpecialAuto(m_autoBuilder, "Special"));
  m_chooser.AddOption("Figure Eight", new FigureEightAuto(m_autoBuilder, "Figure Eight"));
  m_chooser.AddOption("Two Score: High/Mid Cubes", new TwoScoreHighMidCubeAuto(m_autoBuilder, "Two Score High-Mid Cube"));
  m_chooser.AddOption("One Score: High Cube + Taxi", new OneScoreHighCubeTaxiAuto(m_autoBuilder, "One Score + Taxi"));
  m_chooser.AddOption("One Score: High Cube + Balance", new OneScoreHighCubeBalanceAuto(m_autoBuilder, "One Score + Balance"));
  m_chooser.AddOption("Two Score: High Cube + Balance + Ultrashoot", new TwoScoreHighCubeUltrashootAuto(m_autoBuilder, "One Score + Pickup + Balance + Ultrashoot"));
  m_chooser.AddOption("Two Score: Wide Sweep High/Mid Cubes", new TwoScoreWideSweepHighMidCubeAuto(m_autoBuilder, "Two Score Wide Sweep"));
  m_chooser.AddOption("Five Score", new FiveScoreAuto(m_autoBuilder, "Five Score"));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Auto command will be user-selected auto from smartdashboard
  return m_chooser.GetSelected();
}

void RobotContainer::Sync() {
  m_swerve->SyncSmartdashBoardValues();
  frc::SmartDashboard::PutNumber("Swerve Gyro Y'all ", m_swerve->GetRobotYaw());
}

void RobotContainer::ResetGyroscope() {
  m_swerve->ResetGyro();
  m_swerve->ResetOdometry(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}});
}