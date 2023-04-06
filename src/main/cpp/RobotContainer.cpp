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
  AutoParameters::thetaPIDController.EnableContinuousInput(units::radian_t{-PI}, units::radian_t{PI});
  AutoParameters::thetaPIDController.SetTolerance(units::radian_t{1.0 / 30.0});
}

void RobotContainer::ConfigureButtonBindings() {
  // --------------------Driver controls-----------------------
  m_outtakeButton = m_bill.Button(PS5_BUTTON_RBUMPER);
  m_outtakeButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  m_outtakeButton.WhileTrue(
    frc2::SequentialCommandGroup(
      RunIntake(m_intake, OUTTAKE_ROLLER_POWER, OUTTAKE_CONVEYOR_POWER, 0),
      ShootFromCarriage(m_grabber, GRABBER_OUTTAKE_SPEED)
    )
    .ToPtr());
  m_outtakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0, 0);
    m_intake->SetPistonExtension(false);
  },{m_intake}).ToPtr());

  m_intakeButton = m_bill.Button(PS5_BUTTON_LBUMPER);
  m_intakeButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  m_intakeButton.WhileTrue(
    frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER, 0)
      ),
      RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER, INTAKE_CONE_CORRECT_POWER)
    ).ToPtr());
  m_intakeButton.OnFalse(frc2::SequentialCommandGroup(
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

  m_translateLeftButton = m_bill.Button(PS5_BUTTON_SQR);
  m_translateLeftButton.WhileTrue(SlowTranslate(m_swerve, 0.0_mps, SwerveDriveConstants::leftTranslateSpeed).ToPtr());

  m_translateRightButton = m_bill.Button(PS5_BUTTON_O);
  m_translateRightButton.WhileTrue(SlowTranslate(m_swerve, 0.0_mps, SwerveDriveConstants::rightTranslateSpeed).ToPtr());

  m_rotateTo0Button = m_bill.Button(PS5_BUTTON_TRI);
  m_rotateTo0Button.OnTrue(RotateTo(m_swerve, 0.0).ToPtr());

  m_rotateTo180Button = m_bill.Button(PS5_BUTTON_X);
  m_rotateTo180Button.OnTrue(RotateTo(m_swerve, 180.0).ToPtr());

  m_resetOdometryButton = m_bill.Button(PS5_BUTTON_TOUCHPAD);
  m_resetOdometryButton.OnTrue(
    frc2::SequentialCommandGroup(
      ResetOdometry(m_swerve, frc::Pose2d{0.0_m, 0.0_m, {0.0_deg}}),
      frc2::InstantCommand([this]{m_swerve->SetRobotYaw(0.0);},{m_swerve})
    ).ToPtr());

  m_toggleATan2RotButton = m_bill.Button(PS5_BUTTON_RSTICK);
  m_toggleATan2RotButton.OnTrue(
    frc2::InstantCommand([this]{
      m_isMagnitudeRot = !m_isMagnitudeRot;
      m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot));
    },{m_swerve}).ToPtr()
  );
  f
  m_autoBalanceButton = m_bill.Button(PS5_BUTTON_LTRIGGER);
  m_autoBalanceButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());
  
  m_resetEncodersToAbsoluteButton = m_bill.Button(PS5_BUTTON_CREATE);
  m_resetEncodersToAbsoluteButton.WhileTrue(AutoBalance(m_swerve).ToPtr());
  
  m_leftAimAssistButton = m_bill.Button(PS5_BUTTON_CREATE);
  // m_leftAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, -0.4, 0.0)
  // ).ToPtr());

  m_rightAimAssistButton = m_bill.Button(PS5_BUTTON_MENU);
  // m_rightAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, 0.4, 0.0)
  // ).ToPtr());

  m_centerAimAssistButton = m_bill.Button(PS5_BUTTON_PS);
  // m_centerAimAssistButton.OnTrue(frc2::ParallelRaceGroup(
  //   frc2::WaitCommand(3.0_s), TrajectoryAimAssist(m_vision, m_swerve, 1.0, 0.0, 0.0)
  // ).ToPtr());
 
  m_lockWheelsButton = m_bill.Button(PS5_BUTTON_RTRIGGER);
  m_lockWheelsButton.WhileTrue(frc2::InstantCommand([this]{m_swerve->LockWheels();},{m_swerve}).ToPtr().Repeatedly());

  // ---------------------Ted's controls----------------------
  m_elevator->SetDefaultCommand(ElevatorRawDrive(m_elevator, m_grabber, m_intake, &m_ted));

  // When codriver buttons are pressed, elevator will go to corresponding position
  // When codriver releases button (i.e. they should hold it down until they want this to happen),
  // the shooter will shoot for a specified amount of time

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

  m_cancelElevatorPIDControl = m_ted.Button(PS5_BUTTON_LSTICK);
  m_cancelElevatorPIDControl.OnTrue(ElevatorPID(m_elevator, m_intake, 0, true).ToPtr());

  m_runConveyorButton = m_ted.Button(PS5_BUTTON_CREATE);
  m_runConveyorButton.OnTrue(
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
  m_runConveyorButton.OnFalse(frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0, 0);
      m_grabber->SetSpeed(0);
    },{m_intake, m_grabber}).ToPtr()
  );

  m_coneCorrectButton = m_ted.Button(PS5_BUTTON_TOUCHPAD);
  m_coneCorrectButton.WhileTrue(frc2::InstantCommand([this]{m_intake->SetPower((INTAKE_ROLLER_POWER / 2), -INTAKE_CONVEYOR_POWER, 0);},{m_intake}).ToPtr());
  m_coneCorrectButton.OnFalse(RunIntake(m_intake, 0, 0, 0).ToPtr());

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
    [this]() { return m_swerve->GetPose(); }, // Function to supply current robot pose
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
  m_chooser.AddOption("Test: Line", new TestLineAuto(m_autoBuilder));
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