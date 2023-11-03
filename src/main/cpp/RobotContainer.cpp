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
  // Create paths and add to auto selector
  CreateAutoPaths();
  frc::PowerDistribution pdh{1, frc::PowerDistribution::ModuleType::kRev};
  frc::SmartDashboard::PutNumber("PDH Current", pdh.GetTotalCurrent());

  // Joystick operated by DEFAULT
  m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, false, RotationMode::normal));
  frc::SmartDashboard::PutData("Auto Routines", &m_chooser);
  AutoParameters::thetaPIDController.EnableContinuousInput(units::radian_t{-pi}, units::radian_t{pi});
  AutoParameters::thetaPIDController.SetTolerance(units::radian_t{1.0 / 30.0});
}

void RobotContainer::ConfigureButtonBindings() {
  // --------------------Driver controls-----------------------
  // Extends pistons, runs intake and conveyor outwards while held
  frc2::Trigger outtakeButton{m_bill.Button(PS5_BUTTON_RBUMPER)};
  outtakeButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  outtakeButton.WhileTrue(
    frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
      ),
      frc2::ParallelCommandGroup(
        RunIntake(m_intake, OUTTAKE_ROLLER_POWER, OUTTAKE_CONVEYOR_POWER),
        RunGrabber(m_grabber, GRABBER_OUTTAKE_SPEED)
      )
    )
    .ToPtr());
  outtakeButton.OnFalse(frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0);
    m_intake->SetPistonExtension(false);
  },{m_intake}).ToPtr());

  // Extends pistons, runs intake and conveyor inwards while held
  frc2::Trigger intakeButton{m_bill.Button(PS5_BUTTON_LBUMPER)};
  intakeButton.OnTrue(frc2::InstantCommand([this]{
    m_intake->SetPistonExtension(true);
  },{m_intake}).ToPtr());
  intakeButton.WhileTrue(
    frc2::SequentialCommandGroup(
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
      ),
      frc2::ParallelCommandGroup(
        RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER),
        RunGrabber(m_grabber, GrabberAction::Grab)
      )
    ).ToPtr());
  // When the button is released, bring pistons in and bring piece towards grabber
  intakeButton.OnFalse(frc2::SequentialCommandGroup(
    frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0);
      m_intake->SetPistonExtension(false);
    },{m_intake}),
    frc2::WaitCommand(0.5_s),
    frc2::ParallelDeadlineGroup(
      frc2::WaitCommand(1.0_s), 
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER)
      // Perhaps automatically grab piece - or maybe not
      // RunGrabber(m_grabber, GrabberAction::Grab)
    ),
    frc2::InstantCommand([this]{m_intake->SetPower(0, 0); m_grabber->SetSpeed(0);},{m_intake, m_grabber})
  ).ToPtr());

  // Toggles pistons out with help of intake motors
  frc2::Trigger togglePistonsButton{m_bill.Button(PS5_BUTTON_PS)};
  togglePistonsButton.OnTrue(ToggleIntakePistons(m_intake).ToPtr());
  togglePistonsButton.OnFalse(frc2::InstantCommand([this]{m_intake->SetPower(0, 0);},{m_intake}).ToPtr());

  // ----------------------------------------------- POV Button Example ----------------------------------------------------------------------------
  // m_Trigger = frc2::Trigger(m_controller.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop()));
  // m_Trigger.OnTrue/WhileTrue(frc2::Command);
  // ------------------------------------------------------------------------------------------------------------------------------------------------

  frc2::Trigger BalanceAt0Button{m_bill.Button(PS5_BUTTON_SQR)};
  BalanceAt0Button.WhileTrue(PathPlannerAutoBalance(m_swerve, frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{}}).ToPtr());
  BalanceAt0Button.OnFalse(frc2::InstantCommand([this]{m_swerve->PercentDrive(0_mps, 0_mps, 0_rad / 1_s, true);},{m_swerve}).ToPtr());

  // Point the robot down the field (0 deg) when pressed 
  frc2::Trigger rotateTo0Button{m_bill.Button(PS5_BUTTON_TRI)};
  rotateTo0Button.OnTrue(RotateTo(m_swerve, 0.0).ToPtr());

  // Point the robot towards alliance wall (180 deg) when pressed
  frc2::Trigger rotateTo180Button{m_bill.Button(PS5_BUTTON_X)};
  rotateTo180Button.OnTrue(RotateTo(m_swerve, 180.0).ToPtr());

  // Resets the odometry and gyroscope on the fly when pressed
  // Useful for testing or if something is messed up during teleop
  // Robot acts as if it were just turned on and the field is reset
  frc2::Trigger SetCurrentPoseButton{m_bill.Button(PS5_BUTTON_MIC)};
  SetCurrentPoseButton.OnTrue(
    frc2::SequentialCommandGroup(
      SetCurrentPose(m_swerve, frc::Pose2d{0.0_m, 0.0_m, {0.0_deg}}),
      frc2::InstantCommand([this]{m_swerve->SetRobotYaw(0.0);},{m_swerve})
    ).ToPtr()
  );

  // Toggle drive interface between using setpoints for rotation and using constant rotation
  // By default, right joystick controls holonomic rotation of the robot, where the angle of the joystick
  // Matches the holonomic rotation (i.e. pointing to the right causes the robot to have a holonomic rotation of -90 degrees)
  frc2::Trigger toggleATan2RotButton{m_bill.Button(PS5_BUTTON_RSTICK)};
  toggleATan2RotButton.OnTrue(
    frc2::InstantCommand([this]{
      m_isMagnitudeRot = !m_isMagnitudeRot;
      m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot, RotationMode::normal));
    },{m_swerve}).ToPtr()
  );

  frc2::Trigger toggleFieldRelativeDriveButton{m_bill.Button(PS5_BUTTON_LSTICK)};
  toggleFieldRelativeDriveButton.OnTrue(
    frc2::InstantCommand([this]{
      m_isFieldRelative = !m_isFieldRelative;
      m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot, RotationMode::normal, m_isFieldRelative));
    },{m_swerve}).ToPtr()
  );
  
  // Rotates clockwise at a set speed around the front left module while held
  // When released, goes back to normal center of rotation
  frc2::Trigger cornerRotateCWButton{m_bill.Button(PS5_BUTTON_LTRIGGER)};
  cornerRotateCWButton.OnTrue(frc2::InstantCommand([this] {m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot, RotationMode::frontLeftCW)); }, {m_swerve}).ToPtr());
  cornerRotateCWButton.OnFalse(frc2::InstantCommand([this] {m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot, RotationMode::normal)); }, {m_swerve}).ToPtr());

  // Rotates counter-clockwise at a set speed around the front left module while held
  // When released, goes back to normal center of rotation
  frc2::Trigger cornerRotateCCWButton{m_bill.Button(PS5_BUTTON_RTRIGGER)};
  cornerRotateCCWButton.OnTrue(frc2::InstantCommand([this] {m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot, RotationMode::frontLeftCCW)); }, {m_swerve}).ToPtr());
  cornerRotateCCWButton.OnFalse(frc2::InstantCommand([this] {m_swerve->SetDefaultCommand(OISwerveDrive(&m_bill, m_swerve, m_isMagnitudeRot, RotationMode::normal)); }, {m_swerve}).ToPtr());

  // Auto balances the robot with pathplanner trajectory while held -- locks wheels at the end
  frc2::Trigger PPAutoBalanceButton{m_bill.POVLeft(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  PPAutoBalanceButton.WhileTrue(PathPlannerAutoBalance(m_swerve, FieldCoordinates::chargeStationCenter).ToPtr());
  PPAutoBalanceButton.OnFalse(frc2::InstantCommand([this]{m_swerve->PercentDrive(0_mps, 0_mps, 0_rad / 1_s, true);},{m_swerve}).ToPtr());

  // Auto balances the robot while held -- locks wheels at the end
  frc2::Trigger simpleAutoBalanceButton{m_bill.POVUp(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  simpleAutoBalanceButton.WhileTrue(SimpleAutoBalance(m_swerve).ToPtr());
  simpleAutoBalanceButton.OnFalse(frc2::InstantCommand([this]{m_swerve->PercentDrive(0_mps, 0_mps, 0_rad / 1_s, true);},{m_swerve}).ToPtr());

  // Auto balances the robot while held -- locks wheels at the end
  frc2::Trigger PIDAutoBalanceButton{m_bill.POVDown(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  PIDAutoBalanceButton.WhileTrue(PIDAutoBalance(m_swerve).ToPtr());
  PIDAutoBalanceButton.OnFalse(frc2::InstantCommand([this]{m_swerve->PercentDrive(0_mps, 0_mps, 0_rad / 1_s, true);},{m_swerve}).ToPtr());

  // Locks wheels of the swerve in an "X" pattern while held (for anti-push or anti-slipping on charge station)
  frc2::Trigger lockWheelsButton{m_bill.POVRight(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop())};
  lockWheelsButton.WhileTrue(
    frc2::RunCommand([this]{m_swerve->LockWheels();},{m_swerve}).ToPtr()
  );

  // ---------------------Ted's controls----------------------
  // Co-driver's left stick manually controls elevator velocity
  m_elevator->SetDefaultCommand(ElevatorRawDrive(m_elevator, m_grabber, m_intake, &m_ted));

  // When codriver buttons are pressed, elevator will go to corresponding position
  // When codriver releases button (i.e. they should hold it down until they want this to happen),
  // the shooter will shoot for a specified amount of time

  // When pressing codriver elevator buttons, moves elevator up to target
  // On release, shoot the piece
  
  // Hold to move elevator to low target, release to shoot
  frc2::Trigger elevatorLowLevelButton{m_ted.Button(PS5_BUTTON_X)};  
  elevatorLowLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
      ),
      ElevatorPID(m_elevator, ElevatorLevel::Low, false)
    ).ToPtr()
  );
  elevatorLowLevelButton.OnFalse(
    frc2::SequentialCommandGroup( 
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(.5_s), 
        RunGrabber(m_grabber, GrabberAction::Shoot)
      ),
      frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
      ElevatorPID(m_elevator, 0, false),
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
    ).ToPtr()
  );
  
  // Hold to move elevator to mid target, release to shoot
  frc2::Trigger elevatorMidLevelButton{m_ted.Button(PS5_BUTTON_O)};
  elevatorMidLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(0.5_s),
          RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
        ),
        ElevatorPID(m_elevator, ElevatorLevel::Mid, false)
    ).ToPtr()
  );
  elevatorMidLevelButton.OnFalse(
    frc2::SequentialCommandGroup( 
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(.5_s), 
        RunGrabber(m_grabber, GrabberAction::Shoot)
      ),
      frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
      ElevatorPID(m_elevator, 0, false),
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
    ).ToPtr()
  );

  // Hold to move elevator to high target, release to shoot
  frc2::Trigger elevatorHighLevelButton{m_ted.Button(PS5_BUTTON_TRI)};
  elevatorHighLevelButton.OnTrue(
    frc2::SequentialCommandGroup(
        frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(0.5_s),
          RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
        ),
        ElevatorPID(m_elevator, ElevatorLevel::High, false)
    ).ToPtr()
  );
  elevatorHighLevelButton.OnFalse(
    frc2::SequentialCommandGroup( 
      frc2::ParallelDeadlineGroup( 
        frc2::WaitCommand(.5_s), 
        RunGrabber(m_grabber, GrabberAction::Shoot)
      ),
      frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber}),
      ElevatorPID(m_elevator, 0, false),
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false);},{m_intake})
    ).ToPtr()
  );

  std::vector<std::unique_ptr<frc2::Command>> commands;
  commands.emplace_back(ElevatorPID(m_elevator, ElevatorLevel::DoubleSubstation, false).Repeatedly().Unwrap());
  // Hold to move elevator to double substation target and grab with grabber,
  // release to send elevator back to bottom
  frc2::Trigger grabDoubleStationButton{m_ted.Button(PS5_BUTTON_SQR)};
  grabDoubleStationButton.OnTrue(
    frc2::SequentialCommandGroup(

      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(0.5_s),
          RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
        ),
      ElevatorPID(m_elevator, ElevatorLevel::DoubleSubstation, false),
      // RunGrabber(m_grabber, GrabberAction::Grab)
      frc2::ParallelCommandGroup(
        frc2::SequentialCommandGroup(std::move(commands)),
        RunGrabber(m_grabber, GrabberAction::Grab)
      )
    ).ToPtr()
  );
  grabDoubleStationButton.OnFalse(
    frc2::SequentialCommandGroup(
      frc2::InstantCommand([this]{ m_grabber->SetSpeed(0); },{m_grabber}),
      ElevatorPID(m_elevator, 0, false),
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(false); },{m_intake})
    ).ToPtr()
  );

  frc2::Trigger cancelElevatorPIDControl{m_ted.Button(PS5_BUTTON_LSTICK)};
  cancelElevatorPIDControl.OnTrue(ElevatorPID(m_elevator, 0, true).ToPtr());

  // Brings piece inwards inside robot and grabs with grabber and elevator when pressed
  frc2::Trigger interiorGrabButton{m_ted.Button(PS5_BUTTON_CREATE)};
  interiorGrabButton.WhileTrue(
    frc2::ParallelCommandGroup(
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER),
      RunGrabber(m_grabber, GRABBER_INTERIOR_GRAB_SPEED * 2.0)
    ).ToPtr()
  );
  interiorGrabButton.OnFalse(frc2::InstantCommand([this]{
      m_intake->SetPower(0, 0);
      m_grabber->SetSpeed(0);
    },{m_intake, m_grabber}).ToPtr()
  );

  // Tips over cone inside robot while held
  frc2::Trigger coneCorrectButton{m_ted.Button(PS5_BUTTON_MENU)};
  coneCorrectButton.WhileTrue(frc2::InstantCommand([this]{m_intake->SetPower((INTAKE_ROLLER_POWER / 2), -INTAKE_CONVEYOR_POWER);},{m_intake}).ToPtr());
  coneCorrectButton.OnFalse(RunIntake(m_intake, 0, 0).ToPtr());

  // Manual control of grabber -- hold to run inwards
  frc2::Trigger grabButton{m_ted.Button(PS5_BUTTON_LTRIGGER)};
  grabButton.WhileTrue(RunGrabber(m_grabber, GrabberAction::Grab).ToPtr());
  grabButton.OnFalse(RunGrabber(m_grabber, 0).ToPtr());

  // Manual control of grabber -- hold to run outwards
  frc2::Trigger shootButton{m_ted.Button(PS5_BUTTON_RTRIGGER)};
  shootButton.WhileTrue(RunGrabber(m_grabber, GrabberAction::Shoot).ToPtr());
  shootButton.OnFalse(RunGrabber(m_grabber, 0).ToPtr());

  // Stops all co-driver subsystems when pressed -- panic button
  frc2::Trigger codriverStopButton{m_ted.Button(PS5_BUTTON_PS)};
  codriverStopButton.OnTrue((frc2::InstantCommand([this]{
    m_intake->SetPower(0, 0);
    m_elevator->Drive(0);
    m_grabber->SetSpeed(0);
  },{m_intake, m_grabber, m_elevator}).ToPtr()));

  // Shoots the elevator to max height and fires grabber on the way up when pressed
  frc2::Trigger ultraShootButton{m_ted.Button(PS5_BUTTON_RBUMPER)};
  ultraShootButton.OnTrue(UltraShoot(m_elevator, m_intake, m_grabber).ToPtr());

  // Toggles the mode of the robot from cube to cone or vice versa
  // Changes functionality of other commands based on whether handling cubes or cones
  // E.g.: elevator will move to different heights depending on cube or cone
  frc::SmartDashboard::PutBoolean("Is Cone Mode?", m_isConeMode);
  frc2::Trigger pieceModeToggleButton{m_ted.Button(PS5_BUTTON_TOUCHPAD)};
  pieceModeToggleButton.OnTrue(frc2::InstantCommand([this]{ m_isConeMode = !m_isConeMode; frc::SmartDashboard::PutBoolean("Is Cone Mode?", m_isConeMode); }, {}).ToPtr());

  // ---------------------Test's controls----------------------
  // If test controller is connected, takes controls
  if (m_test.IsConnected()) {
    // Takes raw control from co-driver
    m_elevator->SetDefaultCommand(ElevatorRawDrive(m_elevator, m_grabber, m_intake, &m_test));

    // Hold down to spin all modules forward at constant speed
    frc2::Trigger forwardSpeedTestButton{m_test.Button(PS5_BUTTON_TRI)};
    forwardSpeedTestButton.WhileTrue(
      frc2::RunCommand([this]{
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::fl, 0.5);
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::fr, 0.5);
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::bl, 0.5);
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::br, 0.5);
      },{m_swerve}).ToPtr()
    );

    // Hold down to spin all modules backward at constant speed
    frc2::Trigger backwardSpeedTestButton{m_test.Button(PS5_BUTTON_X)};
    backwardSpeedTestButton.WhileTrue(
      frc2::RunCommand([this]{
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::fl, -0.5);
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::fr, -0.5);
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::bl, -0.5);
        m_swerve->ManualModuleSpeed(SwerveModuleLocation::br, -0.5);
      },{m_swerve}).ToPtr()
    );

    // Hold down to rotate all modules clockwise at constant speed
    frc2::Trigger turnCWTestButton{m_test.Button(PS5_BUTTON_SQR)};
    turnCWTestButton.WhileTrue(
      frc2::RunCommand([this]{
        m_swerve->ManualModuleTurn(SwerveModuleLocation::fl, 0.5);
        m_swerve->ManualModuleTurn(SwerveModuleLocation::fr, 0.5);
        m_swerve->ManualModuleTurn(SwerveModuleLocation::bl, 0.5);
        m_swerve->ManualModuleTurn(SwerveModuleLocation::br, 0.5);
      },{m_swerve}).ToPtr()
    );

    // Hold down to rotate all modules counter-clockwise at constant speed
    frc2::Trigger turnCCWTestButton{m_test.Button(PS5_BUTTON_O)};
    turnCCWTestButton.WhileTrue(
      frc2::RunCommand([this]{
        m_swerve->ManualModuleTurn(SwerveModuleLocation::fl, -0.5);
        m_swerve->ManualModuleTurn(SwerveModuleLocation::fr, -0.5);
        m_swerve->ManualModuleTurn(SwerveModuleLocation::bl, -0.5);
        m_swerve->ManualModuleTurn(SwerveModuleLocation::br, -0.5);
      },{m_swerve}).ToPtr()
    );

    // Press to toggle intake pistons in or out
    frc2::Trigger toggleIntakePistonButton{m_test.Button(PS5_BUTTON_TOUCHPAD)};
    toggleIntakePistonButton.OnTrue(
      frc2::SequentialCommandGroup(
        ToggleIntakePistons(m_intake),
        frc2::WaitCommand(0.5_s),
        RunIntake(m_intake, 0, 0)
      ).ToPtr()
    );

    frc2::Trigger runIntakeInButton{m_test.Button(PS5_BUTTON_LBUMPER)};
    runIntakeInButton.WhileTrue(
      RunIntake(m_intake, INTAKE_ROLLER_POWER, 0).ToPtr()
    );
    runIntakeInButton.OnFalse(
      RunIntake(m_intake, 0, 0).ToPtr()
    );

    frc2::Trigger runIntakeOutButton{m_test.Button(PS5_BUTTON_RBUMPER)};
    runIntakeOutButton.WhileTrue(
      RunIntake(m_intake, OUTTAKE_ROLLER_POWER, 0).ToPtr()
    );
    runIntakeOutButton.OnFalse(
      RunIntake(m_intake, 0, 0).ToPtr()
    );

    frc2::Trigger runConveyorInButton{m_test.Button(PS5_BUTTON_CREATE)};
    runConveyorInButton.WhileTrue(
      RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER).ToPtr()
    );
    runConveyorInButton.OnFalse(
      RunIntake(m_intake, 0, 0).ToPtr()
    );

    frc2::Trigger runConveyorOutButton{m_test.Button(PS5_BUTTON_MENU)};
    runConveyorOutButton.WhileTrue(
      RunIntake(m_intake, 0, OUTTAKE_CONVEYOR_POWER).ToPtr()
    );
    runConveyorOutButton.OnFalse(
      RunIntake(m_intake, 0, 0).ToPtr()
    );

    frc2::Trigger runGrabberInButton{m_test.Button(PS5_BUTTON_LTRIGGER)};
    runGrabberInButton.WhileTrue(
      RunGrabber(m_grabber, GRABBER_CONE_SHOOT_SPEED).ToPtr()
    );
    runGrabberInButton.OnFalse(
      RunGrabber(m_grabber, 0).ToPtr()
    );

    frc2::Trigger runGrabberOutButton{m_test.Button(PS5_BUTTON_RTRIGGER)};
    runGrabberOutButton.WhileTrue(
      RunGrabber(m_grabber, -GRABBER_CONE_SHOOT_SPEED).ToPtr()
    );
    runGrabberOutButton.OnFalse(
      RunGrabber(m_grabber, 0).ToPtr()
    );
  }
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
  * Mostly uses copies of teleop commands, with slight modifications
  */ 

  // --------------------------------START OF AUTO EVENTS---------------------------------

  AutoParameters::eventMap.emplace(
    "intake_pistons_out", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::InstantCommand([this]{ m_intake->SetPistonExtension(true);},{m_intake}),
      frc2::ParallelDeadlineGroup(
        frc2::WaitCommand(0.75_s)
        // RunIntake(m_intake, -INTAKE_ROLLER_POWER, -INTAKE_CONVEYOR_POWER)
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
        ElevatorPID(m_elevator, ElevatorLevel::Low, false),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(.5_s), 
          RunGrabber(m_grabber, GrabberAction::Shoot)
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "score_mid", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        ElevatorPID(m_elevator, ElevatorLevel::Mid, false),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(.5_s), 
          RunGrabber(m_grabber, GrabberAction::Shoot)
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "score_high", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        ElevatorPID(m_elevator, ElevatorLevel::High, false),
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(.5_s), 
          RunGrabber(m_grabber, GrabberAction::Shoot)
        ),
        frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      )
    )
  );
  
  AutoParameters::eventMap.emplace(
    "elevator_to_0", 
    std::make_shared<frc2::ParallelRaceGroup>(
      frc2::WaitCommand(1.0_s),
      ElevatorPID(m_elevator, 0, false)
    )
  );

  AutoParameters::eventMap.emplace(
    "grab", 
    std::make_shared<frc2::SequentialCommandGroup>(
      frc2::SequentialCommandGroup(
        frc2::ParallelDeadlineGroup(
          frc2::WaitCommand(2.0_s),
          RunGrabber(m_grabber, GRABBER_INTERIOR_GRAB_SPEED / 1.75),
          RunIntake(m_intake, 0, INTAKE_CONVEYOR_POWER)
        ),
        frc2::InstantCommand([this]{
          m_intake->SetPower(0, 0);
          m_grabber->SetSpeed(0);
        },{m_intake, m_grabber})
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "run_intake_in", 
    std::make_shared<frc2::ParallelDeadlineGroup>(
      frc2::ParallelDeadlineGroup(
        RunIntake(m_intake, INTAKE_ROLLER_POWER, INTAKE_CONVEYOR_POWER),
        RunGrabber(m_grabber, GRABBER_CUBE_GRAB_SPEED)
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "run_intake_out", 
    std::make_shared<frc2::ParallelDeadlineGroup>(
      frc2::ParallelDeadlineGroup(
        RunIntake(m_intake, OUTTAKE_ROLLER_POWER, OUTTAKE_CONVEYOR_POWER),
        RunGrabber(m_grabber, GRABBER_OUTTAKE_SPEED)
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "ultra_shoot", 
    std::make_shared<frc2::SequentialCommandGroup>(
      // frc2::SequentialCommandGroup(
      //   frc2::ParallelDeadlineGroup(
      //     ElevatorPID(m_elevator, ELEVATOR_ULTRA_SHOOT_TARGET, false),
      //     frc2::RunCommand([this]{
      //       m_swerve->LockWheels();
      //       if (m_elevator->GetPosition() > ELEVATOR_ULTRA_SHOOT_RELEASE_POINT) {
      //         m_grabber->SetSpeed(ELEVATOR_ULTRA_SHOOT_POWER);
      //       }
      //     },{m_swerve, m_grabber})
      //   ),
      //   frc2::InstantCommand([this]{m_grabber->SetSpeed(0);},{m_grabber})
      // )
    )
  );

  AutoParameters::eventMap.emplace(
    "stop_intake", 
    std::make_shared<RunIntake>(
      m_intake, 0, 0
    )
  );

  AutoParameters::eventMap.emplace(
    "balance",
    std::make_shared<frc2::ParallelRaceGroup>(
      frc2::ParallelRaceGroup(
        PIDAutoBalance(m_swerve)
      )
    )
  );

  AutoParameters::eventMap.emplace(
    "lock_wheels",
    std::make_shared<frc2::ParallelDeadlineGroup>(
      frc2::WaitCommand(5.0_s),
      frc2::RunCommand([this]{m_swerve->LockWheels();},{m_swerve})
    )
  );

  AutoParameters::eventMap.emplace(
    "set_mode_cube",
    std::make_shared<frc2::InstantCommand>(
      frc2::InstantCommand([this]{frc::SmartDashboard::PutBoolean("Is Cone Mode?", false);},{})
    )
  );

  AutoParameters::eventMap.emplace(
    "set_mode_cone",
    std::make_shared<frc2::InstantCommand>(
      frc2::InstantCommand([this]{frc::SmartDashboard::PutBoolean("Is Cone Mode?", true);},{})
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
    [this]() { return m_swerve->GetEstimatedPose(); }, // Function to supply current robot pose
    // Pathplanner resets odometry to 2 meters higher than it should be
    [this](auto initPose) { m_swerve->SetCurrentPose(initPose); std::cout << "Reset odometry from PPLib\n"; },
    // [this](auto initPose) { m_swerve->SetCurrentPose(initPose.TransformBy(frc::Transform2d{frc::Translation2d{0.0_m, -2.0_m}, frc::Rotation2d{0_deg}})); }, // Function used to reset odometry at the beginning of auto
    pathplanner::PIDConstants(AutoConstants::autoKP, AutoConstants::autoKI, AutoConstants::autoKD), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    pathplanner::PIDConstants(AutoConstants::autoRotP, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    [this](auto speeds) { m_swerve->PercentDrive(speeds); }, // Output function that accepts field relative ChassisSpeeds
    AutoParameters::eventMap, // Our event map
    { m_swerve }, // Drive requirements, usually just a single drive subsystem
    true // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
  );
  
  // Creates instances of each auto command
  // Add auto commands to auto chooser
  m_chooser.SetDefaultOption("N/A", nullptr);
  m_chooser.AddOption("Test: Line", new TestLineAuto(m_swerve, m_autoBuilder, "Test - Line"));
  m_chooser.AddOption("Test - S", new TestSAuto(m_swerve, m_autoBuilder, "Test - S"));
  m_chooser.AddOption("Test - Line + Rotate", new TestLineRotateAuto(m_swerve, m_autoBuilder, "Test - Line + Rotate"));
  m_chooser.AddOption("Test - Line + Intake", new TestLineIntakeAuto(m_swerve, m_autoBuilder, "Test - Line + Intake"));
  m_chooser.AddOption("Test - S + Rotate", new TestSRotateAuto(m_swerve, m_autoBuilder, "Test - S + Rotate"));
  m_chooser.AddOption("Test - Balance", new TestBalance(m_swerve, m_autoBuilder, "Test - Balance"));
  m_chooser.AddOption("Special", new SpecialAuto(m_swerve, m_autoBuilder, "Special"));
  m_chooser.AddOption("Figure Eight", new FigureEightAuto(m_swerve, m_autoBuilder, "Figure Eight"));
  m_chooser.AddOption("Outtake", new Outtake(m_swerve, m_intake));
  m_chooser.AddOption("One Score: High Cube + Taxi", new OneScoreHighCubeTaxiAuto(m_swerve, m_autoBuilder, "One Score + Taxi"));
  m_chooser.AddOption("One Score: High Cone + Taxi", new OneScoreTaxiCone(m_swerve, m_autoBuilder, "One Score + Taxi Cone"));
  m_chooser.AddOption("One Score: High Cube + Balance", new OneScoreHighCubeBalanceAuto(m_swerve, m_autoBuilder, "One Score + Balance"));
  m_chooser.AddOption("One Score: High Cube + Pickup", new OneScorePickupCube(m_swerve, m_autoBuilder, "One Score + Pickup"));
  m_chooser.AddOption("One Score: High Cube + Pickup Bump Side", new OneScoreCubeBump(m_swerve, m_autoBuilder, "One Score + Pickup Bump"));
  m_chooser.AddOption("Two Score: Center Cubes + Ultrashoot", new TwoScoreCenter(m_swerve, m_autoBuilder, "Two Score Center Balance + Ultrashoot"));
  m_chooser.AddOption("Two Score: High/Mid Cubes + Pickup", new TwoScoreHighMidCubeAuto(m_swerve, m_autoBuilder, "Two Score High-Mid Cube + Pickup"));
  m_chooser.AddOption("Two Score: High Cone + High Cube + Pickup", new TwoScoreHighConeCubePickup(m_swerve, m_autoBuilder, "Two Score Cone + Cube + Pickup"));
  m_chooser.AddOption("Two Score: High/Mid Cubes Bump Side", new TwoScoreHighMidCubeBump(m_swerve, m_autoBuilder, "Two Score High-Mid Cube Bump"));
  m_chooser.AddOption("Two Score: High Cube + Balance + Ultrashoot", new TwoScoreHighCubeUltrashootAuto(m_swerve, m_autoBuilder, "One Score + Pickup + Balance + Ultrashoot"));
  m_chooser.AddOption("Two Score: Wide Sweep High/Mid Cubes", new TwoScoreWideSweepHighMidCubeAuto(m_swerve, m_autoBuilder, "Two Score Wide Sweep"));
  m_chooser.AddOption("Three Score: Ultrashoot", new ThreeScore(m_swerve, m_autoBuilder, "Three Score"));
  m_chooser.AddOption("Five Score", new FiveScoreAuto(m_swerve, m_autoBuilder, "Five Score"));
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
  m_swerve->SetCurrentPose(frc::Pose2d{0.0_m, 0.0_m, frc::Rotation2d{0.0_deg}});
}

void RobotContainer::SetSwerveBrake() {
  m_swerve->SetModulesBrake();
}

void RobotContainer::SetSwerveCoast() {
  m_swerve->SetModulesCoast();
}