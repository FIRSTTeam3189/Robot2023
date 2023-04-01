// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/RamseteCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>

#include "Autos/TestAuto.h"
#include "Autos/OneCargo.h"
#include "Autos/Balance.h"
#include "Autos/OneCargoBalance.h"
#include "Autos/OneCargoPickupBalance.h"
#include "Autos/OneCargoPickupOne.h"
#include "Autos/Outtake.h"
#include "Autos/TwoCargo.h"
#include "Autos/TwoPieceWithVision.h"
#include "Autos/TwoCargoUltrashootBalance.h"

#include "commands/SingleModTest.h"
#include "commands/OISwerveDrive.h"
#include "commands/ResetOdometry.h"
#include "commands/ResetEncodersToAbsolute.h"
#include "commands/UpdatePIDValues.h"
#include "commands/AutoBalance.h"
#include "commands/RotateTo.h"
#include "commands/SlowTranslate.h"
#include "commands/AimAssist.h"
#include "commands/ElevatorRawDrive.h"
#include "commands/UltraShoot.h"
#include "commands/DriveToPose.h"
#include "commands/TrajectoryAimAssist.h"

#include "subsystems/SwerveDrive.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Grabber.h"

#include <iostream>
#include "subsystems/Vision.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();
  void Sync();
  void ResetGyroscope();
  void CheckPOV();

 private:
  // The robot's subsystems and commands are defined here...
  Vision *m_vision = new Vision();
  SwerveDrive *m_swerve = new SwerveDrive();
  Elevator *m_elevator = new Elevator();
  Grabber *m_grabber = new Grabber();
  Intake *m_intake = new Intake();

  // Add autonomous routines to chooser
  TestAuto m_testAuto1{m_swerve, m_elevator, m_intake, m_grabber, 1};
  TestAuto m_testAuto2{m_swerve, m_elevator, m_intake, m_grabber, 2};
  TestAuto m_testAuto3{m_swerve, m_elevator, m_intake, m_grabber, 3};
  TestAuto m_testAuto4{m_swerve, m_elevator, m_intake, m_grabber, 4};
  TestAuto m_testAuto5{m_swerve, m_elevator, m_intake, m_grabber, 5};
  Outtake m_outtake{m_intake, m_swerve};
  Balance m_balance{m_swerve};
  OneCargo m_oneCargoHigh{m_swerve, m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET};
  OneCargo m_oneCargoMid{m_swerve, m_elevator, m_grabber, m_intake, ELEVATOR_MID_TARGET};
  OneCargoBalance m_oneCargoHighBalance{m_swerve, m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET};
  OneCargoBalance m_oneCargoMidBalance{m_swerve, m_elevator, m_grabber, m_intake, ELEVATOR_MID_TARGET};
  OneCargoPickupBalance m_oneCargoHighPickupBalanceRed{m_swerve, m_elevator, m_grabber, m_intake, true, ELEVATOR_HIGH_TARGET};
  OneCargoPickupBalance m_oneCargoHighPickupBalanceBlue{m_swerve, m_elevator, m_grabber, m_intake, false, ELEVATOR_HIGH_TARGET};
  OneCargoPickupBalance m_oneCargoMidPickupBalanceRed{m_swerve, m_elevator, m_grabber, m_intake, true, ELEVATOR_MID_TARGET};
  OneCargoPickupBalance m_oneCargoMidPickupBalanceBlue{m_swerve, m_elevator, m_grabber, m_intake, false, ELEVATOR_MID_TARGET};
  OneCargoPickupOne m_oneCargoHighPickupOne{m_swerve, m_elevator, m_grabber, m_intake, ELEVATOR_HIGH_TARGET};
  OneCargoPickupOne m_oneCargoMidPickupOne{m_swerve, m_elevator, m_grabber, m_intake, ELEVATOR_MID_TARGET};
  TwoCargo m_twoCargo{m_swerve, m_elevator, m_grabber, m_intake};
  // TwoPieceWithVision m_twoPieceWithVision{m_swerve, m_elevator, m_grabber, m_intake, m_vision};
  TwoCargoUltrashootBalance m_twoCargoRedUltrashoot{m_swerve, m_elevator, m_grabber, m_intake, true};
  TwoCargoUltrashootBalance m_twoCargoBlueUltrashoot{m_swerve, m_elevator, m_grabber, m_intake, false};

  frc::SendableChooser<frc2::Command*> m_chooser;

  // Bill controls drivetrain + intake etc. (lower half)
  // Ted controls output + aiming etc. (upper half)
  // frc::Joystick m_ted{1};

  frc2::CommandJoystick m_bill{PS5_BILL_CONTROLLER_PORT};
  frc2::CommandJoystick m_ted{PS5_TED_CONTROLLER_PORT};

  // Driver's controls
  // frc2::Trigger m_frontLeftSpeedTestButton;
  // frc2::Trigger m_frontRightSpeedTestButton;
  // frc2::Trigger m_backLeftSpeedTestButton;
  // frc2::Trigger m_backRightSpeedTestButton;
  frc2::Trigger m_translateLeftButton;
  frc2::Trigger m_rotateTo0Button;
  frc2::Trigger m_translateRightButton;
  frc2::Trigger m_rotateTo180Button;
  // frc2::Trigger m_frontLeftRotTestButton;
  // frc2::Trigger m_frontRightRotTestButton;
  // frc2::Trigger m_backLeftRotTestButton;
  // frc2::Trigger m_backRightRotTestButton;
  frc2::Trigger m_resetOdometryButton;
  frc2::Trigger m_resetEncodersToAbsoluteButton;
  frc2::Trigger m_toggleATan2RotButton;
  bool m_isMagnitudeRot = false;
  frc2::Trigger m_updatePIDButton;
  frc2::Trigger m_autoBalanceButton;
  frc2::Trigger m_leftAimAssistButton;
  frc2::Trigger m_centerAimAssistButton;
  frc2::Trigger m_slideStationAimAssistButton;
  frc2::Trigger m_rightAimAssistButton;
  frc2::Trigger m_spinIntakeInButton;
  frc2::Trigger m_spinIntakeOutButton;
  frc2::Trigger m_toggleIntakePistonsDriver;
  frc2::Trigger m_driverStopButton;
  frc2::Trigger m_ultraShootButton;
  frc2::Trigger m_lockWheelsButton;

  // Co-driver's controls
  frc2::Trigger m_elevatorLowLevelButton;
  frc2::Trigger m_elevatorMidLevelButton;
  frc2::Trigger m_elevatorHighLevelButton;
  frc2::Trigger m_cancelElevatorPIDControl;
  frc2::Trigger m_runIntakeInButton;
  frc2::Trigger m_runIntakeOutButton;
  frc2::Trigger m_coneCorrectButton;
  frc2::Trigger m_shootButton;
  frc2::Trigger m_grabButton;
  frc2::Trigger m_codriverStopButton;
  frc2::Trigger m_toggleIntakePistonsButton;
  frc2::Trigger m_runConveyorButton;

  void ConfigureButtonBindings();
};
