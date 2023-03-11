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
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/Joystick.h>

#include "Autos/TestAuto.h"
#include "Autos/OneCargo.h"
#include "Autos/OneCargoBalance.h"
#include "Autos/OneCargoPickupBalance.h"
#include "Autos/OneCargoPickupOne.h"
#include "Autos/TwoCargo.h"
#include "Autos/TwoPieceWithVision.h"

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

 private:
  // The robot's subsystems and commands are defined here...
  Vision *m_vision = new Vision();
  SwerveDrive *m_swerve = new SwerveDrive();
  Elevator *m_elevator = new Elevator();
  Grabber *m_grabber = new Grabber();
  Intake *m_intake = new Intake();

  // Add autonomous routines to chooser
  TestAuto m_TestAuto{m_swerve};
  OneCargo m_oneCargo{m_elevator, m_grabber};
  OneCargoBalance m_oneCargoBalance{m_swerve, m_elevator, m_grabber};
  OneCargoPickupBalance m_oneCargoPickupBalanceRed{m_swerve, m_elevator, m_grabber, m_intake, true};
  OneCargoPickupBalance m_oneCargoPickupBalanceBlue{m_swerve, m_elevator, m_grabber, m_intake, false};
  OneCargoPickupOne m_oneCargoPickupOne{m_swerve, m_elevator, m_grabber, m_intake};
  TwoCargo m_twoCargo{m_swerve, m_elevator, m_grabber, m_intake};
  TwoPieceWithVision m_twoPieceWithVision{m_swerve, m_elevator, m_grabber, m_intake, m_vision};

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
  frc2::Trigger m_updatePIDButton;
  frc2::Trigger m_autoBalanceButton;
  // frc2::Trigger m_leftAimAssistButton;
  frc2::Trigger m_centerAimAssistButton;
  // frc2::Trigger m_rightAimAssistButton;
  frc2::Trigger m_spinIntakeInButton;
  frc2::Trigger m_spinIntakeOutButton;
  frc2::Trigger m_toggleIntakePistonsDriver;
  frc2::Trigger m_leftTranslateTrajectoryButton;
  frc2::Trigger m_rightTranslateTrajectoryButton;
  frc2::Trigger m_driverStopButton;

  // Co-driver's controls
  frc2::Trigger m_elevatorLowLevelButton;
  frc2::Trigger m_elevatorMidLevelButton;
  frc2::Trigger m_elevatorHighLevelButton;
  frc2::Trigger m_cancelElevatorPIDControl;
  frc2::Trigger m_toggleIntakePistonsButton;
  frc2::Trigger m_shootButton;
  frc2::Trigger m_grabButton;
  frc2::Trigger m_codriverStopButton;

  void ConfigureButtonBindings();
};
