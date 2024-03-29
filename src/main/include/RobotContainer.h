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
#include <frc2/command/ParallelCommandGroup.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RepeatCommand.h>
#include <frc/Joystick.h>
#include <frc/PowerDistribution.h>

#include "Autos/FigureEightAuto.h"
#include "Autos/FiveScoreAuto.h"
#include "Autos/OneScorePickupCube.h"
#include "Autos/OneScoreHighCubeBalanceAuto.h"
#include "Autos/OneScoreHighCubeTaxiAuto.h"
#include "Autos/OneScoreTaxiCone.h"
#include "Autos/OneScoreCubeBump.h"
#include "Autos/SpecialAuto.h"
#include "Autos/TestBalance.h"
#include "Autos/TestLineAuto.h"
#include "Autos/TestLineIntakeAuto.h"
#include "Autos/TestLineRotateAuto.h"
#include "Autos/TestSAuto.h"
#include "Autos/TestSRotateAuto.h"
#include "Autos/TwoScoreCenter.h"
#include "Autos/TwoScoreHighCubeUltrashootAuto.h"
#include "Autos/TwoScoreHighMidCubeAuto.h"
#include "Autos/TwoScoreHighConeCubePickup.h"
#include "Autos/TwoScoreHighMidCubeBump.h"
#include "Autos/TwoScoreWideSweepHighMidCubeAuto.h"
#include "Autos/ThreeScore.h"
#include "Autos/Outtake.h"

#include "commands/SingleModTest.h"
#include "commands/OISwerveDrive.h"
#include "commands/SetCurrentPose.h"
#include "commands/ResetEncodersToAbsolute.h"
#include "commands/UpdatePIDValues.h"
#include "commands/PathPlannerAutoBalance.h"
#include "commands/SimpleAutoBalance.h"
#include "commands/PIDAutoBalance.h"
#include "commands/RotateTo.h"
#include "commands/SlowTranslate.h"
#include "commands/ElevatorRawDrive.h"
#include "commands/UltraShoot.h"
#include "commands/TrajectoryAimAssist.h"
#include "commands/ToggleIntakePistons.h"
#include "commands/RunGrabber.h"

#include "subsystems/SwerveDrive.h"
#include "subsystems/Elevator.h"
#include "subsystems/Intake.h"
#include "subsystems/Grabber.h"
#include "subsystems/Vision.h"
#include "subsystems/LEDSystem.h"
#include "RobotParameters.h"

#include <pathplanner/lib/auto/SwerveAutoBuilder.h>
#include <pathplanner/lib/PathPlanner.h>
#include "subsystems/SwerveDrive.h"

#include <iostream>
#include <vector>

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
  void SetSwerveBrake();
  void SetSwerveCoast();

 private:
  // The robot's subsystems and commands are defined here...
  Vision *m_vision = new Vision();
  SwerveDrive *m_swerve = new SwerveDrive(m_vision);
  Elevator *m_elevator = new Elevator();
  Grabber *m_grabber = new Grabber();
  Intake *m_intake = new Intake();
  LEDSystem *m_LED = new LEDSystem();

  frc::SendableChooser<frc2::Command*> m_chooser;
  pathplanner::SwerveAutoBuilder *m_autoBuilder = nullptr;

  bool m_isMagnitudeRot = false;
  bool m_isFieldRelative = true;
  bool m_isConeMode = false;

  // Bill controls drivetrain + intake etc. (lower half)
  // Ted controls output + aiming etc. (upper half)
  frc2::CommandJoystick m_bill{PS5_BILL_CONTROLLER_PORT};
  frc2::CommandJoystick m_ted{PS5_TED_CONTROLLER_PORT};
  frc2::CommandJoystick m_test{PS5_TEST_CONTROLLER_PORT};

  void ConfigureButtonBindings();
  void BuildEventMap();
  void CreateAutoPaths();
};
