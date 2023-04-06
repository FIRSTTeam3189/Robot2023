// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <networktables/NTSendableBuilder.h>
#include <string>
#include <numbers>
#include <iostream>

#include "util/SwerveModule.h"
#include "RobotParameters.h"
#include "Constants.h"

// hold instances of swerve modules and translation vectors

/*
class SwerveConfig {
public:
  double speedP = SwerveDriveConstants::speedP;
  double speedI = SwerveDriveConstants::speedI;
  double speedD = SwerveDriveConstants::speedD;
  double angleP = SwerveDriveConstants::angleP;
  double angleI = SwerveDriveConstants::angleI;
  double angleD = SwerveDriveConstants::angleD;

  friend std::ostream &operator<<(std::ostream &out, const SwerveConfig &config);
  friend std::istream &operator>>(std::istream &in, SwerveConfig &config);
};
*/

enum class ManualModuleDriveType {forward, turn, stop};

enum class SwerveModuleLocation{fl, fr, bl, br};
struct SwerveModules {
  frc::Translation2d m_frontLeftLocation;
  frc::Translation2d m_frontRightLocation;
  frc::Translation2d m_backLeftLocation;
  frc::Translation2d m_backRightLocation;

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_backLeft;
  SwerveModule m_backRight;
};

class SwerveDrive : public frc2::SubsystemBase {
 public:
  SwerveDrive();

  void PercentDrive(units::meters_per_second_t xSpeed,
   units::meters_per_second_t ySpeed,
   units::radians_per_second_t rot,
   bool fieldRelative);

  void PercentDrive(frc::ChassisSpeeds speeds);

  void Drive(units::meters_per_second_t xSpeed,
   units::meters_per_second_t ySpeed,
   units::radians_per_second_t rot,
   bool fieldRelative);

  void UpdateOdometry();
  void InitSmartDashboard();
  void SyncSmartdashBoardValues();
  void UpdatePIDValues();
  void LogModuleStates(SwerveModuleTelemetry telemetryArray[]);
  void Log2DField();
  void ManualModuleSpeed(SwerveModuleLocation location, double speed);
  void ManualModuleTurn(SwerveModuleLocation location, double speed);
  void ResetSpeedEncoders();
  void ResetEncodersToAbsolute();
  void ResetOdometry(frc::Pose2d pose);
  void SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates);
  void SetPercentModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates);
  void SetActiveTrajectory(frc::Trajectory trajectory);

  double GetRobotYaw();
  void SetRobotYaw(double yaw);
  double GetPitch();
  double GetRoll();
  void ResetGyro();
  double GetNormalizedYaw();
  frc::Pose2d GetPose();
  void DriveFast();
  void LockWheels();
  frc2::SwerveControllerCommand<4> CreateSwerveCommand(frc::Trajectory trajectory);

  // Access functions for rotation PID controller
  double Calculate(double measurement, double setpoint);
  void EnableContinuousInput(double minimumInput, double maximumInput);

  // Kinematics and odometry to pass into library
  frc::SwerveDriveOdometry<4> m_odometry;
  
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void Stop();

 private:
  // Declare sensors in struct
  SwerveModules m_SM;
  
  WPI_Pigeon2 m_pigeon;

  // Create wpi::array of swerve positions
  wpi::array<frc::SwerveModulePosition, 4> m_modulePositions;
  frc::Trajectory m_activeTrajectory{};
  frc::Field2d m_fieldObject{};
  frc::Pose2d initialPose = frc::Pose2d{};
  
  // Swerve tab we are writing to
  frc::ShuffleboardTab &SwerveRotationPIDTab = frc::Shuffleboard::GetTab("Rotation PID Values");

  frc::ShuffleboardTab &SwerveFLPIDTab = frc::Shuffleboard::GetTab("Swerve Front Left PID Values");
  frc::ShuffleboardTab &SwerveFRPIDTab = frc::Shuffleboard::GetTab("Swerve Front Right PID Values");
  frc::ShuffleboardTab &SwerveBLPIDTab = frc::Shuffleboard::GetTab("Swerve Back Left PID Values");
  frc::ShuffleboardTab &SwerveBRPIDTab = frc::Shuffleboard::GetTab("Swerve Back Right PID Values");

  frc::ShuffleboardTab &SwerveSpeedTab = frc::Shuffleboard::GetTab("Swerve Speed");
  frc::ShuffleboardTab &SwervePositionTab = frc::Shuffleboard::GetTab("Swerve Position");
  frc::ShuffleboardTab &SwerveAngleVelocityTab = frc::Shuffleboard::GetTab("Swerve Angle Velocity");
  frc::ShuffleboardTab &SwerveSpeedMotorCurrentTab = frc::Shuffleboard::GetTab("Swerve Speed Motor Current");
  frc::ShuffleboardTab &SwerveSpeedMotorVoltageTab = frc::Shuffleboard::GetTab("Swerve Speed Motor Voltage");
  frc::ShuffleboardTab &SwerveAngleMotorCurrentTab = frc::Shuffleboard::GetTab("Swerve Angle Motor Current");
  frc::ShuffleboardTab &SwerveAngleMotorVoltageTab = frc::Shuffleboard::GetTab("Swerve Angle Motor Voltage");
  frc::ShuffleboardTab &SwerveAbsoluteAngleTab = frc::Shuffleboard::GetTab("Swerve Absolute Angle");
  frc::ShuffleboardTab &SwerveRelativeAngleTab = frc::Shuffleboard::GetTab("Swerve Relative Angle");

  // Swerve telemetry entries - store telemetry and write to tab
  nt::GenericEntry* EntryRotationP;
  nt::GenericEntry* EntryRotationI;
  nt::GenericEntry* EntryRotationD;
  nt::GenericEntry* EntryFrontLeftSpeedP;
  nt::GenericEntry* EntryFrontLeftSpeedI;
  nt::GenericEntry* EntryFrontLeftSpeedD;
  nt::GenericEntry* EntryFrontLeftAngleP;
  nt::GenericEntry* EntryFrontLeftAngleI;
  nt::GenericEntry* EntryFrontLeftAngleD;
  nt::GenericEntry* EntryFrontRightSpeedP;
  nt::GenericEntry* EntryFrontRightSpeedI;
  nt::GenericEntry* EntryFrontRightSpeedD;
  nt::GenericEntry* EntryFrontRightAngleP;
  nt::GenericEntry* EntryFrontRightAngleI;
  nt::GenericEntry* EntryFrontRightAngleD;
  nt::GenericEntry* EntryBackLeftSpeedP;
  nt::GenericEntry* EntryBackLeftSpeedI;
  nt::GenericEntry* EntryBackLeftSpeedD;
  nt::GenericEntry* EntryBackLeftAngleP;
  nt::GenericEntry* EntryBackLeftAngleI;
  nt::GenericEntry* EntryBackLeftAngleD;
  nt::GenericEntry* EntryBackRightSpeedP;
  nt::GenericEntry* EntryBackRightSpeedI;
  nt::GenericEntry* EntryBackRightSpeedD;
  nt::GenericEntry* EntryBackRightAngleP;
  nt::GenericEntry* EntryBackRightAngleI;
  nt::GenericEntry* EntryBackRightAngleD;

  nt::GenericEntry* EntryFrontLeftSpeed;
  nt::GenericEntry* EntryFrontLeftPosition;
  nt::GenericEntry* EntryFrontLeftAngleVelocity;
  nt::GenericEntry* EntryFrontLeftSpeedMotorCurrent;
  nt::GenericEntry* EntryFrontLeftSpeedMotorVoltage;
  nt::GenericEntry* EntryFrontLeftAngleMotorCurrent;
  nt::GenericEntry* EntryFrontLeftAngleMotorVoltage;
  nt::GenericEntry* EntryFrontLeftAbsoluteAngle;
  nt::GenericEntry* EntryFrontLeftRelativeAngle;
  nt::GenericEntry* EntryFrontRightSpeed;
  nt::GenericEntry* EntryFrontRightPosition;
  nt::GenericEntry* EntryFrontRightAngleVelocity;
  nt::GenericEntry* EntryFrontRightSpeedMotorCurrent;
  nt::GenericEntry* EntryFrontRightSpeedMotorVoltage;
  nt::GenericEntry* EntryFrontRightAngleMotorCurrent;
  nt::GenericEntry* EntryFrontRightAngleMotorVoltage;
  nt::GenericEntry* EntryFrontRightAbsoluteAngle;
  nt::GenericEntry* EntryFrontRightRelativeAngle;
  nt::GenericEntry* EntryBackLeftSpeed;
  nt::GenericEntry* EntryBackLeftPosition;
  nt::GenericEntry* EntryBackLeftAngleVelocity;
  nt::GenericEntry* EntryBackLeftSpeedMotorCurrent;
  nt::GenericEntry* EntryBackLeftSpeedMotorVoltage;
  nt::GenericEntry* EntryBackLeftAngleMotorCurrent;
  nt::GenericEntry* EntryBackLeftAngleMotorVoltage;
  nt::GenericEntry* EntryBackLeftAbsoluteAngle;
  nt::GenericEntry* EntryBackLeftRelativeAngle;
  nt::GenericEntry* EntryBackRightSpeed;
  nt::GenericEntry* EntryBackRightPosition;
  nt::GenericEntry* EntryBackRightAngleVelocity;
  nt::GenericEntry* EntryBackRightSpeedMotorCurrent;
  nt::GenericEntry* EntryBackRightSpeedMotorVoltage;
  nt::GenericEntry* EntryBackRightAngleMotorCurrent;
  nt::GenericEntry* EntryBackRightAngleMotorVoltage;
  nt::GenericEntry* EntryBackRightAbsoluteAngle;
  nt::GenericEntry* EntryBackRightRelativeAngle;

  double GetEntryRotationP();
  double GetEntryRotationI();
  double GetEntryRotationD();
  double GetEntryFLSpeedP();
  double GetEntryFLSpeedI();
  double GetEntryFLSpeedD();
  double GetEntryFLAngleP();
  double GetEntryFLAngleI();
  double GetEntryFLAngleD();
  double GetEntryFRSpeedP();
  double GetEntryFRSpeedI();
  double GetEntryFRSpeedD();
  double GetEntryFRAngleP();
  double GetEntryFRAngleI();
  double GetEntryFRAngleD();
  double GetEntryBLSpeedP();
  double GetEntryBLSpeedI();
  double GetEntryBLSpeedD();
  double GetEntryBLAngleP();
  double GetEntryBLAngleI();
  double GetEntryBLAngleD();
  double GetEntryBRSpeedP();
  double GetEntryBRSpeedI();
  double GetEntryBRSpeedD();
  double GetEntryBRAngleP();
  double GetEntryBRAngleI();
  double GetEntryBRAngleD();
  
  std::function<double(void)> GetEntryRotationPFunc();
  std::function<double(void)> GetEntryRotationIFunc();
  std::function<double(void)> GetEntryRotationDFunc();

  std::function<double(void)> GetEntryFLSpeedIFunc();
  std::function<double(void)> GetEntryFLSpeedDFunc();
  std::function<double(void)> GetEntryFLSpeedPFunc();
  std::function<double(void)> GetEntryFLAnglePFunc();
  std::function<double(void)> GetEntryFLAngleIFunc();
  std::function<double(void)> GetEntryFLAngleDFunc();

  std::function<double(void)> GetEntryFRSpeedPFunc();
  std::function<double(void)> GetEntryFRSpeedIFunc();
  std::function<double(void)> GetEntryFRSpeedDFunc();
  std::function<double(void)> GetEntryFRAnglePFunc();
  std::function<double(void)> GetEntryFRAngleIFunc();
  std::function<double(void)> GetEntryFRAngleDFunc();

  std::function<double(void)> GetEntryBLSpeedPFunc();
  std::function<double(void)> GetEntryBLSpeedIFunc();
  std::function<double(void)> GetEntryBLSpeedDFunc();
  std::function<double(void)> GetEntryBLAnglePFunc();
  std::function<double(void)> GetEntryBLAngleIFunc();
  std::function<double(void)> GetEntryBLAngleDFunc();

  std::function<double(void)> GetEntryBRSpeedPFunc();
  std::function<double(void)> GetEntryBRSpeedIFunc();
  std::function<double(void)> GetEntryBRSpeedDFunc();
  std::function<double(void)> GetEntryBRAnglePFunc();
  std::function<double(void)> GetEntryBRAngleIFunc();
  std::function<double(void)> GetEntryBRAngleDFunc();
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};