// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ctre/Phoenix.h"
#include <frc/Encoder.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <numbers>
#include "rev/CANSparkMax.h"
#include <math.h>
#include <iostream>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

struct PIDValues {
  const double p;
  const double i;
  const double d;
};

struct SwerveInfo {
  const int speedMotorID;
  const int angleMotorID;
  const int CANCoderID;
  const PIDValues speedMotorPID;
  const PIDValues angleMotorPID;
  const double encoderOffset;
};

struct SwerveModuleTelemetry {
  const double speed;
  const double position;
  const double angleVelocity;
  const double speedMotorCurrent;
  const double speedMotorVoltage;
  const double angleMotorCurrent;
  const double angleMotorVoltage;
  const double absoluteAngle;
  const double relativeAngle;
};

// SysID robot characterization values -- **varies by robot**
constexpr auto ks = 0.208_V;
constexpr auto kv = 2.206 * 1_V * 1_s / 1_m;
constexpr auto ka = 0.409 * 1_V * 1_s * 1_s / 1_m;

class SwerveModule {
 public:
  SwerveModule(SwerveInfo SI);
  void Stop();
  void SetDesiredState(const frc::SwerveModuleState &state);
  void ManualModuleSpeed(double speed);
  void ManualModuleTurn(double speed);
  frc::SwerveModuleState GetState();
  double GetAbsolutePosition();
  double GetRelativeAngle();
  double GetVelocity();
  frc::SwerveModulePosition GetSwerveModulePosition();

  void DriveFast();
  void UpdateModulePosition();
  SwerveModuleTelemetry GetTelemetry();
  void ResetSpeedEncoder();
  void ResetAngleToAbsolute();
  frc::SwerveModuleState OptimizeAngle(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle);
  double NormalizeTo0To360(double currentAngle, double targetAngle);

  double DegreesToCANCoder(double degrees);
  double DegreesToFalcon(double degrees);
  double CANCoderToDegrees(double encoderTicks);
  double FalconToDegrees(double encoderTicks);
  units::meter_t FalconToMeters(double encoderTicks);

  double GetSpeedP();
  double GetSpeedI();
  double GetSpeedD();
  double GetAngleP();
  double GetAngleI();
  double GetAngleD();

  void SetSpeedP(double value);
  void SetSpeedI(double value);
  void SetSpeedD(double value);
  void SetAngleP(double value);
  void SetAngleI(double value);
  void SetAngleD(double value);

  // Motor controller for speed control
  WPI_TalonFX m_speedMotor;
  // Motor controller for angle
  WPI_TalonFX m_angleMotor;
  
 private:
    SwerveInfo SI;
    
    double speedP;
    double speedI;
    double speedD;
    double angleP;
    double angleI;
    double angleD;

    double m_angleOffset;
    double m_lastAngle;
  
    // Create swerve module position for odometry
    frc::SwerveModulePosition m_swervePosition{0_m, frc::Rotation2d{}};
    // frc::SimpleMotorFeedforward<units::meters> ff{SwerveDriveConstants::ks, SwerveDriveConstants::kv, SwerveDriveConstants::ka};
    frc::SimpleMotorFeedforward<units::meters> ff{ks, kv, ka};
    WPI_CANCoder m_absoluteEncoder; 
    units::meters_per_second_t m_lastSpeed;
    units::second_t m_lastTime;
};
