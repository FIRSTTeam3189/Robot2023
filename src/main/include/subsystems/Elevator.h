// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
 public:
  Elevator();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  double GetPosition();
  void Drive(double power);
  void GoToPosition(double target);
  bool AtSetpoint();
  void SetPID(double kP, double kI, double kD);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  // Remember to switch to correct, Falcon motor
  rev::CANSparkMax m_motor;
  // WPI_TalonFX m_motor;

  rev::SparkMaxAbsoluteEncoder m_encoder;
  rev::SparkMaxPIDController m_PIDcontroller;
  frc::DigitalInput m_lowerLimitSwitch;
  frc::DigitalInput m_upperLimitSwitch;
  bool m_atSetpoint;
  double m_target;
  double m_power;
};
