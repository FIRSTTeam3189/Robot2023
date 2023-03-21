// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>
#include <frc/controller/ElevatorFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Timer.h>
#include "Constants.h"
#include <iostream>

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
  double ElevatorTicksToMeters(double encoderTicks);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_motor;
  // WPI_TalonFX m_motor;

  // rev::SparkMaxAbsoluteEncoder m_encoder;
  rev::SparkMaxRelativeEncoder m_encoder;
  // rev::SparkMaxAlternateEncoder m_encoder;
  // rev::SparkMaxPIDController m_PIDcontroller;
  frc::TrapezoidProfile<units::meters>::Constraints m_constraints{ELEVATOR_MAX_SPEED, ELEVATOR_MAX_ACCELERATION};
  frc::ProfiledPIDController<units::meters> m_controller{ELEVATOR_P, ELEVATOR_I, ELEVATOR_D, m_constraints, 20_ms};
  frc::ElevatorFeedforward ff{ELEVATOR_KS, ELEVATOR_KG, ELEVATOR_KV, ELEVATOR_KA};
  // frc::DigitalInput m_lowerLimitSwitch;
  // frc::DigitalInput m_upperLimitSwitch;
  rev::SparkMaxLimitSwitch m_lowerLimitSwitch;
  rev::SparkMaxLimitSwitch m_upperLimitSwitch;
  bool m_atSetpoint;
  double m_target;
  double m_power;
  units::meters_per_second_t m_lastSpeed;
  units::second_t m_lastTime;
};
