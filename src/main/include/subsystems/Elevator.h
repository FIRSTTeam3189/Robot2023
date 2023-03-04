// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>

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
  void GoToPosition(double position);
  bool AtSetpoint();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_motor;
  rev::SparkMaxAlternateEncoder m_encoder;
  rev::SparkMaxPIDController m_PIDcontroller;
  bool m_atSetpoint;
  double m_position;
};
