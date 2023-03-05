// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "util/Piston.h"
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void ToggleIntake();
  void SetPower(double rollerPower, double conveyorPower, double coneCorrectPower);
  
private:
  Piston *m_leftPiston;
  Piston *m_rightPiston;
  rev::CANSparkMax *m_rollerMotor;
  rev::CANSparkMax *m_conveyorMotor;
  rev::CANSparkMax *m_leftConeCorrectMotor;
  rev::CANSparkMax *m_rightConeCorrectMotor;
};
