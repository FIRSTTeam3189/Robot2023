// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/Phoenix.h"
#include "Constants.h"
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/util/Color.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/SuppliedValueWidget.h>

class Grabber : public frc2::SubsystemBase {
public:
  Grabber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SetSpeed(double power);

private:
  WPI_TalonFX m_motor;
  rev::ColorSensorV3 m_colorSensor;
  rev::ColorMatch m_colorMatcher;
  frc::Color m_detectedColor;
  frc::Color m_matchedColor;
  
  static constexpr frc::Color kCubeTarget = frc::Color(0.284, 0.154, 0.561);
  static constexpr frc::Color kConeTarget = frc::Color(0.586, 0.414, 0.0);
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
