// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "ctre/phoenix/led/RainbowAnimation.h"

#include "Constants.h"

class LED : public frc2::SubsystemBase {
 public:
  LED();
  void SetLEDs(int r, int g, int b, int startIndex, int pixelCount);
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::led::CANdle m_candle;
  CANdleConfiguration m_candleConfig;
  bool m_pieceGrabbed;
  int m_r, m_g, m_b, m_startIndex, m_pixelCount;
  ctre::phoenix::led::Animation *m_animation = NULL;
};
