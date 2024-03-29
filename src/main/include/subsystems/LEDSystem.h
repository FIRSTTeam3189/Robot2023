// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <ctre/phoenix/led/ColorFlowAnimation.h>
#include <ctre/phoenix/led/FireAnimation.h>
#include <ctre/phoenix/led/LarsonAnimation.h>
#include <ctre/phoenix/led/RainbowAnimation.h>
#include <ctre/phoenix/led/RgbFadeAnimation.h>
#include <ctre/phoenix/led/SingleFadeAnimation.h>
#include <ctre/phoenix/led/StrobeAnimation.h>
#include <ctre/phoenix/led/TwinkleAnimation.h>
#include <ctre/phoenix/led/TwinkleOffAnimation.h>

#include "Constants.h"

#include <map>
// #include <

enum class LEDAnimationType { Clear, ColorFlow, Fire, Larson, Rainbow, RGBFade, SingleFade, Strobe, Twinkle, TwinkleOff };
enum class LEDSection { 
  All, Candle, BackStrip, RFrontStrip, RFrontElevatorStrip, 
  RBackElevatorStrip, RUnderGlow, LUnderGlow, LBackElevatorStrip, LFrontElevatorStrip, LFrontStrip,
  LSide, RSide, Backside
};

class LEDSystem : public frc2::SubsystemBase {
 public:
  LEDSystem();
  // 
  void SetAnimation(LEDAnimationType animation, LEDSection section = LEDSection::All, int r = 0, int g = 0, int b = 0, double speed = 0.7, bool reverse = false, int animSlot = 0);
  void SetColor(int r, int g, int b, LEDSection section = LEDSection::All);
  void ClearColor(LEDSection section = LEDSection::All);
  void ClearAll();
  void StartingAnimation();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix::led::CANdle m_candleControl;
  CANdleConfiguration m_candleConfig;
  ctre::phoenix::led::Animation *m_animation;
  std::map<LEDSection, std::pair<uint8_t, uint8_t>> m_ledSections;
  bool m_grabberAnimationRunning;
  bool m_modeShouldChangeColor;
  bool m_lastMode;
  bool m_shouldStartup;
  bool m_startupRunning;
  bool m_lastEnableState{false};
  frc::Timer m_timer{};
  std::array<int, 3> m_currentModeRGB;
};
