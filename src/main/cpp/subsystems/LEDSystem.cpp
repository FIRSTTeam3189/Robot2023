// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSystem.h"

LEDSystem::LEDSystem() : m_candleControl(CANDLE_DEVICE_ID), m_candleConfig(), m_animation(NULL), m_ledSections(), m_grabberAnimationRunning(false),
m_modeShouldChangeColor(true), m_lastMode(true), m_shouldStartup(true), m_startupRunning(false), m_currentModeRGB({0, 0, 0}) {
    m_candleConfig.stripType = ctre::phoenix::led::LEDStripType::RGB;
    m_candleConfig.brightnessScalar = LED_BRIGHTNESS;
    m_candleControl.ConfigAllSettings(m_candleConfig);
    // The Full Setup
    m_ledSections[LEDSection::All] = {0, 152};
    // Candle length 8
    m_ledSections[LEDSection::Candle] = {0, 8};
    // length 11
    m_ledSections[LEDSection::BackStrip] = {8, 19};
    // length 15
    m_ledSections[LEDSection::RFrontStrip] = {19, 34};
    // length 10
    m_ledSections[LEDSection::RFrontElevatorStrip] = {34, 43};
    // length 19
    m_ledSections[LEDSection::RBackElevatorStrip] = {43, 62};
    // length 24
    m_ledSections[LEDSection::RUnderGlow] = {62, 86};
    // length 25
    m_ledSections[LEDSection::LUnderGlow] = {86, 111};
    // length 19
    m_ledSections[LEDSection::LBackElevatorStrip] = {111, 130};
    // length 10
    m_ledSections[LEDSection::LFrontElevatorStrip] = {130, 140};
    // length 13
    m_ledSections[LEDSection::LFrontStrip] = {140, 153};
    // length of right side
    m_ledSections[LEDSection::RSide] = {19, 61};
    // length of left side
    m_ledSections[LEDSection::LSide] = {111, 153};
    // length of back side
    m_ledSections[LEDSection::Backside] = {0, 19};
    StartingAnimation();
}

// This method will be called once per scheduler run
void LEDSystem::Periodic() {
    bool grabbed = frc::SmartDashboard::GetBoolean("Piece Grabbed", false);
    bool isConeMode = frc::SmartDashboard::GetBoolean("Is Cone Mode?", false);

    if (isConeMode) {
        m_currentModeRGB = {150, 100, 0};
    } else {
        m_currentModeRGB = {150, 0, 150};
    }

    if (isConeMode != m_lastMode) {
        m_modeShouldChangeColor = true;
    }

    if (grabbed && !m_grabberAnimationRunning) {
        SetAnimation(LEDAnimationType::Clear);
        SetAnimation(LEDAnimationType::Larson, LEDSection::LSide, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.75, false, 0);
        SetAnimation(LEDAnimationType::Larson, LEDSection::RSide, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.75, true, 1);
        SetAnimation(LEDAnimationType::Larson, LEDSection::Backside, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.4, true, 2);
        SetAnimation(LEDAnimationType::SingleFade, LEDSection::LUnderGlow, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.65, false, 3);
        SetAnimation(LEDAnimationType::SingleFade, LEDSection::RUnderGlow, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.65, false, 4);

        m_grabberAnimationRunning = true;
    } else if (m_grabberAnimationRunning && !grabbed) {
        SetAnimation(LEDAnimationType::Clear);
        m_grabberAnimationRunning = false;
    }

    if (m_modeShouldChangeColor) {
        if (!grabbed) {
            SetAnimation(LEDAnimationType::SingleFade, LEDSection::All, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.65, false, 5);
        } else {
            SetAnimation(LEDAnimationType::SingleFade, LEDSection::LUnderGlow, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.65, false, 3);
            SetAnimation(LEDAnimationType::SingleFade, LEDSection::RUnderGlow, m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], 0.65, false, 4);
        }
        m_modeShouldChangeColor = false;
    }

    
    m_lastMode = isConeMode;
}

void LEDSystem::SetAnimation(LEDAnimationType newAnimation, LEDSection section, int r, int g, int b, double speed, bool reverse, int animSlot) {
    auto len = m_ledSections[section].second - m_ledSections[section].first;
    auto cfDir = ColorFlowAnimation::Direction::Forward;
    if (reverse) {
        cfDir = ColorFlowAnimation::Direction::Backward;
    }
    switch (newAnimation) {
    case LEDAnimationType::ColorFlow:
        m_animation = new ColorFlowAnimation(r, g, b, 0, speed, len, cfDir, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Fire:
        m_animation = new FireAnimation(m_candleConfig.brightnessScalar, speed, len, 0.7, 0.5, reverse, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Larson:
        m_animation = new LarsonAnimation(r, g, b, 0, speed, len, LarsonAnimation::BounceMode::Front, 20, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Rainbow:
        m_animation = new RainbowAnimation(m_candleConfig.brightnessScalar, speed, len, reverse, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::RGBFade:
        m_animation = new RgbFadeAnimation(m_candleConfig.brightnessScalar, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::SingleFade:
        m_animation = new SingleFadeAnimation(r, g, b, 0, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Strobe:
        m_animation = new StrobeAnimation(r, g, b, 0, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Twinkle:
        m_animation = new TwinkleAnimation(r, g, b, 0, speed, len, TwinkleAnimation::TwinklePercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::TwinkleOff:
        m_animation = new TwinkleOffAnimation(r, g, b, 0, speed, len, TwinkleOffAnimation::TwinkleOffPercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, animSlot);
        break;
    case LEDAnimationType::Clear:
        m_candleControl.ClearAnimation(0);
        m_candleControl.ClearAnimation(1);
        m_candleControl.ClearAnimation(2);
        m_candleControl.ClearAnimation(3);
        m_candleControl.ClearAnimation(4);
        m_candleControl.ClearAnimation(5);
        m_candleControl.ClearAnimation(6);
        m_candleControl.ClearAnimation(7);
        m_candleControl.ClearAnimation(8);
        break;
    default:
        m_candleControl.ClearAnimation(0);
        m_candleControl.ClearAnimation(1);
        m_candleControl.ClearAnimation(2);
        m_candleControl.ClearAnimation(3);
        m_candleControl.ClearAnimation(4);
        m_candleControl.ClearAnimation(5);
        m_candleControl.ClearAnimation(6);
        m_candleControl.ClearAnimation(7);
        m_candleControl.ClearAnimation(8);
        break;
    }
}

void LEDSystem::SetColor(int r, int g, int b, LEDSection section) {
    m_candleControl.SetLEDs(r, g, b, 0, m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}

void LEDSystem::ClearColor(LEDSection section) {
    m_candleControl.SetLEDs(0, 0, 0, 0, m_ledSections[section].first, m_ledSections[section].second - m_ledSections[section].first);
}

void LEDSystem::ClearAll() {
    m_candleControl.ClearAnimation(0);
    m_candleControl.ClearAnimation(1);
    m_candleControl.ClearAnimation(2);
    m_candleControl.ClearAnimation(3);
    m_candleControl.ClearAnimation(4);
    m_candleControl.ClearAnimation(5);
    m_candleControl.ClearAnimation(6);
    m_candleControl.ClearAnimation(7);
    m_candleControl.ClearAnimation(8);

    m_candleControl.SetLEDs(0, 0, 0, 0, m_ledSections[LEDSection::All].first, m_ledSections[LEDSection::All].second);
}

void LEDSystem::StartingAnimation() {
    while (m_shouldStartup) {
        if (!m_startupRunning) {
            SetAnimation(LEDAnimationType::Clear);
            m_startupRunning = true;
            m_timer.Start();
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 255, 255, 0, 0.5, true);
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 0, 0, 255, 0.5, false, 1);
        }
        if (m_timer.Get() > 3.0_s) {
            m_shouldStartup = false;
            SetAnimation(LEDAnimationType::Clear);
            m_timer.Stop();
        }
    }
}
