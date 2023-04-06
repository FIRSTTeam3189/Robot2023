// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSystem.h"

LEDSystem::LEDSystem() : m_candleControl(CANDLE_DEVICE_ID), m_candleConfig(), m_animation(NULL), m_ledSections(), m_grabberAnimationRunning(false),
m_modeShouldChangeColor(true), m_lastMode(false), m_shouldStartup(true), m_startupRunning(false), m_currentModeRGB({0, 0, 0}) {
    m_candleConfig.stripType = ctre::phoenix::led::LEDStripType::RGB;
    m_candleConfig.brightnessScalar = LED_BRIGHTNESS;
    m_candleControl.ConfigAllSettings(m_candleConfig);
    // The Full Setup
    m_ledSections.at(LEDSection::All) = {0, 154};
    // Candle length 8
    m_ledSections.at(LEDSection::Candle) = {0, 8};
    // length 11
    m_ledSections.at(LEDSection::BackStrip) = {8, 19};
    // length 5
    m_ledSections.at(LEDSection::RIntakeCrossStrip) = {19, 24};
    //length 10
    m_ledSections.at(LEDSection::RFrontStrip) = {24, 34};
    // length 10
    m_ledSections.at(LEDSection::RFrontElevatorStrip) = {34 , 44};
    // length 19
    m_ledSections.at(LEDSection::RBackElevatorStrip) ={44, 63};
    // length 24
    m_ledSections.at(LEDSection::RUnderGlow) = {63, 87};
    // length 25
    m_ledSections.at(LEDSection::LUnderGlow) = {87, 112};
    // length 19
    m_ledSections.at(LEDSection::LBackElevatorStrip) = {112, 131};
    // length 10
    m_ledSections.at(LEDSection::LFrontElevatorStrip) = {131, 141};
    // length 7
    m_ledSections.at(LEDSection::LFrontStrip) = {141, 148};
    // length 6
    m_ledSections.at(LEDSection::LIntakeCrossStrip) = {148, 154};
    // length of right side
    m_ledSections.at(LEDSection::RSide) = {8, 87};
    // length of left side
    m_ledSections.at(LEDSection::LSide) = {86, 154};
}

// This method will be called once per scheduler run
void LEDSystem::Periodic() {
    if (m_shouldStartup) {
        StartingAnimation();
    }
    else {
        bool grabbed = frc::SmartDashboard::GetBoolean("Piece Grabbed", false);
        bool isCubeMode = frc::SmartDashboard::GetBoolean("Piece Mode", false);

        if (isCubeMode) {
            m_currentModeRGB = {150, 100, 0};
        } else {
            m_currentModeRGB = {150, 0, 150};
        }

        if (isCubeMode != m_lastMode) {
            m_modeShouldChangeColor = true;
        }

        if (grabbed) {
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::LBackElevatorStrip, 0, 150, 0, 1.0);
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::LFrontElevatorStrip, 0, 150, 0, 1.0);
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::RBackElevatorStrip, 0, 150, 0, 1.0);
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::RFrontElevatorStrip, 0, 150, 0, 1.0);
            SetAnimation(LEDAnimationType::ColorFlow, LEDSection::BackStrip, 0, 150, 0, 1.0);
            m_grabberAnimationRunning = true;
        } else if (m_grabberAnimationRunning && !grabbed) {
            SetAnimation(LEDAnimationType::Clear);
            m_grabberAnimationRunning = false;
        }

        if (m_modeShouldChangeColor) {
            SetColor(m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], LEDSection::LUnderGlow);
            SetColor(m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], LEDSection::RUnderGlow);
            SetColor(m_currentModeRGB[0], m_currentModeRGB[1], m_currentModeRGB[2], LEDSection::Candle);
            m_modeShouldChangeColor = false;
        }

        m_lastMode = isCubeMode;
    }
}

void LEDSystem::SetAnimation(LEDAnimationType newAnimation, LEDSection section, int r, int g, int b, double speed, bool reverse) {
    auto len = m_ledSections[section].second - m_ledSections[section].first;
    switch (newAnimation) {
    case LEDAnimationType::ColorFlow:
        m_animation = new ColorFlowAnimation(r, g, b, 0, speed, len, ColorFlowAnimation::Direction::Forward, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Fire:
        m_animation = new FireAnimation(m_candleConfig.brightnessScalar, speed, len, 0.7, 0.5, reverse, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Larson:
        m_animation = new LarsonAnimation(r, g, b, 0, speed, len, LarsonAnimation::BounceMode::Front, 3, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Rainbow:
        m_animation = new RainbowAnimation(m_candleConfig.brightnessScalar, speed, len, reverse, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::RGBFade:
        m_animation = new RgbFadeAnimation(m_candleConfig.brightnessScalar, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::SingleFade:
        m_animation = new SingleFadeAnimation(r, g, b, 0, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Strobe:
        m_animation = new StrobeAnimation(r, g, b, 0, speed, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Twinkle:
        m_animation = new TwinkleAnimation(r, g, b, 0, speed, len, TwinkleAnimation::TwinklePercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::TwinkleOff:
        m_animation = new TwinkleOffAnimation(r, g, b, 0, speed, len, TwinkleOffAnimation::TwinkleOffPercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
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
    if (!m_startupRunning) {
        m_startupRunning = true;
        m_timer.Start();
        SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 255, 255, 0, 0.5, true);
        SetAnimation(LEDAnimationType::ColorFlow, LEDSection::All, 0, 0, 255, 0.5, false);
    }
    if (m_timer.Get() > 3.0_s) {
        m_shouldStartup = false;
        m_timer.Stop();
    }
}
