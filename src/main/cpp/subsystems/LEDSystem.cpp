// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LEDSystem.h"

LEDSystem::LEDSystem(int brightness) : m_candleControl(CANDLE_DEVICE_ID), m_animation(NULL), m_candleConfig(), m_ledSections() {
    m_candleConfig.stripType = ctre::phoenix::led::LEDStripType::RGB;
    m_candleConfig.brightnessScalar = brightness;
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
}

// This method will be called once per scheduler run
void LEDSystem::Periodic() {}

void LEDSystem::SetAnimation(LEDAnimationType newAnimation, LEDSection section = LEDSection::All, int r = 0, int g = 0, int b = 0) {
    auto len = m_ledSections[section].second - m_ledSections[section].first;
    switch (newAnimation) {
    case LEDAnimationType::ColorFlow:
        m_animation = new ColorFlowAnimation(r, g, b, 0, 0.7, len, ColorFlowAnimation::Direction::Forward, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Fire:
        m_animation = new FireAnimation(m_candleConfig.brightnessScalar, 0.7, len, 0.7, 0.5, false, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Larson:
        m_animation = new LarsonAnimation(r, g, b, 0, 1, len, LarsonAnimation::BounceMode::Front, 3, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Rainbow:
        m_animation = new RainbowAnimation(1, 0.1, len, false, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::RGBFade:
        m_animation = new RgbFadeAnimation(0.7, 0.4, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::SingleFade:
        m_animation = new SingleFadeAnimation(r, g, b, 0, 0.5, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Strobe:
        m_animation = new StrobeAnimation(r, g, b, 0, 0.75, len, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::Twinkle:
        m_animation = new TwinkleAnimation(r, g, b, 0, 0.5, len, TwinkleAnimation::TwinklePercent::Percent18, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::TwinkleOff:
        m_animation = new TwinkleOffAnimation(r, g, b, 0, 0.8, len, TwinkleOffAnimation::TwinkleOffPercent::Percent100, m_ledSections[section].first);
        m_candleControl.Animate(*m_animation, 0);
        break;
    case LEDAnimationType::MultiAnim:
    // Need to Work on after Testing
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

void LEDSystem::SetColor(int r, int g, int b, LEDSection section = LEDSection::All) {
    m_candleControl.SetLEDs(r, g, b, 0, m_ledSections[section].first, m_ledSections[section].second);
}

void LEDSystem::ClearColor(LEDSection section = LEDSection::All) {
    m_candleControl.SetLEDs(0, 0, 0, 0, m_ledSections[section].first, m_ledSections[section].second);
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

}
