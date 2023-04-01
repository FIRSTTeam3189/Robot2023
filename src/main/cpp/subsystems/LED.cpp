// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LED.h"

LED::LED() 
: 
m_candle(CANDLE_DEVICE_ID),
m_pieceGrabbed(false), 
m_candleConfig(),
m_r(0),
m_g(0),
m_b(0),
m_startIndex(0),
m_pixelCount(CANDLE_LED_PIXEL_COUNT) {
    m_candleConfig.stripType = ctre::phoenix::led::LEDStripType::RGB;
    m_animation = new RainbowAnimation(1, 0.1, CANDLE_LED_PIXEL_COUNT);
}

// This method will be called once per scheduler run
void LED::Periodic() {
    m_pieceGrabbed = frc::SmartDashboard::GetBoolean("Piece Grabbed", false);
    if (m_pieceGrabbed) {
        m_candle.Animate(*m_animation, 0);
    } else {
        m_candle.ClearAnimation(0);
        m_candle.SetLEDs(m_r, m_g, m_b, 0, m_startIndex, m_pixelCount);
    }
}

void LED::SetLEDs(int r, int g, int b, int startIndex, int pixelCount) {
    m_r = r;
    m_g = g;
    m_b = b;
    m_startIndex = startIndex;
    m_pixelCount = pixelCount;
}