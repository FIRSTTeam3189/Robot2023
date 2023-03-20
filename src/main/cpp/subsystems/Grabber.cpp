// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Grabber.h"

Grabber::Grabber() : m_motor(GRABBER_MOTOR_ID), m_colorSensor(frc::I2C::Port::kMXP) {
    m_motor.ConfigFactoryDefault();
    m_motor.ConfigOpenloopRamp(0);
    m_motor.ConfigClosedloopRamp(0);
    m_colorMatcher.AddColorMatch(kConeTarget);
    m_colorMatcher.AddColorMatch(kCubeTarget);
}

// This method will be called once per scheduler run
void Grabber::Periodic() {
    m_detectedColor = m_colorSensor.GetColor();

    m_colorMatcher.SetConfidenceThreshold(GRABBER_SENSOR_CONFIDENCE);
    m_matchedColor = *m_colorMatcher.MatchColor(m_detectedColor);
    uint32_t m_proximity = m_colorSensor.GetProximity();
    if (m_matchedColor == kConeTarget && m_proximity >= GRABBER_SENSOR_PROXIMITY_THRESHOLD) {
        frc::Shuffleboard::GetTab("Driver Tab")
        .Add("Grabber Piece Color", true)
        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
        .WithProperties({
        {"colorWhenTrue", nt::Value::MakeString("yellow")}
        }).GetEntry();
    } else if (m_matchedColor == kCubeTarget && m_proximity >= GRABBER_SENSOR_PROXIMITY_THRESHOLD) {
        frc::Shuffleboard::GetTab("Driver Tab")
        .Add("Grabber Piece Color", true)
        .WithWidget(frc::BuiltInWidgets::kBooleanBox)
        .WithProperties({
        {"colorWhenTrue", nt::Value::MakeString("blue")}
        }).GetEntry();
    } else {
        frc::Shuffleboard::GetTab("Driver Tab")
        .Add("Grabber Piece Color", false)
        .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    }   
}

void Grabber::SetSpeed(double power) {
    m_motor.Set(power);
} 
