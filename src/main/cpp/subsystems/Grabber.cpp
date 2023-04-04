// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Grabber.h"

Grabber::Grabber() : m_motor(GRABBER_MOTOR_ID), m_pieceGrabbed(false), m_encoderVelocity(0.0) {
    m_motor.ConfigFactoryDefault();
    m_motor.ConfigOpenloopRamp(0);
    m_motor.ConfigClosedloopRamp(0);
    m_motor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
    // m_colorMatcher.AddColorMatch(kConeTarget);
    // m_colorMatcher.AddColorMatch(kCubeTarget);
    // m_colorSensor.ConfigureColorSensor(rev::ColorSensorV3::ColorResolution::k13bit, rev::ColorSensorV3::ColorMeasurementRate::k500ms);
}

// This method will be called once per scheduler run
void Grabber::Periodic() {
    // m_detectedColor = m_colorSensor.GetColor();
    m_encoderVelocity = m_motor.GetSelectedSensorVelocity();

    // m_colorMatcher.SetConfidenceThreshold(GRABBER_SENSOR_CONFIDENCE);
    // m_matchedColor = *m_colorMatcher.MatchColor(m_detectedColor);
    // uint32_t m_proximity = m_colorSensor.GetProximity();
    // if (m_matchedColor == kConeTarget) {
    //     frc::SmartDashboard::PutBoolean("Has Cone?", true);
    // } else if (m_matchedColor == kCubeTarget) {
    //     // frc::Shuffleboard::GetTab("Driver Tab")
    //     // .Add("Grabber Piece Color", true)
    //     // .WithWidget(frc::BuiltInWidgets::kBooleanBox)
    //     // .WithProperties({
    //     // {"colorWhenTrue", nt::Value::MakeString("blue")}
    //     // }).GetEntry();
    //     frc::SmartDashboard::PutBoolean("Has Cube?", true);
    // } else {
    //     // frc::Shuffleboard::GetTab("Driver Tab")
    //     // .Add("Grabber Piece Color", false)
    //     // .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    //     frc::SmartDashboard::PutBoolean("Has Cone?", false);
    //     frc::SmartDashboard::PutBoolean("Has Cube?", false);
    // }   

    if (abs(m_encoderVelocity) < 200 && m_motor.GetStatorCurrent() != 0.0) {
        m_pieceGrabbed = true;
    } else {
        m_pieceGrabbed = false;
    }
    
    frc::SmartDashboard::PutBoolean("Piece Grabbed", m_pieceGrabbed);

}

void Grabber::SetSpeed(double power) {
    m_motor.Set(power);
} 

bool Grabber::IsPieceGrabbed() {
    return m_pieceGrabbed;
}
