// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Grabber.h"

Grabber::Grabber() : m_motor(GRABBER_MOTOR_ID), m_pieceGrabbed(false), m_encoderVelocity(0.0) {
    m_motor.ConfigFactoryDefault();
    m_motor.ConfigOpenloopRamp(0);
    m_motor.ConfigClosedloopRamp(0);
    m_motor.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::IntegratedSensor);
}

// This method will be called once per scheduler run
void Grabber::Periodic() {
    m_encoderVelocity = m_motor.GetSelectedSensorVelocity();

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
