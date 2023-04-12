// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Grabber.h"

Grabber::Grabber() :
m_motor(GRABBER_MOTOR_ID, 
rev::CANSparkMax::MotorType::kBrushless), 
m_encoder(m_motor.GetEncoder()), 
m_pieceGrabbed(false), 
m_encoderVelocity(0.0) {
    m_motor.RestoreFactoryDefaults();
    m_motor.SetClosedLoopRampRate(0);
    m_motor.SetOpenLoopRampRate(0);
}

// This method will be called once per scheduler run
void Grabber::Periodic() {
    m_encoderVelocity = m_encoder.GetVelocity();

    // Checks the encoder velocity if the motor is trying to spin to determine if there is resistance
    // If the motor is trying to spin but isn't spinning, it is meeting resistance
    // Most likely, this means it is stuck (hopefully not) or a piece is successfully in the grabber
    // Then, the grabber will stop
    if (abs(m_motor.GetAppliedOutput()) > 0.1) {
        m_timer.Start();
        if (abs(m_encoderVelocity) < 100 && m_timer.HasElapsed(0.25_s)) {
            // frc::SmartDashboard::PutNumber("Grabber applied output", m_motor.GetAppliedOutput());
            m_pieceGrabbed = true;
        } else {
            m_pieceGrabbed = false;
            m_timer.Reset();
        }
    }
    
    frc::SmartDashboard::PutBoolean("Piece Grabbed", m_pieceGrabbed);
}

void Grabber::SetSpeed(double power) {
    m_motor.Set(power);
} 

bool Grabber::IsPieceGrabbed() {
    return m_pieceGrabbed;
}
