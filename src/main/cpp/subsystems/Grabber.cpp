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
    
    m_motor.SetSmartCurrentLimit(25);
    m_motor.SetSecondaryCurrentLimit(30);
}

// This method will be called once per scheduler run
void Grabber::Periodic() {
    m_encoderVelocity = m_encoder.GetVelocity();

    // Checks the encoder velocity if the motor is trying to spin to determine if there is resistance
    // If the motor is trying to spin but isn't spinning, it is meeting resistance
    // Most likely, this means it is stuck (hopefully not) or a piece is successfully in the grabber
    // Then, the grabber will stop
    if (abs(m_motor.GetAppliedOutput()) > 0.05) {
        m_timer.Start();
        if (abs(m_encoderVelocity) < GRABBER_GRABBED_RPM && m_timer.HasElapsed(GRABBER_MINIMUM_SPIN_TIME)) {
            m_pieceGrabbed = true;
            m_timer.Stop();
            m_timer.Reset();
        } else {
            m_pieceGrabbed = false;
        }
    }
    // Once motor doesn't get any power, reset timer so next time it will still run at least 0.25s
    else if (m_motor.GetAppliedOutput() == 0.0) {
        m_timer.Stop();
        m_timer.Reset();
    }
    frc::SmartDashboard::PutBoolean("Piece Grabbed", m_pieceGrabbed);
}

void Grabber::SetSpeed(double power) {
    m_motor.Set(power);
} 

bool Grabber::IsPieceGrabbed() {
    return m_pieceGrabbed;
}
