// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() :
m_motor(ELEVATOR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
m_encoder(m_motor.GetAlternateEncoder(ELEVATOR_CPR)), 
m_PIDcontroller(m_motor.GetPIDController()) 
{
    m_PIDcontroller.SetP(ELEVATOR_P);
    m_PIDcontroller.SetI(ELEVATOR_I);
    m_PIDcontroller.SetD(ELEVATOR_D);
    m_PIDcontroller.SetFeedbackDevice(m_encoder);
    m_PIDcontroller.SetSmartMotionAllowedClosedLoopError(.01);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
    if (abs(m_position - m_encoder.GetPosition()) < .01) {
        m_atSetpoint = true;
    }
}

double Elevator::GetPosition() {
    return m_encoder.GetPosition();
}

void Elevator::Drive(double power) {
    m_motor.Set(power);
}

void Elevator::GoToPosition(double position) {
    // Resets position marker upon getting a new position
    m_atSetpoint = false;
    m_PIDcontroller.SetReference(position, rev::CANSparkMax::ControlType::kPosition);
    m_position = position;
}

bool Elevator::AtSetpoint() {
    return m_atSetpoint;
}