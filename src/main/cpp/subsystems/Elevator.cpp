// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() :
m_motor(ELEVATOR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
// m_encoder(m_motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)), 
m_PIDcontroller(m_motor.GetPIDController()),
m_lowerLimitSwitch(ELEVATOR_LOWER_LIMIT_SWITCH_ID),
m_upperLimitSwitch(ELEVATOR_UPPER_LIMIT_SWITCH_ID)
{
    m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    // m_encoder.SetPositionConversionFactor(ELEVATOR_CPR);
    // m_PIDcontroller.SetP(ELEVATOR_P);
    // m_PIDcontroller.SetI(ELEVATOR_I);
    // m_PIDcontroller.SetD(ELEVATOR_D);
    // m_PIDcontroller.SetFeedbackDevice(m_encoder);
    // m_PIDcontroller.SetSmartMotionAllowedClosedLoopError(.01);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
    // frc::SmartDashboard::PutNumber("Elevator Position", m_encoder.GetPosition());
    // // If carridge is close to target slow down elevator
    // if (abs(m_target - m_encoder.GetPosition()) < ELEVATOR_SLOW_DISTANCE) {
    //     m_PIDcontroller.SetP(ELEVATOR_SLOW_P);
    // } else if (abs(m_target - m_encoder.GetPosition()) < ELEVATOR_STOP_DISTANCE) {
    //     m_atSetpoint = true;
    // } 
    // If lower limit switch is hit and going down, stop and set encoder to 0
    // If upper limit switch is hit and going up, stop
    // if (m_lowerLimitSwitch.Get()) {
    //     m_power = (m_power < 0) ? 0 : m_power;
    //     // TODO: FIX FOR CORRECT ENCODER TYPE
    //     // m_encoder.SetPosition(0);
    // } else if (m_upperLimitSwitch.Get()) {
    //     m_power = (m_power > 0) ? 0 : m_power;
    // }
}

double Elevator::GetPosition() {
    // return m_encoder.GetPosition();
    return 0.0;
}

void Elevator::Drive(double power) {
    m_power = power;
    m_motor.Set(m_power);
}

void Elevator::GoToPosition(double target) {
    // // Resets position marker upon getting a new position
    // m_atSetpoint = false;
    // // Set to normal speed after new PID command
    // m_PIDcontroller.SetP(ELEVATOR_P);
    // m_PIDcontroller.SetReference(target, rev::CANSparkMax::ControlType::kPosition);
    // m_target = target;
}

bool Elevator::AtSetpoint() {
    // return m_atSetpoint;
    return false;
}