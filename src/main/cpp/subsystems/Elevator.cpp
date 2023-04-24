// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() :
m_motor(ELEVATOR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
m_encoder(m_motor.GetEncoder()),
m_lowerLimitSwitch(m_motor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)),
m_upperLimitSwitch(m_motor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen))
{   
    // Configure motor and limit switches
    m_motor.RestoreFactoryDefaults();
    m_motor.SetInverted(true);
    m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_motor.SetClosedLoopRampRate(0.0);
    m_motor.SetOpenLoopRampRate(0.0);
    m_upperLimitSwitch.EnableLimitSwitch(true);
    m_lowerLimitSwitch.EnableLimitSwitch(true);
    // Makes the encoder return encoder counts instead of rotations
    m_encoder.SetPositionConversionFactor(ELEVATOR_INTEGRATED_CPR);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
    frc::SmartDashboard::PutBoolean("Elevator Top Limit Switch", m_upperLimitSwitch.Get());
    frc::SmartDashboard::PutBoolean("Elevator Lower Limit Switch", m_lowerLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Elevator Position (ticks)", (int)m_encoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elevator Position (m)", ElevatorTicksToMeters(m_encoder.GetPosition()));

    // If lower limit switch is hit and going down, stop and set encoder to 0
    // If upper limit switch is hit and going up, stop
    if (m_lowerLimitSwitch.Get()) {
        m_power = (m_power < 0) ? 0 : m_power;
        m_encoder.SetPosition(0);
    } else if (m_upperLimitSwitch.Get()) {
        m_power = (m_power > 0) ? 0 : m_power;
    }
}

double Elevator::GetPosition() {
    return m_encoder.GetPosition();
}

void Elevator::Drive(double power) {
    m_motor.Set(m_power);
    m_power = power;
}

double Elevator::ElevatorTicksToMeters(double encoderTicks) {
    // 0.97 multiplier is arbitrary but makes encoder distance output more accurate
    return 0.97 * encoderTicks / ELEVATOR_INTEGRATED_CPR / ELEVATOR_GEAR_RATIO * ELEVATOR_OUTPUT_CIRCUMFERENCE;
}

void Elevator::GoToPosition(double target) {
    // Calculates PID value in volts based on position and target
    units::volt_t pidValue = units::volt_t{m_controller.Calculate(units::meter_t{ElevatorTicksToMeters(m_encoder.GetPosition())}, 
                                             units::meter_t{ElevatorTicksToMeters(target)})};

    // Calculates the change in velocity (acceleration) since last control loop
    // Uses the acceleration value and desired velocity to calculate feedforward gains
    // Feedforward gains are approximated based on the current state of the system and a known physics model
    // Gains calculated with SysID                                   
    auto acceleration = (m_controller.GetSetpoint().velocity - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);
    units::volt_t ffValue = ff.Calculate(m_controller.GetSetpoint().velocity, acceleration);

    // Don't forget to remove limiter that sets it to 8 volts max and less PID for 0 position in ElevatorPID.cpp
    // Limit the applied voltage to 12
    if (abs(m_target - m_encoder.GetPosition()) < ELEVATOR_SLOW_DISTANCE) {
        m_motor.SetVoltage(std::clamp((pidValue + ffValue), -12.0_V, 12.0_V) / 2.0);
    } else {
        m_motor.SetVoltage(std::clamp((pidValue + ffValue), -12.0_V, 12.0_V));
    }

    m_lastSpeed = m_controller.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();
    m_target = target;
}

void Elevator::SetPID(double kP, double kI, double kD) {
    m_controller.SetP(kP);
    m_controller.SetI(kI);
    m_controller.SetD(kD);
}

bool Elevator::AtSetpoint() {
    // If within stopping distance, elevator will return true
    return abs(m_target - m_encoder.GetPosition()) < ELEVATOR_STOP_DISTANCE;
}

bool Elevator::LowerLimitSwitchHit() {
    return m_lowerLimitSwitch.Get();
}