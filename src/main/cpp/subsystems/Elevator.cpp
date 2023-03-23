// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Elevator.h"

Elevator::Elevator() :
// m_motor(ELEVATOR_MOTOR),
m_motor(ELEVATOR_MOTOR, rev::CANSparkMaxLowLevel::MotorType::kBrushless), 
// m_encoder(m_motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)),
m_encoder(m_motor.GetEncoder()),
// m_encoder(m_motor.GetAlternateEncoder(rev::SparkMaxAlternateEncoder::Type::kQuadrature, ELEVATOR_THROUGHBORE_CPR))
// m_PIDcontroller(m_motor.GetPIDController())
// m_lowerLimitSwitch(ELEVATOR_LOWER_LIMIT_SWITCH_ID),
// m_upperLimitSwitch(ELEVATOR_UPPER_LIMIT_SWITCH_ID)
m_lowerLimitSwitch(m_motor.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen)),
m_upperLimitSwitch(m_motor.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyOpen))
{   
    // Configure falcon PID and miscellaneous settings
    // m_motor.ConfigFactoryDefault();
    // m_motor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    // m_motor.ConfigFeedbackNotContinuous(true);
    // m_motor.SetNeutralMode(NeutralMode::Brake);
    // m_motor.Config_kP(0, ELEVATOR_P, 50);
    // m_motor.Config_kI(0, ELEVATOR_I, 50);
    // m_motor.Config_kD(0, ELEVATOR_D, 50);
    // m_motor.ConfigClosedloopRamp(0);
    // m_motor.ConfigOpenloopRamp(0);
    // m_motor.SetNeutralMode(ctre::phoenix::motorcontrol::Brake);

    m_motor.RestoreFactoryDefaults();
    m_motor.SetInverted(true);
    m_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_upperLimitSwitch.EnableLimitSwitch(true);
    m_lowerLimitSwitch.EnableLimitSwitch(true);
    m_encoder.SetPositionConversionFactor(ELEVATOR_INTEGRATED_CPR);
    // m_PIDcontroller.SetP(ELEVATOR_P);
    // m_PIDcontroller.SetI(ELEVATOR_I);
    // m_PIDcontroller.SetD(ELEVATOR_D);
    // m_PIDcontroller.SetFeedbackDevice(m_encoder);
}

// This method will be called once per scheduler run
void Elevator::Periodic() {
    // frc::SmartDashboard::PutNumber("Elevator Position", m_motor.GetSelectedSensorPosition());
    // if (abs(m_target - m_motor.GetSelectedSensorPosition()) < ELEVATOR_SLOW_DISTANCE) {
    //     m_motor.Config_kP(0, ELEVATOR_SLOW_P);
    // }  else if (abs(m_target - m_motor.GetSelectedSensorPosition()) < ELEVATOR_STOP_DISTANCE) {
    //     m_atSetpoint = true;
    // } 
    // if (m_lowerLimitSwitch.Get()) {
    //     m_motor.SetSelectedSensorPosition(0);
    //     if (m_motor.Get() < 0) {
    //         m_power = 0;
    //         m_motor.StopMotor();
    //     }
    // } else if (m_upperLimitSwitch.Get()) {
    //     if (m_motor.Get() > 0) {
    //         m_power = 0;
    //         m_motor.StopMotor();
    //     }
    // }

    frc::SmartDashboard::PutBoolean("Elevator Top Limit Switch", m_upperLimitSwitch.Get());
    frc::SmartDashboard::PutBoolean("Elevator Lower Limit Switch", m_lowerLimitSwitch.Get());
    frc::SmartDashboard::PutNumber("Elevator Position (ticks)", (int)m_encoder.GetPosition());
    frc::SmartDashboard::PutNumber("Elevator Position (m)", ElevatorTicksToMeters(m_encoder.GetPosition()));
    // If carridge is close to target slow down elevator
    // if (abs(m_target - m_encoder.GetPosition()) < ELEVATOR_SLOW_DISTANCE) {
    //     m_PIDcontroller.SetP(ELEVATOR_SLOW_P);
    // }

    // If lower limit switch is hit and going down, stop and set encoder to 0
    // If upper limit switch is hit and going up, stop
    if (m_lowerLimitSwitch.Get()) {
        m_power = (m_power < 0) ? 0 : m_power;
        // TODO: FIX FOR CORRECT ENCODER TYPE
        m_encoder.SetPosition(0);
    } else if (m_upperLimitSwitch.Get()) {
        m_power = (m_power > 0) ? 0 : m_power;
    }
}

double Elevator::GetPosition() {
    // return m_motor.GetSelectedSensorPosition()
    return m_encoder.GetPosition();
}

void Elevator::Drive(double power) {
    m_motor.Set(m_power);
    // m_motor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_power);
    m_power = power;
}

// 4.36418e-06 meters per tick for trougbore encoder
double Elevator::ElevatorTicksToMeters(double encoderTicks) {
    // return encoderTicks / ELEVATOR_THROUGHBORE_CPR / ELEVATOR_GEAR_RATIO * ELEVATOR_OUTPUT_CIRCUMFERENCE;
    return 0.97 * encoderTicks / ELEVATOR_INTEGRATED_CPR / ELEVATOR_GEAR_RATIO * ELEVATOR_OUTPUT_CIRCUMFERENCE;
}

void Elevator::GoToPosition(double target) {
    // Resets atSetpoint marker on new position
    // m_atSetpoint = false;
    // // Set to normal PIDs after new command
    // m_motor.Config_kP(0, ELEVATOR_P);
    // m_motor.Config_kI(0, ELEVATOR_I);
    // m_motor.Config_kD(0, ELEVATOR_D);
    // m_motor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, target);
    // m_target = target;

    // Resets position marker upon getting a new position
    // Set to normal speed after new PID command
    if (m_controller.GetP() != ELEVATOR_P) {
        SetPID(ELEVATOR_P, ELEVATOR_I, ELEVATOR_D);
    }

    units::volt_t pidValue = units::volt_t{m_controller.Calculate(units::meter_t{ElevatorTicksToMeters(m_encoder.GetPosition())}, 
                                             units::meter_t{ElevatorTicksToMeters(target)})};
    auto acceleration = (m_controller.GetSetpoint().velocity - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);

    units::volt_t ffValue = ff.Calculate(m_controller.GetSetpoint().velocity, acceleration);

    m_motor.SetVoltage(std::clamp((pidValue + ffValue), -12.0_V, 12.0_V) / 2.0);
    m_lastSpeed = m_controller.GetSetpoint().velocity;
    m_lastTime = frc::Timer::GetFPGATimestamp();
    
    // m_PIDcontroller.SetReference(target, rev::CANSparkMax::ControlType::kPosition);
    m_target = target;
}

void Elevator::SetRunningState(bool isRunning) {
    m_isRunning = isRunning;
}

bool Elevator::GetRunningState() {
    return m_isRunning;
}

void Elevator::SetPID(double kP, double kI, double kD) {
    // m_motor.Config_kP(0, kP);
    // m_motor.Config_kI(0, kI);
    // m_motor.Config_kD(0, kD);
    // m_PIDcontroller.SetP(kP);
    // m_PIDcontroller.SetI(kI);
    // m_PIDcontroller.SetD(kD);
    m_controller.SetP(kP);
    m_controller.SetI(kI);
    m_controller.SetD(kD);
}

bool Elevator::AtSetpoint() {
    return abs(m_target - m_encoder.GetPosition()) < ELEVATOR_STOP_DISTANCE;
}