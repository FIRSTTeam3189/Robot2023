// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "units/current.h"

Intake::Intake() {
    m_leftPiston = new Piston(INTAKE_PISTON_L, 1, 0);
    m_rightPiston = new Piston(INTAKE_PISTON_R, 1, 0);
    m_topMotor = new rev::CANSparkMax(INTAKE_MOTOR_L, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_bottomMotor = new rev::CANSparkMax(INTAKE_MOTOR_R, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::ToggleIntake() {
    m_leftPiston->TogglePiston();
    m_rightPiston->TogglePiston();
}

void Intake::SetPower(double power) {
    m_topMotor->Set(power);
    m_bottomMotor->Set(power);
}
