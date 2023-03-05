// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "units/current.h"

Intake::Intake() {
    m_leftPiston = new Piston(INTAKE_PISTON_L, 1, 0);
    m_rightPiston = new Piston(INTAKE_PISTON_R, 1, 0);
    m_rollerMotor = new rev::CANSparkMax(INTAKE_ROLLER_MOTOR_ID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_conveyorMotor = new rev::CANSparkMax(INTAKE_CONVEYOR_MOTOR_ID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_leftConeCorrectMotor = new rev::CANSparkMax(INTAKE_L_CONE_CORRECT_MOTOR_ID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
    m_rightConeCorrectMotor = new rev::CANSparkMax(INTAKE_R_CONE_CORRECT_MOTOR_ID, rev::CANSparkMax::CANSparkMaxLowLevel::MotorType::kBrushless);
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::ToggleIntake() {
    m_leftPiston->TogglePiston();
    m_rightPiston->TogglePiston();
}

void Intake::SetPower(double intakePower, double conveyorPower, double coneCorrectPower) {
    m_rollerMotor->Set(intakePower);
    m_conveyorMotor->Set(conveyorPower);
    m_leftConeCorrectMotor->Set(coneCorrectPower);
    m_rightConeCorrectMotor->Set(coneCorrectPower);
}
