// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"

#include "units/current.h"

Intake::Intake()
:
m_intakePiston(INTAKE_PISTON_OUT, INTAKE_PISTON_IN),
m_rollerMotor(INTAKE_ROLLER_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless),
m_conveyorMotor(INTAKE_CONVEYOR_MOTOR_ID, rev::CANSparkMax::MotorType::kBrushless) {
    m_intakePiston.SetExtended(false);
    m_rollerMotor.RestoreFactoryDefaults();
    m_conveyorMotor.RestoreFactoryDefaults();
    // Inverts because negative value on robot is intaking by default
    m_rollerMotor.SetInverted(true);
    m_conveyorMotor.SetInverted(true);
    
    m_conveyorMotor.SetSmartCurrentLimit(10);
    m_conveyorMotor.SetSecondaryCurrentLimit(15);
}

// This method will be called once per scheduler run
void Intake::Periodic() {}

void Intake::ToggleIntake() {
    m_intakePiston.TogglePiston();
}

void Intake::SetPistonExtension(bool isExtended) {
    m_intakePiston.SetExtended(isExtended);
}

bool Intake::GetPistonExtentionState() {
    return m_intakePiston.IsExtended();
}

void Intake::SetPower(double intakePower, double conveyorPower) {
    m_rollerMotor.Set(intakePower);
    m_conveyorMotor.Set(conveyorPower);
}
