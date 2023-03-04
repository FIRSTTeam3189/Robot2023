// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"

Shooter::Shooter() : m_motor(SHOOTER_MOTOR_ID) {}

// This method will be called once per scheduler run
void Shooter::Periodic() {}

void Shooter::SetSpeed(double power) {
    m_motor.Set(power);
} 
