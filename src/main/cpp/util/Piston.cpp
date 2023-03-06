// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "util/Piston.h"

Piston::Piston(int forwardChannel, int reverseChannel) {
    m_solenoid_e = new frc::Solenoid(1, frc::PneumaticsModuleType::CTREPCM, forwardChannel);
    m_solenoid_r = new frc::Solenoid(1, frc::PneumaticsModuleType::CTREPCM, reverseChannel);

    m_solenoid_r->Set(true);
    m_solenoid_e->Set(false);
}

void Piston::SetExtended(bool extend) {
    m_solenoid_e->Set(extend);
    m_solenoid_r->Set(!extend);
}

void Piston::TogglePiston() {
    bool state = m_solenoid_e->Get();
    m_solenoid_e->Set(!state);
    m_solenoid_r->Set(state);
}

bool Piston::IsExtended() {
    return m_solenoid_e->Get();
}