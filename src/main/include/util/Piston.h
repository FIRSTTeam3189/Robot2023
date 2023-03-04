// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Solenoid.h>

class Piston {
public:
  Piston(int module, int forwardChannel, int reverseChannel);
  void SetExtended(bool extend);
  void TogglePiston();
  bool IsExtended();
private:
  frc::Solenoid *m_solenoid_r;
  frc::Solenoid *m_solenoid_e;
};
