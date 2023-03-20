// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Autos/TwoCargoUltrashootBalance.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
TwoCargoUltrashootBalance::TwoCargoUltrashootBalance(SwerveDrive *swerve, Elevator *elevator, Grabber *grabber, Intake *intake)
: m_swerve(swerve), m_elevator(elevator), m_grabber(grabber), m_intake(intake) {
  // Add your commands here, e.g.
  // AddCommands(FooCommand{}, BarCommand{});
  AddCommands(
    OneCargoPickupBalance(m_swerve, m_elevator, m_grabber, m_intake, true),
    UltraShoot(m_elevator, m_intake, m_grabber)
  );
}
