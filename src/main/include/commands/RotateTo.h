// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/PIDCommand.h>
#include "subsystems/SwerveDrive.h"
#include "Constants.h"

class RotateTo
    : public frc2::CommandHelper<frc2::PIDCommand, RotateTo> {
 public:
  RotateTo(SwerveDrive *swerveDrive, double targetAngle);

  bool IsFinished() override;

 private:
  SwerveDrive *m_swerve;
  double m_targetAngle;
};
