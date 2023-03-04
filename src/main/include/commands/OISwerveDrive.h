// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Joystick.h>
#include "subsystems/SwerveDrive.h"
#include <units/dimensionless.h>
#include <units/time.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "Constants.h"
#include <math.h>
#include <cmath>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class OISwerveDrive
    : public frc2::CommandHelper<frc2::CommandBase, OISwerveDrive> {
 public:
  OISwerveDrive(frc::Joystick *m_bill, SwerveDrive *swerve_drive, bool isMagnitudeRot);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  units::angular_velocity::radians_per_second_t SetDesiredRotationalVelocity();

 private:
 frc::Joystick *m_bill;
 SwerveDrive *m_swerve_drive;
 frc2::PIDController m_rotationPIDController;
 bool m_isMagnitudeRot;

 bool fieldRelative = true;
 // Limits rate of change of voltage so it doesn't explode
 frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{1 / 10_ms};
 frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{1 / 10_ms};
 frc::SlewRateLimiter<units::scalar> m_rotLimiter{1 / 10_ms};
};