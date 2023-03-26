// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/RotateTo.h"

// NOTE:  Consider using this command inline, rather than writing a subclass.
// For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
RotateTo::RotateTo(SwerveDrive *swerveDrive, double targetAngle)
    : m_targetAngle(targetAngle), m_swerve(swerveDrive),
      CommandHelper{frc2::PIDController{10.0, 0, 0},
                    // This should return the measurement
                    [swerveDrive] { return -swerveDrive->GetNormalizedYaw(); },
                    // This should return the setpoint (can also be a constant)
                    targetAngle,
                    // This uses the output
                    [swerveDrive](double output) {
                      // Use the output here
                      if (output > 0) {
                        swerveDrive->Drive(0_mps, 0_mps, 
                                         units::radians_per_second_t{(-output / SwerveDriveConstants::DEGToRAD)}, true);
                      } else {
                        swerveDrive->Drive(0_mps, 0_mps, 
                                         units::radians_per_second_t{(-output / SwerveDriveConstants::DEGToRAD)}, true);
                      }
                    }, 
                    {swerveDrive}} 
  {
    m_controller.EnableContinuousInput(-180, 180);
    AddRequirements(swerveDrive);
  }

// Returns true when the command should end.
bool RotateTo::IsFinished() {
  return (abs(-m_swerve->GetNormalizedYaw() - m_targetAngle) < 7.5);
}
