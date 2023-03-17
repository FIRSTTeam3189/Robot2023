// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AimAssist.h"

AimAssist::AimAssist(Vision *vision, SwerveDrive *swerve, double targetXDistance, double targetYDistance, double targetRotAngle)
: m_vision(vision), m_visionData(m_vision->GetData()),
  m_swerve(swerve), 
  m_xController(frc::PIDController{VISION_X_KP, VISION_X_KI, VISION_X_KD}),
  m_yController(frc::PIDController{VISION_Y_KP, VISION_Y_KI, VISION_Y_KD}),
  m_rotationController(frc::PIDController{VISION_ROT_KP, VISION_ROT_KI, VISION_ROT_KD}),
  m_targetXDistance(targetXDistance),
  m_targetYDistance(targetYDistance),
  m_targetRotAngle(targetRotAngle) {
  // Use addRequirements() here to declare subsystem dependencies.
  // std::cout << "Constructing aim assist\n";
  AddRequirements(vision);
  AddRequirements(swerve);
  // m_vision->Toggle();
  // m_xController.SetTolerance(0.05);
  // m_yController.SetTolerance(0.05);
  m_rotationController.SetTolerance(1.0);
  m_rotationController.EnableContinuousInput(-180, 180);
}

// Called when the command is initially scheduled.
void AimAssist::Initialize() {
  
}

// Called repeatedly when this Command is scheduled to run
void AimAssist::Execute() {
  // std::cout << "Executing aim assist\n";
  m_visionData = m_vision->GetData();
  // std::cout << "Got data\n";
  // std::cout << m_visionData.ID;
  // std::cout << m_visionData.cosZRot << "\n";
  // std::cout << m_visionData.translationMatrix.front();
  // std::cout << m_visionData.translationMatrix[0] << " " << m_visionData.translationMatrix[1] << "\n";
  xOutput = -m_xController.Calculate((double)m_visionData.translationMatrix[0], m_targetXDistance);
  yOutput = -m_yController.Calculate((double)m_visionData.translationMatrix[1], m_targetYDistance);
  // std::cout << "Calculated\n";
  // Converts cosine of z theta to radians
  // double zAngleDeg = acos(m_visionData.cosZRot);
  // rotOutput = m_rotationController.Calculate(zAngleDeg, AIM_ASSIST_TARGET_ROTATION);

  rotOutput = -m_rotationController.Calculate(m_swerve->GetNormalizedYaw(), m_targetRotAngle);

  if (abs((double)m_visionData.translationMatrix[0] - m_targetXDistance) < 0.04) {
    xOutput = 0.0;
  }
  if (abs((double)m_visionData.translationMatrix[1] - m_targetYDistance) < 0.04) {
    yOutput = 0.0;
  }
  
  // std::cout << "About to drive with aim assist\n";
  m_swerve->Drive(units::meters_per_second_t(xOutput), units::meters_per_second_t(yOutput), units::angular_velocity::radians_per_second_t(rotOutput), true);
}

// Called once the command ends or is interrupted.
void AimAssist::End(bool interrupted) {

}

// Returns true when the command should end.
bool AimAssist::IsFinished() {
  if ((abs((double)m_visionData.translationMatrix[0] - m_targetXDistance) < 0.04 && 
       abs((double)m_visionData.translationMatrix[1] - m_targetYDistance) < 0.04)) {
    m_swerve->Drive(units::meters_per_second_t(0.0), units::meters_per_second_t(0.0), units::angular_velocity::radians_per_second_t(0.0), true);
    return true;
  }
  return false;
}
