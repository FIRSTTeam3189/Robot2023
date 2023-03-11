// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/OISwerveDrive.h"

OISwerveDrive::OISwerveDrive(frc::Joystick *m_bill, SwerveDrive *swerve_drive, bool isMagnitudeRot) 
:  m_bill(m_bill), m_swerve_drive(swerve_drive),
   m_rotationPIDController(SwerveDriveConstants::rotP, SwerveDriveConstants::rotI, SwerveDriveConstants::rotD),
   m_isMagnitudeRot(isMagnitudeRot) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_drive);
  m_rotationPIDController.SetTolerance(1.0);
  m_rotationPIDController.EnableContinuousInput(0, 360);
  m_swerve_drive->EnableContinuousInput(-180, 180);
  // Ensures that drive command continues once it's toggled
  // m_swerve_drive->SetDefaultCommand(OISwerveDrive(m_bill, m_swerve_drive));
}

// Called when the command is initially scheduled.
void OISwerveDrive::Initialize() {

}

units::angular_velocity::radians_per_second_t OISwerveDrive::SetDesiredRotationalVelocity() {
  // Get raw (-1.0 to 1.0) joystick positions for x and y axis
  // Left, up are -1.0; right, down are 1.0
  double joystickX = frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_RSTICK_Y), 0.05);
  double joystickY = frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_RSTICK_X), 0.05);

  // Don't rotate if joystick input is near 0 in both axes
  if ((fabs(joystickX) < .05) && (fabs(joystickY) < .05)) 
    return units::angular_velocity::radians_per_second_t{0.0};

  // Convert joystick positions to goal angle in degrees
  // Normalized from -180, 180
  // Uses arctan2 function -- converts Cartesian coordinates to a polar angle, then multiplies by radians to degrees conversion
  double goalAngle = SwerveDriveConstants::DEGToRAD * atan2(joystickY, joystickX);

  frc::SmartDashboard::PutNumber("Robot Desired Angle", goalAngle);

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              units::angular_velocity::radians_per_second_t{
                m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerve_drive->GetNormalizedYaw(), goalAngle))
                * SwerveDriveConstants::maxAngularVelocity};

  frc::SmartDashboard::PutNumber("Wrapped Gyro Yaw", m_swerve_drive->GetNormalizedYaw());
  frc::SmartDashboard::PutNumber("Swerve Gyro Yaw", m_swerve_drive->GetRobotYaw());
  frc::SmartDashboard::PutNumber("Theta PID output", m_rotationPIDController.Calculate(m_swerve_drive->GetNormalizedYaw(), goalAngle))
                * SwerveDriveConstants::maxAngularVelocity;
  
  // Stop rotating if within tolerance
  if (m_rotationPIDController.AtSetpoint())
    rot = units::angular_velocity::radians_per_second_t {0.0};
  return rot;
}

// Called repeatedly when this Command is scheduled to run
void OISwerveDrive::Execute() {
  // Limits speed and rotation to max speed
  const auto xSpeed = -m_xspeedLimiter.Calculate(
                frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_LSTICK_Y), 0.05)) *
              SwerveDriveConstants::kMaxSpeed;

  const auto ySpeed = m_yspeedLimiter.Calculate(
                frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_LSTICK_X), 0.05)) *
              SwerveDriveConstants::kMaxSpeed;

  // Convert from joystick positions to goal angle
  // Then use PIDController to convert from goal angle to next rotation velocity
  // If using magnitude-based rotation (robot turns infinitely in
  // direction pointed, just use x-axis of joystick)
  // If using atan2 rotation (robot points in direction of joystick)
  // Use the rotation velocity function
  if (m_isMagnitudeRot) {
    const auto rot = -m_rotLimiter.Calculate(
                frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_RSTICK_X), 0.05)) *
              SwerveDriveConstants::maxAngularVelocity;
    m_swerve_drive->Drive(xSpeed, ySpeed, rot, fieldRelative);
  } 
  else {
    const auto rot = SetDesiredRotationalVelocity();
    m_swerve_drive->Drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}

// Called once the command ends or is interrupted.
void OISwerveDrive::End(bool interrupted) {
  m_swerve_drive->Stop();
}

// Returns true when the command should end.
bool OISwerveDrive::IsFinished() {
  return false;
}


