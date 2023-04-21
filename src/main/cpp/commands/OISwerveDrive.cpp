// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/OISwerveDrive.h"

OISwerveDrive::OISwerveDrive(frc::Joystick *m_bill, SwerveDrive *swerve_drive, bool isMagnitudeRot, RotationMode mode, bool fieldRelative) 
:  m_bill(m_bill), m_swerve_drive(swerve_drive),
   m_rotationPIDController(SwerveDriveConstants::rotP, SwerveDriveConstants::rotI, SwerveDriveConstants::rotD),
   m_isMagnitudeRot(isMagnitudeRot),
   m_fieldRelative(fieldRelative),
   m_currentMode(mode) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(swerve_drive);
  m_rotationPIDController.SetTolerance(1.0);
  m_rotationPIDController.EnableContinuousInput(0, 360);
  m_swerve_drive->EnableContinuousInput(-180, 180);
}

// Called when the command is initially scheduled.
void OISwerveDrive::Initialize() {}

units::angular_velocity::radians_per_second_t OISwerveDrive::GetDesiredRotationalVelocity() {
  // Get raw (-1.0 to 1.0) joystick positions for x and y axis
  // Left, up are -1.0; right, down are 1.0
  // Inverted so forward on joystick is down the field
  double joystickX = -frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_RSTICK_Y), 0.05);
  double joystickY = -frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_RSTICK_X), 0.05);

  // Don't rotate if joystick input is near 0 in both axes
  if ((fabs(joystickX) < .05) && (fabs(joystickY) < .05)) 
    return units::angular_velocity::radians_per_second_t{0.0};

  // Convert joystick positions to goal angle in degrees
  // Normalized from -180, 180
  // Uses arctan2 function -- converts Cartesian coordinates (1, 1) to a polar angle (pi / 4), then multiplies by radians to degrees conversion
  double goalAngle = SwerveDriveConstants::DEGToRAD * atan2(joystickY, joystickX);

  frc::SmartDashboard::PutNumber("Robot Desired Angle", goalAngle);

  // Return next velocity in radians per second as calculated by PIDController and limited by rotLimiter
  units::angular_velocity::radians_per_second_t rot = 
              units::angular_velocity::radians_per_second_t{
                m_rotLimiter.Calculate(m_rotationPIDController.Calculate(m_swerve_drive->GetNormalizedYaw(), goalAngle))
                * SwerveDriveConstants::maxAngularVelocity};
  
  // Stop rotating if within tolerance
  if (abs(m_swerve_drive->GetNormalizedYaw() - goalAngle) < 2.5) {
    rot = units::angular_velocity::radians_per_second_t{0.0};
  }

  return rot;
}

// Called repeatedly when this Command is scheduled to run
void OISwerveDrive::Execute() {
  // Read's drive joystick inputs to determine direction to travel
  // Limits speed and rotation to max speed
  const auto xSpeed = -m_xspeedLimiter.Calculate(
                frc::ApplyDeadband(m_bill->GetRawAxis(PS5_AXIS_LSTICK_Y), 0.05)) *
              SwerveDriveConstants::kMaxSpeed;

  const auto ySpeed = -m_yspeedLimiter.Calculate(
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
              SwerveDriveConstants::maxAngularVelocity * 2.0;

    m_swerve_drive->TeleopPercentDrive(xSpeed, ySpeed, rot, m_fieldRelative);
  } 
  else {
    units::radians_per_second_t rot{};
    frc::Translation2d centerOfRotation{};

    // If mode is normal, the robot's center of rotation will be the physical center
    // Otherwise, the center of rotation will be one of the four modules
    // And the robot will rotate at a constant speed clockwise or counter-clockwise
    switch (m_currentMode) {
      case RotationMode::normal:
        rot = GetDesiredRotationalVelocity();
        break;
      case RotationMode::frontLeftCW:
        rot = -10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{+SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::frontLeftCCW:
        rot = 10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{+SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::frontRightCW:
        rot = -10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{+SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::frontRightCCW:
        rot = 10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{+SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::backLeftCW:
        rot = -10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{-SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::backLeftCCW:
        rot = 10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{-SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::backRightCW:
        rot = -10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{-SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter};
        break;
      case RotationMode::backRightCCW:
        rot = 10.0_rad / 1_s;
        centerOfRotation = frc::Translation2d{-SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter};
        break;
      default:
        break;
    }

    m_swerve_drive->TeleopPercentDrive(xSpeed, ySpeed, rot, m_fieldRelative, centerOfRotation);
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


