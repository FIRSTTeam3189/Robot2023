// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveDrive.h"
#include <frc/smartdashboard/SmartDashboard.h>

SwerveDrive::SwerveDrive() : 
// Initialize swerve drive info into SwerveModules struct
// Using WPILib kinematics and odometry, pass in sensors and locations
m_SM{
  {+SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter},
  {+SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter},
  {-SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter},
  {-SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter},
  {SwerveDriveConstants::kLeftFrontInfo},
  {SwerveDriveConstants::kRightFrontInfo},
  {SwerveDriveConstants::kLeftBackInfo},
  {SwerveDriveConstants::kRightBackInfo}
},
m_pigeon(SwerveDriveConstants::gyroCANID, "Swerve"),
m_modulePositions(
  m_SM.m_frontLeft.GetSwerveModulePosition(),
  m_SM.m_frontRight.GetSwerveModulePosition(),
  m_SM.m_backLeft.GetSwerveModulePosition(),
  m_SM.m_backRight.GetSwerveModulePosition()),
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
m_poseEstimator(SwerveDriveParameters::kinematics, m_pigeon.GetRotation2d(), m_modulePositions, frc::Pose2d{}, VisionConstants::stateStdDevs, VisionConstants::visionStdDevs)
{
  // Implementation of subsystem constructor goes here.
  // Create Shuffleboard outputs
  InitSmartDashboard();
  SyncSmartdashBoardValues();
}

void SwerveDrive::Periodic() {
  // Implementation of subsystem periodic method goes here.
  UpdateEstimator();
}

double SwerveDrive::GetRobotYaw() {
  return m_pigeon.GetYaw();
}

void SwerveDrive::ResetGyro() {
  m_pigeon.SetYaw(0.0);
}

void SwerveDrive::PercentDrive(frc::ChassisSpeeds speeds) {
  // Version of drive function that uses basic percentage control (100% on axis is 100% power, 5% is 5%, etc.)
  // Takes a chassis speeds instead of desired velocities
  auto states = SwerveDriveParameters::kinematics.ToSwerveModuleStates(speeds);
  SwerveDriveParameters::kinematics.DesaturateWheelSpeeds(&states, SwerveDriveConstants::kMaxSpeed);
  m_SM.m_frontLeft.SetDesiredPercentState(states[0]);
  m_SM.m_frontRight.SetDesiredPercentState(states[1]);
  m_SM.m_backLeft.SetDesiredPercentState(states[2]);
  m_SM.m_backRight.SetDesiredPercentState(states[3]);
}

void SwerveDrive::PercentDrive(
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rot,
  bool fieldRelative,
  frc::Translation2d centerOfRotation) {
  // Version of drive function that uses basic percentage control (100% on axis is 100% power, 5% is 5%, etc.)
  UpdateEstimator();

  auto states = SwerveDriveParameters::kinematics.ToSwerveModuleStates(
    (fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, -m_pigeon.GetRotation2d())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot}),
    centerOfRotation);

  SwerveDriveParameters::kinematics.DesaturateWheelSpeeds(&states, SwerveDriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  frc::SmartDashboard::PutNumber("FL speed", (double)fl.speed);
  frc::SmartDashboard::PutNumber("FL angle", (double)fl.angle.Degrees());
  frc::SmartDashboard::PutNumber("FR speed", (double)fr.speed);
  frc::SmartDashboard::PutNumber("FR angle", (double)fr.angle.Degrees());
  frc::SmartDashboard::PutNumber("BL speed", (double)bl.speed);
  frc::SmartDashboard::PutNumber("BL angle", (double)bl.angle.Degrees());
  frc::SmartDashboard::PutNumber("BR speed", (double)br.speed);
  frc::SmartDashboard::PutNumber("BR angle", (double)br.angle.Degrees());

  frc::SmartDashboard::PutNumber("X speed", (double)xSpeed);
  frc::SmartDashboard::PutNumber("Y speed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("Rotation", (double)(SwerveDriveConstants::DEGToRAD * rot));

  double AdvantageScopeDesiredStates[] = 
    {(double)fl.angle.Degrees(), (double)fl.speed,
     (double)fr.angle.Degrees(), (double)fr.speed,
     (double)bl.angle.Degrees(), (double)bl.speed,
     (double)br.angle.Degrees(), (double)br.speed};

  frc::SmartDashboard::PutNumberArray("AdvantageScope Desired States", AdvantageScopeDesiredStates);

  m_SM.m_frontLeft.SetDesiredPercentState(states[0]);
  m_SM.m_frontRight.SetDesiredPercentState(states[1]);
  m_SM.m_backLeft.SetDesiredPercentState(states[2]);
  m_SM.m_backRight.SetDesiredPercentState(states[3]);
}

void SwerveDrive::Drive(
  // Pass in speeds and rotations, get back module states
  units::meters_per_second_t xSpeed,
  units::meters_per_second_t ySpeed,
  units::radians_per_second_t rot,
  bool fieldRelative) {
  // Version of function that uses feedforward control
  UpdateEstimator();
  
  auto states = SwerveDriveParameters::kinematics.ToSwerveModuleStates(
    fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, -m_pigeon.GetRotation2d())
                        : frc::ChassisSpeeds{xSpeed, ySpeed, rot});
  SwerveDriveParameters::kinematics.DesaturateWheelSpeeds(&states, SwerveDriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;
  
  frc::SmartDashboard::PutNumber("FL speed", (double)fl.speed);
  frc::SmartDashboard::PutNumber("FL angle", (double)fl.angle.Degrees());
  frc::SmartDashboard::PutNumber("FR speed", (double)fr.speed);
  frc::SmartDashboard::PutNumber("FR angle", (double)fr.angle.Degrees());
  frc::SmartDashboard::PutNumber("BL speed", (double)bl.speed);
  frc::SmartDashboard::PutNumber("BL angle", (double)bl.angle.Degrees());
  frc::SmartDashboard::PutNumber("BR speed", (double)br.speed);
  frc::SmartDashboard::PutNumber("BR angle", (double)br.angle.Degrees());

  frc::SmartDashboard::PutNumber("X speed", (double)xSpeed);
  frc::SmartDashboard::PutNumber("Y speed", (double)ySpeed);
  frc::SmartDashboard::PutNumber("Rotation", (double)(SwerveDriveConstants::DEGToRAD * rot));

  // Desired states to be logged in AdvantageScope (requires different format)
  double AdvantageScopeDesiredStates[] = 
    {(double)fl.angle.Degrees(), (double)fl.speed,
     (double)fr.angle.Degrees(), (double)fr.speed,
     (double)bl.angle.Degrees(), (double)bl.speed,
     (double)br.angle.Degrees(), (double)br.speed};

  frc::SmartDashboard::PutNumberArray("AdvantageScope Desired States", AdvantageScopeDesiredStates);
  SetModuleStates(states);
}

void SwerveDrive::SetModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) {
  // Sets all modules using feedforward
  m_SM.m_frontLeft.SetDesiredState(desiredStates[0]);
  m_SM.m_frontRight.SetDesiredState(desiredStates[1]);
  m_SM.m_backLeft.SetDesiredState(desiredStates[2]);
  m_SM.m_backRight.SetDesiredState(desiredStates[3]);
}

void SwerveDrive::SetPercentModuleStates(std::array<frc::SwerveModuleState, 4> desiredStates) {
  // Sets all modules using basic control
  m_SM.m_frontLeft.SetDesiredPercentState(desiredStates[0]);
  m_SM.m_frontRight.SetDesiredPercentState(desiredStates[1]);
  m_SM.m_backLeft.SetDesiredPercentState(desiredStates[2]);
  m_SM.m_backRight.SetDesiredPercentState(desiredStates[3]);
}

double SwerveDrive::GetNormalizedYaw() {
  // Normalize angle from (-inf, inf) to (-180, 180)
  // The PIDController already uses input modulus but
  // it's easier to read diagnostics when the yaw and desired angle match
  int yaw = m_pigeon.GetYaw(); 
  // (-360, 360)
  int normalizedYaw = (yaw % 360);
  // (-180, 180)
  if (normalizedYaw > 180)
    normalizedYaw -= 360;
  else if (normalizedYaw < -180)
    normalizedYaw += 360;

  return normalizedYaw;
}

void SwerveDrive::SetRobotYaw(double yaw) {
  m_pigeon.SetYaw(yaw);
}

double SwerveDrive::GetPitch() {
  return m_pigeon.GetPitch();
}

double SwerveDrive::GetRoll() {
  return m_pigeon.GetRoll();
}

void SwerveDrive::ManualModuleSpeed(SwerveModuleLocation location, double speed) {
  // Switch over enum cases to set individual modules to spin at a constant speed manually
  switch(location) {
    case SwerveModuleLocation::fl :
      m_SM.m_frontLeft.ManualModuleSpeed(speed);
      break;
    case SwerveModuleLocation::fr :
      m_SM.m_frontRight.ManualModuleSpeed(speed);
      break;
    case SwerveModuleLocation::bl :
      m_SM.m_backLeft.ManualModuleSpeed(speed);
      break;
    case SwerveModuleLocation::br :
      m_SM.m_backRight.ManualModuleSpeed(speed);
      break;
  }
}

void SwerveDrive::ManualModuleTurn(SwerveModuleLocation location, double speed) {
  // Same thing but for wheel turning rather than forward movement
  switch(location) {
    case SwerveModuleLocation::fl :
      m_SM.m_frontLeft.ManualModuleTurn(speed);
      break;
    case SwerveModuleLocation::fr :
      m_SM.m_frontRight.ManualModuleTurn(speed);
      break;
    case SwerveModuleLocation::bl :
      m_SM.m_backLeft.ManualModuleTurn(speed);
      break;
    case SwerveModuleLocation::br :
      m_SM.m_backRight.ManualModuleTurn(speed);
      break;
  }
}

void SwerveDrive::ResetSpeedEncoders() {
  m_SM.m_frontLeft.ResetSpeedEncoder();
  m_SM.m_frontRight.ResetSpeedEncoder();
  m_SM.m_backLeft.ResetSpeedEncoder();
  m_SM.m_backRight.ResetSpeedEncoder();
}

void SwerveDrive::ResetEncodersToAbsolute() {
  m_SM.m_frontLeft.ResetAngleToAbsolute();
  m_SM.m_frontRight.ResetAngleToAbsolute();
  m_SM.m_backLeft.ResetAngleToAbsolute();
  m_SM.m_backRight.ResetAngleToAbsolute();
}

frc::Pose2d SwerveDrive::GetEstimatedPose() {
  UpdateEstimator();
  return m_poseEstimator.GetEstimatedPosition();
}

void SwerveDrive::UpdateEstimator() {
  // Update position and rotation of modules
  m_SM.m_frontLeft.UpdateModulePosition();
  m_SM.m_frontRight.UpdateModulePosition();
  m_SM.m_backLeft.UpdateModulePosition();
  m_SM.m_backRight.UpdateModulePosition();

  // Get current positions after update
  m_modulePositions[0] = m_SM.m_frontLeft.GetSwerveModulePosition();
  m_modulePositions[1] = m_SM.m_frontRight.GetSwerveModulePosition();
  m_modulePositions[2] = m_SM.m_backLeft.GetSwerveModulePosition();
  m_modulePositions[3] = m_SM.m_backRight.GetSwerveModulePosition();

  m_poseEstimator.Update(m_pigeon.GetRotation2d(), m_modulePositions);
}

void SwerveDrive::SetCurrentPose(frc::Pose2d pose) {
  ResetSpeedEncoders();
  m_poseEstimator.ResetPosition(m_pigeon.GetRotation2d(),
                           m_modulePositions,
                           pose);
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
// SHOULD CHANGE VISION STD DEVS TO REFLECT DISTANCE
}

double SwerveDrive::Calculate(double measurement, double setpoint) {
  return AutoParameters::thetaPIDController.Calculate(units::radian_t{measurement}, units::radian_t{setpoint});
}

void SwerveDrive::EnableContinuousInput(double minimumInput, double maximumInput) {
  AutoParameters::thetaPIDController.EnableContinuousInput(units::radian_t{minimumInput}, units::radian_t{maximumInput});
}

void SwerveDrive::Stop() {
  m_SM.m_frontLeft.Stop();
  m_SM.m_frontRight.Stop();
  m_SM.m_backLeft.Stop();
  m_SM.m_backRight.Stop();
}

void SwerveDrive::InitSmartDashboard() {
  // Add swerve PID values to PID tabs
  // Allows for editing of PID values during runtime
  frc::SmartDashboard::PutNumber("Robot yaw", GetNormalizedYaw());
  EntryRotationP = SwerveRotationPIDTab.Add("Rotation P", AutoParameters::thetaPIDController.GetP()).GetEntry();
  EntryRotationI = SwerveRotationPIDTab.Add("Rotation I", AutoParameters::thetaPIDController.GetI()).GetEntry();
  EntryRotationD = SwerveRotationPIDTab.Add("Rotation D", AutoParameters::thetaPIDController.GetD()).GetEntry();
  // EntryFrontLeftSpeedP = SwerveFLPIDTab.Add("Front Left Speed P", m_SM.m_frontLeft.GetSpeedP()).GetEntry();
  // EntryFrontLeftSpeedI = SwerveFLPIDTab.Add("Front Left Speed I", m_SM.m_frontLeft.GetSpeedI()).GetEntry();
  // EntryFrontLeftSpeedD = SwerveFLPIDTab.Add("Front Left Speed D", m_SM.m_frontLeft.GetSpeedD()).GetEntry();
  // EntryFrontLeftAngleP = SwerveFLPIDTab.Add("Front Left Angle P", m_SM.m_frontLeft.GetAngleP()).GetEntry();
  // EntryFrontLeftAngleI = SwerveFLPIDTab.Add("Front Left Angle I", m_SM.m_frontLeft.GetAngleI()).GetEntry();
  // EntryFrontLeftAngleD = SwerveFLPIDTab.Add("Front Left Angle D", m_SM.m_frontLeft.GetAngleD()).GetEntry();

  // EntryFrontRightSpeedP = SwerveFRPIDTab.Add("Front Right Speed P", m_SM.m_frontRight.GetSpeedP()).GetEntry();
  // EntryFrontRightSpeedI = SwerveFRPIDTab.Add("Front Right Speed I", m_SM.m_frontRight.GetSpeedI()).GetEntry();
  // EntryFrontRightSpeedD = SwerveFRPIDTab.Add("Front Right Speed D", m_SM.m_frontRight.GetSpeedD()).GetEntry();
  // EntryFrontRightAngleP = SwerveFRPIDTab.Add("Front Right Angle P", m_SM.m_frontRight.GetAngleP()).GetEntry();
  // EntryFrontRightAngleI = SwerveFRPIDTab.Add("Front Right Angle I", m_SM.m_frontRight.GetAngleI()).GetEntry();
  // EntryFrontRightAngleD = SwerveFRPIDTab.Add("Front Right Angle D", m_SM.m_frontRight.GetAngleD()).GetEntry();

  // EntryBackLeftSpeedP = SwerveBLPIDTab.Add("Back Left Speed P", m_SM.m_backLeft.GetSpeedP()).GetEntry();
  // EntryBackLeftSpeedI = SwerveBLPIDTab.Add("Back Left Speed I", m_SM.m_backLeft.GetSpeedI()).GetEntry();
  // EntryBackLeftSpeedD = SwerveBLPIDTab.Add("Back Left Speed D", m_SM.m_backLeft.GetSpeedD()).GetEntry();
  // EntryBackLeftAngleP = SwerveBLPIDTab.Add("Back Left Angle P", m_SM.m_backLeft.GetAngleP()).GetEntry();
  // EntryBackLeftAngleI = SwerveBLPIDTab.Add("Back Left Angle I", m_SM.m_backLeft.GetAngleI()).GetEntry();
  // EntryBackLeftAngleD = SwerveBLPIDTab.Add("Back Left Angle D", m_SM.m_backLeft.GetAngleD()).GetEntry();

  // EntryBackRightSpeedP = SwerveBRPIDTab.Add("Back Right Speed P", m_SM.m_backRight.GetSpeedP()).GetEntry();
  // EntryBackRightSpeedI = SwerveBRPIDTab.Add("Back Right Speed I", m_SM.m_backRight.GetSpeedI()).GetEntry();
  // EntryBackRightSpeedD = SwerveBRPIDTab.Add("Back Right Speed D", m_SM.m_backRight.GetSpeedD()).GetEntry();
  // EntryBackRightAngleP = SwerveBRPIDTab.Add("Back Right Angle P", m_SM.m_backRight.GetAngleP()).GetEntry();
  // EntryBackRightAngleI = SwerveBRPIDTab.Add("Back Right Angle I", m_SM.m_backRight.GetAngleI()).GetEntry();
  // EntryBackRightAngleD = SwerveBRPIDTab.Add("Back Right Angle D", m_SM.m_backRight.GetAngleD()).GetEntry();

  // Add all 40 swerve entries to drive tab in Shuffleboard
  EntryFrontLeftSpeed = SwerveSpeedTab.Add("Front Left Speed", 0).GetEntry();
  EntryFrontLeftPosition = SwervePositionTab.Add("Front Left Position", 0).GetEntry();
  EntryFrontLeftAngleVelocity = SwerveAngleVelocityTab.Add("Front Left Angle Velocity", 0).GetEntry();
  EntryFrontLeftSpeedMotorCurrent = SwerveSpeedMotorCurrentTab.Add("Front Left Speed Motor Current", 0).GetEntry();
  EntryFrontLeftSpeedMotorVoltage = SwerveSpeedMotorVoltageTab.Add("Front Left Speed Motor Voltage", 0).GetEntry();
  EntryFrontLeftAngleMotorCurrent = SwerveAngleMotorCurrentTab.Add("Front Left Angle Motor Current", 0).GetEntry();
  EntryFrontLeftAngleMotorVoltage = SwerveAngleMotorVoltageTab.Add("Front Left Angle Motor Voltage", 0).GetEntry();
  EntryFrontLeftAbsoluteAngle = SwerveAbsoluteAngleTab.Add("Front Left Absolute Angle", 0).GetEntry();
  EntryFrontLeftRelativeAngle = SwerveRelativeAngleTab.Add("Front Left Relative Angle", 0).GetEntry();
  
  EntryFrontRightSpeed = SwerveSpeedTab.Add("Front Right Speed", 0).GetEntry();
  EntryFrontRightPosition = SwervePositionTab.Add("Front Right Position", 0).GetEntry();
  EntryFrontRightAngleVelocity = SwerveAngleVelocityTab.Add("Front Right Angle Velocity", 0).GetEntry();
  EntryFrontRightSpeedMotorCurrent = SwerveSpeedMotorCurrentTab.Add("Front Right Speed Motor Current", 0).GetEntry();
  EntryFrontRightSpeedMotorVoltage = SwerveSpeedMotorVoltageTab.Add("Front Right Speed Motor Voltage", 0).GetEntry();
  EntryFrontRightAngleMotorCurrent = SwerveAngleMotorCurrentTab.Add("Front Right Angle Motor Current", 0).GetEntry();
  EntryFrontRightAngleMotorVoltage = SwerveAngleMotorVoltageTab.Add("Front Right Angle Motor Voltage", 0).GetEntry();
  EntryFrontRightAbsoluteAngle = SwerveAbsoluteAngleTab.Add("Front Right Absolute Angle", 0).GetEntry();
  EntryFrontRightRelativeAngle = SwerveRelativeAngleTab.Add("Front Right Relative Angle", 0).GetEntry();
  
  EntryBackLeftSpeed = SwerveSpeedTab.Add("Back Left Speed", 0).GetEntry();
  EntryBackLeftPosition = SwervePositionTab.Add("Back Left Position", 0).GetEntry();
  EntryBackLeftAngleVelocity = SwerveAngleVelocityTab.Add("Back Left Angle Velocity", 0).GetEntry();
  EntryBackLeftSpeedMotorCurrent = SwerveSpeedMotorCurrentTab.Add("Back Left Speed Motor Current", 0).GetEntry();
  EntryBackLeftSpeedMotorVoltage = SwerveSpeedMotorVoltageTab.Add("Back Left Speed Motor Voltage", 0).GetEntry();
  EntryBackLeftAngleMotorCurrent = SwerveAngleMotorCurrentTab.Add("Back Left Angle Motor Current", 0).GetEntry();
  EntryBackLeftAngleMotorVoltage = SwerveAngleMotorVoltageTab.Add("Back Left Angle Motor Voltage", 0).GetEntry();
  EntryBackLeftAbsoluteAngle = SwerveAbsoluteAngleTab.Add("Back Left Absolute Angle", 0).GetEntry();
  EntryBackLeftRelativeAngle = SwerveRelativeAngleTab.Add("Back Left Relative Angle", 0).GetEntry();
  
  EntryBackRightSpeed = SwerveSpeedTab.Add("Back Right Speed", 0).GetEntry();
  EntryBackRightPosition = SwervePositionTab.Add("Back Right Position", 0).GetEntry();
  EntryBackRightAngleVelocity = SwerveAngleVelocityTab.Add("Back Right Angle Velocity", 0).GetEntry();
  EntryBackRightSpeedMotorCurrent = SwerveSpeedMotorCurrentTab.Add("Back Right Speed Motor Current", 0).GetEntry();
  EntryBackRightSpeedMotorVoltage = SwerveSpeedMotorVoltageTab.Add("Back Right Speed Motor Voltage", 0).GetEntry();
  EntryBackRightAngleMotorCurrent = SwerveAngleMotorCurrentTab.Add("Back Right Angle Motor Current", 0).GetEntry();
  EntryBackRightAngleMotorVoltage = SwerveAngleMotorVoltageTab.Add("Back Right Angle Motor Voltage", 0).GetEntry();
  EntryBackRightAbsoluteAngle = SwerveAbsoluteAngleTab.Add("Back Right Absolute Angle", 0).GetEntry();
  EntryBackRightRelativeAngle = SwerveRelativeAngleTab.Add("Back Right Relative Angle", 0).GetEntry();

  frc::SmartDashboard::PutData(this);
}

void SwerveDrive::SetActiveTrajectory(frc::Trajectory trajectory) {
  m_activeTrajectory = trajectory;
}

void SwerveDrive::Log2DField() {
  // Uses Field2d class as it implements sendable for 2d poses
  // Also supports trajectories, game pieces, etc. as field objects
  m_fieldObject.SetRobotPose(GetEstimatedPose());
  auto trajectoryObject = m_fieldObject.GetObject("Trajectory");
  trajectoryObject->SetTrajectory(m_activeTrajectory);
  frc::SmartDashboard::PutData("Field object", &m_fieldObject);
}

void SwerveDrive::LockWheels() {
  // Sets modules to point in an "X" shape, which means it is less likely to be pushed or slide
  m_SM.m_frontLeft.Lock(frc::SwerveModuleState{0.0_mps, units::radian_t{-pi/ 4.0}});
  m_SM.m_frontRight.Lock(frc::SwerveModuleState{0.0_mps, units::radian_t{pi/ 4.0}});
  m_SM.m_backLeft.Lock(frc::SwerveModuleState{0.0_mps, units::radian_t{pi/ 4.0}});
  m_SM.m_backRight.Lock(frc::SwerveModuleState{0.0_mps, units::radian_t{-pi/ 4.0}});
}

void SwerveDrive::LogModuleStates(SwerveModuleTelemetry telemetryArray[]) {
// Measured states to be logged in AdvantageScope (requires different format)
  double AdvantageScopeMeasuredStates[] = 
    {telemetryArray[0].relativeAngle, ((double)m_SM.m_frontLeft.FalconToMeters(telemetryArray[0].speed)),
     telemetryArray[1].relativeAngle, ((double)m_SM.m_frontLeft.FalconToMeters(telemetryArray[1].speed)),
     telemetryArray[2].relativeAngle, ((double)m_SM.m_frontLeft.FalconToMeters(telemetryArray[2].speed)),
     telemetryArray[3].relativeAngle, ((double)m_SM.m_frontLeft.FalconToMeters(telemetryArray[3].speed))};
  frc::SmartDashboard::PutNumberArray("AdvantageScope Measured States", AdvantageScopeMeasuredStates);
}

void SwerveDrive::UpdatePIDValues() {
  // AutoParameters::thetaPIDController.SetP(GetEntryRotationP());
  // AutoParameters::thetaPIDController.SetI(GetEntryRotationI());
  // AutoParameters::thetaPIDController.SetD(GetEntryRotationD());
  // m_SM.m_frontLeft.SetSpeedP(GetEntryFLSpeedP());
  // m_SM.m_frontLeft.SetSpeedI(GetEntryFLSpeedI());
  // m_SM.m_frontLeft.SetSpeedD(GetEntryFLSpeedD());
  // m_SM.m_frontLeft.SetAngleP(GetEntryFLAngleP());
  // m_SM.m_frontLeft.SetAngleI(GetEntryFLAngleI());
  // m_SM.m_frontLeft.SetAngleD(GetEntryFLAngleD());
  // m_SM.m_frontRight.SetSpeedP(GetEntryFRSpeedP());
  // m_SM.m_frontRight.SetSpeedI(GetEntryFRSpeedI());
  // m_SM.m_frontRight.SetSpeedD(GetEntryFRSpeedD());
  // m_SM.m_frontRight.SetAngleP(GetEntryFRAngleP());
  // m_SM.m_frontRight.SetAngleI(GetEntryFRAngleI());
  // m_SM.m_frontRight.SetAngleD(GetEntryFRAngleD());
  // m_SM.m_backLeft.SetSpeedP(GetEntryBLSpeedP());
  // m_SM.m_backLeft.SetSpeedI(GetEntryBLSpeedI());
  // m_SM.m_backLeft.SetSpeedD(GetEntryBLSpeedD());
  // m_SM.m_backLeft.SetAngleP(GetEntryBLAngleP());
  // m_SM.m_backLeft.SetAngleI(GetEntryBLAngleI());
  // m_SM.m_backLeft.SetAngleD(GetEntryBLAngleD());
  // m_SM.m_backRight.SetSpeedP(GetEntryBRSpeedP());
  // m_SM.m_backRight.SetSpeedI(GetEntryBRSpeedI());
  // m_SM.m_backRight.SetSpeedD(GetEntryBRSpeedD());
  // m_SM.m_backRight.SetAngleP(GetEntryBRAngleP());
  // m_SM.m_backRight.SetAngleI(GetEntryBRAngleI());
  // m_SM.m_backRight.SetAngleD(GetEntryBRAngleD());
}

void SwerveDrive::SyncSmartdashBoardValues() {
  frc::SmartDashboard::PutNumber("Robot Pitch", GetPitch());
  frc::SmartDashboard::PutNumber("Robot Roll", GetRoll());

  // Collect telemetry values - set shuffleboard entries to struct.variable
  SwerveModuleTelemetry frontLeftTelemetry = m_SM.m_frontLeft.GetTelemetry();
  SwerveModuleTelemetry frontRightTelemetry = m_SM.m_frontRight.GetTelemetry();
  SwerveModuleTelemetry backLeftTelemetry = m_SM.m_backLeft.GetTelemetry();
  SwerveModuleTelemetry backRightTelemetry = m_SM.m_backRight.GetTelemetry();

  SwerveModuleTelemetry telemetryArray[] = {frontLeftTelemetry, frontRightTelemetry, backLeftTelemetry, backRightTelemetry};
  LogModuleStates(telemetryArray);
  Log2DField();
  
  EntryFrontLeftSpeed->SetDouble(frontLeftTelemetry.speed);
  EntryFrontLeftPosition->SetDouble(frontLeftTelemetry.position);
  EntryFrontLeftAngleVelocity->SetDouble(frontLeftTelemetry.angleVelocity);
  EntryFrontLeftSpeedMotorCurrent->SetDouble(frontLeftTelemetry.speedMotorCurrent);
  EntryFrontLeftSpeedMotorVoltage->SetDouble(frontLeftTelemetry.speedMotorVoltage);
  EntryFrontLeftAngleMotorCurrent->SetDouble(frontLeftTelemetry.angleMotorCurrent);
  EntryFrontLeftAngleMotorVoltage->SetDouble(frontLeftTelemetry.angleMotorVoltage);
  EntryFrontLeftAbsoluteAngle->SetDouble(frontLeftTelemetry.absoluteAngle);
  EntryFrontLeftRelativeAngle->SetDouble(frontLeftTelemetry.relativeAngle);
  EntryFrontRightSpeed->SetDouble(frontRightTelemetry.speed);
  EntryFrontRightPosition->SetDouble(frontRightTelemetry.position);
  EntryFrontRightAngleVelocity->SetDouble(frontRightTelemetry.angleVelocity);
  EntryFrontRightSpeedMotorCurrent->SetDouble(frontRightTelemetry.speedMotorCurrent);
  EntryFrontRightSpeedMotorVoltage->SetDouble(frontRightTelemetry.speedMotorVoltage);
  EntryFrontRightAngleMotorCurrent->SetDouble(frontRightTelemetry.angleMotorCurrent);
  EntryFrontRightAngleMotorVoltage->SetDouble(frontRightTelemetry.angleMotorVoltage);
  EntryFrontRightAbsoluteAngle->SetDouble(frontRightTelemetry.absoluteAngle);
  EntryFrontRightRelativeAngle->SetDouble(frontRightTelemetry.relativeAngle);
  EntryBackLeftSpeed->SetDouble(backLeftTelemetry.speed);
  EntryBackLeftPosition->SetDouble(backLeftTelemetry.position);
  EntryBackLeftAngleVelocity->SetDouble(backLeftTelemetry.angleVelocity);
  EntryBackLeftSpeedMotorCurrent->SetDouble(backLeftTelemetry.speedMotorCurrent);
  EntryBackLeftSpeedMotorVoltage->SetDouble(backLeftTelemetry.speedMotorVoltage);
  EntryBackLeftAngleMotorCurrent->SetDouble(backLeftTelemetry.angleMotorCurrent);
  EntryBackLeftAngleMotorVoltage->SetDouble(backLeftTelemetry.angleMotorVoltage);
  EntryBackLeftAbsoluteAngle->SetDouble(backLeftTelemetry.absoluteAngle);
  EntryBackLeftRelativeAngle->SetDouble(backLeftTelemetry.relativeAngle);
  EntryBackRightSpeed->SetDouble(backRightTelemetry.speed);
  EntryBackRightPosition->SetDouble(backRightTelemetry.position);
  EntryBackRightAngleVelocity->SetDouble(backRightTelemetry.angleVelocity);
  EntryBackRightSpeedMotorCurrent->SetDouble(backRightTelemetry.speedMotorCurrent);
  EntryBackRightSpeedMotorVoltage->SetDouble(backRightTelemetry.speedMotorVoltage);
  EntryBackRightAngleMotorCurrent->SetDouble(backRightTelemetry.angleMotorCurrent);
  EntryBackRightAngleMotorVoltage->SetDouble(backRightTelemetry.angleMotorVoltage);
  EntryBackRightAbsoluteAngle->SetDouble(backRightTelemetry.absoluteAngle);
  EntryBackRightRelativeAngle->SetDouble(backRightTelemetry.relativeAngle);
}

void SwerveDrive::SimulationPeriodic() {
  // Implementation of subsystem simulation periodic method goes here.
}

double SwerveDrive::GetEntryRotationP() { return EntryRotationP->GetDouble(0); }
double SwerveDrive::GetEntryRotationI() { return EntryRotationI->GetDouble(0); }
double SwerveDrive::GetEntryRotationD() { return EntryRotationD->GetDouble(0); }
double SwerveDrive::GetEntryFLSpeedP() { return EntryFrontLeftSpeedP->GetDouble(0); }
double SwerveDrive::GetEntryFLSpeedI() { return EntryFrontLeftSpeedI->GetDouble(0); }
double SwerveDrive::GetEntryFLSpeedD() { return EntryFrontLeftSpeedD->GetDouble(0); }
double SwerveDrive::GetEntryFLAngleP() { return EntryFrontLeftAngleP->GetDouble(0); }
double SwerveDrive::GetEntryFLAngleI() { return EntryFrontLeftAngleI->GetDouble(0); }
double SwerveDrive::GetEntryFLAngleD() { return EntryFrontLeftAngleD->GetDouble(0); }
double SwerveDrive::GetEntryFRSpeedP() { return EntryFrontRightSpeedP->GetDouble(0); }
double SwerveDrive::GetEntryFRSpeedI() { return EntryFrontRightSpeedI->GetDouble(0); }
double SwerveDrive::GetEntryFRSpeedD() { return EntryFrontRightSpeedD->GetDouble(0); }
double SwerveDrive::GetEntryFRAngleP() { return EntryFrontRightAngleP->GetDouble(0); }
double SwerveDrive::GetEntryFRAngleI() { return EntryFrontRightAngleI->GetDouble(0); }
double SwerveDrive::GetEntryFRAngleD() { return EntryFrontRightAngleD->GetDouble(0); }
double SwerveDrive::GetEntryBLSpeedP() { return EntryBackLeftSpeedP->GetDouble(0); }
double SwerveDrive::GetEntryBLSpeedI() { return EntryBackLeftSpeedI->GetDouble(0); }
double SwerveDrive::GetEntryBLSpeedD() { return EntryBackLeftSpeedD->GetDouble(0); }
double SwerveDrive::GetEntryBLAngleP() { return EntryBackLeftAngleP->GetDouble(0); }
double SwerveDrive::GetEntryBLAngleI() { return EntryBackLeftAngleI->GetDouble(0); }
double SwerveDrive::GetEntryBLAngleD() { return EntryBackLeftAngleD->GetDouble(0); }
double SwerveDrive::GetEntryBRSpeedP() { return EntryBackRightSpeedP->GetDouble(0); }
double SwerveDrive::GetEntryBRSpeedI() { return EntryBackRightSpeedI->GetDouble(0); }
double SwerveDrive::GetEntryBRSpeedD() { return EntryBackRightSpeedD->GetDouble(0); }
double SwerveDrive::GetEntryBRAngleP() { return EntryBackRightAngleP->GetDouble(0); }
double SwerveDrive::GetEntryBRAngleI() { return EntryBackRightAngleI->GetDouble(0); }
double SwerveDrive::GetEntryBRAngleD() { return EntryBackRightAngleD->GetDouble(0); }

std::function<double(void)> SwerveDrive::GetEntryRotationPFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryRotationP, 0); }
std::function<double(void)> SwerveDrive::GetEntryRotationIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryRotationI, 0); }
std::function<double(void)> SwerveDrive::GetEntryRotationDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryRotationD, 0); }
std::function<double(void)> SwerveDrive::GetEntryFLSpeedPFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontLeftSpeedP, 0); }
std::function<double(void)> SwerveDrive::GetEntryFLSpeedIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontLeftSpeedI, 0); }
std::function<double(void)> SwerveDrive::GetEntryFLSpeedDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontLeftSpeedD, 0); }
std::function<double(void)> SwerveDrive::GetEntryFLAnglePFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontLeftAngleP, 0); }
std::function<double(void)> SwerveDrive::GetEntryFLAngleIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontLeftAngleI, 0); }
std::function<double(void)> SwerveDrive::GetEntryFLAngleDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontLeftAngleD, 0); }
std::function<double(void)> SwerveDrive::GetEntryFRSpeedPFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontRightSpeedP, 0); }
std::function<double(void)> SwerveDrive::GetEntryFRSpeedIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontRightSpeedI, 0); }
std::function<double(void)> SwerveDrive::GetEntryFRSpeedDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontRightSpeedD, 0); }
std::function<double(void)> SwerveDrive::GetEntryFRAnglePFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontRightAngleP, 0); }
std::function<double(void)> SwerveDrive::GetEntryFRAngleIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontRightAngleI, 0); }
std::function<double(void)> SwerveDrive::GetEntryFRAngleDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryFrontRightAngleD, 0); }
std::function<double(void)> SwerveDrive::GetEntryBLSpeedPFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackLeftSpeedP, 0); }
std::function<double(void)> SwerveDrive::GetEntryBLSpeedIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackLeftSpeedI, 0); }
std::function<double(void)> SwerveDrive::GetEntryBLSpeedDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackLeftSpeedD, 0); }
std::function<double(void)> SwerveDrive::GetEntryBLAnglePFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackLeftAngleP, 0); }
std::function<double(void)> SwerveDrive::GetEntryBLAngleIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackLeftAngleI, 0); }
std::function<double(void)> SwerveDrive::GetEntryBLAngleDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackLeftAngleD, 0); }
std::function<double(void)> SwerveDrive::GetEntryBRSpeedPFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackRightSpeedP, 0); }
std::function<double(void)> SwerveDrive::GetEntryBRSpeedIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackRightSpeedI, 0); }
std::function<double(void)> SwerveDrive::GetEntryBRSpeedDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackRightSpeedD, 0); }
std::function<double(void)> SwerveDrive::GetEntryBRAnglePFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackRightAngleP, 0); }
std::function<double(void)> SwerveDrive::GetEntryBRAngleIFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackRightAngleI, 0); }
std::function<double(void)> SwerveDrive::GetEntryBRAngleDFunc() { return std::bind(&nt::GenericEntry::GetDouble, EntryBackRightAngleD, 0); }