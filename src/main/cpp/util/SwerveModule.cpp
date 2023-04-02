// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "util/SwerveModule.h"

SwerveModule::SwerveModule(SwerveInfo SI) :
SI(SI),
speedP(SI.speedMotorPID.p),
speedI(SI.speedMotorPID.i),
speedD(SI.speedMotorPID.d),
angleP(SI.angleMotorPID.p),
angleI(SI.angleMotorPID.i),
angleD(SI.angleMotorPID.d),
m_speedMotor(SI.speedMotorID, "Swerve"),
m_angleMotor(SI.angleMotorID, "Swerve"),
m_angleOffset(SI.encoderOffset),
m_absoluteEncoder(SI.CANCoderID, "Swerve")
{
    m_speedMotor.ConfigFactoryDefault();
    m_angleMotor.ConfigFactoryDefault();
    m_speedMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    m_speedMotor.ConfigFeedbackNotContinuous(true);
    m_speedMotor.SetNeutralMode(NeutralMode::Brake);
    m_speedMotor.SetInverted(true);
    m_speedMotor.Config_kP(0, speedP, 50);
    m_speedMotor.Config_kI(0, speedI, 50);
    m_speedMotor.Config_kD(0, speedD, 50);
    // m_speedMotor.ConfigClosedloopRamp(SwerveDriveConstants::loopRampRate);
    m_speedMotor.ConfigClosedloopRamp(0);
    m_speedMotor.ConfigOpenloopRamp(0.01);
    m_speedMotor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{
                                            true, SwerveDriveConstants::ampLimit, SwerveDriveConstants::ampLimit + 5.0, 0.1});

    m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::IntegratedSensor);
    // m_angleMotor.ConfigRemoteFeedbackFilter(m_absoluteEncoder.GetDeviceNumber(), RemoteSensorSource::RemoteSensorSource_CANCoder, 0);
    // m_angleMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor0);
    // Allows PIDController to treat two angles as the same point on a circle
    m_angleMotor.ConfigIntegratedSensorAbsoluteRange(AbsoluteSensorRange::Unsigned_0_to_360);
    m_angleMotor.ConfigIntegratedSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    m_angleMotor.SetNeutralMode(NeutralMode::Brake);
    m_angleMotor.Config_kP(0, angleP, 50);
    m_angleMotor.Config_kI(0, angleI, 50);
    m_angleMotor.Config_kD(0, angleD, 50);
    m_angleMotor.ConfigFeedbackNotContinuous(false);
    // Limits acceleration of motors and current drawn
    m_angleMotor.ConfigOpenloopRamp(SwerveDriveConstants::loopRampRate);
    // Enable current limiting, set current to limit down to as ampLimit, current to start limiting to as the same, and
    // time before limiting to 1 second (arbitrary)
    m_angleMotor.ConfigStatorCurrentLimit(StatorCurrentLimitConfiguration{
                                            true, SwerveDriveConstants::ampLimit, SwerveDriveConstants::ampLimit + 10.0, 0.1});

    m_absoluteEncoder.ConfigSensorDirection(true);
    m_absoluteEncoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Unsigned_0_to_360);
    m_absoluteEncoder.ConfigSensorInitializationStrategy(SensorInitializationStrategy::BootToAbsolutePosition);
    ResetSpeedEncoder();
    ResetAngleToAbsolute();
    // m_anglePIDController.SetTolerance({.5});
    // Absolute position currently working weirdly
    // m_absoluteEncoder.SetPositionToAbsolute(); 
    // Configs distance per encoder pulse -- necessary for movement by position
    // Doesn't exist for test encoders -> m_speedEncoder.SetDistancePerPulse(1 / 4096);
}

void SwerveModule::Stop() {
    m_speedMotor.StopMotor();
    m_angleMotor.StopMotor();
}

// Encoder conversion functions -- CANCoder has different CPR than Falcon
// CANCoders are 1:1 with output shaft, Falcons have a gear ratio
double SwerveModule::DegreesToCANCoder(double degrees) {
    return degrees * 
                    (SwerveDriveConstants::cancoderTicksPerRevolution / 360);
}

double SwerveModule::DegreesToFalcon(double degrees) {
    return degrees * 
                    ((SwerveDriveConstants::encoderTurnGearRatio * SwerveDriveConstants::falconEncoderTicksPerRevolution) / 360);
}

double SwerveModule::CANCoderToDegrees(double encoderTicks) {
    return encoderTicks / 
                    (SwerveDriveConstants::cancoderTicksPerRevolution / 360);
}

double SwerveModule::FalconToDegrees(double encoderTicks) {
    return encoderTicks / 
                    ((SwerveDriveConstants::encoderTurnGearRatio * SwerveDriveConstants::falconEncoderTicksPerRevolution) / 360);
}

units::meter_t SwerveModule::FalconToMeters(double encoderTicks) {
    return units::meter_t{encoderTicks * (SwerveDriveConstants::wheelCircumferenceMeters / (SwerveDriveConstants::encoderSpeedGearRatio * SwerveDriveConstants::falconEncoderTicksPerRevolution))};
}

void SwerveModule::DriveFast() {
    m_speedMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 1.0);
}

void SwerveModule::SetDesiredPercentState(const frc::SwerveModuleState &input_state) {
    const auto state = OptimizeAngle(
        input_state, units::degree_t{FalconToDegrees(m_angleMotor.GetSelectedSensorPosition())});
    double vel = state.speed.value();
    double speedPercent = vel / (double)SwerveDriveConstants::kMaxSpeed;
    m_speedMotor.Set(TalonFXControlMode::PercentOutput, speedPercent);
    double turnSetpoint = DegreesToFalcon(state.angle.Degrees().value());
    if (fabs(vel) < .025 && (m_lastAngle - turnSetpoint) < 5.0) {
        Stop();
        turnSetpoint = m_lastAngle;
    } else {
        m_angleMotor.Set(TalonFXControlMode::Position, turnSetpoint);
    }
    m_lastAngle = turnSetpoint;
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &input_state) {
    // First optimize swerve states (figures out whether CW or CCW is less turning angle)
    frc::SmartDashboard::PutNumber("Input state angle", input_state.angle.Degrees().value());
    const auto state = OptimizeAngle(
        input_state, units::degree_t{FalconToDegrees(m_angleMotor.GetSelectedSensorPosition())});
    
    // const auto state = input_state;
    double vel = state.speed.value();
    
    auto acceleration = (state.speed - m_lastSpeed) /
      (frc::Timer::GetFPGATimestamp() - m_lastTime);

    units::volt_t ffValue = std::clamp(ff.Calculate(state.speed, acceleration), -12.0_V, 12.0_V);
    m_speedMotor.Set(TalonFXControlMode::PercentOutput, (ffValue / 12.0_V));

    m_lastSpeed = state.speed;
    m_lastTime = frc::Timer::GetFPGATimestamp();

    // double speedPercent = vel / (double)SwerveDriveConstants::kMaxSpeed;
    frc::SmartDashboard::PutNumber("Target Swerve Speed Percent", (double)(ffValue / 12.0_V));

    // Tells PID controller to run with calculated module speed
    // m_speedMotor.Set(TalonFXControlMode::PercentOutput, speedPercent);
    
    // Calculate position to set angle PID to
    double turnSetpoint = DegreesToFalcon(state.angle.Degrees().value());
    // double turnSetpoint = -25000.0;
    // If robot wants to barely move or rotate module, stop motors
    // Also sets next setpoint as current angle so no rotation happens
    // std::cout << "Velocity in mps: " << vel << " Turn difference: " << (m_lastAngle - turnSetpoint) << std::endl;

    // Keep wheels in current spot instead of resetting to 0
    if (fabs(vel) < .025 && (m_lastAngle - turnSetpoint) < 5.0) {
        // std::cout << "Stopping motors";
        Stop();
        turnSetpoint = m_lastAngle;
    } else {
        m_angleMotor.Set(TalonFXControlMode::Position, turnSetpoint);
        frc::SmartDashboard::PutNumber("Target Swerve Angle Percent", m_angleMotor.GetMotorOutputPercent());
    }

    m_lastAngle = turnSetpoint;
}

frc::SwerveModuleState SwerveModule::OptimizeAngle(frc::SwerveModuleState desiredState, frc::Rotation2d currentAngle) {
    double targetAngle = NormalizeTo0To360(
                        currentAngle.Degrees().value(), 
                        desiredState.angle.Degrees().value());
    // double targetAngle = desiredState.angle.Degrees().value();
    frc::SmartDashboard::PutNumber("Optimize target angle", targetAngle);

    auto targetSpeed = desiredState.speed;
    double delta = targetAngle - currentAngle.Degrees().value();
    // If degrees to target is > 90, reverse the speed motor direction
    if (std::abs(delta) > 90) {
        targetSpeed = -targetSpeed;
        // If difference is positive, subtract 180 deg and reverse; if negative, vice versa
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        frc::SmartDashboard::PutNumber("Optimized target angle", targetAngle);
    }
    auto targetAngleRad = units::radian_t{targetAngle / SwerveDriveConstants::DEGToRAD};
    return frc::SwerveModuleState{targetSpeed, targetAngleRad};
}

double SwerveModule::NormalizeTo0To360(double currentAngle, double targetAngle) {
    // 0 to 360 range we want to be in
    double lowerBound;
    double upperBound;
    // Offset from current angle to lower bound
    int lowerOffset = (int)currentAngle % 360;
    frc::SmartDashboard::PutNumber("Normalize Lower Offset", lowerOffset);
    /* If angle is positive, angle will be set from 0 to 360, else -360 to 0
    * These degree numbers are relative to input angle
    * This is because WPILib optimize expects a continuous input from a PIDController
    * But Falcon500s use internal PIDControllers without continuous input
    */ 
    if (lowerOffset >= 0) {
          lowerBound = currentAngle - lowerOffset;
          upperBound = currentAngle + (360 - lowerOffset);
      } else {
          upperBound = currentAngle - lowerOffset;
          lowerBound = currentAngle - (360 + lowerOffset);
      }
    frc::SmartDashboard::PutNumber("Normalize Lower Bound", lowerBound);
    frc::SmartDashboard::PutNumber("Normalize Upper Bound", upperBound);
      // Target angle is now normalized between either 0 to 360 or -360 to 0
      while (targetAngle < lowerBound) {
          targetAngle += 360;
          frc::SmartDashboard::PutNumber("Increasing to bound", targetAngle);
      }
      while (targetAngle > upperBound) {
          targetAngle -= 360;
          frc::SmartDashboard::PutNumber("Decreasing to bound", targetAngle);
      }
      if (targetAngle - currentAngle > 180) {
          targetAngle -= 360;
      } else if (targetAngle - currentAngle < -180) {
          targetAngle += 360;
      }
      return targetAngle;
}

void SwerveModule::ManualModuleSpeed(double speed) {
    m_speedMotor.Set(speed);
}

void SwerveModule::ManualModuleTurn(double speed) {
    m_angleMotor.Set(speed);
}

void SwerveModule::ResetSpeedEncoder() {
    m_speedMotor.SetSelectedSensorPosition(0);
}

void SwerveModule::ResetAngleToAbsolute() {
    /* Keep resetting if absolute encoder reports timeout error
    * This prevents accidentally zeroing on boot if the CANCoder 
    * takes too long to respond instead of resetting to absolute
    * Since CANCoder is set to boot to absolute, we can just use normal position
    * Normal get position call runs 10X faster so there's less chance of
    * taking too long to respond
    * Modulo the position call since it is unwrapped (goes above 360 deg)
    */ 
    double absoluteAngle;
    int counter = 1;
    do {
        absoluteAngle = DegreesToFalcon(((int)m_absoluteEncoder.GetPosition() % 360) - m_angleOffset);
        std::cout << "CANCoder seeding" << counter << std::endl;
        counter++;
    }
    while (m_absoluteEncoder.GetLastError() != ErrorCode::OK && (counter < 5)); 
    std::cout << "Seeding complete" << std::endl;
    m_angleMotor.SetSelectedSensorPosition(absoluteAngle);
}

frc::SwerveModuleState SwerveModule::GetState() {
    return {units::meters_per_second_t{m_speedMotor.GetSelectedSensorVelocity()},
        units::radian_t{m_angleMotor.GetSelectedSensorPosition()}};
}

double SwerveModule::GetAbsolutePosition() {
    return m_absoluteEncoder.GetAbsolutePosition();
}

frc::SwerveModulePosition SwerveModule::GetSwerveModulePosition() {
    return m_swervePosition;
}

void SwerveModule::UpdateModulePosition() {
    m_swervePosition.distance = FalconToMeters(m_speedMotor.GetSelectedSensorPosition());
    m_swervePosition.angle = frc::Rotation2d{units::degree_t{(GetRelativeAngle())}};
}

double SwerveModule::GetRelativeAngle() {
    return FalconToDegrees(m_angleMotor.GetSelectedSensorPosition());
}

double SwerveModule::GetVelocity() {
    return m_speedMotor.GetSelectedSensorVelocity();
}

SwerveModuleTelemetry SwerveModule::GetTelemetry() {
    // Get values from sensors - init telemetry struct with values
    // then use in SwerveDrive.cpp dashboard functions
    double speed = m_speedMotor.GetSelectedSensorVelocity();
    double position = m_speedMotor.GetSelectedSensorPosition();
    double angleVelocity = m_angleMotor.GetSelectedSensorVelocity();
    double speedCurrent = m_speedMotor.GetOutputCurrent();
    double speedVoltage = m_speedMotor.GetBusVoltage();
    double angleCurrent = m_angleMotor.GetOutputCurrent();
    double angleVoltage = m_angleMotor.GetBusVoltage();
    double absoluteAngle = m_absoluteEncoder.GetAbsolutePosition();
    double relativeAngle = FalconToDegrees(m_angleMotor.GetSelectedSensorPosition());

    // Organizes module data into struct
    SwerveModuleTelemetry SMT{speed, position, 
                              angleVelocity, speedCurrent,
                              speedVoltage,  angleCurrent,
                              angleVoltage,  absoluteAngle, 
                              relativeAngle};
    return SMT;
}

double SwerveModule::GetSpeedP() { return SI.speedMotorPID.p; }
double SwerveModule::GetSpeedI() { return SI.speedMotorPID.i; }
double SwerveModule::GetSpeedD() { return SI.speedMotorPID.d; }
double SwerveModule::GetAngleP() { return SI.angleMotorPID.p; }
double SwerveModule::GetAngleI() { return SI.angleMotorPID.i; }
double SwerveModule::GetAngleD() { return SI.angleMotorPID.d; }

void SwerveModule::SetSpeedP(double value) { m_speedMotor.Config_kP(0, value); }
void SwerveModule::SetSpeedI(double value) { m_speedMotor.Config_kI(0, value); }
void SwerveModule::SetSpeedD(double value) { m_speedMotor.Config_kD(0, value); }
void SwerveModule::SetAngleP(double value) { m_angleMotor.Config_kP(0, value); }
void SwerveModule::SetAngleI(double value) { m_angleMotor.Config_kI(0, value); }
void SwerveModule::SetAngleD(double value) { m_angleMotor.Config_kD(0, value); }