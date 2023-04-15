// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include "util/SwerveModule.h"
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/trajectory/Trajectory.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
// Don't do this
#define pi 3.141592653

namespace SwerveDriveConstants {
    // PID, sensor IDs passed in via structs in namespace
    constexpr double MPSToRPM {600.0};
    constexpr double DEGToRAD {57.2957795131};
    constexpr double wheelRadius {2.0};
    constexpr double wheelRadiusMeters {0.0508};
    constexpr double wheelCircumferenceMeters {2.0 * pi* wheelRadiusMeters};
    constexpr double encoderSpeedGearRatio {8.1};
    constexpr double encoderTurnGearRatio {15.43};
    constexpr int falconEncoderTicksPerRevolution {2048};
    constexpr int cancoderTicksPerRevolution {4096};
    constexpr units::second_t kDt {0.020};
    constexpr auto swerveConstantSpinSpeed {1.0_rad / 1.0_s};
    constexpr auto leftTranslateSpeed {-0.5_mps};
    constexpr auto rightTranslateSpeed {0.5_mps};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as right
    constexpr auto xDistanceFromCenter {0.282575_m};
    constexpr auto yDistanceFromCenter {0.282575_m};
    
    // 0-max ramp time, current limit in amps
    constexpr double loopRampRate {.1};
    constexpr double ampLimit {35.0};

    // SysID robot characterization values -- **varies by robot**
    constexpr auto ks {0.408_V};
    constexpr auto kv {3.206 * 1_V * 1_s / 1_m};
    constexpr auto ka {3.409 * 1_V * 1_s * 1_s / 1_m};

    // Robot maxes - approximated and varies by robot
    constexpr auto kMaxSpeed {4.0_mps};
    constexpr auto kMaxAcceleration {3.0_mps_sq};
    constexpr units::radians_per_second_t maxAngularVelocity {pi};
    constexpr units::radians_per_second_squared_t maxAngularAcceleration {pi / 2};

    constexpr double speedP {5.0};
    constexpr double speedI {0.0};
    constexpr double speedD {0.0};
    constexpr double angleP {2.0};
    constexpr double angleI {0.0};
    constexpr double angleD {0.0};
    // These are for robot rotation, not wheel rotation
    constexpr double rotP {0.05};
    constexpr double rotI {0.0};
    constexpr double rotD {0.005};

    // Default speed + angle PID values
    constexpr PIDValues speedMotorPID {
        speedP, speedI, speedD,
    };

    constexpr PIDValues angleMotorPID {
        angleP, angleI, angleD,
    };

    // Sensor IDs for motors + encoders - labeled on robot
    constexpr int gyroCANID {1};
    constexpr int rightFrontTurningCANID {1};
    constexpr int rightFrontSpeedCANID {2};
    constexpr int rightBackTurningCANID {3};
    constexpr int rightBackSpeedCANID {4};
    constexpr int leftFrontTurningCANID {5};
    constexpr int leftFrontSpeedCANID {6};
    constexpr int leftBackSpeedCANID {7};
    constexpr int leftBackTurningCANID {8};
    constexpr int leftFrontCancoderID {9};
    constexpr int rightFrontCancoderID {10};
    constexpr int leftBackCancoderID {11};
    constexpr int rightBackCancoderID {12};

    // Swerve angle offsets -- difference between actual degrees heading and absolute degree values
    constexpr double frontLeftOffset {235.5};
    constexpr double frontRightOffset {334.5};
    constexpr double backLeftOffset {9.8};
    constexpr double backRightOffset {83.5};

    constexpr SwerveInfo kLeftFrontInfo {
        leftFrontSpeedCANID, leftFrontTurningCANID, leftFrontCancoderID, 
        speedMotorPID, angleMotorPID, frontLeftOffset
    };

    constexpr SwerveInfo kRightFrontInfo {
        rightFrontSpeedCANID, rightFrontTurningCANID, rightFrontCancoderID,
        speedMotorPID, angleMotorPID, frontRightOffset
    };

    constexpr SwerveInfo kLeftBackInfo {
        leftBackSpeedCANID, leftBackTurningCANID, leftBackCancoderID,
        speedMotorPID, angleMotorPID, backLeftOffset
    };

    constexpr SwerveInfo kRightBackInfo {
        rightBackSpeedCANID, rightBackTurningCANID, rightBackCancoderID,
        speedMotorPID, angleMotorPID, backRightOffset
    };
};

namespace AutoConstants {
    // Multiplier for trajectory translation values
    constexpr auto TrajectoryScale {1.0};
    // Ramsete controller (trajectory following) parameters
    constexpr auto kRamseteB {2.0 * 1_rad * 1_rad / (1_m * 1_m)};
    constexpr auto kRamseteZeta {0.7 / 1_rad};
    constexpr double autoRotP {5.0};
    constexpr double autoRotI {0.0};
    constexpr double autoRotD {0.0};
    constexpr int autoBalanceSettleLoops {25};

    // Y and rotation are inverted
    // PID Controllers for x and y movement in auto mode -- theta controller is part of drive object
    constexpr double autoKP {0.75};
    constexpr double autoKI {0.0};
    constexpr double autoKD {0.1};

    // PID constants for auto balance PID controllers
    constexpr double balanceKP {0.5};                                              
    constexpr double balanceKI {0.0};                                              
    constexpr double balanceKD {0.075};                                              
    constexpr double balanceRotKP {5.0};                                              
    constexpr double balanceRotKI {0.0};                                              
    constexpr double balanceRotKD {0.0};                                              
};

namespace VisionConstants {
    // Camera offsets to center of robot in meters
    constexpr double cameraXOffset {0.295275};
    constexpr double cameraYOffset {-0.180492};
    // Allowed standard deviations for pose estimation, or "trust/confidence score"
    // Higher means you trust the data less and it is less factored into the estimation
    const wpi::array<double, 3> stateStdDevs {0.1, 0.1, 0.1};
    constexpr bool shouldUseVision {false};
    const wpi::array<double, 3> visionStdDevs {0.5, 0.5, 0.0};
}

namespace FieldCoordinates {
    const frc::Pose2d apriltag1{15.51_m, 1.07_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag2{15.51_m, 2.75_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag3{15.51_m, 4.42_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag4{16.18_m, 6.75_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag5{0.36_m, 6.75_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag6{1.03_m, 4.42_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag7{1.03_m, 2.75_m, frc::Rotation2d{}};
    const frc::Pose2d apriltag8{1.03_m, 1.07_m, frc::Rotation2d{}};
    const frc::Pose2d chargeStationCenter{3.85_m, 2.75_m, frc::Rotation2d{}};
}

// PS5 Controls
#define PS5_BILL_CONTROLLER_PORT 0
#define PS5_TED_CONTROLLER_PORT 1
#define PS5_TEST_CONTROLLER_PORT 2
#define PS5_BUTTON_SQR 1
#define PS5_BUTTON_X 2
#define PS5_BUTTON_O 3
#define PS5_BUTTON_TRI 4
#define PS5_BUTTON_LBUMPER 5
#define PS5_BUTTON_RBUMPER 6
#define PS5_BUTTON_LTRIGGER 7
#define PS5_BUTTON_RTRIGGER 8
#define PS5_BUTTON_CREATE 9
#define PS5_BUTTON_MENU 10
#define PS5_BUTTON_LSTICK 11
#define PS5_BUTTON_RSTICK 12
#define PS5_BUTTON_PS 13
#define PS5_BUTTON_TOUCHPAD 14
#define PS5_BUTTON_MIC 15
#define PS5_AXIS_LSTICK_X 0 // left is -1, right is 1
#define PS5_AXIS_LSTICK_Y 1 // up is -1, down is 1
#define PS5_AXIS_RSTICK_X 2 // left is -1, right is 1
#define PS5_AXIS_LTRIGGER 3 // -1 is untouched, fully pressed is 1
#define PS5_AXIS_RTRIGGER 4 // -1 is untouched, fully pressed is 1
#define PS5_AXIS_RSTICK_Y 5 // up is -1 , down is 1

#define INTAKE_PISTON_IN 0
#define INTAKE_PISTON_OUT 1
#define INTAKE_ROLLER_MOTOR_ID 13
#define INTAKE_CONVEYOR_MOTOR_ID 14
#define INTAKE_L_CONE_CORRECT_MOTOR_ID 15
#define INTAKE_R_CONE_CORRECT_MOTOR_ID 16
#define INTAKE_ROLLER_POWER 1.00
#define INTAKE_CONVEYOR_POWER 0.25
#define INTAKE_CONE_CORRECT_POWER 0.1
#define OUTTAKE_ROLLER_POWER -0.5
#define OUTTAKE_CONVEYOR_POWER -0.5

#define ELEVATOR_MOTOR 17
#define ELEVATOR_P 238.62
#define ELEVATOR_SLOW_P 0.000025
#define ELEVATOR_I 0
#define ELEVATOR_D 17.493
#define ELEVATOR_SETTLE_LOOPS 10
#define ELEVATOR_ULTRA_SHOOT_P 0.1
#define ELEVATOR_ULTRA_SHOOT_POWER 1.0
#define ELEVATOR_MAX_SPEED 3.0_mps
#define ELEVATOR_MAX_ACCELERATION 3.0_mps_sq
#define ELEVATOR_GEAR_RATIO 20.0
#define ELEVATOR_OUTPUT_CIRCUMFERENCE (pi* 0.04552)
// Elevator feedforward constants
#define ELEVATOR_KS 0.17186_V
#define ELEVATOR_KG 0.41493_V
#define ELEVATOR_KV 20.578_V * 1_s / 1_m
#define ELEVATOR_KA 0.67322_V * 1_s * 1_s / 1_m
// Values in encoder ticks
// #define ELEVATOR_THROUGHBORE_CPR 1024
#define ELEVATOR_THROUGHBORE_CPR 8192
#define ELEVATOR_INTEGRATED_CPR 42
#define ELEVATOR_LOW_CUBE_TARGET 1200
#define ELEVATOR_MID_CUBE_TARGET 1900
#define ELEVATOR_HIGH_CUBE_TARGET 2800
#define ELEVATOR_LOW_CONE_TARGET 1300
#define ELEVATOR_MID_CONE_TARGET 2000
#define ELEVATOR_HIGH_CONE_TARGET 2800
#define ELEVATOR_DOUBLE_SUBSTATION_CUBE_TARGET 1800
#define ELEVATOR_DOUBLE_SUBSTATION_CONE_TARGET 2000
#define ELEVATOR_STOP_DISTANCE 100
#define ELEVATOR_INTERIOR_GRAB_TARGET 200
#define ELEVATOR_LOWER_LIMIT_SWITCH_ID 0 
#define ELEVATOR_UPPER_LIMIT_SWITCH_ID 1
#define ELEVATOR_ULTRA_SHOOT_TARGET 2900
#define ELEVATOR_ULTRA_SHOOT_RELEASE_POINT 2200

// Target distances are in meters
#define AIM_ASSIST_TARGET_X_DISTANCE 1.0
#define AIM_ASSIST_TARGET_Y_DISTANCE 0.0
// Target rotation is in degrees
#define AIM_ASSIST_TARGET_ROTATION 0.0

// #define GRABBER_MOTOR_ID 0
#define GRABBER_MOTOR_ID 18
#define GRABBER_CUBE_SHOOT_SPEED 0.75
#define GRABBER_CONE_SHOOT_SPEED -1.0
#define GRABBER_INTERIOR_GRAB_SPEED -0.25
#define GRABBER_CUBE_GRAB_SPEED -0.75
#define GRABBER_CONE_GRAB_SPEED 0.85
#define GRABBER_CARRY_SPEED -0.15
#define GRABBER_OUTTAKE_SPEED 0.25
#define GRABBER_GRABBED_RPM 100
#define GRABBER_MINIMUM_SPIN_TIME 0.25_s

#define CANDLE_DEVICE_ID 1
#define LED_BRIGHTNESS 1.0