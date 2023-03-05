// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#ifndef CONSTANTS_H
#define CONSTANTS_H

#pragma once

#include "util/SwerveModule.h"
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
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
#define PI 3.141592653

namespace SwerveDriveConstants {
    // PID, sensor IDs passed in via structs in namespace
    constexpr double MPSToRPM {600.0};
    constexpr double DEGToRAD {57.2957795131};
    constexpr double wheelRadius = 4;
    constexpr double wheelRadiusMeters = 0.1016;
    constexpr double wheelCircumferenceMeters = 2.0 * PI * wheelRadiusMeters;
    constexpr double encoderSpeedGearRatio = 6.55;
    constexpr double encoderTurnGearRatio = 10.29;
    constexpr int falconEncoderTicksPerRevolution = 2048;
    constexpr int cancoderTicksPerRevolution = 4096;
    constexpr units::second_t kDt {0.020};

    // Coordinate plane distance in meters to each swerve drive
    // This has x-positive as forward, y-positive as right
    constexpr auto xDistanceFromCenter {0.282575_m};
    constexpr auto yDistanceFromCenter {0.282575_m};

    // For some reason, this has +x and +y to forward and right
    // Which doesn't match swerve module locations
    // But it works and doing it correctly doesn't work
    // So that probably needs to be fixed
    // Kinematics object for both drive and auto
    static frc::SwerveDriveKinematics<4> kinematics{
        frc::Translation2d{+xDistanceFromCenter, -yDistanceFromCenter},
        frc::Translation2d{+xDistanceFromCenter, +yDistanceFromCenter},
        frc::Translation2d{-xDistanceFromCenter, -yDistanceFromCenter},
        frc::Translation2d{-xDistanceFromCenter, +yDistanceFromCenter}
    };
    
    // 0-max ramp time, current limit in amps
    constexpr double loopRampRate {.1};
    constexpr double ampLimit {20.0};

    // SysID robot characterization values -- **varies by robot**
    constexpr auto ks = 0.208_V;
    constexpr auto kv = 2.206 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.409 * 1_V * 1_s * 1_s / 1_m;

    // Distance traveled by robot per encoder tick
    constexpr double driveEncoderDistancePerPulse = 
        (wheelRadiusMeters * PI) / 
        (static_cast<double>(falconEncoderTicksPerRevolution) * encoderSpeedGearRatio);

    constexpr double turnEncoderDistancePerPulse = 
        (2 * PI) / 
        (static_cast<double>(cancoderTicksPerRevolution) * encoderTurnGearRatio);

    // Robot maxes - approximated and varies by robot
    constexpr auto kMaxSpeed = 3.0_mps;
    constexpr auto kMaxAcceleration = 2.0_mps_sq;
    constexpr units::radians_per_second_t maxAngularVelocity {PI};
    constexpr units::radians_per_second_squared_t maxAngularAcceleration {PI / 4};

    constexpr double speedP {1.0};
    constexpr double speedI {0.0};
    constexpr double speedD {0.0};
    constexpr double angleP {0.3};
    constexpr double angleI {0.0};
    constexpr double angleD {0.0};
    // These are for robot rotation, not wheel rotation
    constexpr double rotP {0.02};
    constexpr double rotI {0.0001};
    constexpr double rotD {0.00009};

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
    constexpr double frontLeftOffset {291.5};
    constexpr double frontRightOffset {9.6};
    constexpr double backLeftOffset {303.4};
    constexpr double backRightOffset {1.8};

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
    constexpr auto TrajectoryScale = 1.0;
    // Ramsete controller (trajectory following) parameters
    constexpr auto kRamseteB = 2.0 * 1_rad * 1_rad / (1_m * 1_m);
    constexpr auto kRamseteZeta = 0.7 / 1_rad;
    // Gyro is inverted so rotation should be inverted
    constexpr double autoRotP {-0.5};
    constexpr double autoRotI {0.0};
    constexpr double autoRotD {0.0};

    // PID Controllers for x and y movement in auto mode -- theta controller is part of drive object
    constexpr double kPXController = 1.0;
    constexpr double kPYController = kPXController;

    static frc::TrapezoidProfile<units::radians>::Constraints thetaConstraints{SwerveDriveConstants::maxAngularVelocity,
                                                                        SwerveDriveConstants::maxAngularAcceleration};

    static frc2::PIDController autoXPIDController{kPXController, 0.0, 0.0};
    static frc2::PIDController autoYPIDController{kPYController, 0.0, 0.0};
    static frc::ProfiledPIDController<units::radians> thetaPIDController {autoRotP, autoRotI, autoRotD, thetaConstraints};                                                               
};

// PS5 Controls

#define PS5_BILL_CONTROLLER_PORT 0

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

#define PS5_AXIS_LSTICK_X 0 // left is -1, right is 1
#define PS5_AXIS_LSTICK_Y 1 // up is -1, down is 1
#define PS5_AXIS_RSTICK_X 2 // left is -1, right is 1
#define PS5_AXIS_LTRIGGER 3 // -1 is untouched, fully pressed is 1
#define PS5_AXIS_RTRIGGER 4 // -1 is untouched, fully pressed is 1
#define PS5_AXIS_RSTICK_Y 5 // up is -1 , down is 1

// Need to get IDs

#define INTAKE_PISTON_L 0
#define INTAKE_PISTON_R 1
#define INTAKE_ROLLER_MOTOR_ID 2
#define INTAKE_CONVEYOR_MOTOR_ID 3
#define INTAKE_L_CONE_CORRECT_MOTOR_ID 4
#define INTAKE_R_CONE_CORRECT_MOTOR_ID 5

#define ELEVATOR_MOTOR 2
#define ELEVATOR_P 0
#define ELEVATOR_I 0
#define ELEVATOR_D 0
#define ELEVATOR_CPR 1024
#define ELEVATOR_LOW_CUBE_TARGET 0
#define ELEVATOR_MID_CUBE_TARGET 0
#define ELEVATOR_HIGH_CUBE_TARGET 0
#define ELEVATOR_LOW_CONE_TARGET 0
#define ELEVATOR_MID_CONE_TARGET 0
#define ELEVATOR_HIGH_CONE_TARGET 0

// Target distances are in meters
#define AIM_ASSIST_TARGET_X_DISTANCE 1.0
#define AIM_ASSIST_TARGET_Y_DISTANCE 0.0
// Target rotation is in degrees
#define AIM_ASSIST_TARGET_ROTATION 0.0

// Camera lense's offsets in meters from center front of robot
#define CAMERA_X_OFFSET -0.095
#define CAMERA_Y_OFFSET -0.065
#define VISION_X_KP 2.75
#define VISION_X_KI 0.0
#define VISION_X_KD 0.0
#define VISION_Y_KP 2.75
#define VISION_Y_KI 0.0
#define VISION_Y_KD 0.0
#define VISION_ROT_KP 0.25
#define VISION_ROT_KI 0.0
#define VISION_ROT_KD 0.0

#define SHOOTER_MOTOR_ID 0
#define SHOOTER_MOTOR_ID 3
#define SHOOTER_REJECT_SPEED 0
#define SHOOTER_LOW_CUBE_SPEED 0
#define SHOOTER_MID_CUBE_SPEED 0
#define SHOOTER_HIGH_CUBE_SPEED 0
#define SHOOTER_LOW_CONE_SPEED 0
#define SHOOTER_MID_CONE_SPEED 0
#define SHOOTER_HIGH_CONE_SPEED 0

#endif