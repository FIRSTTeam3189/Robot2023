// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc2/command/Command.h>
#include "Constants.h"

// Kinematics object for both drive and auto
namespace SwerveDriveParameters{
    static frc::SwerveDriveKinematics<4> kinematics{
    frc::Translation2d{+SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter},
    frc::Translation2d{+SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter},
    frc::Translation2d{-SwerveDriveConstants::xDistanceFromCenter, +SwerveDriveConstants::yDistanceFromCenter},
    frc::Translation2d{-SwerveDriveConstants::xDistanceFromCenter, -SwerveDriveConstants::yDistanceFromCenter}
    };
} // namespace SwerveDriveConstants

namespace AutoParameters {
    static frc::TrapezoidProfile<units::radians>::Constraints thetaConstraints{SwerveDriveConstants::maxAngularVelocity,
                                                                        SwerveDriveConstants::maxAngularAcceleration};

    static frc::ProfiledPIDController<units::radians> thetaPIDController {
        AutoConstants::autoRotP,
        AutoConstants::autoRotI,
        AutoConstants::autoRotD,
        thetaConstraints
    };

    static std::unordered_map<std::string, std::shared_ptr<frc2::Command>> eventMap;      
} // namespace AutoConstants