// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants
{
    constexpr units::meters_per_second_t maxSpeed = 4.8_mps;
    constexpr units::radians_per_second_t maxAngularSpeed {2 * std::numbers::pi };

    constexpr units::radians_per_second_t directionSlewRate = 1.2_rad_per_s;
    constexpr double magnitudeSlewRate = 1.8;   // percent per second (1 = 100%)
    constexpr double rotationalSlewRate = 2.0;  // percent per second (1 = 100%)
}

namespace Chassis
{
    constexpr units::meter_t trackWidth = 0.0_m;
    constexpr units::meter_t wheelBase = 0.0_m;
}

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants
