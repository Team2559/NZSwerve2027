// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/fields/Field.hpp>
#include <wpi/fields/fields.hpp>
#include <wpi/math/geometry/Transform3d.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/constants.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

  inline constexpr int kDriverControllerPort = 0;

} // namespace OperatorConstants

namespace DriveConstants {
  // Drivebase geometry: distance between centers of right and left wheels on
  // robot; distance between centers of front and back wheels on robot.
  inline constexpr wpi::units::meter_t kWheelbaseWidth = 24.75_in;
  inline constexpr wpi::units::meter_t kWheelbaseLength = 24.75_in;

  // Zero positions for the steer of the swerve modules
  inline constexpr wpi::units::degree_t kFrontLeftSteerOffset = 0.99025878_tr;
  inline constexpr wpi::units::degree_t kFrontRightSteerOffset = 0.018524759_tr;
  inline constexpr wpi::units::degree_t kRearLeftSteerOffset = 0.62443335_tr;
  inline constexpr wpi::units::degree_t kRearRightSteerOffset = 0.3770286_tr;

  constexpr wpi::math::DCMotor driveMotorPlant = wpi::math::DCMotor::KrakenX60();
  constexpr wpi::math::DCMotor steerMotorPlant = wpi::math::DCMotor::NEO();

  // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 8.16 / 0.3192 => ~25.57.

  // SDS Mk4 L1, L2, L3, L4 Gear Ratio: 8.14:1, 6.75:1, 6.12:1, 5.14:1
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 6.75 / 0.3192 => ~21.15.

  // SDS Mk4n L1, L2, L3 Gear Ratio: 7.13:1, 5.90:1, 5.36:1
  // Nominal Wheel Diameter (4"): =0.1016m;
  // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
  // 0.3192 / 5.90 => ~0.05401.

  inline constexpr double kDriveGearRatio = 50.0 / 14.0 * 17.0 / 27.0 * 45.0 / 15.0; // approx 6.75

  // This should be empirically determined!  This is just an initial guess.
  // This is used for both distance and velocity control. If this is off, it
  // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
  inline constexpr wpi::units::unit_t<wpi::units::compound_unit<wpi::units::meter, wpi::units::inverse<wpi::units::turn>>> kDriveDistancePerRotation =
    4.00_in * wpi::units::constants::pi / wpi::units::turn_t{kDriveGearRatio};

  // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
  // Review your motor and swerve module configuration for nominal free speed
  // This is an upper bound, for various reasons. It needs to be empirically
  // measured. Half of theoretical free speed is a reasonable starting value
  // (since something in the ballpark is needed here in order to to drive).
  constexpr wpi::units::meters_per_second_t kMaxDriveSpeed = 14.4_V * driveMotorPlant.Kv * kDriveDistancePerRotation;
  constexpr double kSlowDrivePercent = 0.80;

  inline constexpr double kSteerGearRatio = 50.0 / 14.0 * 60.0 / 10.0; // approx 21.43

  // This is used for rotating the robot in place, about it's center.  This
  // may need to be empirically adjusted, but check kDriveMetersPerRotation
  // before making any adjustment here.
  const wpi::units::meter_t kDriveMetersPerSteerCircle =
    M_PI * wpi::units::math::sqrt(wpi::units::math::pow<2>(kWheelbaseLength) + wpi::units::math::pow<2>(kWheelbaseWidth));

  // This is the maximum rotational speed -- not of a swerve module, but of
  // the entire robot.  This is a function of the maximum drive speed and the
  // geometry of the robot.  This will occur when the robot spins in place,
  // around the center of a circle which passes through all the drive modules
  // (if there is no single such circle, things are analogous).  If the drive
  // modules are turned to be tangential to this circle and run at maximum,
  // the robot is rotating as fast as possible.  This can be derived from
  // kMaxDriveSpeed and the geometry and does not have to be directly
  // measured.  It is a good idea to check this value empirically though.

  // So the maximum rotational velocity (spinning in place) is kMaxDriveSpeed
  // / kDriveMetersPerSteerCircle * 360 degrees.  This should not need to
  // be empirically adjusted (but check).
  const wpi::units::degrees_per_second_t kMaxTurnRate =
    kMaxDriveSpeed / kDriveMetersPerSteerCircle * 360.0_deg;

  // CAN ID assignments.
  constexpr int kFrontLeftDriveMotorCanID = 1;
  constexpr int kFrontLeftSteerMotorCanID = 2;
  constexpr int kRearLeftDriveMotorCanID = 3;
  constexpr int kRearLeftSteerMotorCanID = 4;
  constexpr int kRearRightDriveMotorCanID = 5;
  constexpr int kRearRightSteerMotorCanID = 6;
  constexpr int kFrontRightDriveMotorCanID = 7;
  constexpr int kFrontRightSteerMotorCanID = 8;

  // These can flip because of gearing.
  constexpr bool kDriveMotorInverted = false;
  constexpr bool kSteerMotorInverted = true;
  constexpr bool kSteerSensorInverted = false;

  // Closed loop feedback parameters for module drive speed
  namespace DrivePID {
    constexpr double kP = 0.3;
    constexpr double kI = 0.0;
    constexpr double kD = 0.006;
    constexpr double kS = 0.0;
    constexpr double kV = (1.0 / driveMotorPlant.Kv / kDriveDistancePerRotation).value();
  } // namespace DrivePID

  // Closed loop feedback parameters for module steer position
  namespace SteerPID {
    constexpr double kP = 12.0;
    constexpr double kI = 0.0;
    constexpr double kD = 25.0;
    constexpr double kS = 0.0088207;
  } // namespace SteerPID

  // Closed loop feedback for chassis translation
  namespace TranslationPID {
    constexpr double kP = 1.4;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
  } // namespace TranslationPID

  // Closed loop feedback for chassis orientation
  namespace OrientationPID {
    constexpr double kP = 2.0;
    constexpr double kI = 0.0;
    constexpr double kD = 0.0;
  } // namespace OrientationPID
} // namespace DriveConstants

namespace VisionConstants {
  // Camera network name
  constexpr char kName[] = "OV9281";
  // AprilTag field data; FMA events should all use the welded field, but off-season events may use the AndyMark field instead.
  const wpi::fields::Field kFieldLayout = wpi::fields::GetField(wpi::fields::FieldId::FRC_2026_REBUILT_WELDED);
  // Camera focal point position and orientation relative to the robot origin
  constexpr wpi::math::Transform3d kRobotToCam = wpi::math::Transform3d(
    wpi::math::Translation3d(0.5_m, 0.0_m, 0.5_m),
    wpi::math::Rotation3d(0_rad, 0_rad, 0_rad)
  );
} // namespace VisionConstants
