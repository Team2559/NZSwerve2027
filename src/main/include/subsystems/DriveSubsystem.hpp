// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <choreo/Choreo.hpp>
#include <wpi/math/controller/PIDController.hpp>
#include <wpi/math/estimator/SwerveDrivePoseEstimator3d.hpp>
#include <wpi/math/kinematics/SwerveDriveKinematics.hpp>
#include <wpi/math/kinematics/SwerveDriveOdometry.hpp>
#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/smartdashboard/Field2d.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/commands2/sysid/SysIdRoutine.hpp>
#include <wpi/nt/DoubleTopic.hpp>
#include <AHRS.h>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include <memory>

#include "Constants.hpp"
#include "helpers/PIDTuner.hpp"
#include "helpers/SysIdSelectable.hpp"
#include "subsystems/SwerveModule.hpp"

class DriveSubsystem : public wpi::cmd::SubsystemBase {
public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Will be called periodically whenever the CommandScheduler runs during
   * simulation.
   */
  void SimulationPeriodic() override;

  void TestInit();
  void TestExit();

  /**
   *  Reset forward for the driver to be the way the robot is currently facing
   */
  void ResetFieldOrientation(bool inverted = false);

  /**
   * Move at the requested set of speeds measured relative to either the robot or the field
   */
  void Drive(wpi::units::meters_per_second_t xSpeed, wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot, bool fieldRelative) {
    Drive(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }

  /**
   * Move at the requested set of speeds measured relative to either the robot or the field, with an arbitrary center of rotation
   */
  void Drive(wpi::units::meters_per_second_t xSpeed, wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot, bool fieldRelative, wpi::units::meter_t x_center, wpi::units::meter_t y_center);

  /**
   * Move towards the choreo trajectory sample position (with PID feedback)
   */
  void FollowTrajectory(const choreo::SwerveSample &sample);

  /**
   * Move the module steering to be aligned for the requested direction of travel
   */
  void SteerTo(wpi::units::meters_per_second_t xSpeed, wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot, bool fieldRelative) {
    SteerTo(xSpeed, ySpeed, rot, fieldRelative, 0.0_m, 0.0_m);
  }

  /**
   * Move the module steering to be aligned for the requested direction of travel, with an arbitrary center of rotation
   */
  void SteerTo(wpi::units::meters_per_second_t xSpeed, wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot, bool fieldRelative, wpi::units::meter_t x_center, wpi::units::meter_t y_center);

  /**
   * Stop all drivetrain movement
   */
  void Stop();

  /**
   * Reset the robot's pose to the provided pose
   */
  void ResetPose(wpi::math::Pose3d pose);

  /**
   * Gets the robot's current pose (position + orientation)
   */
  wpi::math::Pose3d GetPose();

  /**
   * Incorporate a vision pose measurement into the robots cumulative pose estimation
   */
  void UpdateVisionPose(wpi::math::Pose3d measurement, wpi::units::millisecond_t timestamp);

  /**
   * Gets the robot's current velocity (in translation and rotation)
   */
  wpi::math::ChassisVelocities GetChassisVelocities();

  /**
   * Get the current steer angle and wheel positions for all modules for odometry
   */
  const std::array<wpi::math::SwerveModulePosition, 4> GetModulePositions();

  /**
   * Sets the desired steer angle and wheel velocity for all modules
   */
  void SetModuleVelocities(std::array<wpi::math::SwerveModuleVelocity, 4> desiredStates, bool steerOnly = false);

  // Front: +x, Rear: -x; Left: +y, Right -y. Zero heading is to the front
  // and +rotation is counter-clockwise. This is all standard, although it
  // means the robot's front is along the x-axis, which is often pointed to
  // the right, as things are occasionally drawn.
  /**
   * Drive kinematics for converting robot velocities into module velocities and vice-versa
   */
  wpi::math::SwerveDriveKinematics<4> kDriveKinematics{
    wpi::math::Translation2d(+DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
    wpi::math::Translation2d(+DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2),
    wpi::math::Translation2d(-DriveConstants::kWheelbaseLength / 2, +DriveConstants::kWheelbaseWidth / 2),
    wpi::math::Translation2d(-DriveConstants::kWheelbaseLength / 2, -DriveConstants::kWheelbaseWidth / 2)
  };

  /**
   * Field drawing for dashboard debug information
   */
  wpi::Field2d field;

  // Individual mechanism SysId routines
  std::unique_ptr<wpi::cmd::sysid::SysIdRoutine> SteerSysId();
  std::unique_ptr<wpi::cmd::sysid::SysIdRoutine> DriveSysId();

  // Combined SysId routine command
  wpi::cmd::CommandPtr SysId();

private:
  // The four swerve modules.
  std::unique_ptr<SwerveModule>
    frontLeftModule;
  std::unique_ptr<SwerveModule> frontRightModule;
  std::unique_ptr<SwerveModule> rearLeftModule;
  std::unique_ptr<SwerveModule> rearRightModule;

  // The navX gyro sensor.
  AHRS m_navX;

  // Pose estimator combines odometry with vision readings to yield an accurate robot pose; 4 specifies the number of modules.
  wpi::math::SwerveDrivePoseEstimator3d<4> m_poseEstimator;

  // Tuners to adjust PID values live from the dashboard; greatly increases the ease of tuning
  PIDTuner m_driveTuner;
  PIDTuner m_steerTuner;

  // PID controllers for autonomous path following
  wpi::math::PIDController m_xController;
  wpi::math::PIDController m_yController;
  wpi::math::PIDController m_rController;

  // SysId chooser for calibrating during test mode
  std::optional<SysIdSelectable> m_sysIdSelectable{};

  // Dashboard logging entries
  // -----------------------------------

  wpi::nt::DoublePublisher nt_xPosition;
  wpi::nt::DoublePublisher nt_xSetpoint;
  wpi::nt::DoublePublisher nt_xOutput;

  wpi::nt::DoublePublisher nt_yPosition;
  wpi::nt::DoublePublisher nt_ySetpoint;
  wpi::nt::DoublePublisher nt_yOutput;

  wpi::nt::DoublePublisher nt_rPosition;
  wpi::nt::DoublePublisher nt_rSetpoint;
  wpi::nt::DoublePublisher nt_rOutput;
};
