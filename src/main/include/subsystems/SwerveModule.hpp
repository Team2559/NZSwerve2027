// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/math/kinematics/SwerveModulePosition.hpp>
#include <wpi/math/kinematics/SwerveModuleVelocity.hpp>
#include <wpi/commands2/sysid/SysIdRoutine.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>

#include "helpers/PIDTuner.hpp"

class SwerveModule {
public:
  // Initializer and Destructor
  SwerveModule() = default;
  virtual ~SwerveModule() = default;

  // Test Methods
  virtual void TestInit(std::string name) = 0;
  virtual void TestExit() = 0;
  virtual void TestDebug() = 0;

  // Live PID tuning
  virtual void UpdateDrivePID(PIDUpdate &update) = 0;
  virtual void UpdateSteerPID(PIDUpdate &update) = 0;

  // Is the swerve module in a "healthy" state?
  virtual bool GetStatus() const = 0;

  // Steer functions; get, set, and stop moving; set causes a small motion
  // towards the setpoint
  virtual void SetSteerOffset(wpi::units::angle::turn_t offset) = 0;
  virtual wpi::units::turn_t GetSteerPosition() = 0;
  virtual void SetSteerPosition(wpi::units::turn_t position) = 0;
  virtual void StopSteer() = 0;

  // Drive functions; set, reset, and percent control; set causes a small
  // acceleration towards the setpoint, percent is useful for test profiling
  virtual void SetDriveVelocity(wpi::units::meters_per_second_t velocity) = 0;
  virtual void SetDrivePercent(double percent) = 0;
  virtual void StopDrive() = 0;

  // SysID functions; gives low-level access to the motors
  virtual void SetSteerVoltage(wpi::units::volt_t voltage) = 0;
  virtual void LogSteerInfo(wpi::sysid::SysIdRoutineLog::MotorLog log) = 0;
  virtual void SetDriveVoltage(wpi::units::volt_t voltage) = 0;
  virtual void LogDriveInfo(wpi::sysid::SysIdRoutineLog::MotorLog log) = 0;

  // Combined swerve module functions
  virtual const wpi::math::SwerveModuleVelocity GetVelocity() = 0;
  virtual const wpi::math::SwerveModulePosition GetPosition() = 0;
  virtual void SetDesiredVelocity(wpi::math::SwerveModuleVelocity &state) = 0;
  virtual void Stop() = 0;
};
