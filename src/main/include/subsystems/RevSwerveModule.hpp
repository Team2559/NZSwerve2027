// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <rev/SparkFlex.h>

#include <string>

#include "subsystems/SwerveModule.hpp"

class RevSwerveModule : public SwerveModule {
public:
  // Initializer
  RevSwerveModule(wpi::CANPort canPort, int driveCanID, int steerCanID, wpi::units::angle::turn_t offset);

  // Utility Methods
  void UtilInit(wpi::telemetry::TelemetryTable &table) override;
  void UtilExit() override;

  // Live PID tuning
  void UpdateDrivePID(PIDUpdate &update) override;
  void UpdateSteerPID(PIDUpdate &update) override;

  // Is the swerve module in a "healthy" state?
  bool GetStatus() const override;

  void SetSteerOffset(wpi::units::angle::turn_t offset) override;
  wpi::units::angle::turn_t GetSteerPosition() override;
  void SetSteerPosition(wpi::units::angle::turn_t position) override;
  void StopSteer() override;

  wpi::units::velocity::meters_per_second_t GetDriveVelocity();
  void SetDriveVelocity(wpi::units::velocity::meters_per_second_t velocity) override;
  void SetDriveThrottle(double throttle) override;
  void StopDrive() override;

  void SetSteerVoltage(wpi::units::volt_t voltage) override;
  void LogSteerInfo(wpi::sysid::SysIdRoutineLog::MotorLog log) override;
  void SetDriveVoltage(wpi::units::volt_t voltage) override;
  void LogDriveInfo(wpi::sysid::SysIdRoutineLog::MotorLog log) override;

  const wpi::math::SwerveModuleVelocity GetVelocity() override;
  const wpi::math::SwerveModulePosition GetPosition() override;
  void SetDesiredVelocity(wpi::math::SwerveModuleVelocity &velocity) override;
  void Stop() override;

private:
  rev::spark::SparkFlex driveMotor;
  rev::spark::SparkFlex steerMotor;

  double driveVff;

  rev::spark::SparkRelativeEncoder driveEncoder;
  rev::spark::SparkAbsoluteEncoder steerEncoder;

  std::optional<std::reference_wrapper<wpi::telemetry::TelemetryTable>> nt_driveController = {};
  std::optional<std::reference_wrapper<wpi::telemetry::TelemetryTable>> nt_steerController = {};
};
