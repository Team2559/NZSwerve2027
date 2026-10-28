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
  RevSwerveModule(int driveCanID, int steerCanID, wpi::units::angle::turn_t offset);

  // Test Methods
  void TestInit(std::string name) override;
  void TestExit() override;
  void TestDebug() override;

  // Live PID tuning
  void UpdateDrivePID(PIDUpdate &update) override;
  void UpdateSteerPID(PIDUpdate &update) override;

  // Is the swerve module in a "healthy" state?
  bool GetStatus() const override;

  void SetSteerOffset(wpi::units::angle::turn_t offset) override;
  wpi::units::angle::turn_t GetSteerPosition() override;
  void SetSteerPosition(wpi::units::angle::turn_t position) override;
  void StopSteer() override;

  void SetDriveVelocity(wpi::units::velocity::meters_per_second_t velocity) override;
  void SetDrivePercent(double percent) override;
  void StopDrive() override;

  const wpi::math::SwerveModuleVelocity GetVelocity() override;
  const wpi::math::SwerveModulePosition GetPosition() override;
  void SetDesiredVelocity(wpi::math::SwerveModuleVelocity &state) override;
  void Stop() override;

private:
  rev::spark::SparkFlex driveMotor;
  rev::spark::SparkFlex steerMotor;

  double driveVff;

  rev::spark::SparkRelativeEncoder driveEncoder;
  rev::spark::SparkAbsoluteEncoder steerEncoder;

  // std::optional<nt::DoubleArrayPublisher> nt_driveOutput = {};
  // std::optional<nt::DoubleArrayPublisher> nt_steerOutput = {};
};
