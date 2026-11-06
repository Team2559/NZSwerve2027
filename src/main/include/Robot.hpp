// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/framework/OpModeRobot.hpp>

#include "subsystems/DriveSubsystem.hpp"

class Robot : public wpi::OpModeRobot<Robot> {
public:
  Robot();

  DriveSubsystem m_driveSubsystem;

private:
  // Have it empty by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  // std::optional<wpi::cmd::CommandPtr> autonomousCommand;

  void AddOpModes();
};
