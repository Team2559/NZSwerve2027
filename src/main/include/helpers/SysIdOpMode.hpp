// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/Command.hpp>
#include <wpi/commands2/CommandHelper.hpp>
#include <wpi/commands2/SelectCommand.hpp>
#include <wpi/commands2/sysid/SysIdRoutine.hpp>
#include <wpi/opmode/OpMode.hpp>
#include <wpi/driverstation/RobotState.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

class SysIdOpMode : public wpi::OpMode {
public:
  template <std::convertible_to<std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>> SysIdRoutinePtr>
  explicit SysIdOpMode(SysIdRoutinePtr routine) :
    m_routine{routine},
    m_id{wpi::RobotState::GetOpModeId()}
  {}

  ~SysIdOpMode() {};

  wpi::cmd::CommandPtr Quasistatic(wpi::cmd::sysid::Direction direction);

  wpi::cmd::CommandPtr Dynamic(wpi::cmd::sysid::Direction direction);
private:
  std::unique_ptr<wpi::cmd::sysid::SysIdRoutine> m_routine;
  int m_id;
};
