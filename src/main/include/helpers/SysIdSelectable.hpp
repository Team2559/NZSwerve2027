// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/commands2/Command.hpp>
#include <wpi/commands2/CommandHelper.hpp>
#include <wpi/commands2/SelectCommand.hpp>
#include <wpi/commands2/sysid/SysIdRoutine.hpp>
#include <wpi/tunables/Selectable.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

enum class SysIdSubroutine {
  QUASISTATIC_FORWARD,
  QUASISTATIC_REVERSE,
  DYNAMIC_FORWARD,
  DYNAMIC_REVERSE,
};

class SysIdSelectable : public wpi::tunables::ComplexTunable {
public:
  template <std::convertible_to<std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>>... SysIdRoutinePtrs>
  explicit SysIdSelectable(std::pair<std::string, SysIdRoutinePtrs> &&...routines) {
    std::vector<std::string> routine_names{};

    (routine_names.push_back(routines.first), ...);
    ((void)m_routines.emplace_back(std::move(routines.second)), ...);

    for (auto choice : ExpandRoutineChoices(routine_names)) {
      m_sysIdSelectable.Add(choice.first, choice.second);
    }
  }

  explicit SysIdSelectable(std::vector<std::pair<std::string, std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>>> &&routines);

  wpi::cmd::CommandPtr RunSelected();

  std::string_view GetTunableType() const override;
  void PublishTunable(wpi::tunables::TunableTable& table) override;

private:
  static std::vector<std::pair<uint, std::unique_ptr<wpi::cmd::Command>>> ExpandRoutineCommands(
    std::vector<std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>> &routines
  );

  static std::vector<std::pair<std::string, uint>> ExpandRoutineChoices(
    std::vector<std::string> &routine_names
  );

  std::vector<std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>> m_routines{};
  wpi::tunables::Selectable<uint> m_sysIdSelectable{};
};
