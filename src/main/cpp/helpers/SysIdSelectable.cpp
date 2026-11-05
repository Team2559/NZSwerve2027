// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "helpers/SysIdSelectable.hpp"

#include <wpi/commands2/Commands.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

SysIdSelectable::SysIdSelectable(std::vector<std::pair<std::string, std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>>> &&routines) {
  std::vector<std::string> routine_names{};

  for (auto &&routine : routines) {
    routine_names.push_back(routine.first);
    m_routines.emplace_back(std::move(routine.second));
  }

  for (auto choice : ExpandRoutineChoices(routine_names)) {
    m_sysIdSelectable.Add(choice.first, choice.second);
  }
}

wpi::cmd::CommandPtr SysIdSelectable::RunSelected() {
  return (
    wpi::cmd::SelectCommand<uint>(
      [this]() { return m_sysIdSelectable.GetSelected(); },
      ExpandRoutineCommands(m_routines)
    )
      .ToPtr()
  );
}

std::string_view SysIdSelectable::GetTunableType() const {
  return m_sysIdSelectable.GetTunableType();
}

void SysIdSelectable::PublishTunable(wpi::tunables::TunableTable& table) {
  m_sysIdSelectable.PublishTunable(table);
}

std::vector<std::pair<uint, std::unique_ptr<wpi::cmd::Command>>> SysIdSelectable::ExpandRoutineCommands(
  std::vector<std::unique_ptr<wpi::cmd::sysid::SysIdRoutine>> &routines
) {
  std::vector<std::pair<uint, std::unique_ptr<wpi::cmd::Command>>> vec{};
  vec.reserve(routines.size() * 4);

  uint id = 0;
  for (auto &routine : routines) {
    vec.emplace_back(id + (uint)SysIdSubroutine::QUASISTATIC_FORWARD, routine->Quasistatic(wpi::cmd::sysid::Direction::FORWARD).Unwrap());
    vec.emplace_back(id + (uint)SysIdSubroutine::QUASISTATIC_REVERSE, routine->Quasistatic(wpi::cmd::sysid::Direction::REVERSE).Unwrap());
    vec.emplace_back(id + (uint)SysIdSubroutine::DYNAMIC_FORWARD, routine->Dynamic(wpi::cmd::sysid::Direction::FORWARD).Unwrap());
    vec.emplace_back(id + (uint)SysIdSubroutine::DYNAMIC_REVERSE, routine->Dynamic(wpi::cmd::sysid::Direction::REVERSE).Unwrap());
    id += 4;
  }

  return vec;
}

std::vector<std::pair<std::string, uint>> SysIdSelectable::ExpandRoutineChoices(
  std::vector<std::string> &routine_names
) {
  std::vector<std::pair<std::string, uint>> vec{};
  vec.reserve(routine_names.size() * 4);

  uint id = 0;
  for (auto &routine_name : routine_names) {
    vec.emplace_back(routine_name + " Quasistatic Forward", id + (uint)SysIdSubroutine::QUASISTATIC_FORWARD);
    vec.emplace_back(routine_name + " Quasistatic Reverse", id + (uint)SysIdSubroutine::QUASISTATIC_REVERSE);
    vec.emplace_back(routine_name + " Dynamic Forward", id + (uint)SysIdSubroutine::DYNAMIC_FORWARD);
    vec.emplace_back(routine_name + " Dynamic Reverse", id + (uint)SysIdSubroutine::DYNAMIC_REVERSE);
    id += 4;
  }

  return vec;
}
