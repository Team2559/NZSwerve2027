// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "helpers/PIDTuner.hpp"

#include <wpi/tunables/TunableConfig.hpp>
#include <wpi/tunables/Tunable.hpp>


void PIDTuner::LogTo(wpi::telemetry::TelemetryTable& table) const {
  table.Log("P", P);
  table.Log("I", I);
  table.Log("D", D);
  table.Log("FF", FF); // TODO: make custom FF list instead
  table.Log("slot", slot);
}

std::string_view PIDTuner::GetTelemetryType() const {
  return "PIDTuner";
}

std::string_view PIDTuner::GetTunableType() const {
  return "PIDTuner";
}

void PIDTuner::PublishTunable(wpi::tunables::TunableTable& table) {
  table.Publish("P", this, &PIDTuner::P, wpi::tunables::TunableConfig{
    .onTune = [this](auto& tunable, auto* parent) {
      handler({PIDUpdate::PIDTerm::P, P, slot});
    }
  });
  table.Publish("I", this, &PIDTuner::I, wpi::tunables::TunableConfig{
    .onTune = [this](auto& tunable, auto* parent) {
      handler({PIDUpdate::PIDTerm::I, I, slot});
    }
  });
  table.Publish("D", this, &PIDTuner::D, wpi::tunables::TunableConfig{
    .onTune = [this](auto& tunable, auto* parent) {
      handler({PIDUpdate::PIDTerm::D, D, slot});
    }
  });
  table.Publish("I", this, &PIDTuner::FF, wpi::tunables::TunableConfig{
    .onTune = [this](auto& tunable, auto* parent) {
      handler({PIDUpdate::PIDTerm::FF, FF, slot});
    }
  });
  table.Publish("slot", this, &PIDTuner::slot);
}
