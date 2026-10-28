// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/telemetry/TelemetryLoggable.hpp>
#include <wpi/telemetry/TelemetryTable.hpp>
#include <wpi/tunables/ComplexTunable.hpp>
#include <wpi/tunables/TunableTable.hpp>

/**
 * An update to a single PID term, of a specific slot
 */
struct PIDUpdate {
  /**
   * A specific PID term, limited to the basic feedforward model
   */
  enum class PIDTerm {
    P,
    I,
    D,
    FF,
  } term;

  /**
   * The new value for the specified term
   */
  double value;

  /**
   * Which slot to update on multi-slot devices
   */
  unsigned int slot;
};

/**
 * Generic tuner for adjusting arbitrary PID loops live from the dashboard
 * The provided handler is called with a [`PIDUpdate`] whenever a change is made
 */
class PIDTuner : public wpi::telemetry::TelemetryLoggable,
                 public wpi::tunables::ComplexTunable {
public:
  explicit PIDTuner(std::function<void(PIDUpdate)> handler) :
      PIDTuner(handler, 0.0, 0.0, 0.0) {};
  PIDTuner(std::function<void(PIDUpdate)> handler, double P, double I, double D) :
      PIDTuner(handler, P, I, D, 0.0) {};
  PIDTuner(std::function<void(PIDUpdate)> handler, double P, double I, double D, double FF) :
      P{P}, I{I}, D{D}, FF{FF}, handler(handler) {};
  ~PIDTuner() = default;

  void LogTo(wpi::telemetry::TelemetryTable& table) const override;
  std::string_view GetTelemetryType() const override;
  std::string_view GetTunableType() const override;
  void PublishTunable(wpi::tunables::TunableTable& table) override;

  /**
   * The currently selected slot
   */
  unsigned int Slot();

private:
  // The currently selected PID slot
  unsigned int slot = 0;

  // The current proportional term
  double P;
  // The current integral term
  double I;
  // The current derivative term
  double D;
  // The current feedforward term
  double FF;

  // The handler to call with new PID updates
  std::function<void(PIDUpdate)> handler;
};
