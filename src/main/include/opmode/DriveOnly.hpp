#pragma once

#include "wpi/commands2/button/CommandGamepad.hpp"
#include "wpi/opmode/OpMode.hpp"

#include "Constants.hpp"
#include "Robot.hpp"

class DriveOnly final : public wpi::OpMode {
public:
  explicit DriveOnly(Robot &robot);

  void Periodic() override;

private:
  Robot &robot;

  wpi::cmd::CommandGamepad m_driverController{
      OperatorConstants::kDriverControllerPort};

  // Drive control mode flags for holonomic drivetrains (like swerve)
  bool m_fieldOriented = true;
  bool m_triggerSpeedEnabled = false;
  bool m_isRedAlliance = false;

  void ConfigureDriverBindings();

  /// Process controller inputs into percentage robot velocities; returns an x,
  /// y, ω, fieldOriented tuple
  std::tuple<double, double, double, bool> GetDriveTeleopControls();
};
