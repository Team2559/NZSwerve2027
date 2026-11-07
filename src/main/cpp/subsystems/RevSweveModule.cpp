// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <rev/config/SparkFlexConfig.h>

#include <string>

#include "Constants.hpp"
#include "subsystems/RevSwerveModule.hpp"

using namespace DriveConstants;
using namespace rev::spark;

static wpi::units::turn_t wrapOffset(wpi::units::turn_t value) {
  return value - wpi::units::math::floor(value);
}

RevSwerveModule::RevSwerveModule(wpi::CANPort canPort, int driveCanID, int steerCanID, wpi::units::angle::turn_t offset) :
    driveMotor{canPort, driveCanID, SparkFlex::MotorType::kBrushless},
    steerMotor{canPort, steerCanID, SparkFlex::MotorType::kBrushless},
    driveEncoder{driveMotor.GetEncoder()},
    steerEncoder{steerMotor.GetAbsoluteEncoder()} {
  {
    SparkFlexConfig driveConfig;
    driveConfig
      .SetIdleMode(SparkFlexConfig::IdleMode::kBrake)
      .SmartCurrentLimit(40.0)
      .Inverted(kDriveMotorInverted);

    // driveConfig.encoder
    //   .PositionConversionFactor(kDriveDistancePerRotation.value())
    //   .VelocityConversionFactor(kDriveDistancePerRotation.value() * (1.0_s / 1.0_min));

    driveConfig.closedLoop
      .SetFeedbackSensor(FeedbackSensor::kPrimaryEncoder)
      .Pid(DrivePID::kP, DrivePID::kI, DrivePID::kD);
    driveVff = DrivePID::kV;

    driveMotor.Configure(driveConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  }
  {
    SparkFlexConfig steerConfig;
    steerConfig
      .SetIdleMode(SparkFlexConfig::IdleMode::kCoast)
      .SmartCurrentLimit(60.0)
      .Inverted(kSteerMotorInverted);

    // steerConfig.encoder
    //   .PositionConversionFactor(1.00 / kSteerGearRatio)
    //   .VelocityConversionFactor(1.0 / kSteerGearRatio * (1.0_s / 1.0_min));

    steerConfig.absoluteEncoder
      .ZeroOffset(wrapOffset(offset).value())
      .Inverted(kSteerSensorInverted);

    steerConfig.closedLoop
      .SetFeedbackSensor(FeedbackSensor::kAbsoluteEncoder)
      .PositionWrappingEnabled(true)
      .Pid(SteerPID::kP, SteerPID::kI, SteerPID::kD);

    steerMotor.Configure(steerConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kNoPersistParameters);
  }
}

void RevSwerveModule::UtilInit(wpi::telemetry::TelemetryTable &table) {
  nt_driveController = table.GetTable("Drive Controller");
  nt_steerController = table.GetTable("Steer Controller");

  nt_driveController.value().get().Log("", LoopState{0.0_mps, 0.0_mps, 0.0});
  nt_steerController.value().get().Log("", LoopState{0.0_tr, 0.0_tr, 0.0});
}

void RevSwerveModule::UtilExit() {
  nt_driveController = {};
  nt_steerController = {};
}

static inline void BuildPIDConfig(SparkFlexConfig &config, const PIDUpdate &update) {
  ClosedLoopSlot slot;
  switch (update.slot) {
    case 0:
      slot = kSlot0;
      break;
    case 1:
      slot = kSlot1;
      break;
    case 2:
      slot = kSlot2;
      break;
    case 3:
      slot = kSlot3;
      break;
    default:
      return;
  }

  switch (update.term) {
    case PIDUpdate::PIDTerm::P:
      config.closedLoop.P(update.value, slot);
      break;
    case PIDUpdate::PIDTerm::I:
      config.closedLoop.I(update.value, slot);
      break;
    case PIDUpdate::PIDTerm::D:
      config.closedLoop.D(update.value, slot);
      break;
    case PIDUpdate::PIDTerm::FF:
      // TODO: Make for flexible?
      config.closedLoop.feedForward.kV(update.value, slot);
      break;
  }
}

void RevSwerveModule::UpdateDrivePID(PIDUpdate &update) {
  if (update.term == PIDUpdate::PIDTerm::FF) {
    driveVff = update.value;
    return;
  }
  SparkFlexConfig config;
  BuildPIDConfig(config, update);
  driveMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
}

void RevSwerveModule::UpdateSteerPID(PIDUpdate &update) {
  SparkFlexConfig config;
  BuildPIDConfig(config, update);
  steerMotor.Configure(config, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
}

bool RevSwerveModule::GetStatus() const {
  // TODO: actually check things
  return true;
}

void RevSwerveModule::SetSteerOffset(wpi::units::angle::turn_t offset) {
  SparkFlexConfig steerConfig;

  steerConfig.absoluteEncoder.ZeroOffset(wrapOffset(offset).value());

  steerMotor.Configure(steerConfig, rev::ResetMode::kNoResetSafeParameters, rev::PersistMode::kNoPersistParameters);
}

wpi::units::angle::turn_t RevSwerveModule::GetSteerPosition() {
  return wpi::units::angle::turn_t{steerEncoder.GetPosition().Get()};
}

void RevSwerveModule::SetSteerPosition(wpi::units::angle::turn_t position) {
  steerMotor.GetClosedLoopController().SetSetpoint(position.value(), SparkFlex::ControlType::kPosition);

  if (nt_steerController.has_value()) {
    nt_steerController.value().get().Log("", LoopState{position, GetSteerPosition(), steerMotor.GetAppliedOutput().Get()});
  }
}

void RevSwerveModule::StopSteer() {
  steerMotor.StopMotor();

  if (nt_steerController.has_value()) {
    nt_steerController.value().get().Log("", LoopState{0_tr, GetSteerPosition(), steerMotor.GetAppliedOutput().Get()});
  }
}

wpi::units::velocity::meters_per_second_t RevSwerveModule::GetDriveVelocity() {
  return wpi::units::angular_velocity::revolutions_per_minute_t{driveEncoder.GetVelocity().Get()} * kDriveDistancePerRotation;
}

void RevSwerveModule::SetDriveVelocity(wpi::units::velocity::meters_per_second_t velocity) {
  driveMotor.GetClosedLoopController().SetSetpoint((velocity / kDriveDistancePerRotation).convert<wpi::units::rpm>().value(), SparkFlex::ControlType::kVelocity, {}, velocity.value() * driveVff);

  if (nt_driveController.has_value()) {
    nt_driveController.value().get().Log("", LoopState{velocity, GetDriveVelocity(), driveMotor.GetAppliedOutput().Get()});
  }
}

void RevSwerveModule::SetDriveThrottle(double throttle) {
  driveMotor.SetThrottle(throttle);

  if (nt_driveController.has_value()) {
    nt_driveController.value().get().Log("", LoopState{0.0_mps, GetDriveVelocity(), driveMotor.GetAppliedOutput().Get()});
  }
}

void RevSwerveModule::StopDrive() {
  driveMotor.StopMotor();

  if (nt_driveController.has_value()) {
    nt_driveController.value().get().Log("", LoopState{0.0_mps, GetDriveVelocity(), driveMotor.GetAppliedOutput().Get()});
  }
}

const wpi::math::SwerveModuleVelocity RevSwerveModule::GetVelocity() {
  return {GetDriveVelocity(), GetSteerPosition()};
}

const wpi::math::SwerveModulePosition RevSwerveModule::GetPosition() {
  return {wpi::units::angle::turn_t{driveEncoder.GetPosition().Get()} * kDriveDistancePerRotation, GetSteerPosition()};
}

void RevSwerveModule::SetDesiredVelocity(wpi::math::SwerveModuleVelocity &velocity) {
  auto currentAngle = wpi::math::Rotation2d(GetSteerPosition());
  // Allow modules to flip their "positive" direction when rapidly changing requested directions
  velocity.Optimize(currentAngle);

  // Reduce speed of misoriented modules
  velocity.velocity *= (velocity.angle - currentAngle).Cos();

  SetSteerPosition(velocity.angle.Radians());
  SetDriveVelocity(velocity.velocity);
}

void RevSwerveModule::Stop() {
  StopSteer();
  StopDrive();
}
