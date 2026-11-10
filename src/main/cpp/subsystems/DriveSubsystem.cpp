// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.hpp"

#include <wpi/commands2/Commands.hpp>
#include <wpi/commands2/InstantCommand.hpp>
#include <wpi/commands2/RunCommand.hpp>
#include <wpi/commands2/button/RobotModeTriggers.hpp>
#include <wpi/telemetry/Telemetry.hpp>
#include <wpi/tunables/Tunables.hpp>

#include <memory>
#include <string>

#include "Constants.hpp"
#include "subsystems/RevSwerveModule.hpp"

using namespace DriveConstants;

// std::optional<wpi::math::Rotation3d> GetRotation3D(studica::Navx &navX) {
//   wpi::math::Rotation3d rotation;
//   if (navX.GetRotation3D(rotation) == 0) {
//     return rotation;
//   } else {
//     return {};
//   }
// }

DriveSubsystem::DriveSubsystem() :
    SubsystemBase("Drive Subsystem"),
    frontLeftModule{
      new RevSwerveModule(kDriveCanPort, kFrontLeftDriveMotorCanID, kFrontLeftSteerMotorCanID, kFrontLeftSteerOffset)
    },
    frontRightModule{
      new RevSwerveModule(kDriveCanPort, kFrontRightDriveMotorCanID, kFrontRightSteerMotorCanID, kFrontRightSteerOffset)
    },
    rearLeftModule{
      new RevSwerveModule(kDriveCanPort, kRearLeftDriveMotorCanID, kRearLeftSteerMotorCanID, kRearLeftSteerOffset)
    },
    rearRightModule{
      new RevSwerveModule(kDriveCanPort, kRearRightDriveMotorCanID, kRearRightSteerMotorCanID, kRearRightSteerOffset)
    },
    m_navX{wpi::I2C::Port::PORT_0},
    m_poseEstimator{kDriveKinematics, wpi::math::Rotation3d{wpi::units::degree_t{m_navX.GetYaw()}}, GetModulePositions(), wpi::math::Pose3d()},
    m_driveTuner{
      [this](PIDUpdate update) {
        frontLeftModule->UpdateDrivePID(update);
        frontRightModule->UpdateDrivePID(update);
        rearLeftModule->UpdateDrivePID(update);
        rearRightModule->UpdateDrivePID(update);
      },
      DrivePID::kP,
      DrivePID::kI,
      DrivePID::kD,
      DrivePID::kV
    },
    m_steerTuner{
      [this](PIDUpdate update) {
        frontLeftModule->UpdateSteerPID(update);
        frontRightModule->UpdateSteerPID(update);
        rearLeftModule->UpdateSteerPID(update);
        rearRightModule->UpdateSteerPID(update);
      },
      SteerPID::kP,
      SteerPID::kI,
      SteerPID::kD
    },
    m_xController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
    m_yController{TranslationPID::kP, TranslationPID::kI, TranslationPID::kD},
    m_rController{OrientationPID::kP, OrientationPID::kI, OrientationPID::kD},
    table{wpi::telemetry::GetTable("Drive")} {
  const wpi::math::Pose3d initialPose = m_poseEstimator.GetEstimatedPosition();

  auto &xTable = table.GetTable("X");
  xTable.Log("Position", initialPose.X());
  xTable.Log("Setpoint", 0.0_m);
  xTable.Log("Output", 0.0);

  auto &yTable = table.GetTable("Y");
  yTable.Log("Position", initialPose.Y());
  yTable.Log("Setpoint", 0.0_m);
  yTable.Log("Output", 0.0);

  auto &rTable = table.GetTable("R");
  rTable.Log("Position", initialPose.Rotation().Z().convert<wpi::units::deg>());
  rTable.Log("Setpoint", 0.0_deg);
  rTable.Log("Output", 0.0);

  // Bind test init and test exit to mode transition
  wpi::cmd::RobotModeTriggers::Utility()
    .OnTrue(wpi::cmd::InstantCommand([this]() { UtilInit(); }).ToPtr())
    .OnFalse(wpi::cmd::InstantCommand([this]() { UtilExit(); }).ToPtr());

  wpi::telemetry::Log("Field", field);
}

void DriveSubsystem::Periodic() {
  wpi::math::Rotation3d heading = wpi::math::Rotation3d{wpi::units::degree_t{m_navX.GetYaw()}};

  wpi::math::Pose3d pose = m_poseEstimator.Update(heading, GetModulePositions());

  table.Log("X/Position", pose.X());
  table.Log("Y/Position", pose.Y());
  table.Log("R/Position", pose.Rotation().Z().convert<wpi::units::deg>());

  table.Log("FrontLeft/Drive", frontLeftModule->GetPosition().distance);
  table.Log("FrontRight/Drive", frontRightModule->GetPosition().distance);
  table.Log("RearLeft/Drive", rearLeftModule->GetPosition().distance);
  table.Log("RearRight/Drive", rearRightModule->GetPosition().distance);

  table.Log("FrontLeft/Steer", frontLeftModule->GetSteerPosition().convert<wpi::units::deg>());
  table.Log("FrontRight/Steer", frontRightModule->GetSteerPosition().convert<wpi::units::deg>());
  table.Log("RearLeft/Steer", rearLeftModule->GetSteerPosition().convert<wpi::units::deg>());
  table.Log("RearRight/Steer", rearRightModule->GetSteerPosition().convert<wpi::units::deg>());

  field.SetRobotPose(pose.ToPose2d());
  wpi::telemetry::Log("Field", field);
}

void DriveSubsystem::SimulationPeriodic() {
  wpi::math::ChassisVelocities fieldVelocity = m_velocity.ToRobotRelative(GetPose().ToPose2d().Rotation());

  constexpr wpi::units::millisecond_t loopRate = 20_ms;
  m_poseEstimator.ResetPose(
    m_poseEstimator.GetEstimatedPosition() +
    wpi::math::Transform3d(wpi::math::Transform2d(
      fieldVelocity.vx * loopRate,
      fieldVelocity.vy * loopRate,
      fieldVelocity.omega * loopRate
    ))
  );
}

void DriveSubsystem::UtilInit() {
  wpi::tunables::Publish("Drive/Drive PID", m_driveTuner);
  wpi::tunables::Publish("Drive/Steer PID", m_steerTuner);

  // driveSetupTab.Add("Legend", "(Blue) setpoint, (Red) measured, (Green) output");
  // steerSetupTab.Add("Legend", "(Blue) setpoint, (Red) measured, (Green) output");

  frontLeftModule->UtilInit(table.GetTable("Front left"));
  frontRightModule->UtilInit(table.GetTable("Front right"));
  rearLeftModule->UtilInit(table.GetTable("Rear left"));
  rearRightModule->UtilInit(table.GetTable("Rear right"));

  m_sysIdSelectable = SysIdSelectable{std::pair{std::string{"Steer"}, SteerSysId()}, std::pair{std::string{"Drive"}, DriveSysId()}};

  wpi::tunables::Publish("Drive/SysId", m_sysIdSelectable.value());
}

void DriveSubsystem::UtilExit() {
  frontLeftModule->UtilExit();
  frontRightModule->UtilExit();
  rearLeftModule->UtilExit();
  rearRightModule->UtilExit();
}

void DriveSubsystem::ResetFieldOrientation(bool inverted) {
  m_poseEstimator.ResetPose(wpi::math::Pose3d(GetPose().Translation(), inverted ? wpi::math::Rotation3d(180_deg) : wpi::math::Rotation3d()));
}

void DriveSubsystem::Drive(wpi::units::meters_per_second_t xSpeed, wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot, bool fieldRelative, wpi::units::meter_t x_center, wpi::units::meter_t y_center) {
  wpi::math::Rotation2d heading = GetPose().ToPose2d().Rotation();
  m_velocity = {
    xSpeed, ySpeed, rot
  };


  table.Log("X/Output", xSpeed);
  table.Log("Y/Output", ySpeed);
  table.Log("R/Output", rot.convert<wpi::units::deg_per_s>());

  if (fieldRelative) {
    m_velocity = m_velocity.ToFieldRelative(heading);
  }

  SetModuleVelocities(kDriveKinematics.ToSwerveModuleVelocities(m_velocity, {x_center, y_center}));
}

void DriveSubsystem::FollowTrajectory(const choreo::SwerveSample &sample) {
  wpi::math::Pose2d pose = GetPose().ToPose2d();

  wpi::units::radian_t robot_heading = pose.Rotation().Radians();
  wpi::units::radian_t sample_heading = sample.heading;
  wpi::units::radian_t target_heading = wpi::math::AngleModulus(sample_heading - robot_heading) + robot_heading;

  wpi::units::meters_per_second_t xFeedback{m_xController.Calculate(pose.X().value(), sample.x.value())};
  wpi::units::meters_per_second_t yFeedback{m_yController.Calculate(pose.Y().value(), sample.y.value())};
  wpi::units::radians_per_second_t rotFeedback{
    m_rController.Calculate(robot_heading.value(), target_heading.value())
  };

  table.Log("X/Setpoint", sample.x);
  table.Log("Y/Setpoint", sample.y);
  table.Log("R/Setpoint", sample.heading.convert<wpi::units::deg>());

  Drive(
    sample.vx + xFeedback,
    sample.vy + yFeedback,
    sample.omega + rotFeedback,
    true
  );
}

void DriveSubsystem::SteerTo(wpi::units::meters_per_second_t xSpeed, wpi::units::meters_per_second_t ySpeed, wpi::units::radians_per_second_t rot, bool fieldRelative, wpi::units::meter_t x_center, wpi::units::meter_t y_center) {
  wpi::math::Rotation2d heading = GetPose().ToPose2d().Rotation();

  SetModuleVelocities(kDriveKinematics.ToSwerveModuleVelocities(fieldRelative ? wpi::math::ChassisVelocities(xSpeed, ySpeed, rot).ToRobotRelative(heading) : wpi::math::ChassisVelocities{xSpeed, ySpeed, rot}, {x_center, y_center}), true);
}

void DriveSubsystem::Stop() {
  table.Log("X/Output", 0);
  table.Log("Y/Output", 0);
  table.Log("R/Output", 0);

  frontLeftModule->Stop();
  frontRightModule->Stop();
  rearLeftModule->Stop();
  rearRightModule->Stop();
}

const std::array<wpi::math::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() {
  return {
    frontLeftModule->GetPosition(),
    frontRightModule->GetPosition(),
    rearLeftModule->GetPosition(),
    rearRightModule->GetPosition()
  };
}

void DriveSubsystem::SetModuleVelocities(std::array<wpi::math::SwerveModuleVelocity, 4> desiredStates, bool steerOnly) {
  auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

  if (frontLeft.velocity == 0.0_mps &&
      frontRight.velocity == 0.0_mps &&
      rearLeft.velocity == 0.0_mps &&
      rearRight.velocity == 0.0_mps) {
    frontLeftModule->Stop();
    frontRightModule->Stop();
    rearLeftModule->Stop();
    rearRightModule->Stop();

    return;
  }

  if (!steerOnly) {
    frontLeftModule->SetDesiredVelocity(frontLeft);
    frontRightModule->SetDesiredVelocity(frontRight);
    rearLeftModule->SetDesiredVelocity(rearLeft);
    rearRightModule->SetDesiredVelocity(rearRight);
  } else {
    frontLeftModule->SetSteerPosition(frontLeft.angle.Radians());
    frontRightModule->SetSteerPosition(frontRight.angle.Radians());
    rearLeftModule->SetSteerPosition(rearLeft.angle.Radians());
    rearRightModule->SetSteerPosition(rearRight.angle.Radians());
  }
}

void DriveSubsystem::ResetPose(wpi::math::Pose3d pose) {
  m_poseEstimator.ResetPose(pose);
}

wpi::math::Pose3d DriveSubsystem::GetPose() {
  return m_poseEstimator.GetEstimatedPosition();
}

void DriveSubsystem::UpdateVisionPose(wpi::math::Pose3d measurement, wpi::units::millisecond_t timestamp) {
  m_poseEstimator.AddVisionMeasurement(measurement, timestamp, {0.5, 0.5, 0.5, 0.8});
}

wpi::math::ChassisVelocities DriveSubsystem::GetChassisVelocities() {
  return kDriveKinematics.ToChassisVelocities({
    frontLeftModule->GetVelocity(),
    frontRightModule->GetVelocity(),
    rearLeftModule->GetVelocity(),
    rearRightModule->GetVelocity(),
  });
}

std::unique_ptr<wpi::cmd::sysid::SysIdRoutine> DriveSubsystem::SteerSysId() {
  return std::make_unique<wpi::cmd::sysid::SysIdRoutine>(
    wpi::cmd::sysid::Config{{}, {}, {}, nullptr},
    wpi::cmd::sysid::Mechanism{
      [this](wpi::units::volt_t steerVoltage) {
        frontLeftModule->SetSteerVoltage(steerVoltage);
        frontRightModule->SetSteerVoltage(steerVoltage);
        rearLeftModule->SetSteerVoltage(steerVoltage);
        rearRightModule->SetSteerVoltage(steerVoltage);
      },
      [this](wpi::sysid::SysIdRoutineLog *log) {
        frontLeftModule->LogSteerInfo(log->Motor("frontLeftSteer"));
        frontRightModule->LogSteerInfo(log->Motor("frontRightSteer"));
        rearLeftModule->LogSteerInfo(log->Motor("rearLeftSteer"));
        rearRightModule->LogSteerInfo(log->Motor("rearRightSteer"));
      },
      this
    }
  );
}

std::unique_ptr<wpi::cmd::sysid::SysIdRoutine> DriveSubsystem::DriveSysId() {
  return std::make_unique<wpi::cmd::sysid::SysIdRoutine>(
    wpi::cmd::sysid::Config{{}, {}, {}, nullptr},
    wpi::cmd::sysid::Mechanism{
      [this](wpi::units::volt_t driveVoltage) {
        this->SteerTo(1.0_mps, 0.0_mps, 0.0_rad_per_s, false);
        frontLeftModule->SetDriveVoltage(driveVoltage);
        frontRightModule->SetDriveVoltage(driveVoltage);
        rearLeftModule->SetDriveVoltage(driveVoltage);
        rearRightModule->SetDriveVoltage(driveVoltage);
      },
      [this](wpi::sysid::SysIdRoutineLog *log) {
        frontLeftModule->LogDriveInfo(log->Motor("frontLeftDrive"));
        frontRightModule->LogDriveInfo(log->Motor("frontRightDrive"));
        rearLeftModule->LogDriveInfo(log->Motor("rearLeftDrive"));
        rearRightModule->LogDriveInfo(log->Motor("rearRightDrive"));
      },
      this
    }
  );
}

wpi::cmd::CommandPtr DriveSubsystem::SysId() {
  return wpi::cmd::Defer(
    [this]() {
      if (m_sysIdSelectable.has_value()) {
        return m_sysIdSelectable->RunSelected();
      } else {
        return wpi::cmd::None();
      }
    },
    {this}
  );
}
