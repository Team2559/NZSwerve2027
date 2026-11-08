#include <wpi/commands2/InstantCommand.hpp>
#include <wpi/commands2/RunCommand.hpp>
#include <wpi/net/WebServer.hpp>
#include <wpi/system/Filesystem.hpp>

#include "helpers/GamepadUtil.h"
#include "opmode/DriveOnly.hpp"

DriveOnly::DriveOnly(Robot &robot) : robot{robot} {
  wpi::net::WebServer::GetInstance().Start(
      5800, wpi::filesystem::GetDeployDirectory());
}

void DriveOnly::Start() { SetupDriverBindings(); }
void DriveOnly::End() { TeardownDriverBindings(); }

void DriveOnly::SetupDriverBindings() {
  robot.m_driveSubsystem.SetDefaultCommand(
      wpi::cmd::RunCommand(
          [this]() {
            const auto controls = GetDriveTeleopControls();

            robot.m_driveSubsystem.Drive(
                std::get<0>(controls) * DriveConstants::kMaxDriveSpeed,
                std::get<1>(controls) * DriveConstants::kMaxDriveSpeed,
                std::get<2>(controls) * DriveConstants::kMaxTurnRate,
                std::get<3>(controls));
          },
          {&robot.m_driveSubsystem})
          .WithName("TeleopDrive"));

  m_driverController.Guide().OnTrue(
      wpi::cmd::InstantCommand(
          [this]() {
            robot.m_driveSubsystem.ResetFieldOrientation(m_isRedAlliance);
          },
          {&robot.m_driveSubsystem})
          .IgnoringDisable(true)
          .WithName("Reset Field Orientation"));
}

void DriveOnly::TeardownDriverBindings() {}

std::tuple<double, double, double, bool> DriveOnly::GetDriveTeleopControls() {
  /*
  The robot's frame of reference is the standard unit circle, from
  trigonometry. However, the front of the robot is facing along the positive
  X axis. This means the positive Y axis extends outward from the left (or
  port) side of the robot. Positive rotation is counter-clockwise. On the
  other hand, as the controller is held, the Y axis is aligned with forward.
  And, specifically, it is the negative Y axis which extends forward. So,
  the robot's X is the controllers inverted Y. On the controller, the X
  axis lines up with the robot's Y axis. And, the controller's positive X
  extends to the right. So, the robot's Y is the controller's inverted X.
  Finally, the other controller joystick is used for commanding rotation and
  things work out so that this is also an inverted X axis.
  */
  double leftTriggerAnalogValue = m_driverController.GetLeftTrigger();
  double leftStickX = -m_driverController.GetLeftY();
  double leftStickY = -m_driverController.GetLeftX();
  double rightStickRot = -m_driverController.GetRightX();

  // Limit driving speed according to the current maximum speed
  if (leftTriggerAnalogValue < .05) {
    leftStickX *= DriveConstants::kSlowDrivePercent;
    leftStickY *= DriveConstants::kSlowDrivePercent;
  } else {
    // double fastDrivePercent = nt_fastDriveSpeed.Get(1.0);
    double fastDrivePercent = 1;

    leftStickX *= fastDrivePercent;
    leftStickY *= fastDrivePercent;
  }

  // Flip field-relative velocities
  // if the driver is on the red side of the field
  if (m_isRedAlliance) {
    leftStickX *= -1.0;
    leftStickY *= -1.0;
  }

  // Rescale final speeds to be more controllable
  if (m_triggerSpeedEnabled) {
    // scale speed by analog trigger
    double RightTrigAnalogVal = m_driverController.GetRightTrigger();
    RightTrigAnalogVal = ConditionRawTriggerInput(RightTrigAnalogVal);

    if (leftStickX != 0) {
      double LeftStickTheta = atan(leftStickY / leftStickX);
      leftStickX = RightTrigAnalogVal * cos(LeftStickTheta);
      leftStickY = RightTrigAnalogVal * sin(LeftStickTheta);
    } else if (leftStickY != 0) {
      leftStickY = std::copysign(RightTrigAnalogVal, leftStickY);
    }
  } else {
    // scale speed by analog stick
    leftStickX = ConditionRawJoystickInput(leftStickX);
    leftStickY = ConditionRawJoystickInput(leftStickY);
  }

  // Rescale final rotation speed to be more controllable
  rightStickRot = ConditionRawJoystickInput(rightStickRot);

  return std::make_tuple(leftStickX, leftStickY, rightStickRot,
                         m_fieldOriented);
}
