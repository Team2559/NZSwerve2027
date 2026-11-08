// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.hpp"

#include "opmode/DriveOnly.hpp"

Robot::Robot() { AddOpModes(); }

void Robot::RobotPeriodic() { wpi::cmd::CommandScheduler::GetInstance().Run(); }

void Robot::AddOpModes() {
  AddOpMode<DriveOnly>(wpi::RobotMode::TELEOPERATED, "Drive Only",
                       "Driving OpMode");

  PublishOpModes();
}

#ifndef RUNNING_WPILIB_TESTS
int main() { return wpi::StartRobot<Robot>(); }
#endif
