// Copyright (c) FRC 2559, FIRST, and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * Rescale trigger input to be more controllable
 * Adds some deadzone and cubes the output
 */
double ConditionRawTriggerInput(double RawTrigVal) noexcept;

/**
 * Rescale joystick input to be more controllable
 * Adds some deadzone and cubes the output
 */
double ConditionRawJoystickInput(double RawJoystickVal,
                                 double mixer = 0.75) noexcept;
