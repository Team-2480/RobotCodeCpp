// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Robot container handles OI things. Input mappings and command bindings.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ClimbSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  /**
   * (constructor)
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  RobotContainer();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  frc2::Command *GetAutonomousCommand();

private:
  // The driver's controller
  // Perhaps rename so its evident this represents HID not an arbitrary data structure.
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  ClimbSubsystem m_climb;
  float x_mult = -1;
  float y_mult = -1;
  bool global_local = false;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command *> m_chooser;

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a GenericHID object or one of its
   * subclasses eg. Joystick or XboxController, and then calling by passing it
   * to a JoystickButton.
   */
  void ConfigureButtonBindings();
};
