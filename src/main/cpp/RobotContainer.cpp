// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "LimelightHelpers.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Configure the button bindings
    ConfigureButtonBindings();
    ConfigureButtonBindingsJoystick();

    // Set up default drive command
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    // Uses lambda functions, RunCommand takes 2 args
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            printf("default\n");
            if (slowMode)
            {
                m_drive.Drive(
                    -units::meters_per_second_t{frc::ApplyDeadband(
                        std::pow(m_driverJoystick.GetY(), 3), OIConstants::kDriveDeadband,
                        DriveConstants::kSlowTargetSpeed.value())},
                    -units::meters_per_second_t{frc::ApplyDeadband(
                        std::pow(m_driverJoystick.GetX(), 3), OIConstants::kDriveDeadband,
                        DriveConstants::kSlowTargetSpeed.value())},
                    -units::radians_per_second_t{frc::ApplyDeadband(
                        std::pow(m_driverJoystick.GetTwist(), 3), OIConstants::kDriveDeadband,
                        DriveConstants::kSlowTargetSpeed.value())},
                    global_local);
            }
            else if (m_drive.alignMode())
            {
                m_drive.driveAlign();
            }
            else
            {
                m_drive.Drive(
                    -units::meters_per_second_t{frc::ApplyDeadband(
                        std::pow(m_driverJoystick.GetY(), 3), OIConstants::kDriveDeadband,
                        DriveConstants::kTargetSpeed.value())},
                    -units::meters_per_second_t{frc::ApplyDeadband(
                        std::pow(m_driverJoystick.GetX(), 3), OIConstants::kDriveDeadband,
                        DriveConstants::kTargetSpeed.value())},
                    -units::radians_per_second_t{frc::ApplyDeadband(
                        std::pow(m_driverJoystick.GetTwist(), 3), OIConstants::kDriveDeadband,
                        DriveConstants::kTargetSpeed.value())},
                    global_local);
            }
        },
        {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings()
{
    // the while true method will run the lambda function specified until the
    // button class is in false state.

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
        .ToggleOnTrue(new frc2::InstantCommand([this]()
                                               { m_climb.m_regulator.Zero(); }));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
        .ToggleOnTrue(m_shooter.Shoot());
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
        .ToggleOnFalse(m_shooter.Stop());

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
        .ToggleOnTrue(m_shooter.Rev());
    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
        .ToggleOnFalse(m_shooter.Stop());

    frc2::JoystickButton(&m_driverJoystick, 1)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            {
                slowMode = !slowMode;
            },
            {}));
    frc2::JoystickButton(&m_driverJoystick, 1)
        .ToggleOnFalse(new frc2::InstantCommand(
            [this]
            {
                slowMode = !slowMode;
            },
            {}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper)
        .ToggleOnTrue(m_drive.startAlignment())
        .ToggleOnFalse(m_drive.stopAlignment());

    frc2::JoystickButton(&m_driverJoystick, 2)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            { global_local = !global_local;
            printf("global local: %i\n", global_local); },
            {}));

    frc2::JoystickButton(&m_driverJoystick, 11)
        .WhileTrue(new frc2::RunCommand(
            [this]
            { m_drive.m_pigeon.SetYaw((units::degree_t)0); },
            {&m_drive}));

    frc2::POVButton(&m_driverController, 0, 0)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            {
                m_climb.Spool();
            }));

    frc2::POVButton(&m_driverController, 0, 0)
        .ToggleOnFalse(new frc2::InstantCommand([this]()
                                                { m_climb.Stop(); }));

    frc2::POVButton(&m_driverController, 180, 0)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            {
                m_climb.Unspool();
            }));

    frc2::POVButton(&m_driverController, 180, 0)
        .ToggleOnFalse(new frc2::InstantCommand([this]()
                                                { m_climb.Stop(); }));
}

void RobotContainer::ConfigureButtonBindingsJoystick()
{
}

pathplanner::PathPlannerAuto *RobotContainer::GetAutonomousCommand()
{
    pathplanner::NamedCommands::registerCommand("rev", m_shooter.Rev());
    pathplanner::NamedCommands::registerCommand("shoot", m_shooter.Shoot());
    pathplanner::NamedCommands::registerCommand("stop", m_shooter.Stop());
    pathplanner::NamedCommands::registerCommand("startalign", m_drive.startAlignment());
    pathplanner::NamedCommands::registerCommand("stopalign", m_drive.stopAlignment());
    pathplanner::PathPlannerAuto *path = new pathplanner::PathPlannerAuto("RblueSide start");
    m_drive.ResetOdometry(path->getStartingPose());
    printf("reset position.\n");
    return path;
}