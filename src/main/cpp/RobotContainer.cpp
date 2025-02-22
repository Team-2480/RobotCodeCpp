// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <utility>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer()
{
    // Configure the button bindings
    ConfigureButtonBindings();

    // Set up default drive command
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
    // Uses lambda functions, RunCommand takes 2 args
    m_drive.SetDefaultCommand(frc2::RunCommand(
        [this]
        {
            double y_val;
            double x_val;
            y_val = frc::ApplyDeadband(
                m_driverController.GetLeftY(), OIConstants::kDriveDeadband,
                DriveConstants::kTargetSpeed.value());

            x_val = frc::ApplyDeadband(
                m_driverController.GetLeftX(), OIConstants::kDriveDeadband,
                DriveConstants::kTargetSpeed.value());
            m_drive.Drive(
                y_mult * units::meters_per_second_t{y_val},
                x_mult * units::meters_per_second_t{x_val},
                -units::radians_per_second_t{frc::ApplyDeadband(
                    m_driverController.GetRightX(), OIConstants::kDriveDeadband,
                    DriveConstants::kTargetSpeed.value())},
                global_local);
            printf("Yaw: %f\n", m_drive.m_pigeon.GetYaw().GetValue().value());
        },
        {&m_drive}));
}

void RobotContainer::ConfigureButtonBindings()
{
    // the while true method will run the lambda function specified until the
    // button class is in false state.
    frc2::JoystickButton(&m_driverController,
                         frc::XboxController::Button::kRightBumper)
        .WhileTrue(new frc2::RunCommand([this]
                                        { m_drive.SetX(); },
                                        {&m_drive}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
        .WhileTrue(new frc2::RunCommand(
            [this]
            { m_drive.m_pigeon.SetYaw((units::degree_t)0); },
            {&m_drive}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
        .ToggleOnTrue(m_shooter.Shoot());

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            { global_local = !global_local;
            printf("global local: %i\n", global_local); },
            {}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
        .ToggleOnTrue(m_climb.Trigger());

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            { m_climb.Stop(); },
            {}));
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand(
            [this]() {},
            {}));
}