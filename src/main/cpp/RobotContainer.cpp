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
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <utility>

#include "Constants.h"
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

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
        .ToggleOnTrue(new frc2::InstantCommand([this]()
                                               { m_climb.m_regulator.Zero(); }));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
        .ToggleOnTrue(m_shooter.Shoot());

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kA)
        .ToggleOnFalse(new frc2::InstantCommand([this]()
                                                { m_shooter.Stop(); }));

    // frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
    //     .ToggleOnTrue(new frc2::InstantCommand(
    //         [this]
    //         { global_local = !global_local;
    //         printf("global local: %i\n", global_local); },
    //         {}));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            {
                m_climb.Spool();
            }));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
        .ToggleOnFalse(new frc2::InstantCommand([this]()
                                                { m_climb.Stop(); }));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
        .ToggleOnTrue(new frc2::InstantCommand(
            [this]
            {
                m_climb.Unspool();
            }));

    frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kX)
        .ToggleOnFalse(new frc2::InstantCommand([this]()
                                                { m_climb.Stop(); }));
}

void RobotContainer::ConfigureButtonBindingsJoystick()
{
}

frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return new frc2::SequentialCommandGroup(
        frc2::InstantCommand(
            [this]() {},
            {}));

    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

    pathplanner::AutoBuilder::configureCustom([this]()
                                        { return m_drive.GetPose(); }, [this](std::shared_ptr<pathplanner::PathPlannerPath> path){
                                           
                                        }, [this](frc::Pose2d pose)
                                        { m_drive.ResetOdometry(pose); });
}