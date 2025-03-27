// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

// see
// https://github.wpilib.org/allwpilib/docs/release/cpp/classfrc_1_1_iterative_robot_base.html
// for a list of periodic methods.

class Robot : public frc::TimedRobot {
public:
  /*
   * Robot constructor
   */
  Robot();

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  void RobotInit() override;
  /**
   * This function is called every 20 ms, no matter the mode. Use
   * this for items like diagnostics that you want to run during disabled,
   * autonomous, teleoperated and test.
   *
   * <p> This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  void RobotPeriodic() override;

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  void DisabledInit() override;
  void DisabledPeriodic() override;

  /**
   * This autonomous runs the autonomous command selected by your {@link
   * RobotContainer} class.
   */
  void AutonomousInit() override;
  /** This function is called periodically during autonomous. */
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  /**
   * This function is called periodically during operator control.
   */
  void TeleopPeriodic() override;

  void TestInit() override;
  /**
   * This function is called periodically during test mode.
   */
  void TestPeriodic() override;

private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  pathplanner::PathPlannerAuto* m_autonomousCommand = nullptr;

  // Add robot container (main logic) as a member
  RobotContainer m_container;
};
