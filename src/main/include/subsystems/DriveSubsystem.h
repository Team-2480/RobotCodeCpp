// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/ADIS16470_IMU.h>
#include <frc/Compressor.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/InstantCommand.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include "Constants.h"
#include "MAXSwerveModule.h"

struct RobotPoint {
    double x, y, rot;
    RobotPoint(double x_val, double y_val, double rot_val) : x(x_val), y(y_val), rot(rot_val) {}
};

static std::map<std::string, std::vector<RobotPoint>> autoMap {
        {"RR start", {RobotPoint(14.197, 2.055, 119.427)}},
        {"RB start", {RobotPoint(14.180, 5.958, -120.324)}},
        {"RB3 start", {RobotPoint(14.180, 5.958, -120.324)}},
        {"RB2.5 start", {RobotPoint(14.180, 5.958, -120.324)}},
        
        {"BR start", {RobotPoint(3.310, 2.089, 58.841)}},
        {"BB start", {RobotPoint(3.369, 5.988, -59.642)}},
        {"10ft", {RobotPoint(0.895, 6.0, 180.000)}},
    };

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  // Subsystem methods go here.

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading();

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   * Getter for ChassisSpeeds
   *
   * @return frs::ChassisSpeeds the reletive speeds obj
   */
  frc::ChassisSpeeds getChassisSpeeds(void);

  /**
   * Driving helper functions
   *
   * @param speeds: frc::ChassisSpeeds
   */
  void driveRobotRelative(frc::ChassisSpeeds speeds);


  /**
   * Use limelight to align the robot
   */
  void driveAlign();
  

  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  // The gyro sensor
  // frc::ADIS16470_IMU m_gyro;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon;

frc2::CommandPtr startAlignment(){
return frc2::CommandPtr(frc2::InstantCommand([this]{
  printf("align mode\n");
    m_alignMode = true;
  }));
}

frc2::CommandPtr stopAlignment(){
return frc2::CommandPtr(frc2::InstantCommand([this]{
    m_alignMode = false;
    if (!m_autoName.has_value())
      return;
    auto resetPoints = autoMap[m_autoName.value().first];
    auto resetPoint = resetPoints[0];
    m_poseEstimator.ResetPose(frc::Pose2d(units::meter_t{resetPoint.x},units::meter_t{resetPoint.y}, frc::Rotation2d(units::degree_t{resetPoint.rot})));
    m_autoName.value().second++;
  }));
}
bool alignMode() {
  return m_alignMode;
}

  std::optional<std::pair<std::string, uint32_t>> m_autoName;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  // Odometry class for tracking robot pose,
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> m_odometry;
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;
  bool m_alignMode = false;
  frc::Pose2d m_pose;

  frc::ChassisSpeeds m_chassisSpeeds;
};
