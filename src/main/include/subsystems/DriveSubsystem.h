// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// boilerplate headers
#include <frc2/command/SubsystemBase.h>
// hardware headers
#include <ctre/phoenix6/Pigeon2.hpp>
#include <frc/ADIS16470_IMU.h>
#include <frc/Compressor.h>
// math headers
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Rotation2d.h>
// odometry headers
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
// util headers
#include <frc/Timer.h>

// local headers
#include "Constants.h"
#include "MAXSwerveModule.h"
#include "subsystems/Limelight.h"
#include "wpi/array.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:
  DriveSubsystem();

  void Periodic() override;

  // Subsystem methods go here.

  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative);

  void SetX();

  void ResetEncoders();

  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  units::degree_t GetHeading();

  void ZeroHeading();

  double GetTurnRate();

  frc::Pose2d GetPose();

  void ResetOdometry(frc::Pose2d pose);

  frc::ChassisSpeeds getChassisSpeeds(void);

  void driveRobotRelative(frc::ChassisSpeeds speeds);

  void updateOdometry();

  /**
   * Updates angular position based off of limelight values
   */
  void UpdateVisionHeading();

  /**
   * Getter for desired heading value
   * @return frc::Rotation2d
   */
  frc::Rotation2d GetDesiredHeading();

  // Idk if these member vars are supposed to be public but im afraid to break
  // anything - bear
  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         DriveConstants::kTrackWidth / 2},
      frc::Translation2d{-DriveConstants::kWheelBase / 2,
                         -DriveConstants::kTrackWidth / 2}};

  // The old gyro sensor, is not in use
  // frc::ADIS16470_IMU m_gyro;
  ctre::phoenix6::hardware::Pigeon2 m_pigeon;

private:
  frc::SwerveDrivePoseEstimator<4> m_poseEstimator;

  // Swerve modules

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearRight;

  // getter for cur model states, subroutine
  wpi::array<frc::SwerveModuleState, 4> getModuleStates();

  // Odometry class for tracking robot pose,
  // 4 defines the number of modules
  frc::SwerveDriveOdometry<4> m_odometry;
  frc::Pose2d m_pose;

  frc::ChassisSpeeds m_chassisSpeeds;

  // Limelight member vars
  Limelight Limelight;
  frc::Rotation2d desiredHeading;
};
