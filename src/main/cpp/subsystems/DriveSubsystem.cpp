// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "LimelightHelpers.h"
#include "subsystems/MAXSwerveModule.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_pigeon(kPigeonCanId, "rio"),

      m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_poseEstimator{kDriveKinematics, frc::Rotation2d(units::degree_t{m_pigeon.GetYaw().GetValue()}), {m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()}, frc::Pose2d{}},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::degree_t{m_pigeon.GetYaw().GetValue()}),

                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}}

{
  m_alignMode = false;
   // Usage reporting for MAXSwerve template
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
  // m_compressor is on closed loop mode

  pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds.
  // Also optionally outputs individual module feedforwards
  auto pid = std::make_shared<
      pathplanner::PPHolonomicDriveController>( // PPHolonomicController is
                                                // the built in path
                                                // following controller for
                                                // holonomic drive trains
      pathplanner::PIDConstants(0.4, 0.0, 0.2), // Translation PID constants
      pathplanner::PIDConstants(0.4, 0.0, 0.2)  // Rotation PID constants
  );

  // Configure the AutoBuilder last
  pathplanner::AutoBuilder::configure(
      [this]()
      { return m_poseEstimator.GetEstimatedPosition(); }, // Robot pose supplier
      [this](frc::Pose2d pose)
      {
        m_poseEstimator.ResetPose(pose);
      }, // Method to reset odometry (will be called if your auto has a starting
         // pose)
      [this]()
      {
        return getChassisSpeeds();
      }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds, auto feedforwards)
      {
        if (alignMode())
        {
          driveAlign();
          return;
        }
        driveRobotRelative(speeds);
      },
      pid,
      config, // The robot configuration
      []()
      {
        // TODO: figure out alliance logic
        //  Boolean supplier that controls when the path will be mirrored for the
        //  red alliance This will flip the path being followed to the red side
        //  of the field. THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        // auto alliance = pathplanner::DriverStation::GetAlliance();
        // if (alliance) {
        //     return alliance.value() == DriverStation::Alliance::kRed;
        // }
        return false;
      },
      this // Reference to this subsystem to set requirements
  );
}

void DriveSubsystem::driveAlign()
{
  printf("aligning\n");

  double p = 5;
  double targetingSidewaysSpeed = LimelightHelpers::getTX("limelight") * p;

  p = -10;
  // 1.5 = 54"
  // 3.36 = 35.25"
  double targetingForwardSpeed = (3.36 - LimelightHelpers::getTA("limelight")) * p;

  if (LimelightHelpers::getTA("limelight") == 0)
    return;

  targetingForwardSpeed *= DriveConstants::kSlowTargetSpeed.value();
  targetingSidewaysSpeed *= DriveConstants::kSlowTargetSpeed.value();

  Drive(
      units::meters_per_second_t{targetingForwardSpeed},
      units::meters_per_second_t{targetingSidewaysSpeed},
      units::radians_per_second_t{0},
      false);
}

void DriveSubsystem::Periodic()
{
  
  if (alignMode()) {
    driveAlign();
  }
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(
      frc::Rotation2d(units::degree_t{m_pigeon.GetYaw().GetValue()}),
      {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
       m_frontRight.GetPosition(), m_rearRight.GetPosition()});

  // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
  m_poseEstimator.Update(
      frc::Rotation2d(units::degree_t{m_pigeon.GetYaw().GetValue()}),
      {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
       m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::driveRobotRelative(frc::ChassisSpeeds speeds)
{
  m_chassisSpeeds = speeds;
  auto states = kDriveKinematics.ToSwerveModuleStates(speeds);

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative)
{
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  auto speeds =
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::degree_t{m_pigeon.GetYaw().GetValue()}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered};

  driveRobotRelative(speeds);
}

void DriveSubsystem::SetX()
{
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders()
{
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading()
{
  return frc::Rotation2d(units::degree_t{m_pigeon.GetYaw().GetValue()})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { m_pigeon.Reset(); }

double DriveSubsystem::GetTurnRate()
{
  return -m_pigeon.GetAngularVelocityZWorld().GetValue().value();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose)
{
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}

frc::ChassisSpeeds DriveSubsystem::getChassisSpeeds(void)
{
  return m_chassisSpeeds;
}