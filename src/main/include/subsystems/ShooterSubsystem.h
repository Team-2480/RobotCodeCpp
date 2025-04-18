#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <rev/SparkAbsoluteEncoder.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkMax.h>
#include <rev/SparkRelativeEncoder.h>

#include "Configs.h"
#include "Constants.h"

class ShooterSubsystem
{
private:
  rev::spark::SparkMax m_top;
  rev::spark::SparkClosedLoopController m_topClosedLoopController =
      m_top.GetClosedLoopController();
  rev::spark::SparkMax m_bottom;
  rev::spark::SparkClosedLoopController m_bottomClosedLoopController =
      m_bottom.GetClosedLoopController();

public:
  ShooterSubsystem()
      : m_top(ShooterConstants::kTopShooterCanId,
              rev::spark::SparkMax::MotorType::kBrushless),
        m_bottom(ShooterConstants::kBottomShooterCanId,
                 rev::spark::SparkMax::MotorType::kBrushless)
  {
    m_top.Configure(Configs::MAXSwerveModule::DirectConfig(), // TODO: Configure this properly
                    rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kNoPersistParameters);

    m_bottom.Configure(Configs::MAXSwerveModule::DirectConfig(),
                       rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                       rev::spark::SparkBase::PersistMode::kNoPersistParameters);
    Stop();
  }

  ~ShooterSubsystem() {}

  frc2::CommandPtr Rev();
  frc2::CommandPtr Shoot();
  frc2::CommandPtr Stop();
};
