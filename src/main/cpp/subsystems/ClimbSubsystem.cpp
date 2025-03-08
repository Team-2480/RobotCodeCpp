#include "subsystems/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"
#include "rev/SparkMax.h"

ClimbSubsystem::ClimbSubsystem()
    : m_climbingMotor(ClimbConstants::kClimbMotorCanId,
                      SparkMax::MotorType::kBrushless),
      upSensor(ClimbConstants::kUpSensorDio),
      downSensor(ClimbConstants::kDownSensorDio), m_regulator(&m_climbingMotor, &m_climbingClosedLoopController, 0.2)
{

  // FIX: replace drivingConfig with the correct climbing config
  m_climbingMotor.Configure(Configs::MAXSwerveModule::ClimbConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);

  m_regulator.SetLimits(5, 0);
  m_regulator.SetRatio(60);
}

void ClimbSubsystem::Spool()
{
  printf("Spooling\n");
  m_regulator.SetReference(
      1);
}

void ClimbSubsystem::Unspool()
{
  printf("Unspooling\n");
  m_regulator.SetReference(-1);
}
void ClimbSubsystem::Periodic()
{
  m_regulator.Periodic();
}

bool ClimbSubsystem::TestSensorUp() { return upSensor.Get(); }
bool ClimbSubsystem::TestSensorDown() { return downSensor.Get(); }

void ClimbSubsystem::Stop()
{
  m_regulator.SetReference((double)0);
}