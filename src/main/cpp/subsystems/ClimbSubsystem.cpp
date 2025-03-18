#include "subsystems/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"
#include "rev/SparkMax.h"

ClimbSubsystem::ClimbSubsystem()
    : m_climbingMotor(ClimbConstants::kClimbMotorCanId,
                      SparkMax::MotorType::kBrushless),
      upSensor(ClimbConstants::kUpSensorDio),
      downSensor(ClimbConstants::kDownSensorDio), m_regulator(&m_climbingMotor, &m_climbingClosedLoopController, 0.5)
{

  // FIX: replace drivingConfig with the correct climbing config
  m_climbingMotor.Configure(Configs::MAXSwerveModule::ClimbConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);

  m_regulator.SetTargets(4.5, -1);
  m_regulator.SetRatio(60);
}

void ClimbSubsystem::Spool()
{
  printf("Spooling\n");
  m_regulator.Up();
}

void ClimbSubsystem::Unspool()
{
  printf("Unspooling\n");
  m_regulator.Down();
}
void ClimbSubsystem::Periodic()
{
}

bool ClimbSubsystem::TestSensorUp() { return upSensor.Get(); }
bool ClimbSubsystem::TestSensorDown() { return downSensor.Get(); }

void ClimbSubsystem::Stop()
{
  m_regulator.Pause();
}