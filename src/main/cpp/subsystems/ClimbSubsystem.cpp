#include "subsystems/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"
#include "rev/SparkMax.h"

ClimbSubsystem::ClimbSubsystem()
    : m_climbingMotor(ClimbConstants::kClimbMotorCanId,
                      SparkMax::MotorType::kBrushless),
      upSensor(ClimbConstants::kUpSensorDio),
      downSensor(ClimbConstants::kDownSensorDio)
{

  // FIX: replace drivingConfig with the correct climbing config
  m_climbingMotor.Configure(Configs::MAXSwerveModule::DirectConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);

  // zero the controller
  m_climbingEncoder.SetPosition(0);

  m_trigger_command = Trigger();
}

frc2::Command *ClimbSubsystem::Trigger()
{
  return new frc2::InstantCommand([=]()
                                  {
printf("Triggering with stage %i\n", stage);
  switch (stage)
  {
  case STAGE_UP:
     Spool();
    break;
  case STAGE_DOWN:
     Unspool();
    break;
  default:
    printf("Someone's a little trigger happy...\n");
    break;
  } });
}

void ClimbSubsystem::Spool()
{
  printf("Spooling\n");
  stage = STAGE_GOING_DOWN;
  // on an acidental down command the spool with become rough but it shouldnt matter
  // it'll sort itself out when going back up
  m_climbingClosedLoopController.SetReference(
      (double)ClimbConstants::kSpoolSpeed, SparkMax::ControlType::kVelocity);
}

void ClimbSubsystem::Unspool()
{
  printf("Unspooling\n");
  stage = STAGE_GOING_UP;
  // The solenoid will attempt to go up immediatly but be reigned in by the
  // rope
  m_climbingClosedLoopController.SetReference(
      -ClimbConstants::kSpoolSpeed, SparkMax::ControlType::kVelocity);
}
void ClimbSubsystem::Periodic()
{
  // printf("Sensor Up: %i, Sensor Down: %i, Stage: %i\n", !TestSensorUp(), !TestSensorDown(), stage);
  // if (!TestSensorUp() && stage == STAGE_GOING_UP)
  // {
  //   printf("Finished up\n");
  //   Stop();
  //   stage = STAGE_UP;
  // }
  // if (!TestSensorDown() && stage == STAGE_GOING_DOWN)
  // {
  //   printf("Finished down\n");
  //   Stop();
  //   stage = STAGE_DOWN;
  // }
}

bool ClimbSubsystem::TestSensorUp() { return upSensor.Get(); }
bool ClimbSubsystem::TestSensorDown() { return downSensor.Get(); }

void ClimbSubsystem::Stop()
{
  m_climbingClosedLoopController.SetReference((double)0,
                                              SparkMax::ControlType::kVelocity);
}