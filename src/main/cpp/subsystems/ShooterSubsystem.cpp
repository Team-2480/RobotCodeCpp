#include <subsystems/ShooterSubsystem.h>

frc2::Command *ShooterSubsystem::Rev()
{
  return reverseCmd;
}
frc2::Command *ShooterSubsystem::Shoot()
{
  return shootCmd;
}
void ShooterSubsystem::Stop()
{
  printf("bottom stopped\n");
  m_topClosedLoopController.SetReference((double)0,
                                         SparkMax::ControlType::kDutyCycle);

  m_bottomClosedLoopController.SetReference((double)0,
                                            SparkMax::ControlType::kDutyCycle);
}
