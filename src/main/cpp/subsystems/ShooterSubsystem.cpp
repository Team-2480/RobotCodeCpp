#include <subsystems/ShooterSubsystem.h>

void ShooterSubsystem::Rev()
{
  m_topClosedLoopController.SetReference((double)0.1,
                                         SparkMax::ControlType::kVelocity);
}
frc2::Command *ShooterSubsystem::Shoot()
{
  return shootCmd;
}
void ShooterSubsystem::Stop()
{
  printf("bottom stopped\n");
  m_topClosedLoopController.SetReference((double)0,
                                         SparkMax::ControlType::kVelocity);

  m_bottomClosedLoopController.SetReference((double)0,
                                            SparkMax::ControlType::kVelocity);
}
