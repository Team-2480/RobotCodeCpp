#include <subsystems/ShooterSubsystem.h>

frc2::CommandPtr ShooterSubsystem::Rev()
{

  return frc2::CommandPtr(frc2::InstantCommand([=]
                                               { m_topClosedLoopController.SetReference(
                                                     (double)0.2, SparkMax::ControlType::kDutyCycle); }));
}
frc2::CommandPtr ShooterSubsystem::Shoot()
{

  return frc2::CommandPtr(frc2::InstantCommand([=]
                                               {
                                                       m_bottomClosedLoopController.SetReference(
                                                           (double)0.9, SparkMax::ControlType::kDutyCycle);
                                                       m_topClosedLoopController.SetReference(
                                                                                  (double)-0.2, SparkMax::ControlType::kDutyCycle); }));
}
frc2::CommandPtr  ShooterSubsystem::Stop()
{
  return frc2::CommandPtr(frc2::InstantCommand([=]
                                               {
                                                       printf("bottom stopped\n");
  m_topClosedLoopController.SetReference((double)0,
                                         SparkMax::ControlType::kDutyCycle);

  m_bottomClosedLoopController.SetReference((double)0,
                                            SparkMax::ControlType::kDutyCycle); }));
}
