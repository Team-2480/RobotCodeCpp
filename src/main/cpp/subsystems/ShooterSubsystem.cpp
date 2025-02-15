#include <subsystems/ShooterSubsystem.h>

void ShooterSubsystem::Rev() {
  m_topClosedLoopController.SetReference((double)0.1,
                                         SparkMax::ControlType::kVelocity);
}
frc2::Command* ShooterSubsystem::Shoot() {
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand([=]() {
        m_topClosedLoopController.SetReference(
            (double)-1, SparkMax::ControlType::kVelocity);
        m_topClosedLoopController.SetReference(
            (double)1, SparkMax::ControlType::kVelocity);
      }),
      frc2::WaitCommand(1_s),
      frc2::InstantCommand([=]() { Stop(); }));
}
void ShooterSubsystem::Stop() {
  m_topClosedLoopController.SetReference((double)0,
                                         SparkMax::ControlType::kVelocity);

  m_bottomClosedLoopController.SetReference((double)0,
                                            SparkMax::ControlType::kVelocity);
}