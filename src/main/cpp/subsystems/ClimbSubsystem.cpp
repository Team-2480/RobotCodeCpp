#include "subsystems/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"
#include "rev/SparkMax.h"

ClimbSubsystem::ClimbSubsystem()
    : m_climbingMotor(ClimbConstants::kClimbMotorCanId,
                      SparkMax::MotorType::kBrushless),
      m_compressor(ClimbConstants::kPneumaticCanId,
                   frc::PneumaticsModuleType::CTREPCM),
      upSensor(ClimbConstants::kUpSensorDio),
      downSensor(ClimbConstants::kDownSensorDio) {

  // FIX: replace drivingConfig with the correct climbing config
  m_climbingMotor.Configure(Configs::MAXSwerveModule::DrivingConfig(),
                            SparkBase::ResetMode::kResetSafeParameters,
                            SparkBase::PersistMode::kPersistParameters);

  // zero the controller
  m_climbingEncoder.SetPosition(0);

  m_unspool_command = new frc2::InstantCommand([=]() {
    stage = STAGE_GOING_UP;
    // The solenoid will attempt to go up immediatly but be reigned in by the
    // rope
    solenoid.Toggle();
    m_climbingClosedLoopController.SetReference(
        -ClimbConstants::kSpoolSpeed, SparkMax::ControlType::kVelocity);
  });

  m_spool_command = new frc2::InstantCommand([=]() {
    stage = STAGE_GOING_DOWN;
    // on an acidental down command the spool with become rough but it shouldnt matter
    // it'll sort itself out when going back up
    solenoid.Toggle();
    m_climbingClosedLoopController.SetReference(
        (double)ClimbConstants::kSpoolSpeed, SparkMax::ControlType::kVelocity);
  });
  m_null_command = new frc2::InstantCommand(
      [=]() { printf("Someone's a little trigger happy...\n"); });
}

frc2::Command *ClimbSubsystem::Trigger() {
  switch (stage) {
  case STAGE_UP:
    return Spool();
    break;
  case STAGE_DOWN:
    return Unspool();
    break;
  default:
    return m_null_command;
    break;
  }
}

frc2::Command *ClimbSubsystem::Spool() { return m_spool_command; }

frc2::Command *ClimbSubsystem::Unspool() { return m_unspool_command; }
void ClimbSubsystem::Periodic() {
  if (TestSensorUp() && stage == STAGE_GOING_UP) {
    Stop();
    stage = STAGE_UP;
  }
  if (TestSensorDown() && stage == STAGE_GOING_DOWN) {
    Stop();
    stage = STAGE_DOWN;
  }
}

bool ClimbSubsystem::TestSensorUp() { return upSensor.Get(); }
bool ClimbSubsystem::TestSensorDown() { return downSensor.Get(); }

void ClimbSubsystem::Stop() {
  m_climbingClosedLoopController.SetReference((double)0,
                                              SparkMax::ControlType::kVelocity);
}