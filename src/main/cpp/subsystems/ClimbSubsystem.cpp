#include "subsystems/ClimbSubsystem.h"

#include <frc/geometry/Rotation2d.h>

#include "Configs.h"
#include "rev/SparkMax.h"

ClimbSubsystem::ClimbSubsystem()
    : m_climbingMotor(ClimbConstants::kClimbMotorCanId, SparkMax::MotorType::kBrushless),
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
}

void ClimbSubsystem::Spool() {
  m_climbingClosedLoopController.SetReference((double)1,
                                              SparkMax::ControlType::kVelocity);
}

void ClimbSubsystem::Unspool() {
  m_climbingClosedLoopController.SetReference((double)-1,
                                              SparkMax::ControlType::kVelocity);
}

void ClimbSubsystem::PneumaticUp() {}
void ClimbSubsystem::PneumaticDown() {}

bool ClimbSubsystem::TestSensorUp() {return upSensor.Get();}
bool ClimbSubsystem::TestSensorDown() {return downSensor.Get();}

void ClimbSubsystem::ResetEncoders() { m_climbingEncoder.SetPosition(0); }
