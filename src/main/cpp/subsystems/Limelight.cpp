
#include "subsystems/Limelight.h"
#include "networktables/NetworkTableInstance.h"

Limelight::Limelight() {
  limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}
double Limelight::getTx() { return limelightTable->GetNumber("tx", 0.0); }

/*
 * return true if a april tag is detected
 */
bool Limelight::hasTarget() {
  return limelightTable->GetNumber("tv", 0.0) == 1.0;
}
