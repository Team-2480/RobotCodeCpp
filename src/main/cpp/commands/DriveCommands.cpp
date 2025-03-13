#include "frc2/command/Commands.h"
#include "subsystems/DriveSubsystem.h"

frc2::CommandPtr Flip180(DriveSubsystem *drive) {
  auto targetAngle = drive->GetHeading() + 180_deg;
  // Simple proportional control for rotation
  return frc2::cmd::Run(
             [&] {
               auto error = targetAngle - drive->GetHeading();
               units::radians_per_second_t rotationSpeed =
                   error * 0.01; // Adjust gain as needed

               // Drive while rotating
               drive->Drive(0_mps, 0_mps, rotationSpeed, false);
             },
             {drive})
      .Until([&] {
        return units::math::abs(drive->GetHeading() -
                                (drive->GetHeading() + 180_deg)) < 2_deg;
      });
}
