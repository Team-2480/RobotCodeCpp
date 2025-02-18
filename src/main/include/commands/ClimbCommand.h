#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ClimbSubsystem.h"

/**
 * An example command that uses an example subsystem.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimbCommand : public frc2::CommandHelper<frc2::Command, ClimbCommand> {
public:
  /**
   * Creates a new Climb Command.
   *
   * @param subsystem The climb subsystem which is used by this command.
   */
  explicit ClimbCommand(ClimbSubsystem *subsystem);

  // void execute () override;

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

private:
  ClimbSubsystem *m_subsystem;
};
