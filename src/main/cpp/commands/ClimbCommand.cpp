
#include "commands/ClimbCommand.h"

ClimbCommand::ClimbCommand(ClimbSubsystem *subsystem) : m_subsystem{subsystem} {
  // Register that this command requires the subsystem.
  // Prevents two commands using the same subsystemto be run simultaneously
  AddRequirements(m_subsystem);
}

void ClimbCommand::Execute(){
// Climbing recipie here 
}
bool ClimbCommand::IsFinished(){
  return false; // Keep moving until the command is interrupted or stopped
}

void ClimbCommand::End(bool interrupted){
  // Probably logic to lock motor in place,
  // can also disable robot here as is last action taken during a match.
}

