# WPIlib lingo decoded quick refrence
## Acrynyms
- WPI: Worchester Polytechnic Institute
- HID: Human Interface device
- HAL: Hardware Abstraction Layer
- OI: Operator interface
- Rps: rotations per second
## FRC lingo
- Deadband: The deadband is a range which input is ignored from the human operators to prevent movment from environmental pertubations. (see edu.wpi.first.math.MathUtil.applyDeadband documentation)
## Funny Prefixes
> Some parts of the codebase use simple hungarian notation, mainly:
- m_ : please use this as a prefix for member values
- k : possibly stands for "konstans" this is a hungarian notation relic to indicate constants. Is a substitute for using all caps

# WPIlib essentials
Main.java simply passes in the robot class to the startRobot() method which does everything. Editing main will probably cause a lot of side effects. So don't.

Robot.java contains algorithms to be run at specific time intervals basically empty. RobotContainer.java contains everything else and has the bulk of the robots logic.