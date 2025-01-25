# WPIlib lingo decoded quick refrence

## Acroynyms

- WPI: Worchester Polytechnic Institute
- HID: Human Interface device
- HAL: Hardware Abstraction Layer
- OI: Operator interface
- Rps: rotations per second
- Dio: Digital I/O (discrete data)
- Pwm: pulse width modulation (voltage changes in discrete output)

## FRC lingo

- Deadband: The deadband is a range which input is ignored from the human operators to prevent movment from environmental pertubations. (see edu.wpi.first.math.MathUtil.applyDeadband documentation)

## Funny Prefixes

> Some parts of the codebase use simple hungarian notation, mainly:
- m_ : please use this as a prefix for member values
- k : possibly stands for "konstans" this is a hungarian notation relic to indicate constants. Is a substitute for using all caps

## WPIlib essentials

main() is defined in robot. It is recommended to avoid modifying the entry point.

Robot.cpp contains algorithms to be run at specific time intervals. RobotContainer.cpp contains bulk of robot code. Used for creating a command based robot.
