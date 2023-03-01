# Preliminary report

## Task 1

- `create_oi.py`: Constants for the Create2 Open Interface
- `create_robot.py`: Class for Create2 Robot, with functions wrapping common tasks
- `create_serial.py`: Class for serial connections to the Create2 Robot, with wrappers for serial communication with the robot
- `packets.py`: Decodes sensor packet data into named tuples for ease of use and code readability
- `repeat_timer.py`: Periodic timer running in a thread

## Task 2

- What changed between these two files?

  On line 248, the robot is being initialized using the createlib Create2 object. The key callbacks utilize the wrapper functions from the Create2 object when possible.

- Moving forward, how would you use this new code to add program functionality? (To make this easier, ask yourself what would it take to recreate the last lab?)

  Can use the new wrapper functions to more simply send/receive data. Can use the new enumerations/constants to write more readable code, since semantics are now coded in.

- Are there any features from the original file that didn't make it when the migration happened? Do you think these features should be incorporated?

  idk man
