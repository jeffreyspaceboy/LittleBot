# This is a message that holds data to describe the state of a motor.
#
# The state of each motor is defined by:
#  * The name of the motor
#  * The current pwm power driving the motor
#  * The current rotations per minute of the motor
#
# Each motor is uniquely identified by its name
# The header specifies the time at which the motor states were recorded. All the motor states
# in one message have to be recorded at the same time.
#
# This message consists of a multiple arrays, one for each part of the motor state.
# The goal is to make each of the fields optional. When e.g. your motors aren't being used, 
# you can leave the effort array empty.
#
# All arrays in this message should have the same size, or be empty.
# This is the only way to uniquely associate the motor name with the correct
# states.

std_msgs/Header header

string name
int32 pwm_power
float32 rpm

