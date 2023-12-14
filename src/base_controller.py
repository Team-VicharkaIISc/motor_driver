#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

#import a custom message type
from motor_driver.msg import Teensy

wheel_radius = 0.05 #TODO: replace with your measurement of the wheel radius in meters
vehical_width = 0.5 #TODO: replace with your measurement of the vehical width in meters

#define state of the robot: -1: INVALID_STATE, 0: STOP, 1: FORWARD, 2: BACKWARD, 3: TURN_LEFT, 4: TURN_RIGHT
rover_state = 0

linear_velocity = 0
angular_velocity = 0

required_angular_left_velocity = 0
required_angular_right_velocity = 0

actual_angular_left_velocity = 0
actual_angular_right_velocity = 0

actual_linear_velocity = 0
actual_angular_velocity=0


#Define a PID controller for the left and right wheels
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.error = 0
        self.integral = 0
        self.prev_error = 0
        self.setpoint = 0  # Default setpoint

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def update(self, process_variable):
        self.error = self.setpoint - process_variable
        self.integral += self.error
        derivative = self.error - self.prev_error
        output = self.kp * self.error + self.ki * self.integral + self.kd * derivative
        self.prev_error = self.error
        return output

def cmd_vel_callback(msg):
    # This function will be called every time a message is received on the "cmd_vel" topic
    # Process the data in the Twist message (msg) and send commands to the Teensy motor driver

    # Example: Extract linear and angular velocities from Twist message
    linear_velocity = msg.linear.x
    angular_velocity = msg.angular.z
    global rover_state
    if linear_velocity == 0 and angular_velocity == 0:
        rover_state = 0
    elif linear_velocity > 0 and angular_velocity == 0:
        rover_state = 1
    elif linear_velocity < 0 and angular_velocity == 0:
        rover_state = 2
    elif linear_velocity == 0 and angular_velocity > 0:
        rover_state = 3
    elif linear_velocity == 0 and angular_velocity < 0:
        rover_state = 4
    else:
        rover_state = -1
    global required_angular_left_velocity
    global required_angular_right_velocity

    if rover_state!=-1:
        if rover_state==1 or rover_state==2:
            required_angular_left_velocity = linear_velocity/wheel_radius
            required_angular_right_velocity = linear_velocity/wheel_radius
        else:
            #Perform differential drive
            required_angular_left_velocity = (-1*angular_velocity*vehical_width)/(2*wheel_radius)
            required_angular_right_velocity = (angular_velocity*vehical_width)/(2*wheel_radius)
    else:
        rospy.loginfo("INVALID_STATE: Cannot handle non-zero linear and angular velocities at the same time | ROVER STOPPED")
        required_angular_left_velocity = 0
        required_angular_right_velocity = 0


    # Example: Process the data and send commands to the Teensy motor driver
    # Replace the following lines with your motor control logic
    # motor_command = process_data(linear_velocity, angular_velocity)
    # send_to_teensy(motor_command)

def teensy_feedback_callback(msg):
    # This function will be called every time a message is received on the "teensy_feedback" topic
    # Process the data in the Twist message (msg) and send commands to the Teensy motor driver
    global actual_angular_left_velocity
    global actual_angular_right_velocity

    # Example: Extract linear and angular velocities from Twist message
    actual_angular_left_velocity = msg.actualAngularLeft
    actual_angular_right_velocity = msg.actualAngularLeft

    # Example: Process the data and send commands to the Teensy motor driver
    # Replace the following lines with your motor control logic
    # motor_command = process_data(linear_velocity, angular_velocity)
    # send_to_teensy(motor_command)

def cmd_vel_listener():
    # Initialize the ROS node
    rospy.init_node('cmd_vel_listener', anonymous=True)

    # Subscribe to the "cmd_vel" topic with the cmd_vel_callback function
    rospy.Subscriber('cmd_vel', Twist, cmd_vel_callback)
    rospy.Publisher('teensy_vel', Twist, queue_size=10)
    rospy.Subscriber('teensy_feedback', Teensy, teensy_feedback_callback)
    # Spin to keep the script from exiting
    left_pid=PIDController(0.3,0.1,0.2)
    right_pid=PIDController(0.3,0.1,0.2)
    teensy_cmd=Teensy()
    while not rospy.is_shutdown():
        left_pid.set_setpoint(required_angular_left_velocity)
        right_pid.set_setpoint(required_angular_right_velocity)
        left_angular_command=actual_angular_left_velocity+left_pid.update(actual_angular_left_velocity)
        right_angular_command=actual_angular_right_velocity+right_pid.update(actual_angular_right_velocity)
        rospy.loginfo("Left Motor Command: %f",left_angular_command)
        rospy.loginfo("Right Motor Command: %f",right_angular_command)   
        teensy_cmd.actualAngularLeft=left_angular_command
        teensy_cmd.actualAngularRight=right_angular_command
        pub = rospy.Publisher('teensy_vel', Teensy, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    # Run the listener node
    cmd_vel_listener()
