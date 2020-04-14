"""mavic2ros controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import InertialUnit


import rospy
import math
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16


import os

def _get_vector3(roll, pitch,heading):
        vec = Vector3()
        vec.x = roll
        vec.y = pitch
        vec.z = heading
        return vec
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timeStep = int(robot.getBasicTimeStep())
rospy.init_node('python_webots_controller', anonymous=True) # node is called 'python_webots_controller'
rospy.loginfo("Loading Webots Controller")
pub = rospy.Publisher('imu_values_topic', Vector3, queue_size=10)
depth_pub = rospy.Publisher('depth_topic', Int16, queue_size=10)
# Get and enable devices.

IMUsensor = robot.getInertialUnit('inertial unit')  # front central proximity sensor
IMUsensor.enable(timeStep)

GPSsensor = robot.getGPS('gps')
GPSsensor.enable(timeStep)

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timeStep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    roll, pitch, heading = IMUsensor.getRollPitchYaw() 
    
    blah, height, blah2 = GPSsensor.getValues()
    pi = 3.14159265359
    roll=(roll+pi/2)%180
    
    radoeff = 180.0/pi
    # Process sensor data here.
    rospy.loginfo("Sending Simulated IMU Data. Roll: "+str(round(roll*radoeff))+" Pitch: "+str(round(pitch*radoeff))+" Heading: "+str(round(heading*radoeff)))
    pub.publish(_get_vector3(roll*radoeff,pitch*radoeff,heading*radoeff))

    rospy.loginfo("Sending Simulated Depth Data. Depth: "+str(round(roll)))
    depth_msg = Int16()
    depth_msg.data = height
    depth_pub.publish(depth_msg)
    
    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
