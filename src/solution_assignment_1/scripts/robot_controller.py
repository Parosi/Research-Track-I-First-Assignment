#!/usr/bin/env python

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
This script implements in its main function, control(), one
subscriber to the /odom topic to aquire the robot position.
In the callback, odom_callback, of this subscriber are used a 
client (to obtain a new taget if the previous has been reached) 
to service /random_pos_2d and a publisher on /cmd_vel topic to 
send the commands for the robot.
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from solution_assignment_1.srv import RandomPos2D

# Create the publisher on topic cmd_vel
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
# Create the client for random_pos service
client = rospy.ServiceProxy('/random_pos_2d', RandomPos2D)

# Initialize the two variables that contain target position
target_x = 0
target_y = 0
# Initialize the constant multiplicator of linear velocity
k = 1

# Callback function called when there is a new message on odom topic
def odom_callback(odometry):
	
	global target_x, target_y
	# Check if robot has reached the target
	if (abs(target_x - odometry.pose.pose.position.x) < 0.1 and
		 abs(target_y - odometry.pose.pose.position.y) < 0.1):
		# Print on the terminal
		rospy.loginfo('TARGET REACHED! Requiring a new target!')
		# Call service for a new target
		response = client()
		# Store coordinates of new target
		target_x = response.random_x
		target_y = response.random_y
		# Print on the terminal
		rospy.loginfo('New target: x = %f y = %f', target_x, target_y)
	
	# Initialize the message to control the robot
	vel = Twist()
	# Calculate the linear velocities and store them
	vel.linear.x = k*(target_x - odometry.pose.pose.position.x)
	vel.linear.y = k*(target_y - odometry.pose.pose.position.y)
	# Publish the robot speed to cmd_vel topic
	vel_pub.publish(vel)


# Main function of the node
def control():
	
	# Initialize the node
	rospy.init_node('robot_controller', anonymous = True)
	# Subscribe to odom topic
	rospy.Subscriber('/odom', Odometry, odom_callback);
	# Check continously if there are messages from odom topic
	rospy.spin()


# Be sure that service function executes only if we execute
# the script and not when we import it
if __name__ == '__main__':
	try:
		control()
	except rospy.ROSInterruptException:
		pass
