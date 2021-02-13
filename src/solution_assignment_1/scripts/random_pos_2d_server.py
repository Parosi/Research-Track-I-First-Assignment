#!/usr/bin/env python

'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
This script implements a server for the service RandomPos2D.
In fact in its main function, service(), it runs a server. 
Latter's callback, service_callback, provides two random numbers 
in the interval [-6.0, 6.0] as response.
'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

import rospy
import random
from solution_assignment_1.srv import (RandomPos2D, RandomPos2DResponse)

# Callback function called when there is a new request
def service_callback(req):
	# Create a random number for x coordinate
	random_number = random.uniform(-6, 6)
	# Store random x coordinate
	x = random_number
	# Create a random number for y coordinate
	random_number = random.uniform(-6, 6)
	# Store random y coordinate
	y = random_number
	# Create a response
	response = RandomPos2DResponse()
	# Store coordinates in the response
	response.random_x = x
	response.random_y = y
	# Return the response
	return response

# Main function of the node
def service():
	# Initialize the node
	rospy.init_node('random_pos_2d_server', anonymous = True)
	# Create the service
	rospy.Service('random_pos_2d', RandomPos2D, service_callback)
	# Check continously if there are requests
	rospy.spin()


# Be sure that service function executes only if we execute
# the script and not when we import it
if __name__ == '__main__':
	try:
		service()
	except rospy.ROSInterruptException:
		pass
