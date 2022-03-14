#! /usr/bin/env python3
import rospkg
import rospy
from std_srvs.srv import Empty, EmptyRequest # you import the service message python classes generated from Empty.srv.

rospy.init_node('service_move_bot_in_circle_client') # Initialise a ROS node with the name service_client
rospy.wait_for_service('/move_bot_in_circle') # Wait for the service client /move_bot_in_circle to be running
move_bot_in_circle_service_client = rospy.ServiceProxy('/move_bot_in_circle', Empty) # Create the connection to the service
move_bot_in_circle_request_object = EmptyRequest() # Create an object of type EmptyRequest

result = move_bot_in_circle_service_client(move_bot_in_circle_request_object) # Send through the connection the path to the trajectory file to be executed
print(result) # Print the result given by the service called
