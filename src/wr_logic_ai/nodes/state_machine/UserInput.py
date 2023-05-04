#!/usr/bin/env python

import rospy
from wr_logic_ai.srv import EmptySrv, EmptySrvResponse

def wait_for_user_input(req):
    print("Service called")
    while True:
        if input("Enter c to continue:") == "c":
            # shutdown_user_input_server()
            return EmptySrvResponse()


def wait_for_user_input_server():
    rospy.init_node("user_input_server")
    s = rospy.Service('user_input_service', EmptySrv, wait_for_user_input)
    print("wait for user input server initialized")
    rospy.spin()

def shutdown_user_input_server():
    rospy.signal_shutdown("Ended user signal")
    
if __name__ == "__main__":
    wait_for_user_input_server()