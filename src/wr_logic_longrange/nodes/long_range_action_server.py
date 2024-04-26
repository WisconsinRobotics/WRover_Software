#!/usr/bin/env python3

"""@file
@defgroup wr_logic_longrange
@{
@defgroup wr_logic_longrange_action_server Long Range Action Server
@brief Action server for long range navigation
@details This action server is used in conjunction with the state machine to drive the long range 
navigation logics. The action server determines if a navigation is successful (when it reaches the 
destination coordinate), or if it failed (when a timeout is triggered), and triggers a state machine 
event accordingly. 
@{
"""

import rospy
import actionlib
from wr_logic_longrange.msg import LongRangeAction, LongRangeGoal
import obstacle_avoidance

# TODO: check timeout time length validity
## Timeout time for when we declare a long range navigation as failed
LONG_RANGE_TIMEOUT_TIME = rospy.Duration(1000)


class LongRangeActionServer(object):
    """
    Class for long range navigation's action server

    @param object (_type_): Unused
    """

    def __init__(self, name) -> None:
        rospy.loginfo("initing long range action server")
        self._action_name = name
        obstacle_avoidance.initialize()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            LongRangeAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()
        
        

    def execute_callback(self, goal: LongRangeGoal):
        """
        Executes the long range obstacle avoidance code, and triggers the corresponding state machine event
        depending on the result of the navigation

        @param goal (LongRangeGoal): Goal for the navigation segment, which contains the GPS coordinates
        of the target
        """

        obstacle_avoidance.update_target(goal.target_lat, goal.target_long)
        obstacle_avoidance.set_is_active(True)
        start_time = rospy.get_rostime()
        while rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME and not rospy.is_shutdown():
            rospy.Rate(10).sleep()
            if obstacle_avoidance.run_navigation():
                obstacle_avoidance.set_is_active(False)
                return self._as.set_succeeded()
        obstacle_avoidance.set_is_active(False)
        return self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node("long_range_action_server")
    server = LongRangeActionServer("LongRangeActionServer")
    rospy.spin()
    
    
    