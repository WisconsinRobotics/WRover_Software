#!/usr/bin/env python3

import rospy
import actionlib
from wr_logic_ai.msg import LongRangeAction, LongRangeGoal
import obstacle_avoidance

# TODO: check timeout time length validity
LONG_RANGE_TIMEOUT_TIME = rospy.Duration(1000)


class LongRangeActionServer(object):
    def __init__(self, name) -> None:
        print("initing long range action server")
        self._action_name = name
        obstacle_avoidance.initialize()
        self._as = actionlib.SimpleActionServer(
            self._action_name, LongRangeAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        

    def execute_callback(self, goal: LongRangeGoal):
        start_time = rospy.get_rostime()
        rospy.loginfo("STTUFFF")
        while rospy.get_rostime() - start_time < LONG_RANGE_TIMEOUT_TIME and not rospy.is_shutdown():
            if obstacle_avoidance.update_target(goal.target_lat, goal.target_long):
                return self._as.set_succeeded()
        return self._as.set_aborted()


if __name__ == "__main__":
    rospy.init_node("long_range_action_server")
    server = LongRangeActionServer("LongRangeActionServer")
    rospy.spin()
