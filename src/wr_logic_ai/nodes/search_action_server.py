import rospy
import actionlib
from wr_logic_ai.msg import SearchStateAction, SearchStateGoal
import obstacle_avoidance
import travel_timer

'''
@ingroup wr_logic_ai
@defgroup wr_logic_ai Search Action Server
@brief Tells the rover travel to the target coordinates and calls obstacle avoidance to make sure the rover can actually reach these coordinates.
@details Enters a while loop that continually calls obstacle avoidance to update the target to the next coordinate in the list. 
For each coordinate, if the time between the starting time and the current time go above the estimated travel time for that
specific coordinate, the coordinate is assumed to be unreachable, and the rover will skip it and move on to the next coordinate.
'''

class SearchActionServer(object):
    def __init__(self, name) -> None:
        """
        Initializes the action server

        @param name (String): Name of the action server
        """

        self._action_name = name
        obstacle_avoidance.initialize()
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            SearchStateAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self, goal: SearchStateGoal):
        """
        Executes the long range obstacle avoidance code, and triggers the corresponding state machine event
        depending on the result of the navigation

        @param goal (SearchGoal): Goal for the navigation segment, which contains the GPS coordinates
        of the target
        """
        start_time = rospy.get_rostime()
        while (
            rospy.get_rostime() - start_time < travel_timer.calc_point_time(goal.dist)
            and not rospy.is_shutdown()
        ):
            if obstacle_avoidance.update_target(goal.target_lat, goal.target_long):
                return self._as.set_succeeded()
        return self._as.set_aborted()

if __name__ == "__main__":
    rospy.init_node("search_action_server")
    server = SearchActionServer("SearchActionServer")
    rospy.spin()