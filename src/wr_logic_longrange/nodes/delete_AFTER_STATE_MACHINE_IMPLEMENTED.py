    
from std_msgs.msg import Float64
import actionlib
import rospy
import testing_rviz
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import (
    NavigationState,
    LongRangeAction,
    LongRangeGoal,
    LongRangeActionResult,
)
if __name__ == '__main__':
    rospy.init_node('do_dishes_client')
    client = actionlib.SimpleActionClient(
            "LongRangeActionServer", LongRangeAction
        )
    client.wait_for_server()
    goal = LongRangeGoal(target_lat=CoordinateManager.get_coordinate()[
                            "lat"], target_long=CoordinateManager.get_coordinate()["long"])
    
    client.send_goal(goal)
