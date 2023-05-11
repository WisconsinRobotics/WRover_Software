#!/usr/bin/env python

from statemachine import StateMachine, State
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import TargetMsg
from wr_logic_ai.srv import EmptySrv
import rospy
import actionlib
from wr_logic_ai.msg import LongRangeAction, LongRangeGoal
from actionlib_msgs.msg import GoalStatus

class NavStateMachine(StateMachine):
    # Defining states
    stInit = State(initial=True)
    stLongRange = State()
    stLongRangeRecovery = State()
    stShortRange = State()
    stWaypointSuccess = State()
    stComplete = State()

    # Defining events and transitions
    evSuccess = (stLongRange.to(stShortRange) | stLongRangeRecovery.to(
        stLongRange) | stShortRange.to(stWaypointSuccess))
    evError = (stLongRange.to(stLongRangeRecovery) | stLongRangeRecovery.to(
        stLongRangeRecovery) | stShortRange.to(stLongRange))
    evNotWaiting = stWaypointSuccess.to(stLongRange)
    evUnconditional = stInit.to(stLongRange)
    evComplete = stWaypointSuccess.to(stComplete)

    def __init__(self, mgr: CoordinateManager) -> None:
        self._mgr = mgr
        super(NavStateMachine, self).__init__()
        self.currentEvent = -1

    def on_enter_stInit(self) -> None:
        print("\non enter stInit")
        self._mgr.read_coordinates_file()
        self.evUnconditional()

    def on_exit_stInit(self) -> None:
        if (self._mgr.next_line()):
            self.evComplete

    def on_enter_stLongRange(self) -> None:
        print("\non enter stLongRange")
        # print("Running Timer")
        # self.pub_nav = rospy.Publisher('/target_coord', TargetMsg, queue_size=1)
        # self.timer = rospy.Timer(rospy.Duration(0.2), self.publish)
        
        client = actionlib.SimpleActionClient("LongRangeActionServer", LongRangeAction)
        client.wait_for_server()
        goal = LongRangeGoal(target_lat = self._mgr.get_coordinate()["lat"], target_long = self._mgr.get_coordinate()["long"])
        client.send_goal(goal)
        client.wait_for_result()
        as_state = client.get_state()
        if as_state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        elif as_state == GoalStatus.ABORTED:
            self.evError()

    def on_exit_stLongRange(self) -> None:
        print("Exting Long Range")

    def on_enter_stLongRangeRecovery(self) -> None:

        print("\non enter stLongRangeRecovery")
        if self._mgr is None:
            raise ValueError
        else:
            self._mgr.previous_line()
            print(self._mgr.get_coordinate())            
            client = actionlib.SimpleActionClient("LongRangeActionServer", LongRangeAction)
            client.wait_for_server()
            goal = LongRangeGoal(target_lat = self._mgr.get_coordinate()["lat"], target_long = self._mgr.get_coordinate()["long"])
            client.send_goal(goal)
            client.wait_for_result()
            as_state = client.get_state()
            if as_state == GoalStatus.SUCCEEDED:
                self.evSuccess()
                self._mgr.next_line()
            elif as_state == GoalStatus.ABORTED:
                self.evError()

    def on_exit_stLongRangeRecovery(self) -> None:
        pass

    def on_enter_stShortRange(self) -> None:
        print("\non enter stShortRange")
        if CoordinateManager.short_range_complete() != True:
            print("Short Range Not Complete")
            pass
        else:
            print("Short Range Complete")
            self.evSuccess()

    def on_exit_stShortRange(self) -> None:
        pass

    def on_enter_stWaypointSuccess(self) -> None:
        print("\non enter stWaypointSuccess")
        if self._mgr is None:
            raise ValueError
        else:
            rospy.wait_for_service('wait_for_user_input_service')
            try:
                wait_for_user_input = rospy.ServiceProxy('wait_for_user_input_service', EmptySrv)
                wait_for_user_input()
            except rospy.ServiceException as e:
                print(e)
            if (self._mgr.next_line()):
                print("Should Enter event complete")
                self.evComplete()
            else:
                self.evNotWaiting()

    def on_exit_stWaypointSuccess(self) -> None:
        pass  # Frash lights that we are goint to next waypoint

    def on_enter_stComplete(self) -> None:
        print("We finished, wooooo")

    def publish(self, timer):
        # Publish to obstacle avoidance with the target coordinates
        target_coords = TargetMsg()
        target_coords.target_lat = self._mgr.get_coordinate()['lat']
        target_coords.target_long = self._mgr.get_coordinate()['long']
        target_coords.target_type = self._mgr.get_coordinate()['target_type']
        self.pub_nav.publish(target_coords)
        # TODO: Publish to topic /navigation_state

if __name__ == "__main__":
    rospy.init_node('nav_state_machine', anonymous=False)
    statemachine = NavStateMachine(CoordinateManager())
    rospy.spin()
