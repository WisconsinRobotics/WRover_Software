#!/usr/bin/env python

## @file
# @brief Node for state machine that drives the autonomous nagivation code
# @defgroup wr_logic_ai_state_machine State_Machine
# @ingroup wr_logic_ai
# @details The autonomous navigation code is driven by the state machine below. This node uses the statemachine python library to 
# handle the setup of the state machine, including initializing states and events, handling state transitions, executing the state 
# loop, and more. Here, we only need to define our states and events as well as behavior upon entering or leaving states. 
# @{

from __future__ import annotations
from statemachine import StateMachine, State
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import NavigationState, LongRangeAction, LongRangeGoal, LongRangeActionResult
from wr_logic_ai.msg import ShortRangeAction, ShortRangeGoal, ShortRangeActionResult
from wr_led_matrix.srv import led_matrix as LEDMatrix, led_matrixRequest as LEDMatrixRequest
from std_srvs.srv import Empty
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from wr_drive_msgs.msg import DriveTrainCmd
import threading
import time
import pdb

## LED matrix color for when the rover is navigating towards the target using autonomous navigation
COLOR_AUTONOMOUS = LEDMatrixRequest(RED=0, GREEN=0, BLUE=255)
## LED matrix color for when the rover has reached its target
COLOR_COMPLETE = LEDMatrixRequest(RED=0, GREEN=255, BLUE=0)
## LED matrix color for when the rover has encountered an error while executing autonomous navigation
COLOR_ERROR = LEDMatrixRequest(RED=255, GREEN=0, BLUE=0)
## Initial LED matrix color
COLOR_NONE = LEDMatrixRequest(RED=0, GREEN=0, BLUE=0)

def set_matrix_color(color: LEDMatrixRequest) -> None:
    """Helper function for setting the LED matrix color

    @param color The color to set the LED matrix to
    """
    matrix_srv = rospy.ServiceProxy("/led_matrix", LEDMatrix)
    matrix_srv.wait_for_service()
    matrix_srv.call(COLOR_NONE)
    time.sleep(0.5)
    matrix_srv.call(color)

class NavStateMachine(StateMachine):
    """This class implements the state machine used in autonomous navigation

    Args:
        StateMachine (StateMachine): The StateMachine class imported from the statemachine python library, required to declare 
        this class as a state machine
    """

    # Defining states
    ## Starter state of the state machine
    stInit = State(initial=True)
    ## State representing that the robot is running in long range mode
    stLongRange = State()
    ## State representing that the robot is recovering from a error state
    stLongRangeRecovery = State()
    ## State representing that the robot is running in short range mode
    stShortRange = State()
    ## State representing that the robot has completed a task at a waypoint
    stWaypointSuccess = State()
    ## State representing that the robot has completed all autonomous navigation tasks
    stComplete = State()

    # Defining events and transitions
    ## Event representing a successful task execution
    evSuccess = (stLongRange.to(stShortRange) | stLongRangeRecovery.to(
        stLongRange) | stShortRange.to(stWaypointSuccess))
    ## Event representing an error condition being raised
    evError = (stLongRange.to(stLongRangeRecovery) | stLongRangeRecovery.to(
        stLongRangeRecovery) | stShortRange.to(stLongRange))
    ## Event representing a shortcircuit state transition from WaypointSuccess to LongRange
    evNotWaiting = stWaypointSuccess.to(stLongRange)
    ## Event representing an unconditional state transition from Init to LongRange
    evUnconditional = stInit.to(stLongRange)
    ## Event representing all tasks are complete
    evComplete = stWaypointSuccess.to(stComplete)

    def __init__(self, mgr: CoordinateManager) -> None:
        """Initializes the state machine

        Args:
            mgr (CoordinateManager): Helper class for retrieving target waypoint GPS coordinates
        """
        self._mgr = mgr
        self.currentEvent = -1
        self.mux_pub = rospy.Publisher(
            "/navigation_state", NavigationState, queue_size=1)
        self.mux_long_range = NavigationState()
        self.mux_long_range.state = NavigationState.NAVIGATION_STATE_LONG_RANGE
        self.mux_short_range = NavigationState()
        self.mux_short_range.state = NavigationState.NAVIGATION_STATE_SHORT_RANGE
        super(NavStateMachine, self).__init__()

    def init_calibrate(self, pub: rospy.Publisher, stop_time: float) -> None:
        if rospy.get_time() < stop_time:
            pub.publish(DriveTrainCmd(left_value=0.3, right_value=-0.3))
        else:
            pub.publish(DriveTrainCmd(left_value=0, right_value=0))
            self.evUnconditional()

    def init_w_ros(self):

        set_matrix_color(COLOR_AUTONOMOUS)

        pub = rospy.Publisher("/control/drive_system/cmd",
                              DriveTrainCmd, queue_size=1)
        stop_time = rospy.get_time() + 7
        self._init_tmr = rospy.Timer(rospy.Duration.from_sec(
            0.1), lambda _: self.init_calibrate(pub, stop_time))

    def on_enter_stInit(self) -> None:
        print("\non enter stInit")
        rospy.loginfo("\non enter stInit")
        self._mgr.read_coordinates_file()

        threading.Timer(1, lambda: self.init_w_ros()).start()

    def on_exit_stInit(self) -> None:
        self._init_tmr.shutdown()
        if (self._mgr.next_coordinate()):
            self.evComplete()

    def _longRangeActionComplete(self, state: GoalStatus, _: LongRangeActionResult) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stLongRange(self) -> None:
        print("\non enter stLongRange")
        rospy.loginfo("\non enter stLongRange")
        self.timer = rospy.Timer(rospy.Duration(
            0.2), lambda _: self.mux_pub.publish(self.mux_long_range))

        # enter autonomous mode
        set_matrix_color(COLOR_AUTONOMOUS)

        self._client = actionlib.SimpleActionClient(
            "LongRangeActionServer", LongRangeAction)
        self._client.wait_for_server()
        goal = LongRangeGoal(target_lat=self._mgr.get_coordinate()[
                             "lat"], target_long=self._mgr.get_coordinate()["long"])
        self._client.send_goal(goal, done_cb=lambda status, result:
                               self._longRangeActionComplete(status, result))

    def on_exit_stLongRange(self) -> None:
        print("Exting Long Range")
        rospy.loginfo("Exting Long Range")
        self.timer.shutdown()

    def _longRangeRecoveryActionComplete(self, state: GoalStatus, _: LongRangeActionResult) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stLongRangeRecovery(self) -> None:
        print("\non enter stLongRangeRecovery")
        rospy.loginfo("\non enter stLongRangeRecovery")

        set_matrix_color(COLOR_ERROR)

        if self._mgr is None:
            raise ValueError
        else:
            self._mgr.previous_coordinate()
            print(self._mgr.get_coordinate())
            self.timer = rospy.Timer(rospy.Duration(
                0.2), lambda _: self.mux_pub.publish(self.mux_long_range))
            self._client = actionlib.SimpleActionClient(
                "LongRangeActionServer", LongRangeAction)
            self._client.wait_for_server()
            goal = LongRangeGoal(target_lat=self._mgr.get_coordinate()[
                                 "lat"], target_long=self._mgr.get_coordinate()["long"])
            self._client.send_goal(goal, done_cb=lambda status, result:
                                   self._longRangeRecoveryActionComplete(status, result))

    def on_exit_stLongRangeRecovery(self) -> None:
        self.timer.shutdown()

    def _shortRangeActionComplete(self, state: GoalStatus, _: ShortRangeActionResult) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stShortRange(self) -> None:
        print("\non enter stShortRange")
        rospy.loginfo("\non enter stShortRange")

        set_matrix_color(COLOR_AUTONOMOUS)
        self.timer = rospy.Timer(rospy.Duration(
            0.2), lambda _: self.mux_pub.publish(self.mux_short_range))

        self._client = actionlib.SimpleActionClient(
            "ShortRangeActionServer", ShortRangeAction)
        self._client.wait_for_server()
        TARGET_TYPE_MAPPING = [
            ShortRangeGoal.TARGET_TYPE_GPS_ONLY,  # 0 Vision markers
            ShortRangeGoal.TARGET_TYPE_SINGLE_MARKER,  # 1 Vision marker
            ShortRangeGoal.TARGET_TYPE_GATE,  # 2 Vision markers
        ]
        goal = ShortRangeGoal(
            target_type=TARGET_TYPE_MAPPING[self._mgr.get_coordinate()["num_vision_posts"]])
        self._client.send_goal(goal, done_cb=lambda status, result:
                               self._shortRangeActionComplete(status, result))

    def on_exit_stShortRange(self) -> None:
        # self.timer.shutdown()
        pass

    def _blink_complete(self) -> None:
        # Blinks the color off for 0.5 sec, the other 0.5 sec we sleep
        set_matrix_color(COLOR_COMPLETE)

    def _wait_for_user_input(self) -> None:
        rospy.wait_for_service('wait_for_user_input_service')
        try:
            wait_for_user_input = rospy.ServiceProxy(
                'wait_for_user_input_service', Empty)
            wait_for_user_input()
        except rospy.ServiceException as e:
            print(e)
        if (self._mgr.next_coordinate()):
            print("Should Enter event complete")
            self.evComplete()
        else:
            self.evNotWaiting()

    def on_enter_stWaypointSuccess(self) -> None:
        print("\non enter stWaypointSuccess")

        self._complete_blinker = rospy.Timer(
            rospy.Duration.from_sec(1), lambda _: self._blink_complete())

        self._check_input = rospy.Timer(
            rospy.Duration.from_sec(0.5), lambda _: self._wait_for_user_input(), oneshot=True)

    def on_exit_stWaypointSuccess(self) -> None:
        self._complete_blinker.shutdown()
        self._check_input.shutdown()

    def on_enter_stComplete(self) -> None:
        print("We finished, wooooo")


if __name__ == "__main__":
    rospy.init_node('nav_state_machine', anonymous=False)
    statemachine = NavStateMachine(CoordinateManager())
    rospy.spin()

## @}