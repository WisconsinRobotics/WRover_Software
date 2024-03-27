#!/usr/bin/env python

""" 
@defgroup wr_logic_ai_state_machine_ai
@{
@defgroup wr_logic_ai_state_machine_py state_machine.py
@brief Node for state machine that drives the autonomous nagivation code
@details The autonomous navigation code is driven by the state machine below. This node uses the statemachine python library to 
handle the setup of the state machine, including initializing states and events, handling state transitions, executing the state 
loop, and more. Here, we need to define our states and events as well as behavior upon entering or leaving states. 

State: This will determine what the robot is currently doing

Every state will have a function for when running, entering, and exiting

Event: Most events will be either a success or error. This will indicate which state to go to next depending on the event and state it is at.


![](NavigationStateMachine.png)
@{
"""

from __future__ import annotations
from statemachine import StateMachine, State
from wr_logic_ai.coordinate_manager import CoordinateManager
from wr_logic_ai.msg import (
    NavigationState,
    LongRangeAction,
    LongRangeGoal,
    LongRangeActionResult,
)
import coord_calculations, travel_timer
from wr_logic_ai.msg import ShortRangeAction, ShortRangeGoal, ShortRangeActionResult
from wr_logic_ai.msg import SearchStateAction, SearchStateGoal, SearchStateActionResult
from wr_logic_ai.srv import SearchPatternService
from wr_led_matrix.srv import (
    led_matrix as LEDMatrix,
    led_matrixRequest as LEDMatrixRequest,
)
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
    """
    This class implements the state machine used in autonomous navigation

    @param StateMachine (StateMachine): The StateMachine class imported from the statemachine python library, required to declare
    this class as a state machine
    """

    # Defining states
    ## Starter state of the state machine
    stInit = State(initial=True)
    ## State representing that the robot is running in long range mode
    stLongRange = State()
    ## State representing that the robot is recovering from a error state
    stLongRangeRecovery = State()
    ## State representing that the robot is searching for the target
    stSearch = State()
    ## State representing that the robot is running in short range mode
    stShortRange = State()
    ## State representing that the robot has completed a task at a waypoint
    stWaypointSuccess = State()
    ## State representing that the robot has completed all autonomous navigation tasks
    stComplete = State()

    # Defining events and transitions
    ## Event representing a successful task execution
    evSuccess = (
        stLongRange.to(stShortRange)
        | stLongRangeRecovery.to(stLongRange)
        | stSearch.to(stShortRange) 
        | stShortRange.to(stWaypointSuccess)
    )
    ## Event representing an error condition being raised
    evError = (
        stLongRange.to(stLongRangeRecovery)
        | stLongRangeRecovery.to(stLongRangeRecovery)
        | stSearch.to(stLongRange)
        | stShortRange.to(stLongRange)
    )
    
    ## Event representing a shortcircuit state transition from WaypointSuccess to LongRange
    evNotWaiting = stWaypointSuccess.to(stLongRange)
    ## Event representing an unconditional state transition from Init to LongRange
    evUnconditional = stInit.to(stLongRange)
    ## Event representing all tasks are complete
    evComplete = stWaypointSuccess.to(stComplete)

    def __init__(self, mgr: CoordinateManager) -> None:
        """
        Initializes the state machine

        @param mgr (CoordinateManager): Helper class for retrieving target waypoint GPS coordinates
        """
        # Get coordinates into self._mgr
        self._mgr = mgr
        # Start in current event -1, as state machine runs it will go into event 0 (init)
        self.currentEvent = -1
        # State machine will publish into the mux to switch b/w long range and short range
        self.mux_pub = rospy.Publisher(
            "/navigation_state", NavigationState, queue_size=1
        )

        # Initialization of messages to switch the mux
        self.mux_long_range = NavigationState()
        self.mux_long_range.state = NavigationState.NAVIGATION_STATE_LONG_RANGE
        self.mux_search = NavigationState()
        self.mux_search.state = NavigationState.NAVIGATION_STATE_SEARCH
        self.mux_short_range = NavigationState()
        self.mux_short_range.state = NavigationState.NAVIGATION_STATE_SHORT_RANGE
        super(NavStateMachine, self).__init__()

    def init_calibrate(self, pub: rospy.Publisher, stop_time: float) -> None:
        """
        Function to spin the robot for a certain time. The IMU (N,E,S,W) needs to be spinned to be correct.

        @param stop_time Time when the robot should stop
        @param pub (rospy.Publisher): Publishes drive values to motors
        """
        if rospy.get_time() < stop_time:
            pub.publish(DriveTrainCmd(left_value=0.3, right_value=-0.3))
        else:
            pub.publish(DriveTrainCmd(left_value=0, right_value=0))
            self.evUnconditional()

    def init_w_ros(self):
        """Init function to start ROS publisher and time"""
        # Set the LED to autonomous
        set_matrix_color(COLOR_AUTONOMOUS)

        # Publisher for sending init calibrate
        pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=1)
        # Set amount of time to calibrate
        stop_time = rospy.get_time() + 7

        self._init_tmr = rospy.Timer(
            rospy.Duration.from_sec(0.1), lambda _: self.init_calibrate(pub, stop_time)
        )

    def on_enter_stInit(self) -> None:
        print("\non enter stInit")
        rospy.loginfo("\non enter stInit")
        # Get the coordinates that we will have to go to
        self._mgr.read_coordinates_file()

        # Run calibrate for seven seconds
        threading.Timer(1, lambda: self.init_w_ros()).start()

    def on_exit_stInit(self) -> None:
        # Stop calibration code
        self._init_tmr.shutdown()
        # Check if there is a new coordinate. Will go to event complete if ended.
        if self._mgr.next_coordinate():
            self.evComplete()

    def _longRangeActionComplete(
        self, state: GoalStatus, _: LongRangeActionResult
    ) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stLongRange(self) -> None:
        print("\non enter stLongRange")
        rospy.loginfo("\non enter stLongRange")

        # Publish to mux, runs every .2 seconds. (The mux will tell the robot to get drive values from Long Range and not short range)
        self.timer = rospy.Timer(
            rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_long_range)
        )

        # sets autonomous color for the LED
        set_matrix_color(COLOR_AUTONOMOUS)

        # Initialize the action client that will run the long range
        self._client = actionlib.SimpleActionClient(
            "LongRangeActionServer", LongRangeAction
        )
        self._client.wait_for_server()

        # Get the coordinates as a LongRangeGoal. Two coordinates (lat, long)
        goal = LongRangeGoal(
            target_lat=self._mgr.get_coordinate()["lat"],
            target_long=self._mgr.get_coordinate()["long"],
        )

        # Send coordinates to action service
        self._client.send_goal(
            goal,
            done_cb=lambda status, result: self._longRangeActionComplete(
                status, result
            ),
        )

    def on_exit_stLongRange(self) -> None:
        print("Exting Long Range")
        rospy.loginfo("Exting Long Range")
        self.timer.shutdown()  # Stop timer that was being used for MUX

    def _longRangeRecoveryActionComplete(
        self, state: GoalStatus, _: LongRangeActionResult
    ) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stLongRangeRecovery(self) -> None:
        print("\non enter stLongRangeRecovery")
        rospy.loginfo("\non enter stLongRangeRecovery")

        set_matrix_color(COLOR_ERROR)

        # There was a problem in the state machine if mgr is empty (it should have completed)
        if self._mgr is None:
            raise ValueError
        else:
            # Get previous coordinate and go to it as same as long range as a live-action debugging strategy
            self._mgr.previous_coordinate()
            print(self._mgr.get_coordinate())
            self.timer = rospy.Timer(
                rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_long_range)
            )
            self._client = actionlib.SimpleActionClient(
                "LongRangeActionServer", LongRangeAction
            )
            self._client.wait_for_server()
            goal = LongRangeGoal(
                target_lat=self._mgr.get_coordinate()["lat"],
                target_long=self._mgr.get_coordinate()["long"],
            )
            self._client.send_goal(
                goal,
                done_cb=lambda status, result: self._longRangeRecoveryActionComplete(
                    status, result
                ),
            )

    def on_exit_stLongRangeRecovery(self) -> None:
        self.timer.shutdown()  # Shutdown mux timer

    def _searchActionComplete(self, state: GoalStatus, _: SearchStateActionResult) -> None: # SearchActionResult
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stSearch(self) -> None:
        distance = 4
        num_vertices = 22

        print("\non enter stSearch")
        rospy.loginfo("\non enter stSearch")
        self.timer = rospy.Timer(rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_search))

        # enter autonomous mode
        set_matrix_color(COLOR_AUTONOMOUS)

        self._client = actionlib.SimpleActionClient("SearchActionServer", SearchStateAction) # "SearchActionServer", SearchAction
        self._client.wait_for_server()

        coords = coord_calculations.get_coords(self._mgr.get_coordinate()["lat"], self._mgr.get_coordinate()["long"], distance, num_vertices)
        SEARCH_TIMEOUT_TIME = travel_timer.calc_state_time() # default = 20 meters

        i = 0
        start_time = rospy.get_rostime()
        while (
            rospy.get_rostime() - start_time < SEARCH_TIMEOUT_TIME
            and not rospy.is_shutdown()
            and i < num_vertices
        ):
            if (i != 0): # skip starting point because rover is already there
                goal = SearchStateGoal(target_lat=coords[i]["lat"], target_long=coords[i]["long"], dist=coords[i]['distance']) # SearchGoal
                self._client.send_goal(goal, done_cb=lambda status, result: self._searchActionComplete(status, result))

            # Camera Service - still not sure about integrating the camera with the state machine
            camera_service = rospy.ServiceProxy('search_pattern_service', SearchPatternService)
            camera_service.wait_for_service('search_pattern_service')

            if camera_service:
                break # what should be done when the target object is found? how should we enter ShortRange?

            i += 1

    def on_exit_stSearch(self) -> None:
        print("Exiting Search")
        rospy.loginfo("Exiting Search")
        self.timer.shutdown()

    def _shortRangeActionComplete(
        self, state: GoalStatus, _: ShortRangeActionResult
    ) -> None:
        if state == GoalStatus.SUCCEEDED:
            self.evSuccess()
        else:
            self.evError()

    def on_enter_stShortRange(self) -> None:
        print("\non enter stShortRange")
        rospy.loginfo("\non enter stShortRange")

        set_matrix_color(COLOR_AUTONOMOUS)

        # Set mux to switch to short range
        self.timer = rospy.Timer(
            rospy.Duration(0.2), lambda _: self.mux_pub.publish(self.mux_short_range)
        )

        # Initialized short range action server that will do short range
        self._client = actionlib.SimpleActionClient(
            "ShortRangeActionServer", ShortRangeAction
        )
        self._client.wait_for_server()

        # Define different types of short range that will be used
        TARGET_TYPE_MAPPING = [
            ShortRangeGoal.TARGET_TYPE_GPS_ONLY,  # 0 Vision markers
            ShortRangeGoal.TARGET_TYPE_VISION,  # 1 Vision marker
        ]
        # Get what type of short range action it is from the coordinates
        goal = ShortRangeGoal(
            target_type=TARGET_TYPE_MAPPING[
                self._mgr.get_coordinate()["num_vision_posts"]
            ]
        )
        # Run the short range action and send what type of short range it is.
        self._client.send_goal(
            goal,
            done_cb=lambda status, result: self._shortRangeActionComplete(
                status, result
            ),
        )

    def on_exit_stShortRange(self) -> None:
        # self.timer.shutdown()
        pass

    # Defined for lambda function, not part of state machine architecture
    def _blink_complete(self) -> None:
        # Blinks the color off for 0.5 sec, the other 0.5 sec we sleep
        set_matrix_color(COLOR_COMPLETE)

    def _wait_for_user_input(self) -> None:
        # Make sure service is running
        rospy.wait_for_service("wait_for_user_input_service")
        try:
            # Define proxy of service (this would be the client calling the service)
            wait_for_user_input = rospy.ServiceProxy(
                "wait_for_user_input_service", Empty
            )
            # This will run the service. The code will stay here until the user inputs a value ("c").
            wait_for_user_input()
        except rospy.ServiceException as e:
            print(e)

        # If there is a next coordinate, go back to long range. If not, it is complete
        if self._mgr.next_coordinate():
            print("Should Enter event complete")
            self.evComplete()
        else:
            self.evNotWaiting()

    def on_enter_stWaypointSuccess(self) -> None:
        print("\non enter stWaypointSuccess")

        # Runs every one second, this blinks the LED to say we finished short range
        self._complete_blinker = rospy.Timer(
            rospy.Duration.from_sec(1), lambda _: self._blink_complete()
        )

        # This runs the wait for user input just once (oneshot = True).
        self._check_input = rospy.Timer(
            rospy.Duration.from_sec(0.5),
            lambda _: self._wait_for_user_input(),
            oneshot=True,
        )

    def on_exit_stWaypointSuccess(self) -> None:
        self._complete_blinker.shutdown()
        self._check_input.shutdown()

    def on_enter_stComplete(self) -> None:
        print("We finished, wooooo")


if __name__ == "__main__":
    rospy.init_node("nav_state_machine", anonymous=False)
    statemachine = NavStateMachine(CoordinateManager())  # Initialize state machine
    rospy.spin()  # Runs nav_state_machine topic

## @}
## @}
