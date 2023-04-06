from coordinate_manager import Coordinate_manager
from wr_logic_ai.msg import TargetMsg
import rospy

class State:
    mgr: Coordinate_manager = None
    
    def __init__(self) -> None:
        pass

    def enter(self) -> None:
        raise NotImplementedError("This is an abstract method")

    def exit(self) -> None:
        raise NotImplementedError("This is an abstract method")

class stInit(State):   
    _mgr : Coordinate_manager = None

    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        if self.mgr is None:
            raise ValueError
        else:
            self.mgr.read_coordinates_file()

    def exit(self) -> None:
        pass

    @staticmethod
    def set_coordinate_manager(mgr) -> None:
        stInit._mgr = mgr

class stLongRange(State):
    _mgr : Coordinate_manager = None

    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        if self._mgr is None:
            raise ValueError
        else:
            self.timer = rospy.Timer(rospy.Duration(0.2), self.publish)
            self.pub_nav = rospy.Publisher('/target_coord', TargetMsg, queue_size=1)
            rospy.spin()
       
    def publish(self):    
        # Publish to obstacle avoidance with the target coordinates
        target_coords = TargetMsg()
        target_coords.target_lat = stLongRange._mgr.get_coordinate['lat']
        target_coords.target_long = stLongRange._mgr.get_coordinate['long']
        target_coords.target_type = stLongRange._mgr.get_coordinate['target_type']
        self.pub_nav.publish(target_coords)
        # TODO: Publish to topic /navigation_state and publish to /nav_data

    def exit(self) -> None:
        self.timer.shutdown()
    
    @staticmethod
    def set_coordinate_manager(mgr) -> None:
        stLongRange._mgr = mgr

class stError(State):
    _mgr : Coordinate_manager = None
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        self.mgr.previous_line()
    
    @staticmethod
    def set_coordinate_manager(mgr) -> None:
        stError._mgr = mgr

class stLR_Recovery(State):
    _mgr : Coordinate_manager = None
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass
    
    @staticmethod
    def set_coordinate_manager(mgr) -> None:
        stLR_Recovery._mgr = mgr

class stShortRange(State):
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        pass

class stWaypointSuccess(State):
    _mgr : Coordinate_manager = None
    def __init__(self) -> None:
        super().__init__()

    def enter(self) -> None:
        pass

    def exit(self) -> None:
        self.mgr.next_line()
        
    @staticmethod
    def set_coordinate_manager(mgr) -> None:
        stWaypointSuccess._mgr = mgr