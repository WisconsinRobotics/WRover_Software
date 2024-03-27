import rospy
from wr_logic_ai.srv import SearchPatternService, SearchPatternServiceRequest, SearchPatternServiceResponse

'''
@ingroup wr_logic_ai
@defgroup wr_logic_ai Search Pattern Client
@brief The client for the camera that scans the rover's surroundings for the target object.
@details This client is currently waiting for the response from the server and sending the 
result (boolean), which seems a little redundant. This might be changed or completely 
removed in the future.
'''

def main():
    rospy.init_node("search_pattern_client")
    search_pattern_service = rospy.ServiceProxy('search_pattern_service', SearchPatternService)
    rate = rospy.Rate(10)
    rospy.wait_for_service('object_detection')

    while not rospy.is_shutdown():
        try:
            response:SearchPatternServiceResponse = search_pattern_service()
            return response.output
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

        rate.sleep()

if __name__ == '__main__':
    main()