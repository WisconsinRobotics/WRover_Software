'''
@ingroup wr_logic_ai
@defgroup wr_logic_ai Travel Timer
@brief Calculates the maximum travel time between two or all coordinates to know when 
to skip the coordinate or to restart searching

Attributes(s):
  speed - speed of the rover in meters per second

Methods(s):
  calculate_time(distance) - Calculates maximum travel time between two coordinates
  calculate_time(distance) - Calculates maximum travel time between all coordinates
'''

SPEED = 0.5 # in meters per second,
            # purposefully slow to give more time for obstacle avoidance
            # and because true speed is inknown 
            # (motor power outputs are set to some value between 0.3 and 0.5,
            # which does not correlate to speed)

def calc_point_time(distance) -> float:
  '''
  Description: Calculates the maximum time it should take the rover to travel from one 
    coordinate to another coordinate. If the rover reaches this limit, the coordinate 
    is labeled as unreachable due to obstacle avoidance, and it will skip the 
    coordinate and move on to the next one.

  Parameter(s):
    distance - the distance from the current coordinate to the target coordinate/point (in meters)

  Return(s):
    time - the time it takes the rover to travel the distance between two coordinates (in seconds)
        '''
  time = distance / SPEED
  return time

def calc_state_time(radius = 20) -> float:
  '''
  Description: Calculates the maximum time it should take the rover to visit all
    coordinates during the searching state. The spiral pattern uses 4 meters as a base,
    so the distance between every coordinate must be a multiple of 4.

  Parameter(s):
    radius - the radius of the square spiral (in meters),
             correlates to the maximum distance the starting coordinate is to the target

  Return(s):
    time - the time it takes the rover to visit all coordinates (in seconds)
  '''
  total_distance = (0.5 * radius + 1) * (2 * radius + 4)
  total_time = total_distance / SPEED

  return total_time
    