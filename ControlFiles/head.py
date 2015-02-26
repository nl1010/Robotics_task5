import sys
from math import pi
from params import *
from utils import *   

def main():
  SQUARE_LOG_FILE_NAME = "square"
  
  robot_util = RobotUtility() 

  #robot_util.reset_head()  

  #robot_util.rotate_head(-200 * math.pi / 180)

  #robot_util.look_straight()

  angles_distances  = robot_util.look_around()

  for angle_distance in angles_distances:
    print 'Angle ', angle_distance[0], ' distance ', angle_distance[1]


if __name__ == "__main__":
  main()
