import sys
from params import *
from utils import *   
import random  

import time 

def main():
  SQUARE_LOG_FILE_NAME = "square"
  
  robot_util = RobotUtility() 

  if len(sys.argv) == 4: 
    robot_util.set_drift_callibration(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
  else:
    print "No drift calibration values set!"
  
  hit_left, hit_right, ultrasonic_reading = robot_util.move(40)
  angle_distance_map, max_angle_distance = robot_util.look_around()

  for angle_distance in angle_distance_map:
   print "Angle: ", angle_distance[0], " distance ", angle_distance[1]

  print "Max: ", max_angle_distance

  if hit_left:
    robot_util.rotate(random.uniform(pi/2, pi))
  elif hit_right:
    robot_util.rotate(random.uniform(-pi/2, -pi))

if __name__ == "__main__":
  main()
