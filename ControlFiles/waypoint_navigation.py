import sys
from math import pi
from params import *
from utils import *   

def main():
  robot_util = RobotUtility() 
  
  robot_util.move(0.0000001)

  while True:
    print "Point to navigate to:"
    x = float(raw_input("x: "))
    y = float(raw_input("y: "))
    draw = True
    robot_util.move_to_waypoint(x, y, draw)

if __name__ == "__main__":
  main()

