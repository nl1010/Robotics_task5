import sys
from math import pi
from params import *
from utils import *   

def main():
  robot_util = RobotUtility() 
  
  draw = True
  interval = 20

  robot_util.set_location(84,30)
  robot_util.move_to_waypoint(180,30,interval,draw)
  robot_util.move_to_waypoint(180,54,interval,draw)
  robot_util.move_to_waypoint(126,54,interval,draw)
  robot_util.move_to_waypoint(126,128,interval,draw)
  robot_util.move_to_waypoint(126,126,interval,draw)
  robot_util.move_to_waypoint(30,54,interval,draw)
  robot_util.move_to_waypoint(84,54,interval,draw)
  robot_util.move_to_waypoint(84,30,interval,draw)

  robot_util.walls.draw()


if __name__ == "__main__":
  main()

