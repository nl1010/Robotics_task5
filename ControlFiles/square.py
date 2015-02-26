import sys
from math import pi
from params import *
from utils import *   

def main():
  SQUARE_LOG_FILE_NAME = "square"
  
  robot_util = RobotUtility() 

  #if len(sys.argv) == 4: 
  # robot_util.set_drift_callibration(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
  # else:
  #  print "No drift calibration values set!"
  
  draw = True
  size = 40
  if len(sys.argv) > 1:
    size = int(sys.argv[1])
  if len(sys.argv) > 2 and sys.argv[2] == "simple":
    draw = False

  for i in range(4):
    repeat_num = int(size / 10)
    for j in range(repeat_num):
      robot_util.move(10, draw=draw) 
    robot_util.rotate(pi/2, draw=draw)


if __name__ == "__main__":
  main()
