import time
import sys
from math import pi
from params import *
from utils import *   

def main():
  robot_util = RobotUtility() 

  #if len(sys.argv) == 4: 
  # robot_util.set_drift_callibration(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
  # else:
  #  print "No drift calibration values set!"
  
  # map_move_ultrasonic = []
 
  #for i in range (255):
   #hit_left, hit_right, ultrasonic = robot_util.move(-1) 
   #print (i, ultrasonic)
   #map_move_ultrasonic.append((i, ultrasonic))
   #time.sleep(0.1)

  #robot_util.move(1)
  #angle = 0 

  #while True: 
   # angle = angle + 1
    #head_angle_offset, ultrasonic = robot_util.rotate_head(-1 * pi / 180)
    #print "Angle deg: ", head_angle_offset * 180 / math.pi , "Ultrasonic: ", ultrasonic
    #print "Angle deg: ", angle , "Ultrasonic: ", ultrasonic
    #time.sleep(0.1)
  flip = 1 
  for i in range(100): 
    print robot_util.interface.getSensorValue(params.PORT_ULTRASONIC) 
    robot_util.move(6 * flip)
    flip = -flip 

if __name__ == "__main__":
  main()
