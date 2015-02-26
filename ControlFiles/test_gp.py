#!/usr/bin/python

from utils import RobotUtility
import math

class mock_bot:
  def __init__(self, x=0, y=0, theta=0):
    self.x = x
    self.y = y
    self.theta = theta
 
def main():
  RU = RobotUtility()
  test = [0.05, 0.13, 0.2, 0.34, 0.77, 12032.12]
  print( "Solution is: " + str(RU.get_particle(0, test)))

if __name__ == '__main__':
  main()
