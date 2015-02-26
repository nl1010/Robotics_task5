import brickpi
import time

WHEEL_RADIUS = 1.5

'''Move distance cm forward'''
def move(distance):
  interface=brickpi.Interface()
  interface.initialize()
 
  motors = [0,1]
  
  interface.motorEnable(motors[0])
  interface.motorEnable(motors[1])
   
  motorParams = interface.MotorAngleControllerParameters()
  motorParams.maxRotationAcceleration = 7.0
  motorParams.maxRotationSpeed = 12.0
  motorParams.feedForwardGain = 255/20.0
  motorParams.minPWM = 18.0
  motorParams.pidParameters.minOutput = -255
  motorParams.pidParameters.maxOutput = 255
  motorParams.pidParameters.k_p = 20.0
  motorParams.pidParameters.k_i = 0.0
  motorParams.pidParameters.k_d = 0.0

  logfile = "move"

  interface.startLogging("/home/pi/BrickPi/Logfiles/" + logfile)
  
  interface.setMotorAngleControllerParameters(motors[0],motorParams)
  interface.setMotorAngleControllerParameters(motors[1],motorParams)

  interface.increaseMotorAngleReferences(motors,[distance/WHEEL_RADIUS,distance/WHEEL_RADIUS])
  
  interface.stopLogging()

  interface.terminate()

