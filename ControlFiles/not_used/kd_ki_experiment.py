import brickpi
import time
import params
import sys

def main(ku, pu, percentage_modifier_kd=1, percentage_modifier_ki=1, motor=0, distance=20, timeout=5):
  interface=brickpi.Interface()
  interface.initialize()
  motors = [0,1]
  interface.motorEnable(motors[0])
  interface.motorEnable(motors[1])
  motorParams = [];
  motor0_ku = ku if motor == 0 else 0
  motor1_ku = ku if motor == 1 else 0 
  
  # set params for motor 0 (left)
  motorParams.append(interface.MotorAngleControllerParameters())
  motorParams[0].maxRotationAcceleration = 6.0
  motorParams[0].maxRotationSpeed = 12.0
  motorParams[0].feedForwardGain = 255/20.0
  motorParams[0].minPWM = 18.0
  motorParams[0].pidParameters.minOutput = -255
  motorParams[0].pidParameters.maxOutput = 255
  motorParams[0].pidParameters.k_p = 0.6 * motor0_ku
  motorParams[0].pidParameters.k_i = percentage_modifier_ki * 2 * 0.6 * motor0_ku / pu 
  motorParams[0].pidParameters.k_d = percentage_modifier_kd * 0.6 * motor0_ku  * pu / 8 

  # set params for motor 1 (right)
  motorParams.append(interface.MotorAngleControllerParameters())
  motorParams[1].maxRotationAcceleration = 6.0
  motorParams[1].maxRotationSpeed = 12.0
  motorParams[1].feedForwardGain = 255/20.0
  motorParams[1].minPWM = 18.0
  motorParams[1].pidParameters.minOutput = -255
  motorParams[1].pidParameters.maxOutput = 255
  motorParams[1].pidParameters.k_p = 0.6 * motor1_ku
  motorParams[1].pidParameters.k_i = percentage_modifier_ki * 2 * 0.6 * motor1_ku / pu 
  motorParams[1].pidParameters.k_d = percentage_modifier_kd * 0.6 * motor1_ku  * pu / 8

  interface.setMotorAngleControllerParameters(motors[0],motorParams[0])
  interface.setMotorAngleControllerParameters(motors[1],motorParams[1])
  
  logfile_name = "KI_" + str(percentage_modifier_ki) + "KD_" + str(percentage_modifier_kd) + "_MOTOR_" + str(motor) + "_DISTANCE_" + str(distance)
  interface.startLogging("../Logfiles/" + logfile_name + ".log")
  
  values = [0, 0]
  values[motor] = distance
  interface.increaseMotorAngleReferences(motors, values)
  while not (interface.motorAngleReferencesReached(motors) or timeout < 0):
    time.sleep(0.1)
    timeout = timeout - 0.1
  
  print "Test Complete."
  
  interface.stopLogging()
  interface.terminate()
