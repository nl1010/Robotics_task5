PORT_ULTRASONIC = 3
PORT_LEFT_TOUCH = 0
PORT_LEFT_LIGHT = 4 # broken
PORT_RIGHT_TOUCH = 2 
PORT_RIGHT_LIGHT = 1
MOTOR_LEFT = 0
MOTOR_RIGHT = 1
MOTOR_HEAD = 2
DRAW_OFFSET_X = 0 
DRAW_OFFSET_Y = 0
DRAW_SCALE = 1


def setup(interface, perc_mod_kd=1, perc_mod_ki=1, motor0_ku=550, motor1_ku=550, motor0_pu=0.7, motor1_pu=0.7):
  motorParams = [];

  MOTOR0_KP = 0.6 * motor0_ku
  MOTOR1_KP = 0.6 * motor1_ku
  motor2_pu = 0.7
  MOTOR2_KP = MOTOR0_KP 

  # set params for motor 0 (left)
  motorParams.append(interface.MotorAngleControllerParameters())
  motorParams[0].maxRotationAcceleration = 6.0
  motorParams[0].maxRotationSpeed = 12.0
  motorParams[0].feedForwardGain = 255/20.0
  motorParams[0].minPWM = 18.0
  motorParams[0].pidParameters.minOutput = -255
  motorParams[0].pidParameters.maxOutput = 255
  motorParams[0].pidParameters.k_p = MOTOR0_KP
  motorParams[0].pidParameters.k_i = perc_mod_ki * 2 * MOTOR0_KP / motor0_pu
  motorParams[0].pidParameters.k_d = perc_mod_kd * MOTOR0_KP * motor0_pu / 8

  # set params for motor 1 (right)
  motorParams.append(interface.MotorAngleControllerParameters())
  motorParams[1].maxRotationAcceleration = 6.0
  motorParams[1].maxRotationSpeed = 12.0
  motorParams[1].feedForwardGain = 255/20.0
  motorParams[1].minPWM = 18.0
  motorParams[1].pidParameters.minOutput = -255
  motorParams[1].pidParameters.maxOutput = 255
  motorParams[1].pidParameters.k_p = MOTOR1_KP 
  motorParams[1].pidParameters.k_i = 2 * MOTOR1_KP / motor1_pu
  motorParams[1].pidParameters.k_d = MOTOR1_KP * motor1_pu / 8

  # set params for motor 2 (head)
  motorParams.append(interface.MotorAngleControllerParameters())
  motorParams[2].maxRotationAcceleration = 6.0
  motorParams[2].maxRotationSpeed = 12.0
  motorParams[2].feedForwardGain = 255/20.0
  motorParams[2].minPWM = 18.0
  motorParams[2].pidParameters.minOutput = -255
  motorParams[2].pidParameters.maxOutput = 255
  motorParams[2].pidParameters.k_p = MOTOR2_KP 
  motorParams[2].pidParameters.k_i = 2 * MOTOR2_KP / motor2_pu
  motorParams[2].pidParameters.k_d = MOTOR2_KP * motor2_pu / 8

  return motorParams

