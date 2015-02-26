import brickpi
import params
import time

interface=brickpi.Interface()
interface.initialize()

interface.sensorEnable(params.PORT_LEFT_TOUCH, brickpi.SensorType.SENSOR_TOUCH)
interface.sensorEnable(params.PORT_RIGHT_TOUCH, brickpi.SensorType.SENSOR_TOUCH)



while True:
  result_left = interface.getSensorValue(params.PORT_LEFT_TOUCH)
  result_right = interface.getSensorValue(params.PORT_RIGHT_TOUCH)
 
  if result_left and result_right: 
    print "Left: " + str(result_left[0]) + " , right: " + str(result_right[0]) 
  else:
    print "Failed to read"
  time.sleep(0.05)

interface.terminate()
