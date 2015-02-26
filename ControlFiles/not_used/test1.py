import brickpi
import time

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParam1 = interface.MotorAngleControllerParameters()
motorParam1.maxRotationAcceleration = 7.0
motorParam1.maxRotationSpeed = 12.0
motorParam1.feedForwardGain = 255/20.0
motorParam1.minPWM = 18.0
motorParam1.pidParameters.minOutput = -255
motorParam1.pidParameters.maxOutput = 255
motorParam1.pidParameters.k_p = 20.0
motorParam1.pidParameters.k_i = 0.0
motorParam1.pidParameters.k_d = 0.0

motorParam2 = interface.MotorAngleControllerParameters()
motorParam2.maxRotationAcceleration = 7.0
motorParam2.maxRotationSpeed = 12.0
motorParam2.feedForwardGain = 255/20.0
motorParam2.minPWM = 18.0
motorParam2.pidParameters.minOutput = -255
motorParam2.pidParameters.maxOutput = 255
motorParam2.pidParameters.k_p = 20.0
motorParam2.pidParameters.k_i = 0.0
motorParam2.pidParameters.k_d = 0.0


interface.setMotorAngleControllerParameters(motors[0],motorParam1)
interface.setMotorAngleControllerParameters(motors[1],motorParam2)

logfile = raw_input("Specify logfile: ")
#f = open("/home/pi/BrickPi/Logfiles/log0.txt");
interface.startLogging("../Logfiles/" + logfile)

while True:
        angle1 = float(input("Enter a angle to rotate wheel 1 (in radians): "))
        angle2 = float(input("Enter a angle to rotate wheel 2 (in radians): "))

        interface.increaseMotorAngleReferences(motors,[angle1,angle2])

        while not interface.motorAngleReferencesReached(motors) :
                print "Refrences: ", interface.getMotorAngleReferences(motors)
                motorAngles = interface.getMotorAngles(motors)
                if motorAngles :
                        print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
                time.sleep(0.1)

        print "Destination reached!"

interface.stopLogging()


interface.terminate()

