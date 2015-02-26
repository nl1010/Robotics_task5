import brickpi
import time
import params

interface=brickpi.Interface()
interface.initialize()

motors = [0,1]

interface.motorEnable(motors[0])
interface.motorEnable(motors[1])

motorParams = params.setup(interface)

interface.setMotorAngleControllerParameters(motors[0],motorParams[0])
interface.setMotorAngleControllerParameters(motors[1],motorParams[1])

#logfile_name = "0_" + str(motor0_ku) + "_1_" + str(motor1_ku)
#raw_input("Logfile name: ")

#paramsFile = open('../Logfiles/Params/' + logfile_name + "_params.txt", 'w')
#paramsFile.write(str(MOTOR0_KU) + '\n' + str(MOTOR1_KU))
#paramsFile.write(str(motor0_ku) + '\n' + str(motor1_ku))
#paramsFile.close

#interface.startLogging("../Logfiles/" + logfile_name + ".txt")

while True:
	angle = float(input("Enter a angle to rotate (in radians): "))

	interface.increaseMotorAngleReferences(motors,[angle,angle])

	while not interface.motorAngleReferencesReached(motors) :
		print "Refrences: ", interface.getMotorAngleReferences(motors)
		motorAngles = interface.getMotorAngles(motors)
		if motorAngles :
			print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
		time.sleep(0.1)

	print "Destination reached!"

#interface.stopLogging()
	

interface.terminate()
