import brickpi
import utils
import params

interface=brickpi.Interface()
interface.initialize()
motors = [0, 1]
interface.motorEnable(motors[0])
interface.motorEnable(motors[1])
motorParams = params.setup(interface)

verbose = raw_input("Verbose session? (y/n) ")
verbose = verbose[0].lower() == 'y'

while True:
  orders = raw_input("Enter a command in the form '<Command> <Value>', M for Move, R for Rotate: ")
  try:
    command = orders[0]
    value = float(orders[1:len(orders)])

    # TODO: Change Utils to remove motorParam passing requirement, since we can just init them once.

    if command.lower() == "m":
      print "Moving " + str(value) + "cm."
      utils.move(value, interface, motorParams, verbose) 
    elif command.lower() == "r":
      print "Rotating " + str(value) + "radians."
      utils.rotate(value, interface, motorParams, verbose)
    else:
      print "Try Again." 
  except:
    print "Invalid Command! Exiting."
    break

interface.terminate()
