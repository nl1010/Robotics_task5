#!usr/bin/python
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def main(): 
  logfile_name = raw_input("Logfile name: ")
  logfile = open("Logfiles/" + logfile_name + ".txt")
  lines = logfile.readlines()

  angleDif = []
  time = []
  motor0Ref = []
  motor0Ang = []
  motor1Ref = []
  motor1Ang = []
  motor0Dif = []
  motor1Dif = []
  
  # Read logs from the file
  for line in lines: 
    words = line.split()
    if len(words) == 5:
      time.append(float(words[0]))
      
      motor0Ref.append(float(words[1]))
      motor0Ang.append(float(words[2]))
      motor1Ref.append(float(words[3]))
      motor1Ang.append(float(words[4]))
      
      motor0Dif.append(float(words[1]) - float(words[2]))
      motor1Dif.append(float(words[3]) - float(words[4]))
      angleDif.append(float(words[2]) - float(words[4]))
  
  motor0pu = 0
  motor1pu = 0

  paramsfile = open("Logfiles/Params/" + logfile_name + "_params.txt")
  lines = paramsfile.readlines()
  motor0pu = float(lines[0])
  motor1pu = float(lines[1])
    
  grid = gridspec.GridSpec(3, 2)

  # Blue curve is reference
  # Red curve is angle
  # Motor 0 (left) plot angle & reference curves
  plt.subplot(grid[0,0])
  plt.axis([time[0] - 1, 
            time[-1] + 1, 
            min(motor0Ref[0], motor0Ang[0]) - 1, 
            max(motor0Ref[-1], motor0Ang[-1]) + 15
           ])
  plt.plot(time, motor0Ref, 'blue') 
  plt.plot(time, motor0Ang, 'red')
  plt.title('Motor 0 (Left) ku=' + str(motor0pu))  

  # Motor 0 (left) plot angle & reference difference
  #plt.subplot(3,2,2)
  plt.subplot(grid[0,1])
  plt.axis([time[0], time[-1], -0.5, 0.5])
  plt.plot(time, motor0Dif, 'blue')
  plt.title('Motor 0 reference - angle difference')
 

  # Motor 1 (right) plot angle & reference curves
  #plt.subplot(3,2,3)
  plt.subplot(grid[1,0])
  plt.axis([time[0] - 1,
            time[-1] + 1,
            min(motor1Ref[0], motor1Ang[0]) - 1,
            max(motor1Ref[-1], motor1Ang[-1]) +15
           ])
  plt.plot(time, motor1Ref, 'blue')
  plt.plot(time, motor1Ang, 'red')
  plt.title('Motor 1 (Right) ku=' + str(motor1pu))

  # Motor 1 (right) plot angle & reference difference
  #plt.subplot(3,2,4)
  plt.subplot(grid[1,1])
  plt.axis([time[0], time[-1], -0.5, 0.5])
  plt.plot(time, motor1Dif, 'blue')
  plt.title('Motor 1 reference - angle difference')

  # Difference between angles
  plt.subplot(grid[2,:])
  #plt.subplot(3,2,5,rowspan=2)
  plt.axis([time[0], time[-1], -5, 5])
  plt.plot(time, angleDif, 'red')
  plt.title('Angle differences')

  plt.show()

if __name__ == "__main__":
  main()

