import brickpi
import math
import params
import random
import time
import sys

#Provided by ajd + modified START

# A Canvas class for drawing a map and particles:
# 	- it takes care of a proper scaling and coordinate transformation between
#	  the map frame of reference (in cm) and the display (in pixels)
class Canvas:
  def __init__(self,map_size=210):
    self.map_size    = map_size    # in cm;
    self.canvas_size = 768         # in pixels;
    self.margin      = 0.05*map_size
    self.scale       = self.canvas_size/(map_size+2*self.margin)

  def drawLine(self,line):
    x1 = self.__screenX(line[0])
    y1 = self.__screenY(line[1])
    x2 = self.__screenX(line[2])
    y2 = self.__screenY(line[3])
    print "drawLine:" + str((x1,y1,x2,y2))

  def drawParticles(self,data):
    display = [(self.__screenX(d[0]),self.__screenY(d[1])) + d[2:] for d in data]
    print "drawParticles:" + str(display)

  def __screenX(self,x):
    return (x + self.margin)*self.scale

  def __screenY(self,y):
    return (self.map_size + self.margin - y)*self.scale

# A Map class containing walls
class Map:
  def __init__(self):
    self.a = (0,0,0,168)        # a
    self.b = (0,168,84,168)     # b
    self.c = (84,126,84,210)    # c
    self.d = (84,210,168,210)   # d
    self.e = (168,210,168,84)   # e
    self.f = (168,84,210,84)    # f
    self.g = (210,84,210,0)     # g
    self.h = (210,0,0,0)        # h

    self.walls = [self.a, self.b, self.c, self.d, self.e, self.f, self.g, self.h]

  def add_wall(self,wall):
    self.walls.append(wall)

  def clear(self):
    self.walls = []

  def draw(self):
    for wall in self.walls:
     print "drawLine:" + str(wall) 
      
  def which_wall(self, particle, verbose=False):
    # Given a particle, find out which wall it is looking at.
    minimum_m = sys.maxint
    best_wall = None
    incident_angle = 0.0
    for wall in self.walls:
      A_x = wall[0]
      A_y = wall[1]
      B_x = wall[2]
      B_y = wall[3]

      if verbose:
        print("Wall A: ({},\t {})\t - \tWall B: ({},\t {}).".format(A_x, A_y, B_x, B_y))

      m = ( (B_y - A_y) * (A_x - particle.x) - (B_x - A_x) * (A_y - particle.y) ) / ( (B_y - A_y) * math.cos(particle.theta) - (B_x - A_x) * math.sin(particle.theta) )

      if verbose:
        print("M for this wall: {}".format(m))

      x_max = max(A_x, B_x)
      x_min = min(A_x, B_x)
      y_max = max(A_y, B_y)
      y_min = min(A_y, B_y)

      hit_point = (particle.x + m * math.cos(particle.theta), particle.y + m * math.sin(particle.theta))

      if verbose:
        print(hit_point)

      if hit_point[0] >= x_min and hit_point[0] <= x_max and hit_point[1] >= y_min and hit_point[1] <= y_max:
        if m < minimum_m and m >= 0:
          minimum_m = m
          best_wall = wall
          incident_angle = math.acos(( math.cos(particle.theta) * (A_y - B_y) + math.sin(particle.theta) * (B_x - A_x) ) / (math.sqrt((A_y - B_y) ** 2 + (B_x - A_x) ** 2)) )
      
    return best_wall, minimum_m, incident_angle

  # Provided by ajd + modified END
 
class Particle:
  num_particles = 100
  def __init__(self):
    self.x = 0
    self.y = 0
    self.theta = 0
    self.weight = 1.0 / Particle.num_particles

    # offsets from origin for drawing
class RobotUtility:
  def __init__(self):
    interface = brickpi.Interface()
    interface.initialize()
    self.ACCEPT_THRESHOLD = 0.25
    self.WHEEL_RADIUS = 2.85
    self.ROBOT_RADIUS = 5.5
    self.ROBOT_LENGTH = 13.5
    self.PERC_MOD_WHEEL = 1.025
    self.PERC_MOD_BOOST = 1.15
    self.ANGLE_BOOST = 1.141
    self.walls = Map()
    self.particles = [Particle() for i in range(Particle.num_particles)] # (x, y, theta)
    self.sigma = 0.0125
    self.error_theta = 0.5 * math.pi / 180
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0
    self.sonar_sigma = 0.4
    self.interface = interface
    self.motorParams = params.setup(interface)    
    self.motors = [params.MOTOR_LEFT,params.MOTOR_RIGHT,params.MOTOR_HEAD] 

    for motor in self.motors: 
      interface.motorEnable(motor)
      interface.setMotorAngleControllerParameters(motor, self.motorParams[motor])
      
    interface.sensorEnable(params.PORT_LEFT_TOUCH, brickpi.SensorType.SENSOR_TOUCH)
    interface.sensorEnable(params.PORT_RIGHT_TOUCH, brickpi.SensorType.SENSOR_TOUCH)
    interface.sensorEnable(params.PORT_ULTRASONIC, brickpi.SensorType.SENSOR_ULTRASONIC)   
   
    self.HEAD_ANGLE_CENTRE = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    self.HEAD_ANGLE_LEFT = self.HEAD_ANGLE_CENTRE - math.pi / 2 
    self.HEAD_ANGLE_RIGHT = self.HEAD_ANGLE_CENTRE + math.pi / 2
    self.HEAD_STEP = 30 * math.pi / 180    
    self.HEAD_WAIT = 0.25

  def getWalls(self):
    return self.walls

  def set_drift_callibration(self, mod_wheel, boost, degs):
    self.PERC_MOD_WHEEL = mod_wheel
    self.PERC_MOD_BOOST = boost 
    self.ANGLE_BOOST = degs

  
  '''Sets robot location to (x,y)'''
  def set_location(self, x, y, theta=0):
   self.x = x
   self.y = y
   self.theta = theta
   for particle in self.particles:
     particle.x = x
     particle.y = y
     particle.theta = theta

 
  def update_weights(self):
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)
    new_weights = []
    num_acceptable = 0
    
    for particle in self.particles: 
      likelihood, acceptable = self.calculate_likelihood(self.walls, particle, sonar_reading)
      new_weights.append(particle.weight * likelihood)
      if acceptable:
        num_acceptable = num_acceptable + 1

    if num_acceptable > len(self.particles) * self.ACCEPT_THRESHOLD:
      for i in range(len(self.particles)):
        particle = self.particles[i]
        particle.weight = new_weights[i] / sum(new_weights)
    
  

  '''Rotate radians clockwise'''
  def rotate(self, radians, draw=False, verbose=False):
    # angle to rotate motor
    angle = self.ROBOT_RADIUS * (radians * self.ANGLE_BOOST) / self.WHEEL_RADIUS
    
    motors = [params.MOTOR_LEFT, params.MOTOR_RIGHT] 
 
    self.interface.increaseMotorAngleReferences(motors, [angle, -angle * 1.00])
   
    while not self.interface.motorAngleReferencesReached(motors):
      motorAngles = self.interface.getMotorAngles(motors)
      if motorAngles and verbose:
        print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
    # while not end   
    
    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    for particle in self.particles:
      particle.theta = particle.theta + radians + random.gauss(0, self.error_theta)
  
    self.update_weights()
    self.resample()

    if draw:
      draw_particles = []
      for particle in self.particles:
        draw_particles.append((particle.x * params.DRAW_SCALE +
          params.DRAW_OFFSET_X, particle.y * params.DRAW_SCALE +
          params.DRAW_OFFSET_Y, particle.theta))
      print "drawParticles:" + str(draw_particles)

    tot_theta = 0
    for particle in self.particles:
      tot_theta = tot_theta + particle.theta * particle.weight

    self.theta = tot_theta

  # Head methods START
   
  '''Puts head to the nearest default position, either looking left or looking right
     Returns koefficient to be used for direction of rotation'''
  def reset_head(self, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]    
    direction_k = 1
    if (verbose):
      print 'Head before reset', current_head_angle
    if ( abs(current_head_angle - self.HEAD_ANGLE_LEFT) < abs(current_head_angle - self.HEAD_ANGLE_RIGHT)):
      self.look_left(verbose)
    else:
      self.look_right(verbose)
      direction_k = -1
    return direction_k


  '''Move head to look left'''
  def look_left(self, verbose=False):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_LEFT)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      if verbose:
        print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  '''Move head to look right'''
  def look_right(self, verbose=False):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_RIGHT)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      if verbose:
        print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  '''Move head to look straight'''
  def look_straight(self, verbose=False):
    self.interface.setMotorAngleReference(params.MOTOR_HEAD, self.HEAD_ANGLE_CENTRE)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]) :
      if verbose:
        print "Head angle: ", self.interface.getMotorAngle(params.MOTOR_HEAD)


  '''Rotate head radians clockwise.'''
  def rotate_head(self, radians, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    
    if (current_head_angle + radians < self.HEAD_ANGLE_LEFT):
      radians = - current_head_angle + self.HEAD_ANGLE_LEFT
    elif (current_head_angle + radians > self.HEAD_ANGLE_RIGHT):
      radians = - current_head_angle + self.HEAD_ANGLE_RIGHT 

    self.interface.increaseMotorAngleReference(params.MOTOR_HEAD, radians)
    while not self.interface.motorAngleReferenceReached(self.motors[params.MOTOR_HEAD]): 
      current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
      if verbose:
        print "Angle from initial head angle: ", (head_angle_offset) * 180 / math.pi
   
    end_sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)
   
    return current_head_angle, end_sonar_reading


  ''' Prints head angle'''
  def print_head_angle(self):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]    
    print 'Head angle', current_head_angle


  '''Looks arounds between left and right head angles.
     Returns array of tuples with angles and sonar readings'''
  def look_around(self, verbose=False):
    direction_k = self.reset_head()
    #print 'RESETED'
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)     
    angles_distances = [(current_head_angle, sonar_reading)]

    while (current_head_angle >= self.HEAD_ANGLE_LEFT
           and current_head_angle <= self.HEAD_ANGLE_RIGHT) :
      current_head_angle, sonar_reading = self.rotate_head(direction_k * self.HEAD_STEP)
      angles_distances.append((self.get_head_offset(), sonar_reading))
      if verbose:
        print "Head angle: ", head_angle, "Ultrasonic: ", sonar_reading
      time.sleep(self.HEAD_WAIT)

    return angles_distances


  ''' Get offset angle of head from centre in radians '''
  def get_head_offset(self, verbose=False):
    current_head_angle = self.interface.getMotorAngle(params.MOTOR_HEAD)[0]
    return self.HEAD_ANGLE_CENTRE - current_head_angle

  # Methods for head END

  # Methods for move START
  '''Move distance cm forward'''
  def force_move(self, distance, verbose=False):
    self.interface.increaseMotorAngleReferences(self.motors,[(distance * self.PERC_MOD_BOOST)/self.WHEEL_RADIUS, self.PERC_MOD_WHEEL * distance * self.PERC_MOD_BOOST/self.WHEEL_RADIUS])
    
    while not self.interface.motorAngleReferencesReached(self.motors) :
      motorAngles = self.interface.getMotorAngles([0,1])
      if motorAngles and verbose:
        print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]
 

  '''Move distance cm forward with conditions
     Returns'''
  def move(self, distance, draw=False, verbose=False):
    bumper_hit_left = 0
    bumper_hit_right = 0
    
    self.interface.increaseMotorAngleReferences(self.motors,[(distance * self.PERC_MOD_BOOST)/self.WHEEL_RADIUS, self.PERC_MOD_WHEEL * distance * self.PERC_MOD_BOOST/self.WHEEL_RADIUS])

    while not self.interface.motorAngleReferencesReached(self.motors) and not bumper_hit_left and not bumper_hit_right:
      motorAngles = self.interface.getMotorAngles([0,1])
      bumper_hit_left = self.interface.getSensorValue(params.PORT_LEFT_TOUCH)[0]
      bumper_hit_right = self.interface.getSensorValue(params.PORT_RIGHT_TOUCH)[0]
      sonar_reading = self.interface.getSensorValue(params.PORT_ULTRASONIC)      
      
      #drift opposite
      #if (max_from_object > 0):
        #drift closer   

      if verbose:
        if motorAngles:
          print "Motor angles: ", motorAngles[0][0], ", ", motorAngles[1][0]   
        if bumper_hit_left or bumper_hit_right: 
          print "Left: " + str(bumper_hit_left) + " , right: " + str(bumper_hit_right) 
        if sonar_reading:
          print "Ultrasonic: " + str(sonar_reading)
        else:
          print "Failed to read"
    # while not end   

    # Create a list of particles to draw. This list should be filled by tuples (x, y, theta).
    prev_x = self.x
    prev_y = self.y
    tot_x = 0
    tot_y = 0
    
    for particle in self.particles:
      e = random.gauss(0, self.sigma) * distance
      particle.x = particle.x + (distance + e) * math.cos(particle.theta)
      particle.y = particle.y + (distance + e) * math.sin(particle.theta)
      particle.theta = particle.theta + random.gauss(0, self.error_theta)
      

    self.update_weights()
    self.resample() 
    
    for particle in self.particles:
      tot_x = tot_x + particle.x * particle.weight
      tot_y = tot_y + particle.y * particle.weight

    self.x = tot_x
    self.y = tot_y
      
    if draw:
      draw_particles = []
      for particle in self.particles:
        draw_particles.append((particle.x * params.DRAW_SCALE +
          params.DRAW_OFFSET_X, particle.y * params.DRAW_SCALE +
          params.DRAW_OFFSET_Y, particle.theta))
      print "drawParticles:" + str(draw_particles)
      print "drawLine:"+str((params.DRAW_SCALE * prev_x + params.DRAW_OFFSET_X,
                             params.DRAW_SCALE * prev_y + params.DRAW_OFFSET_Y,
                             params.DRAW_SCALE * self.x + params.DRAW_OFFSET_X,
                             params.DRAW_SCALE * self.y + params.DRAW_OFFSET_Y))
    self.reset_motors()
    
    return bumper_hit_left, bumper_hit_right, sonar_reading
  

  def terminate_interface(self):
    self.interface.terminate()

  
  def get_particle(self, val, arr):
    split = len(arr) / 2

    if val <= arr[split]:
      if split == 0:
        return 0
      elif val > arr[split - 1]:
        return split
      else:
        return self.get_particle(val, arr[:split])
    else:
      return self.get_particle(val, arr[split:]) + split
     

  def resample(self):
    cpa = []
    new_particles = []
    tot_p = 0

    for i in range(len(self.particles)): 
      tot_p = tot_p + self.particles[i].weight
      cpa.append(tot_p)
     
    for i in range(len(self.particles)):
      rnd = random.uniform(0, 1)
      index = self.get_particle(rnd, cpa) 
      p = Particle() 
      p.x = self.particles[index].x
      p.y = self.particles[index].y
      p.theta = self.particles[index].theta
      p.weight = self.particles[index].weight
      new_particles.append(p)
    
    self.particles = new_particles

  def move_to_waypoint(self, x, y, interval=0, draw=False):
    delta_y = y - self.y
    delta_x = x - self.x

    if delta_y == 0:
      delta_y = sys.float_info.epsilon    

    beta = math.atan(delta_x / delta_y)
    turn_angle = -self.theta + math.pi/2 - beta
    move_distance = math.sqrt(delta_x ** 2 + delta_y ** 2) 
    if delta_y/math.cos(beta) < 0:
      turn_angle = turn_angle - math.pi

    self.rotate(turn_angle, draw)  
    if interval > 0:
     if move_distance > 1:
        self.move(min(interval, move_distance)) 
        self.move_to_waypoint(x,y,interval,draw)
    else: 
      self.move(move_distance, draw)

  # move methods END

  '''Clears rotation commands on all motors.'''
  def reset_motors(self, reset_left=True, reset_right=True, reset_head=False): 
   if reset_left:
     self.interface.motorDisable(params.MOTOR_LEFT)
     self.interface.motorEnable(params.MOTOR_LEFT)
   if reset_right:
     self.interface.motorDisable(params.MOTOR_RIGHT)
     self.interface.motorEnable(params.MOTOR_RIGHT)
   if reset_head:
     self.interface.motorDisable(params.MOTOR_HEAD)
     self.interface.motorEnable(params.MOTOR_HEAD) 
 
  def calculate_likelihood(self, walls, particle, z):
    self.LIKELIHOOD_OFFSET = 0.5
    wall, m, angle = walls.which_wall(particle)
    if angle > 15 * math.pi / 180:
      return 1, False
    else:
      return self.LIKELIHOOD_OFFSET + math.exp(-((z[0] - m) ** 2) / (2 * (self.sonar_sigma ** 2))), True
