#!/usr/bin/env python

# This work is based on ardrone_tutorial https://github.com/mikehamer/ardrone_tutorials

import roslib
import rospy
import time
import sys, random, math, pygame
from pygame.locals import *
from math import *

# Load the DroneController class, which handles interactions with the drone, and the DroneVideoDisplay class, which handles video display
from drone_controller_gazebo import BasicDroneController
from drone_video_display import DroneVideoDisplay

# Finally the GUI libraries
from PySide import QtCore, QtGui

step_length=10
# Here we define the keyboard map for our controller (note that python has no enums, so we use a class)
class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_W
	PitchBackward    = QtCore.Qt.Key.Key_S
	RollLeft         = QtCore.Qt.Key.Key_A
	RollRight        = QtCore.Qt.Key.Key_D
	YawLeft          = QtCore.Qt.Key.Key_Q
	YawRight         = QtCore.Qt.Key.Key_E
	IncreaseAltitude = QtCore.Qt.Key.Key_Z
	DecreaseAltitude = QtCore.Qt.Key.Key_C
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
        Trajectory       = QtCore.Qt.Key.Key_T
        Persuit_evasion  = QtCore.Qt.Key.Key_P

def move1(x,y):
    drone1_pose=controller.drone1()
    drone1_x=drone1_pose[0]
    drone1_y=drone1_pose[1]
    xr=drone1_pose[3]
    yr=drone1_pose[4]
    zr=drone1_pose[5]
    w=drone1_pose[6]
    yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
    while abs(yaw)>0.05:
      drone1_pose=controller.drone1()
      xr=drone1_pose[3]
      yr=drone1_pose[4]
      zr=drone1_pose[5]
      w=drone1_pose[6]
      yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
      controller.SetCommand1(0,0,-2*yaw,0)
    while abs(drone1_x-x)>0.1 or abs(drone1_y-y)>0.1:
      drone1_pose=controller.drone1()
      drone1_x=drone1_pose[0]
      drone1_y=drone1_pose[1]
      xr=drone1_pose[3]
      yr=drone1_pose[4]
      zr=drone1_pose[5]
      w=drone1_pose[6]
      yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
      controller.SetCommand1(y-drone1_y, x-drone1_x, -2*yaw, 0)

def move2(x,y):
    drone2_pose=controller.drone2()
    drone2_x=drone2_pose[0]
    drone2_y=drone2_pose[1]
    xr=drone2_pose[3]
    yr=drone2_pose[4]
    zr=drone2_pose[5]
    w=drone2_pose[6]
    yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
    while abs(yaw)>0.05:
      drone2_pose=controller.drone2()
      xr=drone2_pose[3]
      yr=drone2_pose[4]
      zr=drone2_pose[5]
      w=drone2_pose[6]
      yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
      controller.SetCommand2(0,0,-2*yaw,0)
    while abs(drone2_x-x)>0.1 or abs(drone2_y-y)>0.1:
      drone2_pose=controller.drone2()
      drone2_x=drone2_pose[0]
      drone2_y=drone2_pose[1]
      xr=drone2_pose[3]
      yr=drone2_pose[4]
      zr=drone2_pose[5]
      w=drone2_pose[6]
      yaw=atan2(2*(w*zr+xr*yr), 1-2*(yr*yr+zr*zr))
      controller.SetCommand2(y-drone2_y, x-drone2_x, -2*yaw, 0)

length=400
width=700
screen = pygame.display.set_mode((length,width))
white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
green = 0, 255, 0
blue = 0, 0, 255
pink=200, 20, 240

def distance(x1,x2):
  return sqrt((x1[0]-x2[0])*(x1[0]-x2[0])+(x1[1]-x2[1])*(x1[1]-x2[1]))

def step(x1,x2):
  if distance(x1,x2)<=step_length:
    return x2
  else:
    angle=atan2(x2[1]-x1[1],x2[0]-x1[0])
    return x1[0]+step_length*cos(angle), x1[1]+step_length*sin(angle)  

class point:
    x=0
    y=0
    last=None
    cost=0
    time=0
    #next=[]
    #next_choice=None
    def __init__(self,x_value,y_value):
         self.x=x_value
         self.y=y_value

def choose_neighborhood(tree):
    count=len(tree)
    neighborhood_calculated=500*sqrt(log(count)/count)
    if neighborhood_calculated<18:
      return neighborhood_calculated
    else:
      return 18

def collision_test(new_vertex,vertex):    # a safe distance of 50 from obstacles
    k=(new_vertex.y-vertex.y)/(new_vertex.x-vertex.x)
    if (new_vertex.x<270 and new_vertex.y>150 and new_vertex.y<260) or (new_vertex.x>130 and new_vertex.y>400 and new_vertex.y<510):
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-150)*(vertex.y-150)<0 and (new_vertex.x-270)*(vertex.x-270)<0 and vertex.y+k*(270-vertex.x)>150:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-260)*(vertex.y-260)<0 and (new_vertex.x-270)*(vertex.x-270)<0 and vertex.y+k*(270-vertex.x)<260:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-400)*(vertex.y-400)<0 and (new_vertex.x-130)*(vertex.x-130)<0 and vertex.y+k*(130-vertex.x)>400:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-510)*(vertex.y-510)<0 and (new_vertex.x-130)*(vertex.x-130)<0 and vertex.y+k*(130-vertex.x)<510:
      print("delete this new vertex because of collision")
      return 0
    else:
      return 1

def extend(tree,vertex_random,ability):
    vertex=tree[0]
    for x in tree:
       if distance([x.x,x.y],[vertex_random.x,vertex_random.y])<distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y]):
         vertex=x
    if distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y])<ability:
      new_vertex=vertex_random
      time=distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y])/ability
    else:
      angle=atan2(vertex_random.y-vertex.y,vertex_random.x-vertex.x)
      new=vertex.x+ability*cos(angle), vertex.y+ability*sin(angle)
      new_vertex=point(new[0],new[1])
      time=1
    new_vertex.last=vertex
    #vertex.next_choice=(new_vertex)
    new_vertex.cost=vertex.cost+distance([x.x,x.y],[vertex_random.x,vertex_random.y])
    new_vertex.time=vertex.time+time
    neighborhood=choose_neighborhood(tree)
    sign_1=collision_test(new_vertex,new_vertex.last)
    if sign_1==1:
      for x in tree:
        if distance([x.x,x.y],[new_vertex.x,new_vertex.y])<neighborhood and x.cost+distance([x.x,x.y],[new_vertex.x,new_vertex.y])<new_vertex.cost:
          sign_2=collision_test(new_vertex,x)
          if sign_2==1:
            new_vertex.last=x
            #x.next_choice=new_vertex
            new_vertex.cost=x.cost+distance([x.x,x.y],[new_vertex.x,new_vertex.y])
            new_vertex.time=x.time+distance([x.x,x.y],[new_vertex.x,new_vertex.y])/ability
            #x.next.append(new_vertex) 
      tree.append(new_vertex)
      #new_vertex.last.next.append(new_vertex)
      pygame.draw.line(screen,black,[new_vertex.last.x,new_vertex.last.y],[new_vertex.x,new_vertex.y])
      pygame.display.flip()
      for i in xrange(len(tree)):
        x=tree[i]
        if x!=new_vertex.last and distance([x.x,x.y],[new_vertex.x,new_vertex.y])<neighborhood and distance([x.x,x.y],[new_vertex.x,new_vertex.y])+new_vertex.cost<x.cost:
          sign_3=collision_test(x,new_vertex)
          if sign_3==1:
            pygame.draw.line(screen,white,[x.x,x.y],[x.last.x,x.last.y])
            x.last=new_vertex
            #new_vertex.next.append(x)
            x.cost=distance([x.x,x.y],[new_vertex.x,new_vertex.y])+new_vertex.cost
            x.time=distance([x.x,x.y],[new_vertex.x,new_vertex.y])/ability+new_vertex.time
            tree[i]=x
            pygame.draw.line(screen,black,[x.x,x.y],[x.last.x,x.last.y])
            pygame.display.flip()
      return new_vertex

def extend_1(tree_1,vertex_random,ability):
    vertex=tree_1[0]
    for x in tree_1:
       if distance([x.x,x.y],[vertex_random.x,vertex_random.y])<distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y]):
         vertex=x
    if distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y])<ability:
      new_vertex=vertex_random
      time=distance([vertex.x,vertex.y],[vertex_random.x,vertex_random.y])/ability
    else:
      angle=atan2(vertex_random.y-vertex.y,vertex_random.x-vertex.x)
      new=vertex.x+ability*cos(angle), vertex.y+ability*sin(angle)
      new_vertex=point(new[0],new[1])
      time=1
    new_vertex.last=vertex
    new_vertex.cost=vertex.cost+distance([x.x,x.y],[vertex_random.x,vertex_random.y])
    new_vertex.time=vertex.time+time
    neighborhood=choose_neighborhood(tree_1)
    sign_1=collision_test(new_vertex,new_vertex.last)
    if sign_1==1:
      for x in tree_1:
        if distance([x.x,x.y],[new_vertex.x,new_vertex.y])<neighborhood and x.cost+distance([x.x,x.y],[new_vertex.x,new_vertex.y])<new_vertex.cost:
          sign_2=collision_test(new_vertex,x)
          if sign_2==1:
            new_vertex.last=x
            new_vertex.cost=x.cost+distance([x.x,x.y],[new_vertex.x,new_vertex.y])
            new_vertex.time=x.time+distance([x.x,x.y],[new_vertex.x,new_vertex.y])/ability 
      tree_1.append(new_vertex)
      #new_vertex.last.next.append(new_vertex)
      #pygame.draw.line(screen,pink,[new_vertex.last.x,new_vertex.last.y],[new_vertex.x,new_vertex.y])
      #pygame.display.flip()

      for i in xrange(len(tree_1)):
        x=tree_1[i]
        if x!=new_vertex.last and distance([x.x,x.y],[new_vertex.x,new_vertex.y])<neighborhood and distance([x.x,x.y],[new_vertex.x,new_vertex.y])+new_vertex.cost<x.cost:
          sign_3=collision_test(x,new_vertex)
          if sign_3==1:
            #pygame.draw.line(screen,white,[x.x,x.y],[x.last.x,x.last.y])
            x.last=new_vertex
            #new_vertex.next.append(x)
            x.cost=distance([x.x,x.y],[new_vertex.x,new_vertex.y])+new_vertex.cost
            x.time=distance([x.x,x.y],[new_vertex.x,new_vertex.y])/ability+new_vertex.time
            tree_1[i]=x
            #pygame.draw.line(screen,pink,[x.x,x.y],[x.last.x,x.last.y])
            #pygame.display.flip()
      return new_vertex

def persuit_evasion_game():
    length=400
    width=700
    vertex_number=2000
    white = 255, 255, 255
    black = 0, 0, 0
    red = 255, 0, 0
    green = 0, 255, 0
    blue = 0, 0, 255
    pink=200, 20, 240
    ability_evader=10
    capture_distance=10
    running=1
    while running:
      pygame.init()
      screen = pygame.display.set_mode((length,width))
      pygame.display.set_caption('persuit_evasion_game')
      screen.fill(white)
      pygame.draw.circle(screen,red,(200,50),10,0)
      pygame.draw.circle(screen,green,(0,600),10,0)
      pygame.draw.rect(screen, (0,0,0), (0,200,220,10), 0)
      pygame.draw.rect(screen, (0,0,0), (180,450,220,10), 0)  
      pygame.display.flip()
      drone1_pose=controller.drone1()
      drone1_x=drone1_pose[0]
      if drone1_x>1.5:
        move1(3,-1)
        move1(0,-1)
      if drone1_x>-1:
        move1(0,2)
      move1(-2.5,0)
      print("drone1 in position")
      controller.SetCommand1(0, 0, 0, 0)
      drone2_pose=controller.drone2()
      drone2_x=drone2_pose[0]
      if drone2_x<-1:
        move2(-2,1.5)
        move2(0,1.5)
      if drone2_x<1.5:
        move2(0,-2)
        move2(3,-2)
      move2(3,1.5)
      print("drone2 in position")
      controller.SetCommand2(0, 0, 0, 0)
      pygame.draw.circle(screen,pink,(350,600),5,0)
      tree=[]
      tree_1=[]
      tree.append(point(200,50))
      evader=tree[0]
      tree_1.append(point(350,600))
      pursuer=tree_1[0]
      target=point(0,600)
      ability_pursuer=3
      sign=1
      for i in range(vertex_number):
        print "iteration = %d" %sign
        sign=sign+1
        vertex_random=point(random.random()*length, random.random()*width)
        symbol=extend(tree,vertex_random,ability_evader)
        if symbol is not None:
          for x in tree_1:
            if distance([x.x,x.y],[symbol.x,symbol.y])<capture_distance:
              if x.time<symbol.time:
                del(tree[-1])
                pygame.draw.line(screen,white,[symbol.x,symbol.y],[symbol.last.x,symbol.last.y])
                pygame.display.flip()        
      #if UEflag==1:
      #  break
      #else:
        vertex_random_1=point(random.random()*length, random.random()*width)
        symbol_1=extend_1(tree_1,vertex_random_1,ability_pursuer)

        if symbol_1 is not None:
          for x in tree:
            if distance([x.x,x.y],[symbol_1.x,symbol_1.y])<capture_distance:
              if x.time>symbol_1.time:
                pygame.draw.line(screen,white,[x.x,x.y],[x.last.x,x.last.y])
                pygame.display.flip()
              #for y in x.next:
              #  pygame.draw.line(screen,white,[y.x,y.y],[x.x,x.y])
              #  pygame.display.flip()
                tree.remove(x)
              #print("dsadsad")
        pygame.draw.circle(screen,red,(200,50),10,0)
        pygame.draw.circle(screen,green,(0,600),10,0)
        pygame.draw.rect(screen, (0,0,0), (0,200,220,10), 0)
        pygame.draw.rect(screen, (0,0,0), (180,450,220,10), 0)  
        pygame.display.flip()
      vertex=tree[0]
      path_1=[]
      for x in tree:
        if distance([x.x,x.y],[target.x,target.y])<distance([vertex.x,vertex.y],[target.x,target.y]):
          vertex=x
      while vertex!=evader:
        path_1.append(vertex)
        pygame.draw.line(screen,blue,[vertex.x,vertex.y],[vertex.last.x,vertex.last.y],5)
        vertex=vertex.last
        pygame.display.flip()
      #select=0
      #for x in path_1[::-1]:
      #  select=select+1
      #  if select%3==0:
      #    x_goal=x.y/100-3
      #    y_goal=x.x/100-2
      #    move1(x_goal,y_goal)
      #controller.SetCommand1(0, 0, 0, 0)
      m=0    
      path_2=[]                                             # The pursuer catching stategy: 
      for i in path_1:                                      # avaible positions: 1. within 100 of the evader's path   2. the time cost of pursuer is less than that of evader
        for x in tree_1:                                    # find the nearest one from target among all the available positions
          if x.time<i.time and abs(x.time-i.time)<2 and distance([x.x,x.y],[i.x,i.y])<150:
            vertex=x
            m=1
            while vertex!=pursuer:
              path_2.append(vertex)
              pygame.draw.line(screen,pink,[vertex.x,vertex.y],[vertex.last.x,vertex.last.y],5)
              vertex=vertex.last
              pygame.display.flip()
            break
        if m==1:
          newpath_1=path_1[::-1]
          newpath_2=path_2[::-1]
          #for x in path_2[::-1]:
            #select=select+1
            #if select%3==0:
          #  x_goal=x.y/100-3
          #  y_goal=x.x/100-2
          #  move2(x_goal,y_goal)
          #controller.SetCommand2(0, 0, 0, 0)
          #break
          #newpath_1=sorted(path_1, reverse=True)
          #newpath_2=sorted(path_2, reverse=True)
          numbers1=len(newpath_1)  
          numbers2=len(newpath_2)
          i=0
          while i < numbers1:
            #if i<numbers2:
            x1_goal=newpath_1[i].y/100-3
            y1_goal=newpath_1[i].x/100-2
            move1(x1_goal,y1_goal)
            if i%5==0 and i/5<numbers2:
              x2_goal=newpath_2[i/5].y/100-3
              y2_goal=newpath_2[i/5].x/100-2
              move2(x2_goal,y2_goal)
              controller.SetCommand2(0, 0, 0, 0)
            i=i+1
            print(i)
          controller.SetCommand1(0, 0, 0, 0)
          controller.SetCommand2(0, 0, 0, 0)
          break
      if m==0:
        print("It is too hard for pursuer to get close to evader. To save energy, the purser will not try to catch")     # if the pursuer cannot find a near enough positon, the pursuer will give up catching process


# Our controller definition, note that we extend the DroneVideoDisplay class
class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

# We add a keyboard handler to the DroneVideoDisplay to react to keypresses
	def keyPressEvent(self, event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Handle the important cases first!
			if key == KeyMapping.Takeoff:
				controller.SetCommand1(0, 0, 0, 1)
				controller.SetCommand2(0, 0, 0, 1)
                                time.sleep(2)
                                controller.SetCommand1(0, 0, 0, 0)
				controller.SetCommand2(0, 0, 0, 0)
			elif key == KeyMapping.Land:
				controller.SetCommand1(0, 0, 0, -1)
				controller.SetCommand2(0, 0, 0, -1)
                                time.sleep(1.5)
                                controller.SetCommand1(0, 0, 0, 0)
				controller.SetCommand2(0, 0, 0, 0)
			else:
				# Now we handle moving, notice that this section is the opposite (+=) of the keyrelease section
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 1
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -1

				elif key == KeyMapping.PitchForward:
					self.pitch += 1
				elif key == KeyMapping.PitchBackward:
					self.pitch += -1

				elif key == KeyMapping.RollLeft:
					self.roll += 1
				elif key == KeyMapping.RollRight:
					self.roll += -1

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 1
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -1

                                elif key == KeyMapping.Trajectory:
                                        move1(-2,1)
                                        move2(3,0)
                                elif key == KeyMapping.Persuit_evasion:
                                        persuit_evasion_game()
			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand1(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

	def keyReleaseEvent(self,event):
		key = event.key()

		# If we have constructed the drone controller and the key is not generated from an auto-repeating key
		if controller is not None and not event.isAutoRepeat():
			# Note that we don't handle the release of emergency/takeoff/landing keys here, there is no need.
			# Now we handle moving, notice that this section is the opposite (-=) of the keypress section
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 1
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -1

			elif key == KeyMapping.PitchForward:
				self.pitch -= 1
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -1

			elif key == KeyMapping.RollLeft:
				self.roll -= 1
			elif key == KeyMapping.RollRight:
				self.roll -= -1

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 1
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -1

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			controller.SetCommand1(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)

# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('persuit_evasion_game')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = BasicDroneController()
	display = KeyboardController()
	
	display.show()
	
	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
