import sys, random, math, pygame, time
from pygame.locals import *
from math import *

length=500
width=700
ability_evader=10
vertex_number=5000
capture_distance=10

white = 255, 255, 255
black = 0, 0, 0
red = 255, 0, 0
green = 0, 255, 0
blue = 0, 0, 255
pink=200, 20, 240

UEflag=0

pygame.init()
screen = pygame.display.set_mode((length,width))
pygame.display.set_caption('persuit-evasion game')

def distance(x1,x2):
  return sqrt((x1[0]-x2[0])*(x1[0]-x2[0])+(x1[1]-x2[1])*(x1[1]-x2[1]))  

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

def collision_test(new_vertex,vertex):    
    k=(new_vertex.y-vertex.y)/(new_vertex.x-vertex.x)
    if (new_vertex.x<280 and new_vertex.y>400 and new_vertex.y<420) or (new_vertex.x>220 and new_vertex.y>250 and new_vertex.y<270):
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-400)*(vertex.y-400)<0 and (new_vertex.x-280)*(vertex.x-280)<0 and vertex.y+k*(280-vertex.x)>400:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-420)*(vertex.y-420)<0 and (new_vertex.x-280)*(vertex.x-280)<0 and vertex.y+k*(280-vertex.x)<420:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-250)*(vertex.y-250)<0 and (new_vertex.x-220)*(vertex.x-220)<0 and vertex.y+k*(220-vertex.x)>250:
      print("delete this new vertex because of collision")
      return 0
    elif (new_vertex.y-270)*(vertex.y-270)<0 and (new_vertex.x-220)*(vertex.x-220)<0 and vertex.y+k*(220-vertex.x)<270:
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

def path_choose(tree, evader, target):
    vertex=tree[0]
    path=[]
    for x in tree:
      if distance([x.x,x.y],[target.x,target.y])<distance([vertex.x,vertex.y],[target.x,target.y]):
        vertex=x
    while vertex!=evader:
      path.append(vertex)
      pygame.draw.line(screen,blue,[vertex.x,vertex.y],[vertex.last.x,vertex.last.y],5)
      vertex=vertex.last
      pygame.display.flip()
    return path

def pursuer_initialization():
    t=0
    state_estimation_x=30
    state_estimation_y=460
    last_state_estimation_x=30
    last_state_estimation_y=460
    r1=0
    last_state_measurement_x=30
    last_state_measurement_y=460
    pursuer=30,460
    w_x=0
    w_y=0
    i=1
    while t<35:
      print("number : %d") %i
      i=i+1
      pursuer_velocity= -3*(1-exp(-t/2))*sin(0.6*t), 6.5*(1-exp(-t/3))*cos(0.6*t)
      pursuer=pursuer[0]+pursuer_velocity[0]*0.01, pursuer[1]+pursuer_velocity[1]*0.01
      t=t+0.01
      if t>0.01:
        state_measurement_x=pursuer[0]
        state_measurement_y=pursuer[1]
        state_difference_x=-0.05*(last_state_estimation_x-last_state_measurement_x)+w_x
        state_difference_y=-0.05*(last_state_estimation_y-last_state_measurement_y)+w_y
        #print("w : %f      state_difference : %f") %(w,state_difference)
        state_estimation_x=last_state_estimation_x+state_difference_x*0.01
        state_estimation_y=last_state_estimation_y+state_difference_y*0.01
        #print("state_estimation: %f   state_measurement: %f") %(state_estimation,state_measurement)
        w_x=-(0.05/(exp(0.0005)-1))*(state_estimation_x-state_measurement_x)
        w_y=-(0.05/(exp(0.0005)-1))*(state_estimation_y-state_measurement_y)
        w=sqrt(w_x*w_x+w_y*w_y)
        last_state_estimation_x=state_estimation_x     
        last_state_measurement_x=state_measurement_x
        last_state_estimation_y=state_estimation_y    
        last_state_measurement_y=state_measurement_y
        #print("last_state_estimation : %f     last_state_measurement : %f") %(last_state_estimation,last_state_measurement)
        if w+0.091>10:
          r_1=10
        else:
          r_1=w+0.091
        if r_1>r1:
          r1=r_1
        else:
          r1=r1
        print("The bound: %f") %r1
        #pursuer=pursuer_last[0]+pursuer_velocity[0]*0.01, pursuer_last[1]+pursuer_velocity[1]*0.01
        pygame.draw.line(screen,pink,pursuer_last,pursuer,3)
        pygame.display.flip()
        #time.sleep(0.01)
      pursuer_last=pursuer
    return r1

def main():
    screen.fill(white)
    pygame.draw.circle(screen,red,(250,150),10,0)
    pygame.draw.circle(screen,green,(450,600),10,0)
    pygame.draw.rect(screen, (0,0,0), (0,400,280,20), 0)
    pygame.draw.rect(screen, (0,0,0), (220,250,280,20), 0)  
    pygame.display.flip()
    if UEflag==0:
      pygame.draw.circle(screen,pink,(30,460),5,0)
    else:
      r1=pursuer_initialization()
    tree=[]
    tree_1=[]
    tree.append(point(250,150))
    evader=tree[0]
    tree_1.append(point(30,460))
    pursuer=tree_1[0]
    target=point(450,600)
    if UEflag==1:
      ability_pursuer=r1
    else:
      ability_pursuer=5
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
      pygame.draw.circle(screen,red,(250,150),10,0)
      pygame.draw.circle(screen,green,(450,600),10,0)
      pygame.draw.rect(screen, (0,0,0), (0,400,280,20), 0)
      pygame.draw.rect(screen, (0,0,0), (220,250,280,20), 0)  
      pygame.display.flip()
    path=path_choose(tree,evader,target)
    m=0                                                   # The pursuer catching stategy: 
    for i in path:                                        # avaible positions: 1. within 100 of the evader's path   2. the time cost of pursuer is less than that of evader
      for x in tree_1:                                    # find the nearest one from target among all the available positions
        if x.time<i.time and abs(x.time-i.time)<2 and distance([x.x,x.y],[i.x,i.y])<100:
          vertex=x
          m=1
          while vertex!=pursuer:
            pygame.draw.line(screen,pink,[vertex.x,vertex.y],[vertex.last.x,vertex.last.y],5)
            vertex=vertex.last
            pygame.display.flip()
          break
      if m==1:
        break

    if m==0:
      print("It is too hard for pursuer to get close to evader. To save energy, the purser will not try to catch")     # if the pursuer cannot find a near enough positon, the pursuer will give up catching process
        
          
        
 

if __name__ == '__main__':
    main()
    running = True
    while running:
       for event in pygame.event.get():
	    if event.type == pygame.QUIT:
               running = False


