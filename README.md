persuit_evasion_game
==
This repo uses pygame simulation and gazebo simulation to realize the [algorithm](https://ieeexplore.ieee.org/document/7525229)
<br>

pygame simulation
--
The python script _persuit_evasion_game.py_ can show the game in a 500x700 window.
<br> In the code, there is a boolean variable called _UEflag_. If you turn this bool to 0, it is just an ordinary persuit evasion game with out uncertainty estimatin; if you turn it to 1, it will estimate the maximum ability of pursuer first and initialize the game.
<br>
<br> When you turn this boolean to 0, you can change the variable _ability_pursuer_ to change the ability of velocity(in this scenario, the velocity of pursuer). The ability of evader is 10.
<br>
<br>
**When UEflag is set to be 0**:

![image](https://github.com/Shicheng-Liu/persuit_evasion_game/blob/master/pursuer_ability_5.jpg)

In this fig, the ability of pursuer is 5(which is quite low), so the evader can reach the target.
<br>

![image](https://github.com/Shicheng-Liu/persuit_evasion_game/blob/master/pursuer_ability_10.jpg)

In this fig, the ability of pursuer is 10(which is high) so that the evader cannot reach the goal.
<br>
<br>
**When UEflag is set to be 1**:

![image](https://github.com/Shicheng-Liu/persuit_evasion_game/blob/master/uncertainty_estimation.jpg)

The estimated ability is 6.5. In this situation, the evader can reach the goal.

![image](https://github.com/Shicheng-Liu/persuit-evasion-game/blob/master/estimation_curve.png)

Gazebo simulation
--
To implement this algorithm on drones, we need drone simulation to prove it. This simulator is based on _hectquadrotor_.
<br> This simulation is tested on Ubuntu16 ROS kinetic
<br>
<br> First, clone this repository into an empty folder:
```
git clone https://github.com/Shicheng-Liu/persuit_evasion_game.git
```
Then change the name of this repo(you just download as a folder) to src, and then:
```
catkin_make
```
If you find that there an error about gazebo-ros-controller, you need to install it first.
<br>
<br>
Now, we have built our environment and we can work on the simulation now. Oh, don't forget to source this workspace first!
<br>
Open a terminal:
```
roslaunch hector_quadrotor_gazebo nrsl_two_drones.launch
```
Now, you can our simulator and then open another terminal:
```
rosrun hector_quadrotor_gazebo persuit_evasion.py
```
When a Qt video window shows, type the key _p_ and you can see the performance.

![image](https://github.com/Shicheng-Liu/persuit_evasion_game/blob/master/persuit-evasion-game.gif)
