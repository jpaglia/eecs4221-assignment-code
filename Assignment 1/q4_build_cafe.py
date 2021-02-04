#!/usr/bin/python3
#
# A ros node to populate the world for a cafe simulation
#
import rospy
import os

def make_cafe(x, y):
  os.system(f"rosrun gazebo_ros spawn_model -database cafe -sdf -model cafe_main -x {x} -y {y}")

def make_table(id, x, y):
  os.system(f"rosrun gazebo_ros spawn_model -database cafe_table -sdf -model table_{id} -x {x} -y {y}")
  
def make_bookshelf(id, x, y, yaw):
  os.system(f"rosrun gazebo_ros spawn_model -database bookshelf -sdf -model shelf_{id} -x {x} -y {y} -Y {yaw}")
  
def make_cabinet(id, x, y):
  os.system(f"rosrun gazebo_ros spawn_model -database cabinet -sdf -model cabinet_{id} -x {x} -y {y}")
  
def make_robo_friend(id, x, y, yaw):
  os.system(f"rosrun gazebo_ros spawn_model -database husky -sdf -model husky_{id} -x {x} -y {y} -Y {yaw}")

if __name__ == '__main__':
  rospy.init_node('q4_build_cafe')
  
  make_cafe(0, 0)
  make_bookshelf(0, -5, -8.5, 1.57)
  make_cabinet(0, 2.34, 9)
  make_robo_friend(0, -2.55, 2, -0.3)
  
  id = 0
  for z in range(0,5):
    if (z < 3):
    	make_table(id, 3, (z) * (-4))
    	id = id + 1
    else:
    	make_table(id, 0, ((z%3)+1) * (-4))
    	id = id + 1
    	make_table(id, -3, ((z%3)+1) * (-4))
    	id = id + 1
    
  rospy.signal_shutdown("all done")
    

