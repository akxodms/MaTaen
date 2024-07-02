# stacking_block

import rospy
import math as mt
import numpy as np
from std_msgs.msg import Int32, String, Float32
from std_msgs.msg import Float32MultiArray as fl
import time

def dtr(dgree):
   return dgree*(np.pi/180)

####################################################################################################################################
# about about position setting
init_position = [145, 0, 200]
current_postion = [0,0,0]

open_gripper = 345
close_gripper = 280
############################################
# about period
sleep_time = 0.5
traj_time = 2 # tf

############################################
# about block_info
block_size = 25

block_gap_diff = 2.5
p2_block_gap_diff = 2.5

block_gap = block_size + block_gap_diff
p2_block_gap = block_size + p2_block_gap_diff

blcok_gap_height = block_size

############################################
# about block_position_info
get_block_angle = 45
get_block_distance = 200

p1_stack_block_angle = 45
p1_stack_block_distance = 250

p2_stack_block_angle = 0
p2_stack_block_distance = 250


height = 73
pick_height = 73
p2_height = 73
above = 50
back = 20
############################################
### about movement
## get_block
get_block_pos = np.zeros((2,3))
get_block_pos[0] = [get_block_distance*mt.cos(dtr(get_block_angle)),get_block_distance*mt.sin(dtr(get_block_angle)),pick_height+above]
get_block_pos[1] = [get_block_distance*mt.cos(dtr(get_block_angle)),get_block_distance*mt.sin(dtr(get_block_angle)),pick_height]

############################################
## stack block
p1_stack_block_pos = np.zeros((10, 3))

# first block
p1_stack_block_pos[0] = [p1_stack_block_distance*mt.cos(dtr(p1_stack_block_angle)),-p1_stack_block_distance*mt.sin(dtr(p1_stack_block_angle)),height]
# floor-1
for i in range(1,4):
    p1_stack_block_pos[i][0] = p1_stack_block_pos[i-1][0] - block_gap*mt.cos(dtr(p1_stack_block_angle)) 
    p1_stack_block_pos[i][1] = p1_stack_block_pos[i-1][1] + block_gap*mt.sin(dtr(p1_stack_block_angle))
    p1_stack_block_pos[i][2] = p1_stack_block_pos[i-1][2]

# floor-2
p1_stack_block_distance_2 = p1_stack_block_distance-(block_size/2)
p1_stack_block_pos[4] = [p1_stack_block_distance_2*mt.cos(dtr(p1_stack_block_angle)),-p1_stack_block_distance_2*mt.sin(dtr(p1_stack_block_angle)),height+blcok_gap_height]

for i in range(5,7):
    p1_stack_block_pos[i][0] = p1_stack_block_pos[i-1][0] - block_gap*mt.cos(dtr(p1_stack_block_angle)) 
    p1_stack_block_pos[i][1] = p1_stack_block_pos[i-1][1] + block_gap*mt.sin(dtr(p1_stack_block_angle))
    p1_stack_block_pos[i][2] = p1_stack_block_pos[i-1][2]

# floor-3
p1_stack_block_distance_3 = p1_stack_block_distance_2-(block_size/2)
p1_stack_block_pos[7] = [p1_stack_block_distance_3*mt.cos(dtr(p1_stack_block_angle)),-p1_stack_block_distance_3*mt.sin(dtr(p1_stack_block_angle)),height+blcok_gap_height*2]

for i in range(8, 9):
    p1_stack_block_pos[i][0] = p1_stack_block_pos[i-1][0] - block_gap*mt.cos(dtr(p1_stack_block_angle)) 
    p1_stack_block_pos[i][1] = p1_stack_block_pos[i-1][1] + block_gap*mt.sin(dtr(p1_stack_block_angle))
    p1_stack_block_pos[i][2] = p1_stack_block_pos[i-1][2]

# floor-4
p1_stack_block_distance_4 = p1_stack_block_distance_3-(block_size/2)
p1_stack_block_pos[9] = [p1_stack_block_distance_4*mt.cos(dtr(p1_stack_block_angle)),-p1_stack_block_distance_4*mt.sin(dtr(p1_stack_block_angle)),height+blcok_gap_height*3]

# value
p1_stack_block_pos[0][2] = p1_stack_block_pos[0][2] + 5
p1_stack_block_pos[1][2] = p1_stack_block_pos[1][2] + 2
p1_stack_block_pos[2][2] = p1_stack_block_pos[2][2] + 0
p1_stack_block_pos[3][2] = p1_stack_block_pos[3][2] -3

p1_stack_block_pos[4][2] = p1_stack_block_pos[4][2] + 2
p1_stack_block_pos[5][2] = p1_stack_block_pos[5][2] + 0
p1_stack_block_pos[6][2] = p1_stack_block_pos[6][2] - 1

############################################
## stack block_2
p2_stack_block_pos = np.zeros((10, 3))

# first block
p2_stack_block_pos[0] = [p2_stack_block_distance*mt.cos(dtr(p2_stack_block_angle)),-p2_stack_block_distance*mt.sin(dtr(p2_stack_block_angle)),p2_height]
# floor-1
for i in range(1,4):
    p2_stack_block_pos[i][0] = p2_stack_block_pos[i-1][0] - p2_block_gap*mt.cos(dtr(p2_stack_block_angle)) 
    p2_stack_block_pos[i][1] = p2_stack_block_pos[i-1][1] + p2_block_gap*mt.sin(dtr(p2_stack_block_angle))
    p2_stack_block_pos[i][2] = p2_stack_block_pos[i-1][2]

# floor-2
p2_stack_block_distance_2 = p2_stack_block_distance-(block_size/2)
p2_stack_block_pos[4] = [p2_stack_block_distance_2*mt.cos(dtr(p2_stack_block_angle)),-p2_stack_block_distance_2*mt.sin(dtr(p2_stack_block_angle)),p2_height+blcok_gap_height]

for i in range(5,7):
    p2_stack_block_pos[i][0] = p2_stack_block_pos[i-1][0] - p2_block_gap*mt.cos(dtr(p2_stack_block_angle)) 
    p2_stack_block_pos[i][1] = p2_stack_block_pos[i-1][1] + p2_block_gap*mt.sin(dtr(p2_stack_block_angle))
    p2_stack_block_pos[i][2] = p2_stack_block_pos[i-1][2]

# floor-3
p2_stack_block_distance_3 = p2_stack_block_distance_2-(block_size/2)
p2_stack_block_pos[7] = [p2_stack_block_distance_3*mt.cos(dtr(p2_stack_block_angle)),-p2_stack_block_distance_3*mt.sin(dtr(p2_stack_block_angle)),p2_height+blcok_gap_height*2]

for i in range(8, 9):
    p2_stack_block_pos[i][0] = p2_stack_block_pos[i-1][0] - p2_block_gap*mt.cos(dtr(p2_stack_block_angle)) 
    p2_stack_block_pos[i][1] = p2_stack_block_pos[i-1][1] + p2_block_gap*mt.sin(dtr(p2_stack_block_angle))
    p2_stack_block_pos[i][2] = p2_stack_block_pos[i-1][2]

# floor-4
p2_stack_block_distance_4 = p2_stack_block_distance_3-(block_size/2)
p2_stack_block_pos[9] = [p2_stack_block_distance_4*mt.cos(dtr(p2_stack_block_angle)),-p2_stack_block_distance_4*mt.sin(dtr(p2_stack_block_angle)),p2_height+blcok_gap_height*3]

# value
p2_stack_block_pos[0][2] = p2_stack_block_pos[0][2] + 7
p2_stack_block_pos[1][2] = p2_stack_block_pos[1][2] + 3
p2_stack_block_pos[2][2] = p2_stack_block_pos[2][2] + 0
p2_stack_block_pos[3][2] = p2_stack_block_pos[3][2] -3

p2_stack_block_pos[4][2] = p2_stack_block_pos[4][2] + 3
p2_stack_block_pos[5][2] = p2_stack_block_pos[5][2] + 1
p2_stack_block_pos[6][2] = p2_stack_block_pos[6][2] - 1

p2_stack_block_pos[7][2] = p2_stack_block_pos[7][2] + 1
p2_stack_block_pos[8][2] = p2_stack_block_pos[8][2] + 0

p2_stack_block_pos[9][2] = p2_stack_block_pos[9][2] + 0

####################################################################################################################################
# functions

def get_block():
    pub_traj(get_block_pos[0])
    pub_traj(get_block_pos[1],1)
    pub_gripper(close_gripper)
    pub_traj(get_block_pos[0],1)
    return get_block_pos[0]

def stack_block(end):
    end_above = [end[0]- back*mt.cos(dtr(p1_stack_block_angle)), end[1]+back*mt.sin(dtr(p1_stack_block_angle)), end[2]+above]
    pub_traj(end_above)
    pub_traj(end)
    pub_gripper(open_gripper)
    pub_traj(end_above,1)
    return end_above

def change_block(start, end):
    global current_postion
    if current_postion[2] <= height+(block_size*3):
        first_postion = [current_postion[0], current_postion[1], height+(block_size*4)]
        pub_traj(first_postion, 1)
    start_above = [start[0]- back*mt.cos(dtr(p1_stack_block_angle)), start[1]+back*mt.sin(dtr(p1_stack_block_angle)), start[2]+above]    
    end_above = [end[0]- back*mt.cos(dtr(p2_stack_block_angle)), end[1]+back*mt.sin(dtr(p2_stack_block_angle)), end[2]+above]
    pub_traj(start_above)
    pub_traj(start)
    pub_gripper(close_gripper)
    start_above_2 = [start[0]- back*mt.cos(dtr(p1_stack_block_angle)), start[1]+back*mt.sin(dtr(p1_stack_block_angle)), height+(block_size*4)]
    pub_traj(start_above_2)
    pub_traj(end_above)
    pub_traj(end)
    pub_gripper(open_gripper)
    pub_traj(end_above, 1)

def main():
    rospy.init_node('stacking_block', anonymous=True)
    # 초기 위치로 이동
    rospy.loginfo("move to init_position")
    pub_position(init_position)
    pub_gripper(open_gripper)
    rospy.sleep(sleep_time) 


    # get_block  
    for i in range(0,10):
        get_block()
        stack_block(p1_stack_block_pos[i])
    rospy.sleep(1)
    
    # chanege_block
    for i in range(0,10):
        change_block(p1_stack_block_pos[9-i], p2_stack_block_pos[i])
        
    pub_traj(init_position)    



def pub_position(position):
    msg = fl()
    global current_postion
    msg.data = [position[0], position[1], position[2]] 
    pub_positions.publish(msg)
    current_postion = position
    rospy.loginfo("pub position is %.2f %.2f %.2f", *position)

    
# trajectory pub(two position pub : start, end) / topic : trajectory
def pub_traj(goal, tf = traj_time):
    msg = fl()
    global current_postion
    traj_pos = [current_postion[0], current_postion[1], current_postion[2], goal[0], goal[1], goal[2], tf]  # 시작점과 끝점 좌표를 하나의 리스트에 넣음
    msg.data = traj_pos
    pub_trajectory.publish(msg)
    rospy.loginfo("start : %.2f %.2f %.2f, end : %.2f %.2f %.2f", *current_postion, *goal)
    current_postion = goal
    if tf == 1:
        local_sleep_time = tf + sleep_time*0.7
    else:
        local_sleep_time = tf + sleep_time
    rospy.sleep(local_sleep_time) 

def pub_gripper(angle):
    pub_gripper_angle.publish(angle)

if __name__ == '__main__':
    try:
        pub_positions = rospy.Publisher('position', fl, queue_size=10) 
        pub_trajectory = rospy.Publisher('trajectory', fl, queue_size=10)
        pub_gripper_angle = rospy.Publisher('gripper', Float32, queue_size=10)
        main()

    except rospy.ROSInterruptException:
        print('program is shut downed')    