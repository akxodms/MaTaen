# task 1
## 196 0 45
import rospy
import math as mt
import numpy as np
from std_msgs.msg import Int32, String, Float32
from std_msgs.msg import Float32MultiArray as fl
import time


#####################################################################################################################################################################################
## point input ##
# draw line
end_plus = 10
start_position = [40+4-2-1+1,45]
end_position = [100-2-1,-60]

# grad = (end_position[1] - start_position[1])/(end_position[0] - start_position[0])
# add_line_init = 5
# add_line_end = 10 
# start_position = [109+add_line_init,-34]
# end_position = [75+add_line_end, 50]

# start_position = [75, 50]
# end_position = [109,-34]
# draw circle
center = [78, -10]
radius = 39+1

## 파라미터 ##
# delay
# time_dif = 0.05
# rate_draw = 1 / time_dif
rv = 15
sleep_time = 2
sleep_time_draw = 1
traj_time = 3 # tf


# draw
circle_point_num = 220
line_point_num = 100

above = 40
offset = 137.5

init_position = [145, 0, offset+above+20]

## point renewer ##
# draw line
x_off = init_position[0]
start_position[0] += x_off
end_position[0] += x_off
line_x_minus = 2
# draw circle
center[0] += x_off


#####################################################################################################################################################################################
# 함수들

# 중심점, 반지름 -> point_num 개의 x,y 좌표
def make_circle():
    theta = np.linspace(0, 2 * np.pi, circle_point_num)
    x = center[0] + radius * np.cos(theta)
    y = center[1] - radius * np.sin(theta)
    z = np.zeros(circle_point_num) + offset
    return x, y, z

def make_line(start_extension=2, end_extension=3):
    # 시작점과 끝점의 차이로 방향 벡터를 계산합니다.
    direction_vector = np.array(end_position) - np.array(start_position)
    # 방향 벡터의 크기를 계산합니다.
    direction_length = np.linalg.norm(direction_vector)
    # 방향 벡터를 단위 벡터로 만듭니다.
    unit_vector = direction_vector / direction_length
    
    # 시작점과 끝점을 각각 지정된 길이만큼 연장합니다.
    new_start = np.array(start_position) - unit_vector * start_extension
    new_end = np.array(end_position) + unit_vector * end_extension
    
    # linspace를 사용하여 x, y 좌표를 생성합니다.
    x = np.linspace(new_start[0], new_end[0], line_point_num) - line_x_minus
    y = np.linspace(new_start[1], new_end[1], line_point_num)
    # z 좌표는 일정한 offset 값으로 설정합니다.
    z = np.zeros(line_point_num) + offset
    
    return x, y, z


#####################################################################################################################################################################################
# main

def main():
    rospy.init_node('task1', anonymous=True)
    rate_cir = rospy.Rate(13)
    rate = rospy.Rate(rv)
    # 초기 위치로 이동
    rospy.loginfo("move to init_position")
    pub_position(init_position)
    rospy.sleep(sleep_time)

    # 원 좌표 계산
    x, y, z = make_circle()

    # 원의 첫 점 위로 이동
    rospy.loginfo("move to first circle point above")    
    position_0 = [x[0], y[0], z[0] + above]
    pub_traj(init_position, position_0)
    rospy.sleep(traj_time)

    # 원의 첫 점으로 이동
    rospy.loginfo("move to first circle point")        
    position_1 = [x[0], y[0], z[0]]
    pub_traj(position_0, position_1)
    rospy.sleep(traj_time)

    # 원 그리기
    circle_position = [None] * circle_point_num
    rospy.loginfo("draw_circle")  
    for i in range(circle_point_num):
        circle_position[i] = [x[i], y[i], z[i]]        
        pub_position(circle_position[i])    
        rate_cir.sleep()
    add_num = int(circle_point_num/36)

    for i in range(0, add_num):
        circle_position[i] = [x[i], y[i], z[i]]
        pub_position(circle_position[i])        
        rate.sleep()

    rospy.sleep(sleep_time_draw)
    
    # 원의 끝 점 위로 이동  ## delete
    rospy.loginfo("move to last circle point above")    
    position_2 = [x[add_num-1], y[add_num-1], z[add_num-1] + above]
    pub_traj(circle_position[add_num-1], position_2)
    rospy.sleep(traj_time)

    # 직선의 첫 점 위로 이동
    rospy.loginfo("move to first line point above")
    x2, y2, z2 = make_line()

    position_3 = [x2[0], y2[0], z2[0] + above+30]
    pub_traj(position_2, position_3)
    rospy.sleep(traj_time)

    # 직선의 첫 점으로 이동
    rospy.loginfo("move to first line point")        
    position_4 = [x2[0], y2[0], z2[0]]
    pub_traj(position_3, position_4)
    rospy.sleep(traj_time)

    # 직선 그리기
    line_position = [None] * line_point_num

    rospy.loginfo("draw_line")  
    for i in range(line_point_num): 
        line_position[i] = [x2[i], y2[i], z2[i]]
        pub_position(line_position[i])
        rate.sleep()
    rospy.sleep(sleep_time_draw)

    # 직선의 끝 점 위로 이동
    rospy.loginfo("move to last line point above")    
    position_5 = [x2[line_point_num-1], y2[line_point_num-1], z2[line_point_num-1] + above]
    pub_traj(line_position[line_point_num-1], position_5)
    rospy.sleep(sleep_time)
    
    rospy.loginfo("take1 is finished")
    rospy.sleep(sleep_time)




#####################################################################################################################################################################################
# ros topic

# 위치 1개 pub / topic : position
def pub_position(position):
    msg = fl()
    msg.data = [position[0], position[1], position[2]] 
    pub_positions.publish(msg)
    rospy.loginfo("pub position is %.2f %.2f %.2f", *position)

    
# trajectory pub(two position pub : start, end) / topic : trajectory
def pub_traj(start, end):
    msg = fl()
    traj_pos = [start[0], start[1], start[2], end[0], end[1], end[2]]  # 시작점과 끝점 좌표를 하나의 리스트에 넣음
    msg.data = traj_pos
    pub_trajectory.publish(msg)
    rospy.loginfo("start : %.2f %.2f %.2f, end : %.2f %.2f %.2f", *start, *end)


if __name__ == '__main__':
    try:
        pub_positions = rospy.Publisher('position', fl, queue_size=10) 
        pub_trajectory = rospy.Publisher('trajectory', fl, queue_size=10)
        main()

    except rospy.ROSInterruptException:
        print('program is shut downed')

        # cycle = 0
        # if (i > circle_point_num*0.55) & (i < circle_point_num*0.65):
        #     if cycle < 15 :
        #         cycle += 1     
        #     else :
        #         cycle -= 1      
        #     circle_position[i] = [x[i]+5/15*cycle, y[i], z[i]]
        # else:
        #     circle_position[i] = [x[i], y[i], z[i]]