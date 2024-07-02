import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Int32, String, Float32
from geometry_msgs.msg import Point
#####################################################################################################################################################################################
# 기본적인 함수들

def dtr(dgree):
   return dgree*(np.pi/180)

#####################################################################################################################################################################################
# 파라미터

rv = 18
# parameter

# (time_divide / tf) per 1sec

# DH parameter
d = [90, 35.25, 0, 0, 0]
a = [0, 0, 158, 173, 124.25]
al = [0, dtr(-90), 0, 0, dtr(-90)]


DOF = 4



#####################################################################################################################################################################################
# chain 정의 및 IK 계산 클래스

class chain:
    def __init__(self):
        self.make_chain()

    def Make_URDF(self, link_name, d, a, al):
        return URDFLink(
            name = link_name,
            origin_translation=[a, 0, d], 
            origin_orientation=[al, 0, 0],
            rotation=[0, 0, 1],
        )
  # 4-DOF robot arm define
    def make_chain(self): 
            self.arm = Chain(name='arm', links=[
            OriginLink(), # base
            self.Make_URDF('link1', d[0], a[0], al[0]),
            self.Make_URDF('link2', d[1], a[1], al[1]),
            self.Make_URDF('link3', d[2], a[2], al[2]),
            self.Make_URDF('link4', d[3], a[3], al[3]),
            self.Make_URDF('link5', d[4], a[4], al[4])],
            active_links_mask=[False, True, True, True, True, False] )

  # IK 계산 매서드 / position -> angle
    def IK(self, target_position):
        angle = self.arm.inverse_kinematics(target_position, target_orientation=[0, 0, -1], orientation_mode="X")
        # orientation mode 를 "X"로 설정하기. EE의 green axis가 x축 이므로.
        self.angles = np.round(np.rad2deg(angle), 3)
        #print(self.angles)
        return self.angles
  
#####################################################################################################################################################################################
# 핵심 연산 관련 함수들 


def trajectory_generation(th_i, th_f, tf):
    tf = int(tf)
    time_divide = rv*tf

    th = np.zeros((time_divide, DOF))
    t = np.linspace(0, tf, time_divide)
    rate = rospy.Rate(rv)
    a0 = th_i
    a1 = np.zeros(DOF)
    a2 = (3 / tf**2) * (th_f - th_i)
    a3 = -(2 / tf**3) * (th_f - th_i)

    # Generate th array
    t_squared = t**2
    t_cubed = t**3

    # Reshape a0, a1, a2, a3 for broadcasting
    a0 = a0.reshape(1, -1)
    a1 = a1.reshape(1, -1)
    a2 = a2.reshape(1, -1)
    a3 = a3.reshape(1, -1)

    # Expand dimensions of t, t_squared, t_cubed for broadcasting
    t = t[:, np.newaxis]
    t_squared = t_squared[:, np.newaxis]
    t_cubed = t_cubed[:, np.newaxis]

    # Compute th
    th = a0 + a1 * t + a2 * t_squared + a3 * t_cubed
    
    for i in range(time_divide):
        pub_angle(th[i])
        rate.sleep()



#####################################################################################################################################################################################
# ros 관련 코드들

# ros main
def main():
    # node name = IK
    rospy.init_node('IK', anonymous=True)
    callback_method = callback()

    # callback_1 : Subscribe Topic : position / callback 함수 : cb_pos
    rospy.Subscriber("position", fl, callback_method.cb_pos)

    # callback_2 : Subscribe Topic : initial_position / callback 함수 : cb_traj
    rospy.Subscriber("trajectory", fl, callback_method.cb_traj)
    rospy.spin()

def pub_angle(angle):
    msg = fl()
    msg.data = angle  
    publisher_angle.publish(msg)
    rospy.loginfo("angle : %.2f %.2f %.2f %.2f", *angle)         

class callback:
    def __init__(self):
        self.arm = chain()

    def cb_pos(self, data):
        goal_position = np.array(data.data)
        rospy.loginfo("position : %.2f %.2f %.2f", *goal_position)
        angle = self.arm.IK(goal_position)
        angle = angle[1:5]
        pub_angle(angle)
        

    def cb_traj(self, data):        
        traj_pos = data.data
        rospy.loginfo("start : %.2f %.2f %.2f, end : %.2f %.2f %.2f", *traj_pos[:6])
        init_position = [traj_pos[0], traj_pos[1], traj_pos[2]]
        end_postion = [traj_pos[3], traj_pos[4], traj_pos[5]]
        tf = traj_pos[6]
        init_angle = self.arm.IK(init_position)
        end_angle = self.arm.IK(end_postion)

        init_angle = np.array(init_angle[1:5])
        end_angle = np.array(end_angle[1:5])
        trajectory_generation(init_angle, end_angle, tf)


#####################################################################################################################################################################################

if __name__ == '__main__':
    try:
        rospy.logwarn("IK Node is on")
        publisher_angle = rospy.Publisher('angle', fl, queue_size=10)  
        main()
    except rospy.ROSInterruptException:
        print('program is shut downed')