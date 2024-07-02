#!/usr/bin/env python

import rospy, math
import numpy as np
from std_msgs.msg import Int32, String, Float32
from std_msgs.msg import Float32MultiArray as fl
import matplotlib.pyplot as plt
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib

# matplotlib.use('Agg')

def dtr(dgree):
   return dgree*(np.pi/180)

class chain:
  def __init__(self):
    self.make_chain()
    
  def make_chain(self):
    # 4-DOF robot arm define
    self.arm = Chain(name='arm', links=[
        OriginLink(), # base
        URDFLink(
          name="link_1", # XM430
          origin_translation=[0, 0, 48.8], 
          origin_orientation=[0, 0, 0],
          rotation=[0, 0, 1],
        ),
        URDFLink( # AX-12
          name="link_2",
          origin_translation=[0, 0, 53.7], 
          origin_orientation=[dtr(90), 0, 0],
          rotation=[0, 0, 1],
        ),
        URDFLink( # AX-18(1)
          name="link_3",
          origin_translation=[166.5, 0, 0],
          origin_orientation=[0, 0, 0],
          rotation=[0, 0, 1],
        ),
        URDFLink( # AX-18(2)
          name="link_4",
          origin_translation=[166.5, 0, 0],
          origin_orientation=[0, 0, 0],
          rotation=[0, 0, 1], # not angle, just 0 or 1 / 1 means the axis of rotating
        ),
        URDFLink( # end point
          name="link_5",
          origin_translation=[63, 0, 0], # 
          origin_orientation=[dtr(-90), 0, 0],
          rotation=[0, 0, 0], # no rotation(no dof)
        )        
    ], active_links_mask=[False, True, True, True, True, False] # 4-True = four moving axis = 4-DOF
    )

  def plot_original_state(self):
    ax = plt.figure().add_subplot(111, projection='3d')
    plt.xlabel('x')
    plt.ylabel('y')
    self.arm.plot([0,0,0,0,0,0], ax) # there are 6-Links, but only 4-links has a DOF
    plt.show()
    # in end-point, green axis : x(n) / blue axis : y(o) / orange axis : z(a)

  def IK(self, target_position):
    # inverse kinematics
    angle = self.arm.inverse_kinematics(target_position, target_orientation=[0, 0, -1], orientation_mode="X")
    # self.plot(angle)
    new_angle = np.zeros(4)
    for i in range(4):
      new_angle[i] = angle[i+1]

    print(np.round(np.rad2deg(angle), 3))

    return np.rad2deg(new_angle)

  # def plot(self, angle):
  #   fig = plt.figure()
  #   ax = plt.add_subplot(111, projection='3d')  
  #   plt.xlabel('x')
  #   plt.ylabel('y')
  #   plt.ylabel('z')
    
  #   self.arm.plot(angle, ax) # there are 6-Links, but only 4-links has a DOF
  #   plt.savefig()
  #   plt.close(fig)


def main():
    rospy.init_node('Node1', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        action()
        
        try:
            position = list(map(float, position.split()))
            msg = fl()
            msg.data = position
            pub_position.publish(msg)
        except ValueError:
            rospy.logwarn("please try again")
        rate.sleep()
def action():
  position = input("Enter the goal positon(x, y, z) : ")   


class action_list():
  def __init__(self):
     arm = chain() # arm is chain class

  def point_move(self, position):
    
  
  def joint_traj():
   

  def move  


def pub_angle(angle):
  pub_angle = rospy.Publisher('angle', fl, queue_size=10)
  msg = fl()
  msg.data = angle
  # angle_type = type(angle)
  # print(angle_type)
  pub_angle.publish(msg)


if __name__ == '__main__':
    try:
        rospy.logwarn("get_angle Node is on")
        main()
    except rospy.ROSInterruptException:
        print('program is shut downed')