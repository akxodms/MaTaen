#!/usr/bin/env python

import rospy
import math as mt
from std_msgs.msg import Int32, String, Float32
from std_msgs.msg import Float32MultiArray as fl
# total 4 topic is exist
# positon -> translate direction or rotaion
# if rotaion, ask the axis
# lastly, ask a value of transformation

def main():
    rospy.init_node('get_position', anonymous=True)
    pub_position = rospy.Publisher('position', fl, queue_size=10)
    rate = rospy.Rate(10) 

    while not rospy.is_shutdown():
        position = input("Enter the goal positon(x, y, z) : ") 
                
        try:
            position = list(map(float, position.split()))
            msg = fl()
            msg.data = position
            pub_position.publish(msg)
        except ValueError:
            rospy.logwarn("please try again")
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print('program is shut downed')