## angle -> move motort / angle(0,0,0,0) -> end point will be located in (396, 0, 102.5)

import os
import sys
import rospy
from dynamixel_sdk import *
import numpy as np
# 현재 스크립트의 디렉토리 경로를 가져옵니다.
module_directory = os.path.join(os.path.dirname(__file__), "DynamixelSDK/ros/dynamixel_sdk")
# sys.path 리스트에 모듈 디렉토리를 추가합니다.
sys.path.append(module_directory)
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.robotis_def import *
from std_msgs.msg import Float32MultiArray as fl
from std_msgs.msg import Float32

DEVICENAME = '/dev/ttyUSB0'

#*************************AX-12A(PROTOCOL_VERSION 1.0)*****************************#

# Control table address
AX_ADDR_TORQUE_ENABLE          = 24 # 토크 활성화(1)/비활성화(0)
AX_ADDR_CW_COMPLIANCE_MARGIN   = 26 # 시계방향 : Goal Position을 도달했다고 판단하는 Margin값, 예를 들어 Goal Position이 30이고 Margin값이 2라면 28~32에 도달하면 goal position에 도달한것으로 판단함
AX_ADDR_CCW_COMPLIANCE_MARGIN  = 27 # 반시계 방향 : ```
AX_ADDR_CW_COMPLIANCE_SLOPE    = 28 # 시계방향 : 가속/김속하는 시간
AX_ADDR_CCW_COMPLIANCE_SLOPE   = 29 # 반시계방향 : ```
AX_ADDR_GOAL_POSITION          = 30 # 목표 각도
AX_ADDR_MOVING_SPEED           = 32 # 목표 속도
AX_ADDR_PRESENT_POSITION       = 36 # 현재 각도
AX_ADDR_PRESENT_SPEED          = 38 # 현재 속도
AX_ADDR_PRESENT_LOAD           = 40
AX_ADDR_MOVING                 = 46
AX_ADDR_PUNCH                  = 48 # 모터에 가하는 최소 전류 -> 다르게 생각하면 최소 속도라고 할 수 있을 듯

AX_PROTOCOL_VERSION = 1.0

AX_DXL_ID = [2, 3]

AX_BAUDRATE = 115200

AX_TORQUE_ENABLE = 1
AX_TORQUE_DISABLE = 0

AX_CW_COMPLIANCE_MARGIN = 1 #실제로 설정하려는 값
AX_CCW_COMPLIANCE_MARGIN = 1
AX_CW_COMPLIANCE_SLOPE = 128
AX_CCW_COMPLIANCE_SLOPE = 128


AX_DXL_ADDR_PRESENT_POSITION = 36  # AX 모터의 현재 위치 값을 읽기 위한 레지스터 주소

#port_handler = PortHandler(DEVICENAME)
#ax_packet_handler = PacketHandler(AX_PROTOCOL_VERSION)

DXL_PRESENT_POSITION_LENGTH = 2  # 현재 위치 데이터의 길이 (2바이트)

#**********************XM430-W350-R(PROTOCOL_VERSION 2.0)**************************#

# Control table address

XM_ADDR_TORQUE_ENABLE           = 64
XM_ADDR_VELOCITY_I_GAIN         = 76
XM_ADDR_VELOCITY_P_GAIN         = 78
XM_ADDR_POTISION_D_GAIN         = 80
XM_ADDR_POSITION_I_GAIN         = 82
XM_ADDR_POSITION_P_GAIN         = 84
XM_ADDR_FEEDFORWARD_2ND_GAIN    = 88
XM_ADDR_FEEDFORWARD_1ST_GAIN    = 90
XM_ADDR_PROFILE_ACCELERATION    = 108
XM_ADDR_PROFILE_VELOCITY        = 112
XM_ADDR_GOAL_POSITION           = 116
XM_ADDR_MOVING                  = 122
XM_ADDR_MOVING_STATUS           = 123
XM_ADDR_PRESENT_POSITION        = 132
XM_ADDR_VELOCITY_LIMIT          = 44

XM_PROTOCOL_VERSION = 2.0

XM_DXL_ID = [0, 1]

XM_BAUDRATE = 115200


XM_TORQUE_ENABLE = 1
XM_TORQUE_DISABLE = 0

#xm_packet_handler = PacketHandler(XM_PROTOCOL_VERSION)

# XC330-T288 Control table address and protocol version
XC_ADDR_TORQUE_ENABLE = 64
XC_ADDR_GOAL_POSITION = 116
XC_ADDR_PRESENT_POSITION = 132

XC_PROTOCOL_VERSION = 2.0

XC_DXL_ID = 4
XC_BAUDRATE = 1000000

XC_TORQUE_ENABLE = 1
XC_TORQUE_DISABLE = 0

#**********************************************************************************#

# 각 모터에 대한 ID 및 레지스터 주소 설정
XM_DXL_ADDR_PRESENT_POSITION = 132  # XM 모터의 현재 위치 값을 읽기 위한 레지스터 주소


def main():
    rospy.init_node('motor_control', anonymous=True)
    motor = DynamixelNode() # change this code to prevent spin of this code 
    rospy.Subscriber("angle", fl, motor.run)
    rospy.Subscriber("gripper", Float32, motor.gripper)
    rospy.spin()

    

class DynamixelNode:
    def __init__(self):

        # XM 모터와 통신을 위한 포트 핸들러 및 패킷 핸들러 초기화
        self.port_handler_xm = PortHandler(DEVICENAME)
        self.packet_handler_xm = PacketHandler(XM_PROTOCOL_VERSION)

        # AX 모터와 통신을 위한 포트 핸들러 및 패킷 핸들러 초기화
        self.port_handler_ax = PortHandler(DEVICENAME)
        self.packet_handler_ax = PacketHandler(AX_PROTOCOL_VERSION)
        
        # XC
        self.port_handler_xc = PortHandler(DEVICENAME)
        self.packet_handler_xc = PacketHandler(XC_PROTOCOL_VERSION)

        # XM 모터 포트 열기
        if self.port_handler_xm.openPort():
            rospy.loginfo("Successfully opened the XM port.")
        else:
            rospy.logerr("Failed to open the XM port.")
            rospy.signal_shutdown("Failed to open the XM port.")
            return

        # AX 모터 포트 열기
        if self.port_handler_ax.openPort():
            rospy.loginfo("Successfully opened the AX port.")
        else:
            rospy.logerr("Failed to open the AX port.")
            rospy.signal_shutdown("Failed to open the AX port.")
            return

        # XC port open
        if self.port_handler_xc.openPort():
            rospy.loginfo("Successfully opened the XC port.")
        else:
            rospy.logerr("Failed to open the XC port.")
            rospy.signal_shutdown("Failed to open the XC port.")
            return
        

        # XM 모터 통신 속도 설정
        if self.port_handler_xm.setBaudRate(XM_BAUDRATE):
            rospy.loginfo("Successfully set the XM baudrate.")
        else:
            rospy.logerr("Failed to set the XM baudrate.")
            rospy.signal_shutdown("Failed to set the XM baudrate.")
            return

        # AX 모터 통신 속도 설정
        if self.port_handler_ax.setBaudRate(AX_BAUDRATE):
            rospy.loginfo("Successfully set the AX baudrate.")
        else:
            rospy.logerr("Failed to set the AX baudrate.")
            rospy.signal_shutdown("Failed to set the AX baudrate.")
            return

        # XC Set baud rate
        if self.port_handler_xc.setBaudRate(XC_BAUDRATE):
            rospy.loginfo("Successfully set the XC baudrate.")
        else:
            rospy.logerr("Failed to set the XC baudrate.")
            rospy.signal_shutdown("Failed to set the XC baudrate.")
            return
        
         # XM 모터의 토크 활성화 및 추가 설정
        for dxl_id in XM_DXL_ID:
            dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_TORQUE_ENABLE, XM_TORQUE_ENABLE)
            dxl_comm_result, dxl_error = self.packet_handler_xm.write1ByteTxRx(self.port_handler_xm, dxl_id, XM_ADDR_PROFILE_ACCELERATION, 5)


            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Failed to enable torque for XM Motor ID: {}".format(dxl_id))
                rospy.signal_shutdown("Failed to enable torque for XM Motor ID: {}".format(dxl_id))
                return
            else:
                rospy.loginfo("Torque enabled for XM Motor ID: {}".format(dxl_id))
        
        # AX 모터의 토크 활성화 및 추가 설정
        for dxl_id in AX_DXL_ID:
            dxl_comm_result, dxl_error = self.packet_handler_ax.write1ByteTxRx(self.port_handler_ax, dxl_id, AX_ADDR_TORQUE_ENABLE, AX_TORQUE_ENABLE)
            dxl_comm_result, dxl_error = self.packet_handler_ax.write2ByteTxRx(self.port_handler_ax, dxl_id, AX_ADDR_MOVING_SPEED, 100) 
            dxl_comm_result, dxl_error = self.packet_handler_ax.write1ByteTxRx(self.port_handler_ax, dxl_id, AX_ADDR_CW_COMPLIANCE_SLOPE, 65)
            dxl_comm_result, dxl_error = self.packet_handler_ax.write1ByteTxRx(self.port_handler_ax, dxl_id, AX_ADDR_CCW_COMPLIANCE_SLOPE, 65)

            
            if dxl_comm_result != COMM_SUCCESS:
                rospy.logerr("Failed to enable torque for AX Motor ID: {}".format(dxl_id))
                rospy.signal_shutdown("Failed to enable torque for AX Motor ID: {}".format(dxl_id))
                return
            else:
                rospy.loginfo("Torque enabled for AX Motor ID: {}".format(dxl_id))
         
        #XC torque inable
        dxl_comm_result, dxl_error = self.packet_handler_xc.write1ByteTxRx(self.port_handler_xc, XC_DXL_ID, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            rospy.logerr("Failed to enable torque for XC Motor ID: {}".format(XC_DXL_ID))
            rospy.signal_shutdown("Failed to enable torque for XC Motor ID: {}".format(XC_DXL_ID))
            return
        else:
            rospy.loginfo("Torque enabled for XC Motor ID: {}".format(XC_DXL_ID))


    def read_motor_position(self, port_handler, packet_handler, dxl_id, addr_present_position):
        # 모터의 현재 위치 읽기
        dxl_present_position, _, _ = packet_handler.read2ByteTxRx(port_handler, dxl_id, addr_present_position)
        return dxl_present_position

    def run(self, data):

        position_IK = data.data  # ik angle      
        
        position_degree = list(position_IK) # dynamixel angle(degree)
        
        position_degree[0] += 180
        position_degree[1] += 180
        position_degree[2] += 150
        position_degree[3] += 150
        
        position_dynamixel = position_degree[0:4]
        position_dynamixel[0] *= 11.375
        position_dynamixel[1] *= 11.375
        position_dynamixel[2] *= 3.41

        
        position_dynamixel[2] = position_dynamixel[2] - 20
        position_dynamixel[3] *= 3.41
        for i in range(4):
            position_dynamixel[i] = int(position_dynamixel[i])
        

        xm_position = list([0, 0])
        for i in range(2):
            xm_position[i] = position_dynamixel[i]

        ax_position = list([0, 0])
        for i in range(2):
            ax_position[i] = position_dynamixel[i+2]

        # goal position
        for dxl_id in XM_DXL_ID:
            # print("Set Goal XM_Position of ID %s = %s" % (XM_DXL_ID[dxl_id], xm_position[dxl_id]))
            self.packet_handler_xm.write4ByteTxRx(self.port_handler_xm, XM_DXL_ID[dxl_id], XM_ADDR_GOAL_POSITION, xm_position[dxl_id])

        for dxl_id in AX_DXL_ID:
            # print("Set Goal AX_Position of ID %s = %s" % (AX_DXL_ID[dxl_id-1], ax_position[dxl_id-1]))
            self.packet_handler_ax.write2ByteTxRx(self.port_handler_ax, AX_DXL_ID[dxl_id-2], AX_ADDR_GOAL_POSITION, ax_position[dxl_id-2])

        # XM 모터의 현재 위치 읽기
        for dxl_id in XM_DXL_ID:
            # present_position = self.read_motor_position(self.port_handler_xm, self.packet_handler_xm, dxl_id, XM_DXL_ADDR_PRESENT_POSITION)
            rospy.loginfo("XM Motor ID: {}, goal angle: {}, motort value : {}".format(dxl_id, position_IK[dxl_id], position_dynamixel[dxl_id]))

            # AX 모터의 현재 위치 읽기
        for dxl_id in AX_DXL_ID:
            # present_position = self.read_motor_position(self.port_handler_ax, self.packet_handler_ax, dxl_id, AX_DXL_ADDR_PRESENT_POSITION)
            rospy.loginfo("AX Motor ID: {}, goal angle: {}, motort value : {}".format(dxl_id, position_IK[dxl_id], position_dynamixel[dxl_id]))

        print("")

    def gripper(self, data):
        # Assuming we get the desired angle in degrees from the message
        desired_angle_deg = data.data
        desired_position = int(desired_angle_deg * 11.378)  # Convert degrees to Dynamixel units
        # Write goal position
        dxl_comm_result, dxl_error = self.packet_handler_xc.write4ByteTxRx(self.port_handler_xc, XC_DXL_ID, XC_ADDR_GOAL_POSITION, desired_position)
        if desired_position >= 3800:
            rospy.loginfo("gripper open")
        else:
            rospy.loginfo("gripper close")                            


    def shutdown(self):
        # 노드 종료 시 AX, XM 시리얼 포트 닫기

        self.port_handler_ax.closePort()
        self.port_handler_xm.closePort()
        self.packet_handler_xc.write1ByteTxRx(self.port_handler_xc, XC_DXL_ID, XC_ADDR_TORQUE_ENABLE, XC_TORQUE_DISABLE)
        self.port_handler_xc.closePort()        
        rospy.loginfo("Shutdown Dynamixel node.")

if __name__ == '__main__':
    try:
        
        main()
    except rospy.ROSInterruptException:
        pass