#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import math

def imu_callback(msg):
    # 메시지에서 쿼터니언을 추출
    quaternion = msg.orientation
    
    # 쿼터니언을 [x, y, z, w] 리스트로 변환
    q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
    
    # 쿼터니언을 Euler 각도로 변환 (Roll, Pitch, Yaw)
    roll, pitch, yaw = euler_from_quaternion(q)
    
    # RPY 값을 라디안에서 각도로 변환
    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)
    
    # 각도 값 출력
    rospy.loginfo(f"\nRoll: {roll_deg}°, Pitch: {pitch_deg}°, Yaw: {yaw_deg}°")

def listener():
    # ROS 노드 초기화
    rospy.init_node('imu_to_rpy_listener', anonymous=True)

    # IMU 토픽 구독
    rospy.Subscriber('/imu/data_madgwick', Imu, imu_callback)

    # ROS 스핀 - 콜백을 기다리면서 노드를 계속 실행
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

