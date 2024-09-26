#!/usr/bin/env python

import rospy
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3Stamped


def imu_mag_callback(msg):
    # 새로운 메시지 객체 생성
    mag = MagneticField()
    
    mag.header = msg.header
    mag.magnetic_field = msg.vector
    mag.magnetic_field_covariance[0] = 0.01
    mag.magnetic_field_covariance[4] = 0.01
    mag.magnetic_field_covariance[8] = 0.01
    
    pub.publish(mag)
    '''
    custom_msg = MagneticFieldWithCovariance()
    
    # 기본 magnetic_field 값을 복사
    custom_msg.magnetic_field.x = msg.vector.x
    custom_msg.magnetic_field.y = msg.vector.y
    custom_msg.magnetic_field.z = msg.vector.z
    
    # 공분산 행렬 예시 값 설정 (3x3 공분산 행렬)
    custom_msg.magnetic_field_covariance[0] = 0.01  # x축 공분산
    custom_msg.magnetic_field_covariance[4] = 0.01  # y축 공분산
    custom_msg.magnetic_field_covariance[8] = 0.01  # z축 공분산

    # 메시지 발행
    pub.publish(custom_msg)
    '''
    
if __name__ == '__main__':
    # 노드 초기화
    rospy.init_node('imu_mag_covariance_publisher')

    # 퍼블리셔 설정
    pub = rospy.Publisher('/imu/mag_with_covariance', MagneticField, queue_size=10)

    # 구독자 설정 (기존의 IMU Mag 메시지를 구독)
    rospy.Subscriber('/imu/mag', Vector3Stamped, imu_mag_callback)

    # ROS 루프
    rospy.spin()

