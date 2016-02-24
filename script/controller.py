#!/usr/bin/python
# -*- coding:utf-8 -*-

import rospy

from sensor_msgs.msg import JointState
from std_msgs.msg import UInt16

motor_params = None
shoulder_left_roll_pub = None

def get_index_from_name(msg, name):
    return msg.name.index(name)

def callback(msg):
    position_rad = msg.position[get_index_from_name(msg, 'left_shoulder_roll')]

    max_position = motor_params['shoulder_left_roll']['max']
    min_position = motor_params['shoulder_left_roll']['min']

    min_rad = motor_params['shoulder_left_roll']['min_rad']
    max_rad = motor_params['shoulder_left_roll']['max_rad']

    position = ((position_rad - min_rad) / (max_rad - min_rad)) * (max_position - min_position) + min_position

    shoulder_left_roll_pub.publish(position)

    rospy.loginfo("min_rad:%lf, max_rad:%lf, position:%d" % (min_rad, max_rad, position))
    # for motor_status in zip(msg.name, msg.position):
    #     rospy.loginfo(motor_status)

# def base_callback(motor_config, msg):
#     target = msg.data
#     target = min(target, motor_config['max'])
#     target = max(target, motor_config['min'])
#     servo_target_msg = ServoTarget(sid = motor_config['id'], target_position = target)
#     servo_target_pub.publish(servo_target_msg)

def get_callback(motor_config):
    return lambda msg: base_callback(motor_config, msg)

def controller():
  rospy.init_node("robovie_controller")

  global motor_params
  motor_params = rospy.get_param('/robovie/vstone_servo/controller_spawner/config')

  global shoulder_left_roll_pub
  shoulder_left_roll_pub = rospy.Publisher('/robovie/vstone_servo/shoulder_left_roll_controller', UInt16, queue_size=10000)

  rospy.Subscriber("/joint_states", JointState, callback)

  rospy.spin()

if __name__ == "__main__":
  try:
    controller()
  except rospy.ROSInterruptException:
    pass
