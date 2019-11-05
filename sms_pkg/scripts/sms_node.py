#!/usr/bin/env python

# Position control for 3 sms and Velocity control for another 3 sms

#dmp_rep.getParam("trajectory_file_path",dmp_rep_obj.trajectory_file_path);

import rospy
import SMSLibrary as sms
import time as t
import math

from sms_pkg.msg import control_with_id
from sms_pkg.msg import control_all

from sms_pkg.srv import encoder_pos, encoder_posResponse
from sms_pkg.srv import encoder_abspos, encoder_absposResponse
from sms_pkg.srv import encoder_vel, encoder_velResponse
from sms_pkg.srv import encoder_cur, encoder_curResponse

motor_id_bias = rospy.get_param("motor_id_bias")
num_of_motors = rospy.get_param("number_of_motors_attached")
motors_in_block = rospy.get_param("motors_in_block")

block_current = 1
motors_in_current_block = motors_in_block
num_of_blocks = num_of_motors // motors_in_block
if (num_of_motors % motors_in_block) > 0:
    num_of_blocks = int(num_of_blocks + 1)

def initialize_motors():
    print("Initializing all motors... ")
    for motor_id in range(num_of_motors):
        sms.start(motor_id_bias + motor_id)
        print "Done with motor ", motor_id_bias + motor_id
        t.sleep(0.1)
    print("End of initialization!\n\n")

""" Control providing motor's ID callbacks """
def control_with_id_relpos_callback(msg):
    sms.moveToRelativePosition(msg.id, msg.value)
    print "Relative Position command", msg.value, "successfully sent to motor", msg.id

def control_with_id_pos_callback(msg):
    sms.moveToAbsolutePosition(msg.id, msg.value)
    print "Position command", msg.value, "successfully sent to motor", msg.id

def control_with_id_vel_callback(msg):
    sms.moveWithVelocity(msg.id, msg.value)
    print "Velocity command", msg.value, "successfully sent to motor", msg.id

""" Control all motors callbacks """
def control_all_relpos_callback(msg):
    for motor_id in range(num_of_motors):
        sms.moveToRelativePosition(motor_id_bias + motor_id, msg.data[motor_id])
        t.sleep(0.01)
    print "Relative Position's data", msg.data, "successfully sent to motors..!"

def control_all_pos_callback(msg):
    for motor_id in range(num_of_motors):
        sms.moveToAbsolutePosition(motor_id_bias + motor_id, msg.data[motor_id])
        t.sleep(0.01)
    print "Position's data", msg.data, "successfully sent to motors..!"

def control_all_vel_callback(msg):
    """for motor_id in range(num_of_motors):
        sms.moveWithVelocity(motor_id_bias + motor_id, msg.data[motor_id])
        #t.sleep(0.01)
    print "Velocity's data", msg.data, "successfully sent to motors..!"
    """
    global block_current, motors_in_current_block
    for motor_id in range(motors_in_current_block):
        sms.moveWithVelocity(motor_id_bias + (block_current-1)*motors_in_block + motor_id, msg.data[(block_current-1)*motors_in_block + motor_id])
        #t.sleep(0.01)
    print "Velocity's data", msg.data, "successfully sent to motors..!"
    block_current = block_current + 1
    if (block_current == num_of_blocks):
        if (num_of_motors % motors_in_block) > 0:
            motors_in_current_block = int(num_of_motors % motors_in_block)
    elif (block_current > num_of_blocks):
        block_current = 1
        motors_in_current_block = int(motors_in_block)

""" Encoder callbacks """
def handle_encoder_pos(req):
    encoder_pos_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getPosition(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        encoder_pos_res.append(int(tmp))
    return encoder_posResponse(encoder_pos_res)

def handle_encoder_abspos(req):
    encoder_abspos_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getAbsolutePosition(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        encoder_abspos_res.append(int(tmp))
    return encoder_absposResponse(encoder_abspos_res)

def handle_encoder_vel(req):
    encoder_vel_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getVelocity(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        encoder_vel_res.append(int(tmp))
    return encoder_velResponse(encoder_vel_res)

def handle_encoder_cur(req):
    encoder_cur_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getCurrent(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        encoder_cur_res.append(int(tmp))
    return encoder_curResponse(encoder_cur_res)

def sms_node_py():
    # Starts a new node
    rospy.init_node('sms_node_node', anonymous=True)

    port = rospy.get_param("~index_of_USB_port") # '/dev/ttyUSB_' (i.e. 0)
    sms.init(port) # initialize USB port
    t.sleep(0.1) # in seconds
    initialize_motors()

    rospy.Subscriber("control_with_id_relpos", control_with_id, control_with_id_relpos_callback)
    rospy.Subscriber("control_with_id_pos", control_with_id, control_with_id_pos_callback)
    rospy.Subscriber("control_with_id_vel", control_with_id, control_with_id_vel_callback)

    rospy.Subscriber("control_all_relpos", control_all, control_all_relpos_callback)
    rospy.Subscriber("control_all_pos", control_all, control_all_pos_callback)
    rospy.Subscriber("control_all_vel_throttled", control_all, control_all_vel_callback)

    s_pos = rospy.Service('encoder_pos_service', encoder_pos, handle_encoder_pos)
    s_abspos = rospy.Service('encoder_abspos_service', encoder_abspos, handle_encoder_abspos)
    s_vel = rospy.Service('encoder_vel_service', encoder_vel, handle_encoder_vel)
    s_cur = rospy.Service('encoder_cur_service', encoder_cur, handle_encoder_cur)

    while not rospy.is_shutdown():
        rospy.spin()

    for motor_id in range(num_of_motors):
        sms.moveToAbsolutePosition(motor_id_bias + motor_id,0)
        t.sleep(0.05)
    t.sleep(2)

    print("Disabling all Motors... ")
    for motor_id in range(num_of_motors):
        sms.stop(motor_id_bias + motor_id)
    print("DONE!\n\n")
    sms.shut_down(port) # close port

if __name__ == '__main__':
    try:
        sms_node_py()
    except rospy.ROSInterruptException:
        pass
