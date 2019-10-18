#!/usr/bin/env python

# Position control for 3 sms and Velocity control for another 3 sms

#dmp_rep.getParam("trajectory_file_path",dmp_rep_obj.trajectory_file_path);

import rospy
import SMSLibrary as sms
import time as t
import math

from std_msgs.msg import Empty

from sms_pkg.msg import relpos_control
from sms_pkg.msg import pos_control
from sms_pkg.msg import vel_control

from sms_pkg.srv import pos_encoder, pos_encoderResponse
from sms_pkg.srv import abspos_encoder, abspos_encoderResponse
from sms_pkg.srv import vel_encoder, vel_encoderResponse
from sms_pkg.srv import cur_encoder, cur_encoderResponse

motor_id_bias = rospy.get_param("motor_id_bias")
num_of_motors = rospy.get_param("number_of_motors_attached")
print motor_id_bias, num_of_motors

def initialize_motors():
    print("Initializing all motors... ")
    for motor_id in range(num_of_motors):
        sms.start(motor_id_bias + motor_id)
        print "Done with motor ", motor_id_bias + motor_id
        t.sleep(0.1)
    print("End of initialization!\n\n")

def relpos_control_callback(msg):
    sms.moveToRelativePosition(msg.id, msg.value)

def pos_control_callback(msg):
    sms.moveToAbsolutePosition(msg.id, msg.value)

def vel_control_callback(msg):
    sms.moveWithVelocity(msg.id, msg.value)

def handle_pos_encoder(req):
    pos_encoder_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getPosition(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        pos_encoder_res.append(int(tmp))
    return pos_encoderResponse(pos_encoder_res)

def handle_abspos_encoder(req):
    abspos_encoder_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getAbsolutePosition(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        abspos_encoder_res.append(int(tmp))
    return abspos_encoderResponse(abspos_encoder_res)

def handle_vel_encoder(req):
    vel_encoder_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getVelocity(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        vel_encoder_res.append(int(tmp))
    return vel_encoderResponse(vel_encoder_res)

def handle_cur_encoder(req):
    cur_encoder_res = []
    for motor_id in range(num_of_motors):
        tmp = sms.getCurrent(motor_id_bias + motor_id)[1]
        tmp = -(tmp & 0x80000000) | (tmp & 0x7fffffff)
        cur_encoder_res.append(int(tmp))
    return cur_encoderResponse(cur_encoder_res)

def sms_node_py():
    # Starts a new node
    rospy.init_node('sms_node_node', anonymous=True)

    port = rospy.get_param("~index_of_USB_port") # '/dev/ttyUSB_' (i.e. 0)
    sms.init(port) # initialize USB port
    t.sleep(0.1) # in seconds
    initialize_motors()

    rospy.Subscriber("relpos_control_topic", relpos_control, relpos_control_callback)
    rospy.Subscriber("pos_control_topic", pos_control, pos_control_callback)
    rospy.Subscriber("vel_control_topic", vel_control, vel_control_callback)

    s_pos = rospy.Service('pos_encoder_service', pos_encoder, handle_pos_encoder)
    s_abspos = rospy.Service('abspos_encoder_service', abspos_encoder, handle_abspos_encoder)
    s_vel = rospy.Service('vel_encoder_service', vel_encoder, handle_vel_encoder)
    s_cur = rospy.Service('cur_encoder_service', cur_encoder, handle_cur_encoder)

    while not rospy.is_shutdown():
        rospy.spin()

    for motor_id in range(num_of_motors):
        sms.moveToAbsolutePosition(motor_id_bias + motor_id,0)
    t.sleep(1)

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
