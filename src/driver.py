#!/usr/bin/env python
import rospy
import math

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

class arm_driver():
    def __init__(self):
        self.joint_name = ["shoulder", "elbow", "wrist_pitch", "wrist_rot", "gripper"]
        self.motor = dict()
        self.motor_pub = dict()
        self.cmd = dict()
        self.angle = dict()
        self.joint_roll = self.angle_cmd(4, 0)
        for i in range(1, 6):
            rospy.Subscriber("/m" + str(i) + "_controller/state", JointState, self.get_motor_stat, callback_args=i)
            self.motor_pub[str(i)] = rospy.Publisher("/m" + str(i) + "_controller/command", Float64, queue_size=1)
            rospy.Subscriber("hek/" + self.joint_name[i-1] + "_joint_position_controller/command", Float64, self.set_cmd, callback_args=i)
            self.angle[str(i)] = rospy.Publisher("/motor" + str(i) + "_angle", Float64, queue_size=1)




    def get_motor_stat(self, data, i):
        self.motor[int(data.motor_ids[0])] = {"name": data.name,
                                           "Id": data.motor_ids,
                                           "temperature": data.motor_temps,
                                           "goal_pos": math.degrees(data.goal_pos),
                                           "current_pos": math.degrees(data.current_pos),
                                           "error": math.degrees(data.error),
                                           "velocity": math.degrees(data.velocity),
                                           "load": data.load,
                                           "is_moving": data.is_moving}

        if (i == 4):
            self.motor_pub[str(4)].publish(math.radians(-self.motor[3]["current_pos"] + self.joint_roll))
            self.angle[str(4)].publish(self.joint_roll)
        else:
            self.angle[str(i)].publish(self.motor[i]["current_pos"])

    def set_cmd(self, data, i):
        command = math.degrees(data.data)
        if (i == 4):
            self.joint_roll = self.angle_cmd(i, command)

        else:
            self.motor_pub[str(i)].publish(math.radians(self.angle_cmd(i, command)))

    def angle_cmd(self, joint, cmd):
        if joint == 1:
            angle = 35.0/18.0*(90-cmd)+45.0/2.0
            return angle
        elif joint == 2:
            angle = 89.0/90.0*cmd+179.0
            return angle
        elif joint == 3:
            angle = -3.0/2.0*cmd+175.0
            return angle
        elif joint == 4:
            angle = -3.0/2.0*cmd+270.0
            return angle
        elif joint == 5:
            angle = 32.0/25.0*(100-cmd)+75.0
            return angle


if __name__ == '__main__':
    rospy.init_node('arm_driver', anonymous=True)
    robotic_arm = arm_driver()

    rospy.spin()
