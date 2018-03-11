#!/usr/bin/env python
import rospy
import math

from dynamixel_msgs.msg import JointState
from std_msgs.msg import Float64

class arm_driver():
    def __init__(self):
        self.motor = dict()
        self.motor_pub = dict()
        self.cmd = dict()
        self.angle = dict()
        self.joint_roll = 0
        for i in range(1, 6):
            rospy.Subscriber("/m" + str(i) + "_controller/state", JointState, self.get_motor_stat, callback_args=i)
            self.motor_pub[str(i)] = rospy.Publisher("/m" + str(i) + "_controller/command", Float64, queue_size=1)
            rospy.Subscriber("cmd_" + str(i), Float64, self.set_cmd, callback_args=i)
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
        if (i == 4 | i == 3):
            self.joint_roll = self.motor[4]["current_pos"] - self.motor[3]["current_pos"]

        if (i == 4):
            self.angle[str(4)].publish(self.joint_roll)

        self.angle[str(i)].publish(self.motor[i]["current_pos"])

    def set_cmd(self, data, i):
        if (i == 3):
            self.motor_pub[str(3)].publish(math.radians(data.data))
            print (-data.data + self.joint_roll)
            self.motor_pub[str(4)].publish(math.radians(-data.data + self.joint_roll))
        elif (i == 4):
            self.joint_roll = data.data
            self.motor_pub[str(4)].publish(math.radians(-self.motor[3]["current_pos"] + self.joint_roll))
        else:
            self.motor_pub[str(i)].publish(math.radians(data.data))


if __name__ == '__main__':
    rospy.init_node('arm_driver', anonymous=True)
    robotic_arm = arm_driver()

    rospy.spin()