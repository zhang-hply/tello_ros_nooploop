#!/usr/bin/env python2
from position_controller import PositionController
import rospy
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import TrajectoryPoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
class TelloRacing:
    def __init__(self):
        self.odom_sub = rospy.Subscriber("/tello/odom", Odometry, self.cb_odom)
        self.position_sub = rospy.Subscriber("/tello/odometry1", Odometry, self.cb_position)
        self.reference_state_sub = rospy.Subscriber("/hummingbird/autopilot/reference_state", TrajectoryPoint, self.cb_reference_state)
        self.cmd_vel_pub = rospy.Publisher("/tello/cmd_vel", Twist, queue_size=10)
        self.start_cmd_sub = rospy.Subscriber("/tello/start_cmd", Bool, self.cb_startCmd)
        
        self.K = np.mat([[1,0,0,0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 0.05]])
        self.position_controller = PositionController(self.K)

        self.curr_state = np.mat([[0.0], [0.0], [0.0], [0.0]])
        self.desired_state = np.mat([[2.5], [1.5], [0.0], [0.0]])

        self.cmd_vel_msg = Twist()
        # The angle between true north and xoy built by nooploop
        self.heading_offset = 0.0
        self.start_cmd = False

    def cb_odom(self, msg):
        quat = [msg.pose.pose.orientation.x, 
                            msg.pose.pose.orientation.y, 
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w]
        r = R.from_quat(quat)
        self.curr_state[3] = r.as_euler('xyz', degrees=True)[2] + self.heading_offset
        print("The yaw is %f"%(self.curr_state[3]))
    
    def cb_startCmd(self, msg):
        self.start_cmd = msg.data
        

    def cb_position(self, msg):
        self.curr_state[0] = msg.pose.pose.position.x
        self.curr_state[1] = msg.pose.pose.position.y
        self.curr_state[2] = msg.pose.pose.position.z

    def cb_reference_state(self, msg):
        self.desired_state[0] = msg.pose.position.x
        self.desired_state[1] = msg.pose.position.y
        self.desired_state[2] = msg.pose.position.z
        self.desired_state[3] = msg.heading * 180.0 / math.pi
        
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.position_controller.set_curr_position(self.curr_state)
            cmd_vel = self.position_controller.get_vel_cmd(self.desired_state)

                        
            yaw = self.curr_state[3] / 180.0 * math.pi
            body_vel_x = cmd_vel[0] * math.cos(yaw) + cmd_vel[1] * math.sin(yaw)
            body_vel_y = -cmd_vel[0] * math.sin(yaw) + cmd_vel[1] * math.cos(yaw)

            self.cmd_vel_msg.linear.x = body_vel_y
            self.cmd_vel_msg.linear.y = body_vel_x
            self.cmd_vel_msg.linear.z = 0.0
            self.cmd_vel_msg.angular.z = cmd_vel[3]
            if(self.start_cmd):
                self.cmd_vel_pub.publish(self.cmd_vel_msg)
                print("Send cmd")
            rate.sleep()


def main():
    rospy.init_node("cnn_control_tello")
    telloRacing = TelloRacing()
    telloRacing.run()

if __name__ == '__main__':
    main()