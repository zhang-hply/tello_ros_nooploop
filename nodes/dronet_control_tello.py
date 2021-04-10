#!/usr/bin/env python2
import rospy
from tellopy._internal import tello
from tello_driver.msg import CNN_out
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class Dronet_control_tello(tello.Tello):
    def __init__(self):
        self.rate = rospy.Rate(10)
        rospy.Subscriber('cnn_predictions', CNN_out, self.cb_cnn_out)
        rospy.Subscriber('state_change', Bool, self.cb_state_change)
        self.desired_velocity_pub = rospy.Publisher('cnn_tello/desired_velocity', Twist, queue_size=1)
        
        self.max_forward_index_  = 1.0      #max velocity forward
        self.alpha_velocity_ = 0.3          #filter parameter for velocity
        self.alpha_yaw = 0.5                #filter parameter for yaw angular velocity
        self.critical_prob_coll_ = 0.7
        self.max_forward_index_ = 0.2
        
        self.probability_of_collision_ = 0
        self.steering_angle_ = 0 

        self.desired_forward_velocity_ = self.max_forward_index_
        self.desired_angular_velocity_ = 0.0

        self.use_network_out_ = False


    
    def cb_cnn_out(self, msg):
        self.probability_of_collision_ = msg.collision_prob
        self.steering_angle_ = msg.steering_angle

        #Output modulation
        self.steering_angle_ = -1.0 if self.steering_angle_ < -1.0 else None 
        self.steering_angle_ = 1.0  if self.steering_angle_ > 1.0  else None

    def cb_state_change(self, msg):
        self.use_network_out_ = msg.data

    def run(self):
        while not rospy.is_shutdown():
            self.desired_forward_velocity_m = (1.0 - self.probability_of_collision_) * self.max_forward_index_ 
            if self.desired_forward_velocity_m <= 0.0:
                rospy.loginfo("Detected negative forward velocity! Drone will now stop!")
                self.desired_forward_velocity_m  = 0
            
            self.desired_forward_velocity_ = (1.0 - self.alpha_velocity_) * self.desired_forward_velocity_ \
                                        +  self.alpha_velocity_ * self.desired_forward_velocity_m

            if self.desired_forward_velocity_ < ((1 - self.critical_prob_coll_) * self.max_forward_index_):
                self.desired_forward_velocity_ = 0.0
            
            self.desired_angular_velocity_ = (1.0 - self.alpha_yaw) * self.desired_angular_velocity_ \
                                        + self.alpha_yaw * self.steering_angle_

            cmd_velocity_ = Twist()
            cmd_velocity_.linear.x = self.desired_forward_velocity_
            cmd_velocity_.linear.z = self.desired_angular_velocity_

            if self.use_network_out_:
                self.desired_velocity_pub(cmd_velocity_)

            self.rate.sleep()

def main():
    rospy.init_node('dronet_control_tello')
    dronet_control = Dronet_control_tello()
    dronet_control.run()


if __name__ == '__main__':
    main()