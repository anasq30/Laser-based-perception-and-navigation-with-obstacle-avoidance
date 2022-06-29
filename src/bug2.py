#! usr/bin/env python
from cmath import pi
from turtle import heading
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
#from tf import transformations
from tf.transformations import euler_from_quaternion
import math

class Bug2():
    def __init__(self):
        rospy.init_node('bug2')
        rospy.Subscriber('base_scan',LaserScan,self.callback)
        self.msg = LaserScan()
        self.vel = Twist()
        rospy.Subscriber('base_pose_ground_truth',Odometry,self.odomcallback)
        self.cmd_vel = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.current_position = Vector3()
        self.state1 = 'goal_seek'
        self.state2 = 'follow_wall'
        self.current_state = self.state1
        self.final_state = 'goal_reached'
        #self.follow_wall = 'follow_wall'
        self.old_pose = 0.0
        self.forward = 0.0
        rospy.set_param('x',4.5)
        rospy.set_param('y',9.0)
        self.dist_thrsh_l = 1.0
        self.dist_thrsh_m = 1.0


    
    def odomcallback(self,msg):
        self.current_position.x = msg.pose.pose.position.x
        self.current_position.y = msg.pose.pose.position.y
        self.current_position.z = 0.0
        orientation = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        e = euler_from_quaternion(orientation)
        self.forward = e[2]
           

        

    def distance(self,curr_x,curr_y,fnl_x,fnl_y):
        self.dist = math.sqrt(math.pow((fnl_x - curr_x),2) +math.pow((fnl_y - curr_y),2))
        return self.dist

    def main(self):
        while not rospy.is_shutdown() and self.distance(curr_x=self.current_position.x,curr_y=self.current_position.y,fnl_x= rospy.get_param('x',4.0), fnl_y= rospy.get_param('y',4.0)) >1.6:

            rospy.Rate(30).sleep
    
    def callback(self,msg):
        if self.current_state == self.state1 and msg.ranges[180]< self.dist_thrsh_m:
            self.current_state = self.state2
            self.old_pose = self.current_position.x
        else:
            angle = math.atan2(rospy.get_param('y',2.0)-self.current_position.y, rospy.get_param('x',2.0)-self.current_position.x)
            if angle<0:
                angle += 2*math.pi
            if self.forward<0:
                self.forward += 2*math.pi
            self.vel.angular.z = angle - self.forward
            self.vel.linear.x = 2.0
        if self.current_state==self.state2 and msg.ranges[180]<=self.dist_thrsh_m:
            self.vel.angular.z = -2.0
        elif self.current_state==self.state2 and msg.ranges[360]>=self.dist_thrsh_l:
            self.vel.angular.z = 2.0
        else:
            if self.current_position.x < self.old_pose:
                self.current_state = self.state1
            else:
                self.vel.linear.x = 2.0
        self.cmd_vel.publish(self.vel)



if __name__=="__main__":
    bug = Bug2()
    bug.main()
    rospy.spin()

