#! usr/bin/env python
from turtle import distance
from xml.dom.expatbuilder import theDOMImplementation
import rospy
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import math
from visualization_msgs.msg import Marker


class Ransac():
    def __init__(self):
        rospy.init_node('listen')
        rospy.Subscriber('/base_scan',LaserScan,self.callback)
        self.msg = LaserScan()
        self.pub = rospy.Publisher('/visualization_marker',Marker,queue_size=1)
        #rospy.spin()
        #rospy.Rate(10).sleep()
        rospy.set_param("iteration",10)
        rospy.set_param("threshold",0.25)
        rospy.set_param("point_threshold",3)
        #rospy.Rate(10).sleep()

    def callback(self,msg):
        
        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.type = self.marker.LINE_LIST
        self.marker.action = self.marker.ADD
        self.marker.lifetime = rospy.Duration()
        # setting marker scale
        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05
        self.marker.scale.z = 0.0
        #setting marker color
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

        range = np.array(msg.ranges)
        range[range==3.0]=0
        angle = np.linspace(start=np.pi/2,stop=(-1)*np.pi/2,num=361)
        x_crd = range*np.cos(angle) 
        y_crd = range*np.sin(angle)
        self.crd = np.stack((x_crd,y_crd),axis=-1)
        self.rans(self.crd,rospy.get_param('iteration'),rospy.get_param('threshold'),rospy.get_param('point_threshold'))
        self.pub.publish(self.marker)
        #rospy.Rate(10).sleep()

        #return self.crd
    def rans(self,points,iteration,threshold,point_threshold):
        points = self.crd 
        #print(points.shape)
        iteration = 10
        point_threshold = 3
        threshold = 0.25
        indexes = range(len(points))
        max = -1
        ymax = -1
        min = -1
        ymin = -1
        px = [-1]
        py = [-1]

        for r in range(5):
            inl = []
            outl = []
            
            for i in range(iteration):
                tempinl = []
                tempoutl = []
                random_points = [points[i] for i in np.random.choice(len(points),2)]
                x1 = random_points[0][0]
                y1 = random_points[0][1]
                x2 = random_points[1][0]
                y2 = random_points[1][1]
                for j in (indexes):
                    x0 = points[j][0]
                    y0 = points[j][1]
                
                    dist = abs((y2 - y1) * x0 - (x2 - x1)*y0 + x2 * y1 - y2 * x1) / math.sqrt((y2 - y1) * (y2 - y1) + (x2 - x1) * (x2 - x1))
                    if dist < threshold:
                        tempinl.append(j)
                    else:
                        tempoutl.append(i)
                if len(inl)< len(tempinl):
                    inl = tempinl
                    outl = tempoutl
                    max = np.max(points[inl,0])
                    print(max)
                    min = np.min(points[inl,0])
                    yout = points[inl,:]
                    ymax = yout[np.argmax(yout[:,0]),1]
                    ymin = yout[np.argmin(yout[:,0]),1]


                
            
            if len(inl) > point_threshold:
                px.append(max)
                py.append(ymax)
                px.append(min)
                py.append(ymin)
                indexes = outl
                print('ind',len(indexes))
            if len(indexes) < 7:
                break

        for i in range(1,len(px)):
            rans = Point()
            rans.x = px[i]
            rans.y = py[i]
            self.marker.points.append(rans)

                
            


    '''
    def main(self):
        while not rospy.is_shutdown():
          
            rospy.Rate(10).sleep()
    '''      





if __name__=="__main__":
    ran = Ransac()
    rospy.spin()
    #rospy.Rate(10).sleep()


