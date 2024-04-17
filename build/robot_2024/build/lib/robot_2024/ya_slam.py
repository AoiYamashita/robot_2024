import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from PIL import Image
import numpy as np
import cv2
import time

size = 500

class YaSlam(Node):
    def __init__(self):
        super().__init__("slam")
        self.sub = self.create_subscription(LaserScan,"/scan",self.cb,10)
        self.Imu = self.create_subscription(Odometry,"/first_robot/odom",self.imu,10)
        self.angle_max = None
        self.angle_min = None
        self.angle_inc = None
        self.map_msg = np.zeros((size,size,3))
        self.x = 0
        self.y = 0
        self.deg = 0
        self.vx = 0
        self.vy = 0
        self.vdeg = 0
        self.time = time.perf_counter()
    def cb(self,data):
        now = time.perf_counter()
        dt = now - self.time
        self.time = now
        #print(dt)
        self.x += self.vx*dt
        self.y += self.vy*dt
        self.deg += self.vdeg*dt
        self.map_msg = np.zeros((size,size,3))
        if self.angle_max is None:
            self.angle_max = data.angle_max
            self.angle_min = data.angle_min
            self.angle_inc = data.angle_increment
        for i in range(len(data.ranges)):
            ran = data.ranges[i]
            if ran > data.range_max:continue
            ran *= size/20
            x_ = int(size/2+ran*np.sin(self.angle_min+self.angle_inc*i+self.deg))
            y_ = int(size/2+ran*np.cos(self.angle_min+self.angle_inc*i+self.deg))
            cv2.circle(self.map_msg,(int(x_-self.x*size/20),int(y_+self.y*size/20)),1,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
            #print((100+int(ran*np.sin(self.angle_min+self.angle_inc*i)),100+int(ran*np.cos(self.angle_min+self.angle_inc*i))))
        
        self.map_msg = np.array(self.map_msg,dtype=np.uint8)
        gray = cv2.cvtColor(self.map_msg, cv2.COLOR_BGR2GRAY)
        lines = cv2.HoughLinesP(gray,rho=1, theta=np.pi/360, threshold=80, minLineLength=0, maxLineGap=5)
        if i is not None:
            for i in lines:
                i = np.squeeze(i)
                cv2.line(self.map_msg,(int(i[0]),int(i[1])),(int(i[2]),int(i[3])),(255,0,0),thickness=3,lineType=cv2.LINE_4,shift=0)
        cv2.circle(self.map_msg,(int(size/2-self.x*size/20),int(size/2+self.y*size/20)),3,(0,255,0),thickness=3,lineType=cv2.LINE_4,shift=0)
        
        plt.imshow(self.map_msg)
        plt.pause(.0001)
        #plt.show()
        #print(self.map_msg)
    def imu(self,data):
        self.vx = data.twist.twist.linear.x*np.cos(self.deg)-data.twist.twist.linear.y*np.sin(self.deg)
        self.vy = data.twist.twist.linear.x*np.sin(self.deg)+data.twist.twist.linear.y*np.cos(self.deg)
        self.vdeg = data.twist.twist.angular.z
        #print(data.twist.twist.linear.x,data.twist.twist.angular.z)
        

def main():
    rclpy.init()
    yaslam = YaSlam()
    rclpy.spin(yaslam)

if __name__ == "__main__":
    main()
