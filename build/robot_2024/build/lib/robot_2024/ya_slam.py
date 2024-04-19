import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from PIL import Image
import random
import numpy as np
import cv2
import time

size = 500

class Particle():
    def __init__(self,x_,y_,deg_):
        self.x = x_
        self.y = y_
        self.deg = deg_

class YaSlam(Node):
    def __init__(self):
        super().__init__("slam")
        self.sub = self.create_subscription(LaserScan,"/scan",self.cb,10)
        self.Imu = self.create_subscription(Odometry,"/first_robot/odom",self.imu,10)
        self.timer = self.create_timer(0.1,self.draw)
        self.angle_max = None
        self.angle_min = None
        self.angle_inc = None
        self.map_msg = np.zeros((size,size,3))
        self.markmap = np.zeros((size,size))
        self.landmarks = []
        self.particles = [Particle(0,0,0) for i in range(100)]
        self.x = 0
        self.y = 0
        self.deg = 0
        self.vx = 0
        self.vy = 0
        self.vdeg = 0
        self.odomDevX = 0.1
        self.odomDevY = 0.1
        self.odomDevTheta = 0.1
        self.time = time.perf_counter()
    def cb(self,data):
        now = time.perf_counter()
        dt = now - self.time
        self.time = now
        self.map_msg = np.zeros((size,size,3))
        for i in self.particles:
            arrsize = 10
            self.moveOdom(i,dt)
            #cv2.circle(self.map_msg,(int(size/2-i.x*size/20),int(size/2+i.y*size/20)),3,(0,255,0),thickness=3,lineType=cv2.LINE_4,shift=0)
            cv2.arrowedLine(self.map_msg,(int(size/2-i.x*size/20),int(size/2+i.y*size/20)),(int(size/2-i.x*size/20-arrsize*np.cos(i.deg)),int(size/2+i.y*size/20+arrsize*np.sin(i.deg))),(255,0,0),thickness=1,line_type=cv2.LINE_4,shift=0)
        self.x = self.particles[0].x
        self.y = self.particles[0].y
        self.deg = self.particles[0].deg
        
        if self.angle_max is None:
            self.angle_max = data.angle_max
            self.angle_min = data.angle_min
            self.angle_inc = data.angle_increment
        #plot lidar
        for i in range(len(data.ranges)):
            ran = data.ranges[i]
            if ran > data.range_max:continue
            ran *= size/20
            x_ = int(size/2+ran*np.sin(self.angle_min+self.angle_inc*i+self.deg))
            y_ = int(size/2+ran*np.cos(self.angle_min+self.angle_inc*i+self.deg))
            cv2.circle(self.map_msg,(int(x_-self.x*size/20),int(y_+self.y*size/20)),1,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
            #print((100+int(ran*np.sin(self.angle_min+self.angle_inc*i)),100+int(ran*np.cos(self.angle_min+self.angle_inc*i))))
        
        #mk landmark
        self.map_msg = np.array(self.map_msg,dtype=np.uint8)
        gray = cv2.cvtColor(self.map_msg, cv2.COLOR_BGR2GRAY)
        lines = cv2.HoughLinesP(gray,rho=1, theta=np.pi/360, threshold=80, minLineLength=0, maxLineGap=7)
        if lines is not None:
            for i in lines:
                i = np.squeeze(i)
                cv2.circle(self.map_msg,(int(i[0]),int(i[1])),3,(0,255,255),thickness=3,lineType=cv2.LINE_4,shift=0)
                #cv2.line(self.map_msg,(int(i[0]),int(i[1])),(int(i[2]),int(i[3])),(0,255,0),thickness=1,lineType=cv2.LINE_4,shift=0)
                cv2.circle(self.map_msg,(int(i[2]),int(i[3])),3,(0,255,255),thickness=3,lineType=cv2.LINE_4,shift=0)
                bit = 5
                self.markmap[int(i[0])-bit:int(i[0])+bit,int(i[1])-bit:int(i[1])+bit] += 1
                self.markmap[int(i[2])-bit:int(i[2])+bit,int(i[3])-bit:int(i[3])+bit] += 1
                if self.markmap[int(i[0]),int(i[1])] > 0.1*np.sum(self.landmarks):
                    self.landmarks.append([i[0],i[1]])
                if self.markmap[int(i[2]),int(i[3])] > 0.1*np.sum(self.landmarks):
                    self.landmarks.append([i[2],i[3]])
        #cv2.circle(self.map_msg,(int(size/2-self.x*size/20),int(size/2+self.y*size/20)),3,(0,255,0),thickness=3,lineType=cv2.LINE_4,shift=0)
        
        for i in self.landmarks:
            cv2.circle(self.map_msg,(int(i[0]),int(i[1])),3,(0,255,255),thickness=3,lineType=cv2.LINE_4,shift=0)
    def moveOdom(self,particle,dt):
        #print(dt)
        np.random.normal(loc= 0,scale = self.odomDevX)
        noize = [self.vx*np.random.normal(loc= 0,scale = self.odomDevX),self.vy*np.random.normal(loc= 0,scale = self.odomDevY),self.vdeg*np.random.normal(loc= 0,scale = self.odomDevTheta)]
        particle.x += self.vx*dt+noize[0]+noize[2]*np.cos(self.deg)
        particle.y += self.vy*dt+noize[1]+noize[2]*np.cos(self.deg)
        particle.deg += self.vdeg*dt
    def draw(self):
        #plt.imshow(self.markmap)
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
