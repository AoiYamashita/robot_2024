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

size = 1000
perticle_num = 100

class Particle():
    def __init__(self,x_,y_,deg_):
        self.x = x_
        self.y = y_
        self.deg = deg_
        self.w = 1/perticle_num
        self.errorSum = 0
        self.coords = []
    #def calcW(self,newMap):

class YaSlam(Node):
    def __init__(self):
        super().__init__("slam")
        self.sub = self.create_subscription(LaserScan,"/scan",self.cb,10)
        self.Imu = self.create_subscription(Odometry,"/first_robot/odom",self.odom,10)
        self.timer = self.create_timer(0.1,self.draw)
        self.angle_max = None
        self.angle_min = None
        self.angle_inc = None
        self.map_msg = np.zeros((size,size,3))
        self.markmap = np.zeros((size,size))
        self.particleMap = np.zeros((size,size,3))
        self.lidarValue = np.zeros((size,size))
        self.particles = [Particle(0,0,0) for i in range(perticle_num)]
        self.x = 0
        self.y = 0
        self.deg = 0
        self.vx = 0
        self.vy = 0
        self.vdeg = 0
        self.odomDevX = 0.2
        self.odomDevY = 0.2
        self.odomDevTheta = 0.2
        self.time = time.perf_counter()
    def cb(self,data):
        now = time.perf_counter()
        dt = now - self.time
        self.time = now
        self.get_logger().info("%lf" % dt)
        
        self.map_msg = np.zeros((size,size,3))
        for i in self.particles:
            arrsize = 50
            self.moveOdom(i,dt)
            #print(i.w)
            #cv2.circle(self.map_msg,(int(size/2-i.x*),int(size/2+i.y*)),3,(0,255,0),thickness=3,lineType=cv2.LINE_4,shift=0)
            cv2.arrowedLine(self.map_msg,(int(size/2-i.x),int(size/2+i.y)),(int(size/2-i.x*-arrsize*np.cos(i.deg)),int(size/2+i.y*+arrsize*np.sin(i.deg))),(55+200*i.w,0,0),thickness=4,line_type=cv2.LINE_4,shift=0)
        self.x = np.average(np.array([i.x for i in self.particles]))
        self.y = np.average(np.array([i.y for i in self.particles]))
        self.deg = np.average(np.array([i.deg for i in self.particles]))

        cv2.circle(self.particleMap,(int(size/2-self.x),int(size/2+self.y)),3,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
        cv2.circle(self.particleMap,(int(size/2),int(size/2)),3,(0,255,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
        
        if self.angle_max is None:
            self.angle_max = data.angle_max
            self.angle_min = data.angle_min
            self.angle_inc = data.angle_increment
        #plot lidar
        for i in range(len(data.ranges)):
            ran = data.ranges[i]
            if ran > data.range_max:continue
            x_ = int(size/2+ran*np.sin(self.angle_min+self.angle_inc*i+self.deg))
            y_ = int(size/2+ran*np.cos(self.angle_min+self.angle_inc*i+self.deg))
            #cv2.circle(self.map_msg,(int(x_-self.x*),int(y_+self.y*)),3,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
            cv2.circle(self.map_msg,(int(int(size/2+ran*np.sin(self.angle_min+self.angle_inc*i))),int(size/2+ran*np.cos(self.angle_min+self.angle_inc*i))),3,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
            #print((100+int(ran*np.sin(self.angle_min+self.angle_inc*i)),100+int(ran*np.cos(self.angle_min+self.angle_inc*i))))
        
        #mk landmark
        self.map_msg = np.array(self.map_msg,dtype=np.uint8)
        gray = cv2.cvtColor(self.map_msg, cv2.COLOR_BGR2GRAY)
        self.lidarValue = np.where(cv2.resize(gray,dsize=None,fx=0.5,fy=0.5) > 0)
        print(self.lidarValue)
        #for i in self.particles: 
        #    if i is not None:

        #    i.coords = self.lidarValue
                
        #for i in 
    def moveOdom(self,particle,dt):
        #print(dt)
        np.random.normal(loc= 0,scale = self.odomDevX)
        dv = np.array([[np.cos(particle.deg),-np.sin(particle.deg)],[np.sin(particle.deg),np.cos(particle.deg)]]) @ np.array([self.vx,self.vy])
        noize = [dv[0]*dt*np.random.normal(loc= 0,scale = self.odomDevX),dv[1]*dt*np.random.normal(loc= 0,scale = self.odomDevY),self.vdeg*dt*np.random.normal(loc= 0,scale = self.odomDevTheta)]
        dv = np.array([dv[0]*dt+noize[0],dv[1]*dt+noize[1]])
        particle.x += dv[0]
        particle.y += dv[1]
        particle.deg += self.vdeg*dt+noize[2]
    def draw(self):
        #plt.imshow(cv2.resize(self.particleMap,dsize = None,fx = 0.3,fy = 0.3))
        plt.imshow(self.lidarValue)
        #plt.imshow(cv2.resize(self.map_msg,dsize = None,fx = 0.3,fy = 0.3))
        plt.pause(.0001)
        self.particleMap = np.zeros((size,size,3))
        #plt.show()
        #print(self.map_msg)
    def odom(self,data):
        self.vx = data.twist.twist.linear.x#*np.cos(self.deg)-data.twist.twist.linear.y*np.sin(self.deg)
        self.vy = data.twist.twist.linear.y#data.twist.twist.linear.x*np.sin(self.deg)+data.twist.twist.linear.y*np.cos(self.deg)
        self.vdeg = data.twist.twist.angular.z
        #print(data.twist.twist.linear.x,data.twist.twist.angular.z)
        

def main():
    rclpy.init()
    yaslam = YaSlam()
    rclpy.spin(yaslam)

if __name__ == "__main__":
    main()
