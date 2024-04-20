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
        self.landmarks = []
    def addLM(self,lm):
        self.landmarks.append(lm)
    def makeLocalMap(self,marks):
        lm = np.array([[np.cos(self.deg),-np.sin(self.deg),self.x],[np.sin(self.deg),np.cos(self.deg),self.y],[0,0,1]]) @ np.array([marks[0],marks[1],1])
        l = np.array([np.sqrt((i[0]-lm[0])**2 + (i[1]-lm[1])**2) for i in self.landmarks])
        self.errorSum += np.min(l)
    def sep(self):
        par = Particle(self.x,self.y,self.deg)
        par.errorSum = self.errorSum/2
        self.errorSum /=2
        par.landmarks = self.landmarks.copy()
        return par

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
        self.landmarks = []
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
            #cv2.circle(self.map_msg,(int(size/2-i.x*size/20),int(size/2+i.y*size/20)),3,(0,255,0),thickness=3,lineType=cv2.LINE_4,shift=0)
            cv2.arrowedLine(self.map_msg,(int(size/2-i.x*size/20),int(size/2+i.y*size/20)),(int(size/2-i.x*size/20-arrsize*np.cos(i.deg)),int(size/2+i.y*size/20+arrsize*np.sin(i.deg))),(55+200*i.w,0,0),thickness=4,line_type=cv2.LINE_4,shift=0)
        self.x = np.average(np.array([i.x for i in self.particles]))
        self.y = np.average(np.array([i.y for i in self.particles]))
        self.deg = np.average(np.array([i.deg for i in self.particles]))

        cv2.circle(self.particleMap,(int(size/2-self.x*size/20),int(size/2+self.y*size/20)),3,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
        cv2.circle(self.particleMap,(int(size/2),int(size/2)),3,(0,255,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
        
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
            #cv2.circle(self.map_msg,(int(x_-self.x*size/20),int(y_+self.y*size/20)),3,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
            cv2.circle(self.map_msg,(int(int(size/2+ran*np.sin(self.angle_min+self.angle_inc*i))),int(size/2+ran*np.cos(self.angle_min+self.angle_inc*i))),3,(255,0,0),thickness=-1,lineType=cv2.LINE_4,shift=0)
            #print((100+int(ran*np.sin(self.angle_min+self.angle_inc*i)),100+int(ran*np.cos(self.angle_min+self.angle_inc*i))))
        
        #mk landmark
        self.map_msg = np.array(self.map_msg,dtype=np.uint8)
        gray = cv2.cvtColor(self.map_msg, cv2.COLOR_BGR2GRAY)
        lines = cv2.HoughLinesP(gray,rho=1, theta=np.pi/360, threshold=80, minLineLength=0, maxLineGap=30)
        if lines is not None:
            for i in lines:
                
                i = np.squeeze(i)
                #cv2.circle(self.map_msg,(int(i[0]),int(i[1])),3,(0,255,255),thickness=3,lineType=cv2.LINE_4,shift=0)
                #cv2.line(self.map_msg,(int(i[0]),int(i[1])),(int(i[2]),int(i[3])),(0,255,0),thickness=1,lineType=cv2.LINE_4,shift=0)
                #cv2.circle(self.map_msg,(int(i[2]),int(i[3])),3,(0,255,255),thickness=3,lineType=cv2.LINE_4,shift=0)
                bit = 5
                self.markmap[int(i[0])-bit:int(i[0])+bit,int(i[1])-bit:int(i[1])+bit] += 1
                self.markmap[int(i[2])-bit:int(i[2])+bit,int(i[3])-bit:int(i[3])+bit] += 1
                lmThreshold = 15#0.001
                bit_range = 15
                if self.markmap[int(i[0]),int(i[1])] > lmThreshold:#*np.sum(self.markmap):
                    flag = True
                    length = np.array([np.sqrt((i[0]-lm[0])**2+(i[1]-lm[1])**2) for lm in self.landmarks])
                    if len(self.landmarks) != 0 and np.min(length) < bit_range:
                        for par in self.particles:
                            par.makeLocalMap([i[0],i[1]])
                    else:
                        self.landmarks.append([i[0],i[1]])
                        for par in self.particles:
                            par.addLM([i[0],i[1]])
                        self.markmap = np.zeros((size,size))
                if self.markmap[int(i[2]),int(i[3])] > lmThreshold:#*np.sum(self.markmap):
                    flag = True
                    length = np.array([np.sqrt((i[2]-lm[0])**2+(i[3]-lm[1])**2) for lm in self.landmarks])
                    if len(self.landmarks) != 0 and np.min(length) < bit_range:
                        for par in self.particles:
                                par.makeLocalMap([i[2],i[3]])
                    else:
                        self.landmarks.append([i[2],i[3]])
                        for par in self.particles:
                            par.addLM([i[0],i[1]])
                        self.markmap = np.zeros((size,size))
        
        #cv2.circle(self.map_msg,(int(size/2-self.x*size/20),int(size/2+self.y*size/20)),3,(0,255,0),thickness=3,lineType=cv2.LINE_4,shift=0)
        errors = np.sum(np.array([i.errorSum for i in self.particles]))
        if errors != 0:
            k = 0
            for i in self.particles:
                i.w = i.errorSum/errors
                if i.w < 0.1/perticle_num:
                    self.particles.pop(k)
                if i.w > 1.1/perticle_num:
                    self.particles.append(i.sep())
                k+=1

        for i in self.landmarks:
            cv2.circle(self.map_msg,(int(i[0]),int(i[1])),3,(0,0,255),thickness=3,lineType=cv2.LINE_4,shift=0)
        #print(len(self.landmarks))
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
        #plt.imshow(self.markmap)
        plt.imshow(cv2.resize(self.map_msg,dsize = None,fx = 0.3,fy = 0.3))
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
