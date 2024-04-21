import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import UInt8
import numpy as np

class controller(Node):
    def __init__(self):
        super().__init__("controller")
        self.pub0 = self.create_publisher(String,"Serial_data0",10)
        self.pub1 = self.create_publisher(String,"Serial_data1",10)
        self.pub2 = self.create_publisher(String,"Serial_data2",10)
        self.sub = self.create_subscription(Twist,"controll",self.cb,10)

        self.pubAir = self.create_publisher(String,"Air",10)
        self.subAir = self.create_subscription(UInt8,"AirControll",self.cbAir,10)
    def cb(self,data):
        speedx = data.linear.x
        speedy = data.linear.y
        arg = data.angular.z
        if arg != 0:
            #arg = 0
            msg = String()
            msg.data = 'speed:{}:deg:{}'.format(int(arg),int(135))
            self.pub0.publish(msg)
            msg = String()
            msg.data = 'speed:{}:deg:{}'.format(int(arg),int(45))
            self.pub1.publish(msg)
            msg = String()
            msg.data = 'speed:{}:deg:{}'.format(int(arg),int(270))
            self.pub2.publish(msg)
            return
        deg = np.arctan2(speedy,speedx)
        msg = String()
        msg.data = 'speed:{}:deg:{}'.format(int(np.sqrt(speedx**2+speedy**2)),int(np.degrees(deg)))
        self.pub0.publish(msg)
        self.pub1.publish(msg)
        self.pub2.publish(msg)
    def cbAir(self,data):
        msg = String()
        msg.data = 'Air:{}'.format(int(data.data))
        self.pubAir.publish(msg)

def main():
    rclpy.init()
    webcontroll = controller()
    rclpy.spin(webcontroll)

if __name__ == "__main__":
    main()
