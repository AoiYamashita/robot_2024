import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class serial_indep(Node):
    def __init__(self):
        self.serial0 = serial.Serial("/dev/ttyACM0",115200,timeout=0.1)
        self.serial1 = serial.Serial("/dev/ttyACM1",115200,timeout=0.1)
        self.serial2 = serial.Serial("/dev/ttyACM2",115200,timeout=0.1)
        super().__init__("serial_indep")
        self.ids = ["","",""]
        self.setId()
        self.subscription0 = self.create_subscription(String,self.ids[0],self.cb0,10)
        self.subscription1 = self.create_subscription(String,self.ids[1],self.cb1,10)
        self.subscription2 = self.create_subscription(String,self.ids[2],self.cb2,10)
        self.subscriptionAir = self.create_subscription(String,"Air",self.AirCb,10)
        self.timer = self.create_timer(0.1,self.read)
    def setId(self):
        self.serial0.write("name".encode('utf-8'))
        data = ""
        while data == "":
            data = self.serial0.readline()
            print("waiting")
        data=data.strip()
        data=data.decode('utf-8')
        if data != "":
            self.get_logger().info("serial0 id is %s" % data)
            self.ids[int(data)] = "Serial_data0"

        self.serial1.write("name".encode('utf-8'))
        data = ""
        while data == "":
            data = self.serial1.readline()
        data=data.strip()
        data=data.decode('utf-8')
        if data != "":
            self.get_logger().info("serial1 id is %s" % data)
            self.ids[int(data)] = "Serial_data1"
        
        self.serial2.write("name".encode('utf-8'))
        data = ""
        while data == "":
            data = self.serial2.readline()
        data=data.strip()
        data=data.decode('utf-8')
        if data != "":
            self.get_logger().info("serial2 id is %s" % data)
            self.ids[int(data)] = "Serial_data2"
    def cb0(self,data):
        a = data.data
        self.serial0.write(a.encode('utf-8'))
        self.get_logger().info("send0: %s" % a)
    def cb1(self,data):
        a = data.data
        self.serial1.write(a.encode('utf-8'))
        self.get_logger().info("send1: %s" % a)
    def cb2(self,data):
        a = data.data
        self.serial2.write(a.encode('utf-8'))
        self.get_logger().info("send2: %s" % a)
    def AirCb(self,data):
        a = data.data
        if self.ids[2] == "Serial_data0":
            self.serial0.write(a.encode('utf-8'))
        if self.ids[2] == "Serial_data1":
            self.serial1.write(a.encode('utf-8'))
        if self.ids[2] == "Serial_data2":
            self.serial2.write(a.encode('utf-8'))
        self.get_logger().info("send2: %s" % a)
    def read(self):
        data = self.serial0.readline()
        data=data.strip()
        data=data.decode('utf-8')
        if data != "":
            self.get_logger().info("read0: %s" % data)
        data = self.serial1.readline()
        data=data.strip()
        data=data.decode('utf-8')
        if data != "":
            self.get_logger().info("read1: %s" % data)
        data = self.serial2.readline()
        data=data.strip()
        data=data.decode('utf-8')
        if data != "":
            self.get_logger().info("read1: %s" % data)

def main():
    rclpy.init()
    serialIndep = serial_indep()
    rclpy.spin(serialIndep)
