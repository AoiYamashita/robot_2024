import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tkinter as tk

class Cmdvel(Node):
    def __init__(self):
        super().__init__("cmdvel")
        self.pub = self.create_publisher(Twist,"first_robot/cmd_vel",10)
        self.timer = self.create_timer(0.1,self.cb)
        self.velocity = Twist()
    def cb(self):
        """
        self.velocity.linear.x = self.linear_x
        self.velocity.linear.y = self.linear_y
        self.velocity.linear.z = self.linear_z
        self.velocity.angular.x = self.angle_x
        self.velocity.angular.y = self.angle_y
        self.velocity.angular.z = self.angle_z
        """
        self.pub.publish(self.velocity)

rclpy.init(args=None)
frc = Cmdvel()
root = tk.Tk()
root.title("Tkinter test")
root.geometry("640x480")

def key_event(e):
    frc.velocity.linear.x = 0.0
    frc.velocity.linear.y = 0.0
    frc.velocity.linear.z = 0.0
    frc.velocity.angular.x = 0.0
    frc.velocity.angular.y = 0.0
    frc.velocity.angular.z = 0.0
    if e.keysym == 'w':
       frc.velocity.linear.x = 10.0
    if e.keysym == 'a':
       frc.velocity.linear.y = 10.0
    if e.keysym == 'd':
       frc.velocity.linear.y = -10.0
    if e.keysym == 's':
       frc.velocity.linear.x = -10.0
    if e.keysym == 'q':
       frc.velocity.angular.z = 1.0
    if e.keysym == 'e':
       frc.velocity.angular.z = -1.0
    frc.cb()

def key(e):
    frc.velocity.linear.x = 0.0
    frc.velocity.linear.y = 0.0
    frc.velocity.linear.z = 0.0
    frc.velocity.angular.x = 0.0
    frc.velocity.angular.y = 0.0
    frc.velocity.angular.z = 0.0
    frc.cb()

def main(args=None):
    root.bind("<KeyPress>", key_event)
    root.bind("<KeyRelease>", key)
    root.mainloop()
    #rclpy.spin(frc)
    rclpy.shutdown()

if __name__ == "__main__":
    main()