#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtle_msgs.msg import KillTurtle
from turtle_msgs.msg import AliveTurtle
import random
import math
from functools import partial


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner_node")
        self.count_ = 1
        self.declare_parameter("Spawn_TimePeriod", 1.5)
        self.spawnTimePeriod = self.get_parameter("Spawn_TimePeriod").value
        self.publisher_ = self.create_publisher(AliveTurtle, "turtle_alive", 10)
        self.subscriber_ = self.create_subscription(KillTurtle, "turtle_kill", self.callbackTopic_turtleKillName, 10)
        self.timer_ = self.create_timer(self.spawnTimePeriod, self.turtleSpawner)
        self.get_logger().warn("Turtle Spawner cum Killer Node has Started...")


    def turtleSpawner(self):
        self.count_ += 1
        nx = round((random.uniform(0.5, 10.5)), 5)
        ny = round((random.uniform(0.5, 10.5)), 5)
        theta = random.uniform(0.0, 2*math.pi)
        turtle_name = str("Turtle"+str(self.count_))

        self.call_turtleSpawner(nx, ny, theta, turtle_name)
        
    def call_turtleSpawner(self, x, y, theta, name):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Turtlesim_Node_Spawn ....")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_turtleSpawner, x=x, y=y, theta=theta, name=name))

    def callback_turtleSpawner(self, future, x, y, theta, name):
        try:
            response = future.result()
            if response.name != "":
                self.publishCallback_AliveTurtle(x, y)
        except Exception as e:
            self.get_logger().error("Request Failed %r " % (e,))

        
    def turtleKiller(self, name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1):
            self.get_logger().warn("Waiting for Server Turtlesim_Node_Kill ....")
        
        request = Kill.Request()
        request.name = name

        future = client.call_async(request)
        future.add_done_callback(self.callback_turtleKiller)

    def callback_turtleKiller(self, future):
        try:
            response = future.result()
            self.get_logger().info("Killed Successfully")
        except Exception as e:
            self.get_logger().error("Request Failed %r " % (e,))


    def callbackTopic_turtleKillName(self, msg):
        kill_name_ = msg.kill_name
        self.turtleKiller(kill_name_)


    def publishCallback_AliveTurtle(self, nx, ny):
        msg = AliveTurtle()
        msg.x_posn = nx
        msg.y_posn = ny
        msg.name_count = self.count_
        self.publisher_.publish(msg)



    
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()


