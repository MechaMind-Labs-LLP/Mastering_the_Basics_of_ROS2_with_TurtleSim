#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtle_msgs.msg import AliveTurtle
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtle_msgs.msg import KillTurtle
import math
    
    
class TurtleControllerNode(Node): 
    def __init__(self):
        super().__init__("turtle_controller_node") 
        self.name_array = [] #unsorted
        self.sorted_names = [] #sorted
        self.current_posn_turtle1 = []
        self.turtle_posn_to_go = [] #unsorted
        self.target_posn_turtle1 = [] #sorted
        self.publish_twist = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.publish_kill = self.create_publisher(KillTurtle, "turtle_kill", 10)
        self.subscriber_ = self.create_subscription(AliveTurtle, "turtle_alive", self.callbackTopic_TurtleAliveData, 10)
        self.subscriber_pose_ = self.create_subscription(Pose, "/turtle1/pose", self.callback_pose, 10)

    def callbackTopic_TurtleAliveData(self, msg):
        x_posn = msg.x_posn
        y_posn = msg.y_posn
        name_count = msg.name_count
        self.turtle_posn_to_go.append((x_posn, y_posn))
        self.name_array.append(name_count)
        self.get_logger().info(str(self.name_array))


    def callback_pose(self, msg):
        current_x = msg.x
        current_y = msg.y
        current_theta = msg.theta
        current_LVel = msg.linear_velocity
        current_AVel = msg.angular_velocity
        self.current_posn_turtle1 = [current_x, current_y, current_theta, current_LVel, current_AVel]

        if self.turtle_posn_to_go:
            self.turtlePosn_Nearest_Calculation()
            self.move_turtle_to_nearest_point()


    def turtlePosn_Nearest_Calculation(self):
        x1 = self.current_posn_turtle1[0]
        y1 = self.current_posn_turtle1[1]
        reference_point = [x1, y1]
        coordinates = self.turtle_posn_to_go
        coordinate_names = self.name_array

        def calculate_distance(coord1, coord2):
            # Calculate Euclidean distance between two coordinates
            return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

        def rearrange_based_on_distance(coordinates, coordinate_names, reference_point):
            # Combine coordinates and their names into a list of tuples
            combined = list(zip(coordinates, coordinate_names))
            
            # Sort combined list based on the distance from the reference point
            combined_sorted = sorted(combined, key=lambda pair: calculate_distance(pair[0], reference_point))
            
            # Unzip the sorted pairs back into two separate lists
            sorted_coordinates, sorted_names = zip(*combined_sorted)
            
            return list(sorted_coordinates), list(sorted_names)

        # Rearrange based on distance from the reference point
        sorted_coordinates, sorted_names = rearrange_based_on_distance(coordinates, coordinate_names, reference_point)
        self.target_posn_turtle1 = sorted_coordinates
        self.sorted_names = sorted_names
        
    
    def move_turtle_to_nearest_point(self):
        if not self.target_posn_turtle1:
            return
        target_x = self.target_posn_turtle1[0][0]
        target_y = self.target_posn_turtle1[0][1]

        kp_linear = 2.0
        kp_angular = 6.0

        current_x = self.current_posn_turtle1[0]
        current_y = self.current_posn_turtle1[1]
        current_theta = self.current_posn_turtle1[2]

        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate the angle to the target point
        angle_to_target = math.atan2(dy, dx)
        
        # Angular control: rotate towards the target
        angular_error = angle_to_target - current_theta
        if(angular_error > math.pi):
            angular_error -= 2*math.pi
        elif(angular_error < -math.pi):
            angular_error += 2*math.pi
        angular_velocity = kp_angular * angular_error
        
        # Linear control: move forward
        linear_velocity = kp_linear * distance

        # Publish the velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.publish_twist.publish(twist_msg)
        
        # Stop the turtle when it's close enough to the target
        if distance < 0.5:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publish_twist.publish(twist_msg)
            self.get_logger().info('Target reached!')

            array_of_arrays = self.turtle_posn_to_go
            sub_array_to_remove = self.target_posn_turtle1[0]
            if sub_array_to_remove in array_of_arrays:
                array_of_arrays.remove(sub_array_to_remove)

            name_to_remove = self.sorted_names[0]
            if name_to_remove in self.name_array:
                self.name_array.remove(name_to_remove)  

            self.turtleKill()

    def turtleKill(self):
        if not self.sorted_names:
            return
        msg = KillTurtle()
        msg.kill_name = str("Turtle"+str(self.sorted_names[0]))
        self.publish_kill.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode() 
    rclpy.spin(node)
    rclpy.shutdown()
    
    
if __name__ == "__main__":
    main()
