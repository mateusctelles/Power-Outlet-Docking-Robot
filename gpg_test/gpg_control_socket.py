#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import Twist, Pose2D, TwistStamped, PointStamped
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import angles
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Float64


class GpgControllerNode(Node):
    def __init__(self):
        super().__init__("gpg_controller")
        self.first = True
        self.goal_x = 0.0
        self.area = 0.0
        self.count = 1

        self.declare_parameter('k_lin', 150.0) #pq? - sao os ganhos
        self.declare_parameter('k_ang', 0.00088) #pq? - sao os ganhos

        self.declare_parameter('tol', 4000)
        self.pose_ = Pose2D()

        self.cmd_vel_publisher_ = self.create_publisher(
            TwistStamped, 
            "/diff_drive_controller/cmd_vel", 
            10 ) # TROCAR NO ROBO
        
        #self.pose_subscriber_ = self.create_subscription(
         #   Odometry, "/diff_drive_controller/odom", self.robot_callback, 10  # TROCAR NO ROBO
        #)
        
        self.distance_subscriber_ = self.create_subscription(
            Float64, "distance", self.callback_distance, 10
        )

        self.angle_subscriber_ = self.create_subscription(
            Float64, "angle", self.callback_angle, 10
        )

        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)


        # Abaixo, estes sao os subscribers
        # segundo tutorial, eh ideal inicializar para evitar warnings
        
    

    def callback_distance(self, msg):
        # self.pose_ = msg # aqui ele passa todos os parametros do pose de uma vez, autoupdate sempre qd callback
        
        self.area = msg

        # self.goal_point_odom.header.frame_id = 'odom'
        # self.goal_point_odom.point.x = self.goal_x
        # self.goal_point_odom.point.y = self.goal_y
        # self.goal_point_odom.point.z = 0.0
    def callback_angle(self, msg):
        self.goal_x = msg
            
    def control_loop(self):

         # Transform goal position to base_link frame
        try:
            
            self.get_logger().warn("Updated Code")
            #goal_point_base_link = self.tf_buffer.transform(self.goal_point_odom, 'base_link')
            msg = TwistStamped()
           
            #if self.count == 1:
             #   msg.twist.angular.z = -0.035
              #  msg.twist.linear.x = 0.0

             #   msg.twist.angular.z = -0.001
                #msg.twist.linear.x = 0.0
            self.cmd_vel_publisher_.publish(msg)

            if self.area is not None and self.goal_x is not None and isinstance(self.area, Float64) and isinstance(self.goal_x, Float64):
                self.count = self.count + 1
                # Calculate distance and angle to the transformed goal
                distance_to_goal = self.area.data
                angle_to_goal = self.goal_x.data

                self.get_logger().info(f"Angle: {angle_to_goal}, Distance (AREA): {distance_to_goal}")

                tol = self.get_parameter('tol').get_parameter_value().double_value
                K_lin = self.get_parameter('k_lin').get_parameter_value().double_value
                K_ang = self.get_parameter('k_ang').get_parameter_value().double_value

                if distance_to_goal < 4000: #definir 0.5 como tol parameter
                    # position
                    self.get_logger().warn("Publishing Message")
                    msg.twist.linear.x = (K_lin * 1/distance_to_goal)/((0.045*angle_to_goal)**2) #+ K_ang*(1/(angle_to_goal**2))
                    #msg.twist.linear.x = (K_lin * 1/distance_to_goal)
                    if msg.twist.linear.x > 3.5:
                        msg.twist.linear.x = 3.5

                    msg.twist.angular.z = K_ang * -angle_to_goal
                    self.cmd_vel_publisher_.publish(msg)

                    self.get_logger().warn(f"Message Published: Linear: {msg.twist.linear.x}, Angular: {msg.twist.angular.z}")
                    

                else:
                    self.get_logger().warn("Tolerance Achieved")
                    # target reached!
                    msg.twist.linear.x = 0.0
                    msg.twist.angular.z = 0.0
                    self.cmd_vel_publisher_.publish(msg)

            else:
                msg.twist.linear.x = 0.0
                msg.twist.angular.z = 0.0
            #    self.get_logger().warn("Distance or angle data is not available or not a Float64 type")
            
           # self.cmd_vel_publisher_.publish(msg)

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"Transform lookup failed: {e}")
            return
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"Transform extrapolation failed: {e}")
            return
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return           

        #self.cmd_vel_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpgControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
