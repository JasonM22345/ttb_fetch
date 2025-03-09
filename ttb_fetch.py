import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import tf2_ros
import tf2_geometry_msgs
from tf2_msgs.msg import TFMessage
import struct

from sensor_msgs.msg import Imu, Range, PointCloud2, LaserScan
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from irobot_create_msgs.msg import KidnapStatus

import random
import numpy as np
import time
import math

class TTBFetchController(Node):

    def __init__(self):
        super().__init__('ttb_fetch')

        # setup publisher for Twist msg to /TTB02/cmd_vel with buffer size = 10
        self.publisher_ = self.create_publisher(
            Twist,
            '/TTB02/cmd_vel',
            10
        )

        # setup subscriber to Imu msg from /TTB01/imu with buffer size = 10
        qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        static_qos_profile = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/TTB02/odom',
            self.odom_callback,
            qos_profile
        )

        self.pc_subscriber = self.create_subscription(
            PointCloud2,
            '/TTB02/realsense/depth/color/points',
            self.pointcloud_callback,
            qos_profile
        )
        self.subscriber_scan = self.create_subscription(
            LaserScan,
            '/TTB02/scan',
            self.scan_callback,
            qos_profile
        )
        self.dynamic_subscriber = self.create_subscription(
            TFMessage, 
            '/TTB02/tf', 
            self.tf_callback, 
            qos_profile
        )
        self.static_subscriber = self.create_subscription(
            TFMessage, 
            '/TTB02/tf_static', 
            self.tf_static_callback, 
            static_qos_profile
        )

        self.pick_up_subscriber = self.create_subscription(
            KidnapStatus, 
            '/TTB02/kidnap_status', 
            self.kidnap_callback, 
            qos_profile
        )
        

        self.ball_marker_pub = self.create_publisher(Marker, '/TTB02/ball_marker', 10)


        # setup controller to run at 10hz (period=.1s) and call method controller_callback
        timer_period=.01
        self.timer = self.create_timer(timer_period, self.controller_callback)
        self.lin_x = 0.0
        self.ang_z = 0.0
        self.initial_x = 0.0
        self.initial_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.old_goal_x = 0.0
        self.old_goal_y = 0.0
        self.first_pos = True
        self.kv = 0.1
        self.kp = 1.8
        self.desired_theta = 0
        self.distance = 0.0
        self.found_ball = False
        self.fetched_ball = False
        self.found_human = False
        self.returned_ball = False
        self.follow_human = False
        self.follow_debug = False
        self.picked_up = False
        self.red_threshold = 65
        self.green_threshold = 45
        self.blue_threshold = 80
        self.stabilization_threshold = 0.01
        self.ball_samples = []
        self.num_ball_samples = 10
        self.current_x = 0.0
        self.current_y = 0.0
        self.yaw_z = 0.0
        self.follow_debug_count = 0.0
        self.dynamic_transforms = {}
        self.static_transforms = {}
        self.all_transforms = {}
        self.static_recieved = False
        self.max_velocity = 0.4
        self.closest_obstacle_distance = float("inf")
        self.closest_obstacle_angle = 0.0
        self.safe_distance = 0.7
        self.max_obs_range = 1.2
        self.robot_front = -np.pi/2 # angle directly infront of the robot

        # Colors [r, g, b] of our actual balls
        self.tennis_ball = [229, 240, 39]
        self.blue_ball = [0, 179, 232]


        # Current ball
        self.ball = self.tennis_ball
        



        
        
    def publish_goal_marker(self, publisher, x, y, z):
        """Publish the goal marker."""
        goal_marker = Marker()
        goal_marker.header.frame_id = "odom"
        goal_marker.id = 0
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = float(x)
        goal_marker.pose.position.y = float(y)
        goal_marker.pose.position.z = float(z)
        goal_marker.scale.x = 0.3
        goal_marker.scale.y = 0.3
        goal_marker.scale.z = 0.3
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        goal_marker.color.a = 1.0
        publisher.publish(goal_marker)



    def controller_callback(self):
        msg = Twist()
        
        if not self.found_ball:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            return
        dbx = self.old_goal_x - self.current_x
        dby = self.old_goal_y - self.current_y
        distance_from_ball = np.sqrt((dbx)**2+(dby)**2)
         # If an obstacle is within the safe distance     
        if self.closest_obstacle_distance < self.safe_distance and self.closest_obstacle_distance > 0.25 and self.closest_obstacle_distance != float("inf") and self.distance > 0.45 and distance_from_ball > 0.45:

            if self.closest_obstacle_distance <= 0.15:
                msg.linear.x = 0.0
            else:
                msg.linear.x = 0.1  # Move forward slowly to steer away

            # Determine if the obstacle is on the left or right
            if self.closest_obstacle_angle < -1.5:
                msg.angular.z += 0.5  # Turn left
            else:
                msg.angular.z += -0.5  # Turn right

        else:
            msg.linear.x = self.lin_x
            msg.angular.z = self.ang_z
        self.publisher_.publish(msg)



    def kidnap_callback(self, msg):
        self.picked_up = msg.is_kidnapped
    
    def pointcloud_callback(self, msg):
        # Convert PointCloud2 to a NumPy structured array
        # Parse the point cloud fields to find offsets for x, y, z, and rgb

        # don't need to find the goal if we already found the ball or returned it
        if self.found_ball and not self.fetched_ball:
            return
        
        # get the field names that contain x, y, z
        field_names = {field.name: field.offset for field in msg.fields}
        point_step = msg.point_step

        # List to store extracted points with RGB
        extracted_points = []

        for i in range(msg.width):  # Loop through all points
            offset = i * point_step

            # Unpack x, y, z
            x, y, z = struct.unpack_from('fff', msg.data, offset + field_names['x'])

            # Unpack RGB
            rgb_offset = offset + field_names['rgb']
            rgb_int = struct.unpack_from('I', msg.data, rgb_offset)[0]
            r = (rgb_int & 0x00FF0000) >> 16
            g = (rgb_int & 0x0000FF00) >> 8
            b = (rgb_int & 0x000000FF)
            extracted_points.append([x, y, z, r, g, b])
        # find the reddest and bluest point
        transformed_points = self.convert_pointcloud2_to_odom(extracted_points, msg.header.frame_id)

        greenest_point, max_green_level = self.find_greenest_point(transformed_points)
        bluest_point, max_blue_level = self.find_bluest_point(transformed_points)
        # ensure that the reddest point passes the threshold and that we have not found the ball
        print("Green Point:", greenest_point)
        print("Max Green Level:", max_green_level)
        if max_green_level >= self.green_threshold and not self.found_ball:
            # sample every 5 points that are detected to "debounce" the ball
            self.ball_samples.append(greenest_point)
            if len(self.ball_samples) > self.num_ball_samples:
                print("Hit", self.num_ball_samples, "samples")
                ball_array = np.array(self.ball_samples)
                # make sure there are minimal oscillations in ball position so we know ball stopped moving
                var_x = np.var(ball_array[:, 0])
                var_y = np.var(ball_array[:, 1])
                var_z = np.var(ball_array[:, 2])
                print("Variance of samples:", var_x, var_y, var_z)
                # make sure ball stays in roughly the same position by checking if variances in samples are small enough
                if var_x <= self.stabilization_threshold and var_y <= self.stabilization_threshold and var_z <= self.stabilization_threshold:
                    self.goal_x = greenest_point[0]
                    self.goal_y = greenest_point[1]
                    print("Ball stabilized")
                    print("Greenest Point:", greenest_point)
                    print("Max Green Level:", max_green_level)
                    # found the ball! now we go to the ball
                    self.found_ball = True
                    self.publish_goal_marker(self.ball_marker_pub, self.goal_x, self.goal_y, 0.0)
                else:
                    # if there are still oscillations restart the samples
                    self.ball_samples = []
        # ensure that the bluest point meets the threshold and that we've already fetched the ball so we know we need to return back to the human
        if max_blue_level >= self.blue_threshold and self.fetched_ball:
            print("Bluest Point:", bluest_point)
            print("Max Blue Level:", max_blue_level)
            x_blue = bluest_point[0]
            y_blue = bluest_point[1]
            # camera data to odom transformation
            self.goal_x = x_blue
            self.goal_y = y_blue
            print("New goal:", self.goal_x, self.goal_y, "Old goal:", self.old_goal_x, self.old_goal_y, "Current pos:", self.current_x, self.current_y)
            self.publish_goal_marker(self.ball_marker_pub, self.goal_x, self.goal_y, bluest_point[2])
            # signal that you found the human
            self.found_human = True
    
    def convert_pointcloud2_to_odom(self, extracted_points, source_frame):
        transformed_points = []
        chain = self.get_transform_chain(source_frame, "odom")
        while chain == None:
                chain = self.get_transform_chain(source_frame, "odom")
        
        # Convert extracted points to a NumPy array for batch processing
        points = np.array([[x, y, z] for x, y, z, _, _, _ in extracted_points])
        transformed_points = self.apply_transform_chain(points, chain)
        
        # Add RGB back to transformed points
        transformed_points = [
            (pt[0], pt[1], pt[2], r, g, b)
            for pt, (_, _, _, r, g, b) in zip(transformed_points, extracted_points)
        ]
        return transformed_points


    def find_reddest_point(self, extracted_points):
        # finds the reddest point given that ideal red is (r, g, b) = (255, 0, 0)
        reddest_point = (0, 0, 0)
        max_red_level = float("-inf")
        for point in extracted_points:
            x, y, z, r, g, b = point
            red_level = r - g - b
            if red_level > max_red_level:
                max_red_level = red_level
                reddest_point = (x, y, z, r, g, b)
        return reddest_point, max_red_level
    
    def find_greenest_point(self, extracted_points):
        # finds the greenest point given that ideal green is (r, g, b) = (0, 255, 0)
        greenest_point = (0, 0, 0)
        min_green_level = float("inf")
        for point in extracted_points:
            x, y, z, r, g, b = point
            green_level = np.sqrt((r-self.ball[0])**2+(g-self.ball[1])**2+(b-self.ball[2])**2)
            if green_level < min_green_level:
                min_green_level = green_level
                greenest_point = (x, y, z, r, g, b)
        return greenest_point, min_green_level
    
    def find_bluest_point(self, extracted_points):
        # finds the bluest point given that ideal blue is (r, g, b) = (0, 0, 255)
        bluest_point = (0, 0, 0)
        max_blue_level = float("-inf")
        for point in extracted_points:
            x, y, z, r, g, b = point
            blue_level = b - g - r
            if blue_level > max_blue_level:
                max_blue_level = blue_level
                bluest_point = (x, y, z, r, g, b)
        return bluest_point, max_blue_level



    def convert_lidar_to_odom(self, msg):
        ranges = msg.ranges
        closest_point = (0, 0)
        closest_range = float("inf")
        range_index = 0
        range_intensity = 0.0
        closest_theta = 0.0
        for r in range(len(ranges)):
            theta = msg.angle_min + r*msg.angle_increment
            # weird angle ranges that the Lidar picks up where the lidar points are on the robot itself
            if (theta >= -0.65 and theta<=-0.5) or (theta >= -2.65 and theta<=-2.5):
                continue
            # theta requires rotation by 90 degrees to properly map to the lidar point
            theta = theta + np.pi/2
            magnitude = msg.ranges[r]
            # skip lidar points that aren't detected
            if magnitude == float("inf") :
                continue
            x_lidar = magnitude*np.cos(theta) 
            y_lidar = magnitude*np.sin(theta)
            # lidar to odom transformation
            x = self.current_x + x_lidar*np.cos(self.yaw_z) - y_lidar*np.sin(self.yaw_z)
            y = self.current_y + x_lidar*np.sin(self.yaw_z) + y_lidar*np.cos(self.yaw_z)
            # find closest lidar point
            if magnitude < closest_range:
                closest_point = (x, y)
                closest_range = magnitude
                range_index = r
                closest_theta = theta
        return closest_point


    
    def scan_callback(self, msg):
            ranges = msg.ranges
            closest_distance = float("inf")
            closest_angle = 0.0

            # Front 180 degrees corresponds to angles between -pi/2 and +pi/2
            for i, distance in enumerate(ranges):
                angle = msg.angle_min + i * msg.angle_increment
                #if -np.pi / 2 <= angle <= np.pi / 2:  # Limit angle
                
                if self.robot_front - np.pi/3 <= angle <= self.robot_front + np.pi/3:  # Limit angle
                    if distance < closest_distance and distance > 0.15 and distance <= self.max_obs_range:
                        closest_distance = distance
                        closest_angle = angle

            self.closest_obstacle_distance = closest_distance
            self.closest_obstacle_angle = closest_angle


    

        	
    def odom_callback(self, msg):
        if self.first_pos:
            self.initial_x = msg.pose.pose.position.x
            self.initial_y = msg.pose.pose.position.y
            self.first_pos = False

        if self.picked_up:
            self.found_ball = False
            self.fetched_ball = False
            self.found_human = False
            self.returned_ball = False
            self.follow_debug = False
            self.lin_x = 0.0
            self.ang_z = 0.0
            self.kv = 0.1
            self.kp = 1.8
            return

        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        dx = self.goal_x-self.current_x
        dy = self.goal_y-self.current_y
        self.distance = (dx)**2+(dy)**2
        # check if robot reached the ball (or if you want to debug following the human)
        #self.found_ball = True
        if (self.distance < 0.05 and self.found_ball and not self.fetched_ball) or (self.follow_debug):
            print("Fetched the ball!")
            if self.follow_debug_count == 0:
                self.old_goal_x = self.goal_x
                self.old_goal_y = self.goal_y
            self.follow_debug_count+=1
            self.fetched_ball = True

            # Added to ensure it gets the ball, and does not stop short of it.
            msg = Twist() 
            msg.linear.x = 0.1
            self.publisher_.publish(msg)
            time.sleep(1)
            msg.linear.x = self.lin_x
            self.publisher_.publish(msg)
        # keep rotating until you found a human
        if self.fetched_ball and not self.found_human:
            self.lin_x = 0.0
            self.ang_z = 0.1
            return
        # if the robot has switched to finding the closest lidar point because it's close to the human, stop if its too close to the human
        if self.distance < 0.2 and self.found_human:
            print("Reached Human! Returning ball")
            self.lin_x = 0.0
            self.ang_z = 0.0
            return

        self.lin_x = min(self.kv*np.sqrt(self.distance), self.max_velocity)
        self.desired_theta = np.arctan2(dy, dx)
        q_x =  msg.pose.pose.orientation.x
        q_y =  msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        t3 = 2.0*(q_w*q_z+q_x*q_y)
        t4 = 1.0 - 2.0*(q_y*q_y + q_z*q_z)
        self.yaw_z = np.arctan2(t3, t4)
        self.ang_z = self.kp*(self.desired_theta -self.yaw_z)


    def tf_callback(self, msg):
        # callback for dynamic transforms 
        self.update_transforms(self.dynamic_transforms, msg)
        self.combine_transforms()

    def tf_static_callback(self, msg):
        # callback for static transforms
        if self.static_recieved:
            return
        self.update_transforms(self.static_transforms, msg)
        self.static_recieved = True

    def update_transforms(self, transform_dict, msg):
        for transform in msg.transforms:
            parent = transform.header.frame_id
            child = transform.child_frame_id
            transform_dict[(parent, child)] = transform
    
    def combine_transforms(self):
        # combine static and dynamic transforms into one tree
       self.all_transforms = {**self.dynamic_transforms, **self.static_transforms}

    def get_transform_chain(self, from_frame, to_frame):
        """Finds the chain of transforms from one frame to another."""
        chain = []
        current_frame = from_frame
        while current_frame != to_frame:
            found = False
            for (parent, child), transform in self.all_transforms.items():
                if child == current_frame:
                    chain.append(transform)
                    current_frame = parent
                    found = True
                    break
            if not found:
                self.get_logger().error(f"Cannot find transform from {current_frame} to {to_frame}")
                return None
        return chain

    def apply_transform_chain(self, points, chain):
        """Applies a chain of transforms to a batch of points."""
        points_h = np.hstack((points, np.ones((points.shape[0], 1))))  # Homogeneous coordinates
        
        for transform in chain:
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            rotation_matrix = self.quaternion_to_matrix(qx, qy, qz, qw)
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, :3] = rotation_matrix
            transformation_matrix[:3, 3] = [tx, ty, tz]

            points_h = points_h @ transformation_matrix.T  # Matrix multiplication
        
        return points_h[:, :3]

    def quaternion_to_matrix(self, qx, qy, qz, qw):
        """Converts a quaternion to a 3x3 rotation matrix."""
        q = np.array([qw, qx, qy, qz])
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2 * (q[2]**2 + q[3]**2)
        R[0, 1] = 2 * (q[1] * q[2] - q[3] * q[0])
        R[0, 2] = 2 * (q[1] * q[3] + q[2] * q[0])
        R[1, 0] = 2 * (q[1] * q[2] + q[3] * q[0])
        R[1, 1] = 1 - 2 * (q[1]**2 + q[3]**2)
        R[1, 2] = 2 * (q[2] * q[3] - q[1] * q[0])
        R[2, 0] = 2 * (q[1] * q[3] - q[2] * q[0])
        R[2, 1] = 2 * (q[2] * q[3] + q[1] * q[0])
        R[2, 2] = 1 - 2 * (q[1]**2 + q[2]**2)
        return R
     	
     
    	
def main(args=None):
    rclpy.init(args=args)
    ttb_controller = TTBFetchController()
    rclpy.spin(ttb_controller)
    ttb_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main(6)
