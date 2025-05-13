#!/usr/bin/env python3
import rclpy
import rclpy.clock
from rclpy.node import Node
from rclpy.timer import Timer
from sensor_msgs.msg import Image, PointCloud2, PointField
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch
from tf_transformations import quaternion_matrix
import struct
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
from scipy.spatial import cKDTree
from efficient_track_anything.build_efficienttam import build_efficienttam_camera_predictor
import message_filters
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time

class InteractiveTrackingNode(Node):
    def __init__(self):
        super().__init__('interactive_tracking_node')
        self.bridge = CvBridge()

        # Segmentation state variables
        self.frame = None
        self.segmentation_done = False
        self.clicked_points = []
        self.clicked_labels = []
        self.current_obj_id = 1
        self.ann_frame_idx = 0  # annotation frame index
        self.init_frame = None
        self.new_frame = None
        self.new_mask = None
        self.reset_pending = False
        self.saved_tracking_result = None  # 用于保存跟踪结果
        
        # Model configuration
        self.checkpoint = "/home/arvin/Documents/real-time-eta/checkpoints/efficienttam_ti_512x512.pt"
        self.model_cfg = "configs/efficienttam/efficienttam_ti_512x512.yaml"
        # self.checkpoint = "/home/arvin/Documents/real-time-eta/checkpoints/efficienttam_s_512x512.pt"
        # self.model_cfg = "configs/efficienttam/efficienttam_s_512x512.yaml"
        # self.checkpoint = "/home/arvin/Documents/real-time-eta/checkpoints/efficienttam_s.pt"
        # self.model_cfg = "configs/efficienttam/efficienttam_s.yaml"
        self.predictor = build_efficienttam_camera_predictor(self.model_cfg, self.checkpoint)
        self.get_logger().info("Model loaded with config {}!".format(self.model_cfg))
        self.is_labeled = False

        # Tracking state variables
        self.depth_image = None
        self.mask_image = None
        self.latest_odom = None
        self.object_points_world = []
        self.track_start_time = None
        self.prev_world = None

        # Camera intrinsics and transforms
        self.K = np.array([
            [462.1379699707031, 0, 320.0],
            [0, 462.1379699707031, 240.0],
            [0, 0, 1]
        ])
        # Camera to base_link transform
        self.camera_to_base = np.array([
            [0.0,  0.0,  1.0,  0.310],
            [-1.0,  0.0,  0.0,  0.033],
            [0.0, -1.0,  0.0,  0.083],
            [0, 0, 0, 1]
        ])
        # LiDAR to base_link transform
        self.livox_to_base = np.array([
            [1.0, 0.0, 0.0, 0.2],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.12],
            [0.0, 0.0, 0.0, 1.0]
        ])

        # Create QoS profile for subscribers to ensure matching behavior
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers with message_filters
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            '/color/image_raw',
            qos_profile=qos)
            
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            '/aligned_depth/image_raw',
            qos_profile=qos)
            
        self.lidar_sub = message_filters.Subscriber(
            self,
            PointCloud2,
            '/mid360_PointCloud2',
            qos_profile=qos)

        self.odom_sub = message_filters.Subscriber(
            self,
            Odometry,
            '/odom',
            qos_profile=qos)

        # Synchronize the three topics with ApproximateTimeSynchronizer
        # Parameters: (subscribers, queue_size, max_interval_sec)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.lidar_sub, self.odom_sub],
            queue_size=10,
            slop=0.05)  # 50ms time difference tolerance
        
        # Register the synchronized callback
        self.sync.registerCallback(self.synchronized_callback)

        # Publishers
        self.mask_publisher = self.create_publisher(
            Image,
            'segmentation_mask',
            10)
            
        self.position_pub = self.create_publisher(
            PointStamped, 
            '/tracking_object/position', 
            10)
            
        self.marker_pub = self.create_publisher(
            Marker, 
            '/tracking_object/marker', 
            10)
            
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 
            '/tracking_object/pointcloud', 
            10)
            
        self.pointcloud_downsample_pub = self.create_publisher(
            PointCloud2, 
            '/tracking_object/pointcloud_downsample', 
            10)
            
        self.pointcloud_downsample_filtered_pub = self.create_publisher(
            PointCloud2, 
            '/tracking_object/pointcloud_downsample_filtered', 
            10)
            
        self.lidar_filtered_pub = self.create_publisher(
            PointCloud2, 
            '/mid360_PointCloud2_filtered', 
            10)

    # ============== Synchronized Callback ===============
    
    def synchronized_callback(self, color_msg, depth_msg, lidar_msg, odom_msg):
        """Process synchronized color, depth, LiDAR and odometry data"""
        try:
            current_time = time.perf_counter()

            # Update latest_odom with the synchronized odometry message
            self.latest_odom = odom_msg
            # Process color image for segmentation/tracking
            frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='rgb8')
            self.frame = frame
            
            # Process depth image
            self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg)
            
            # Process the frame for annotation or tracking
            if not self.segmentation_done:
                self.annotation_phase(frame)
            else:
                # Track objects and update mask
                self.track_objects(frame, color_msg.header)
                
                # After tracking, immediately process mask with synchronized depth data
                if self.mask_image is not None:
                    self.process_mask()
            
                # Process LiDAR data with updated tracking information
                self.process_lidar(lidar_msg)
            self.get_logger().info(f"Computing time: {time.perf_counter() - current_time}")
                
            self.get_logger().info(f"Processed synchronized data with timestamp: {color_msg.header.stamp.sec}.{color_msg.header.stamp.nanosec}")
            
        except Exception as e:
            self.get_logger().error(f'Error in synchronized callback: {str(e)}')

    # ============== Processing Methods ===============
    
    def process_lidar(self, msg):
        """Process incoming LiDAR data and publish filtered version"""
        if not self.object_points_world or self.prev_world is None:
            self.lidar_filtered_pub.publish(msg)
            return

        try:
            pc_points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            points = [[p[0], p[1], p[2]] for p in pc_points]
            if len(points) == 0:
                self.get_logger().warn("Empty point cloud received")
                return
            if self.latest_odom is None:
                self.get_logger().warn("No odometry data available for LiDAR filtering")
                return
            # Transform LiDAR points to odom frame
            self.get_logger().info(f"    {len(points), len(points[0])}")
            points_world = np.array(self.transform_livox_to_world(points))
            # Downsample object points to reduce density
            object_points_world_with_fake_z = [[obj[0], obj[1], 0] for obj in self.object_points_world]
            object_points_xy_with_fake_z = self.downsample_points(np.array(object_points_world_with_fake_z), voxel_size=0.3)
            object_points_xy = object_points_xy_with_fake_z[:, :2]
            # Transform object center to LiDAR frame
            object_centroid_world = self.prev_world            
            filtered_points = self.filter_points_kdtree(points_world, object_points_xy, object_centroid_world[:2], threshold=0.3)
            # Publish filtered pointcloud
            filtered_msg = self.create_pointcloud_msg(filtered_points, "odom")
            self.lidar_filtered_pub.publish(filtered_msg)
            self.get_logger().info(f"Published filtered LiDAR: {len(filtered_points)}/{len(points_world)} points")

        except Exception as e:
            self.get_logger().error(f"Error processing LiDAR data: {str(e)}")

    def mouse_callback(self, event, x, y, flags, param):
        if not self.segmentation_done:  # Only allow clicks during annotation phase
            if event == cv2.EVENT_LBUTTONDOWN:  # Left click for positive
                self.clicked_points.append([x, y])
                self.clicked_labels.append(1)
            elif event == cv2.EVENT_RBUTTONDOWN:  # Right click for negative
                self.clicked_points.append([x, y])
                self.clicked_labels.append(0)

    def annotation_phase(self, frame):
        if not self.is_labeled:
            self.predictor.load_first_frame(frame)
            self.is_labeled = True
            # 保存初始帧用于重置
            self.init_frame = frame.copy()

        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("Image", self.mouse_callback)

        while not self.segmentation_done:
            display_frame = frame.copy()

            # Draw clicked points
            for p, l in zip(self.clicked_points, self.clicked_labels):
                color = (0, 255, 0) if l == 1 else (0, 0, 255)
                cv2.circle(display_frame, tuple(p), 5, color, -1)

            # Display object ID
            cv2.putText(display_frame, f"Object {self.current_obj_id}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            cv2.imshow("Image", cv2.cvtColor(display_frame, cv2.COLOR_RGB2BGR))
            key = cv2.waitKey(1) & 0xFF

            if key == ord('n'):
                if self.clicked_points:
                    # Add object to tracker
                    points = np.array(self.clicked_points, dtype=np.float32)
                    labels = np.array(self.clicked_labels, dtype=np.int32)
                    self.predictor.add_new_prompt(
                        frame_idx=self.ann_frame_idx,
                        obj_id=self.current_obj_id,
                        points=points,
                        labels=labels
                    )
                    # 保存添加的提示点和标签，以便重置时使用
                    # if not hasattr(self, 'saved_tracking_result'):
                    self.saved_tracking_result = {}
                    self.saved_tracking_result[self.current_obj_id] = {
                        'points': points.copy(),
                        'labels': labels.copy()
                    }
                    self.get_logger().info(f"Object {self.current_obj_id} added.")
                    self.current_obj_id += 1
                    self.clicked_points = []
                    self.clicked_labels = []
                else:
                    self.get_logger().warn("Please annotate points first.")
            elif key == ord('s'):  # 's' for segment
                if self.current_obj_id > 1:
                    self.segmentation_done = True
                    cv2.destroyAllWindows()
                    self.get_logger().info("Annotation complete. Starting tracking...")
                    break
                else:
                    self.get_logger().warn("Please add at least one object.")
            elif key == 27:  # ESC key
                rclpy.shutdown()
                self.destroy_node()
                cv2.destroyAllWindows()
                break

    # def track_objects(self, frame, header):
    #     # Track objects using EfficientTAM
    #     if self.init_frame is not None:
    #         self.predictor.load_first_frame(self.init_frame)
    #         self.ann_frame_idx = 0
            
    #         # 重新添加所有保存的对象提示
    #         for obj_id, data in self.saved_tracking_result.items():
    #             self.predictor.add_new_prompt(
    #                 frame_idx=self.ann_frame_idx,
    #                 obj_id=obj_id,
    #                 points=data['points'],
    #                 labels=data['labels']
    #             )
    #             self.get_logger().info(f"Restored object {obj_id} from saved tracking result")
            
    #         # 再次跟踪当前帧
    #         obj_ids, mask_logits = self.predictor.track(frame)
        
    #     combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

    #     for idx, _ in enumerate(obj_ids):
    #         # Get mask for current object
    #         mask = (mask_logits[idx] > 0).squeeze().cpu().numpy().astype(np.uint8)            
    #         # Combine masks
    #         combined_mask = cv2.bitwise_or(combined_mask, mask)

    #     # Save the mask for position tracking
    #     self.mask_image = combined_mask
        
    #     # Publish the combined mask
    #     self.publish_mask(combined_mask, header)


    def track_objects(self, frame, header):
        # Track objects using EfficientTAM
        obj_ids, mask_logits = self.predictor.track(frame)
        self.get_logger().info(f"current info: {mask_logits}")
        
        # 检查是否所有的mask_logits都是负的
        all_negative = True
        for idx in range(len(mask_logits)):
            if torch.any(mask_logits[idx] > 0):
                all_negative = False
                break
        
        # 如果全都是负的，使用保存的结果重新初始化跟踪
        if all_negative and hasattr(self, 'saved_tracking_result') and self.saved_tracking_result:
            self.get_logger().info("All mask logits are negative, resetting tracking with saved results")
            
            # 重新加载初始帧
            if self.init_frame is not None:
                self.predictor.load_first_frame(self.init_frame)
                self.ann_frame_idx = 0
                
                # 重新添加所有保存的对象提示
                for obj_id, data in self.saved_tracking_result.items():
                    self.predictor.add_new_prompt(
                        frame_idx=self.ann_frame_idx,
                        obj_id=obj_id,
                        points=data['points'],
                        labels=data['labels']
                    )
                    self.get_logger().info(f"Restored object {obj_id} from saved tracking result")
                
                # 再次跟踪当前帧
                obj_ids, mask_logits = self.predictor.track(frame)
        
        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)

        for idx, _ in enumerate(obj_ids):
            # Get mask for current object
            mask = (mask_logits[idx] > 0).squeeze().cpu().numpy().astype(np.uint8)            
            # Combine masks
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # Save the mask for position tracking
        self.mask_image = combined_mask
        
        # Publish the combined mask
        self.publish_mask(combined_mask, header)

    def publish_mask(self, mask, header):
        """Publishes the segmentation mask as a ROS2 Image message."""
        try:
            if np.max(mask) <= 1:  # If values are between 0 and 1, scale them to 0-255
                mask = (mask * 255).astype(np.uint8)
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            mask_msg.header = header
            self.mask_publisher.publish(mask_msg)
        except Exception as e:
            self.get_logger().error(f'Error publishing mask: {str(e)}')

    def process_mask(self):
        mask = self.mask_image
        depth_image = self.depth_image
        
        y_indices, x_indices = np.where(mask > 0)
        if len(x_indices) <= 30:
        # if len(x_indices) == 0:
            if self.prev_world is not None:
                maintained_prev_world = [self.prev_world[0], self.prev_world[1], -1.0]
                self.publish_tracking(maintained_prev_world)
            self.get_logger().warn("No object detected in the mask.")
            return
        # make sure less no more than 20
        step = min(int(np.ceil(len(x_indices) / 200)), 20)
        if self.track_start_time is None:
            self.track_start_time = self.get_clock().now()
        object_points_camera = []

        for i, (x, y) in enumerate(zip(x_indices, y_indices)):
            if i % step != 0:  # Skip points unless they are the N-th point
                continue

            depth = depth_image[y, x] / 1000.0  # Convert to meters
            if np.isnan(depth) or np.isinf(depth) or depth <= 0:
                continue

            # Convert pixel coordinates to camera coordinates
            point_camera = self.pixel_to_camera(x, y, depth)
            object_points_camera.append(point_camera)
        self.get_logger().info("8")
        if not object_points_camera:
            if self.prev_world is not None:
                maintained_prev_world = [self.prev_world[0], self.prev_world[1], -1]
                self.publish_tracking(maintained_prev_world)
            self.get_logger().warn("No valid depth data found within the mask.")
            return
        
        # 1. Original object_points_camera as cloud_points for visualization
        # delete the visualization to accelerate
        # object_points_livox = list(self.transform_to_world(object_points_camera))
        # self.publish_pointcloud(object_points_livox, self.pointcloud_pub)

        # 2. Downsample object points for filtering lidar points
        original_length = len(object_points_camera)
        object_points_camera = self.downsample_points(np.array(object_points_camera), voxel_size=0.05)
        self.get_logger().info(f"Downsampled object points from {original_length} to {len(object_points_camera)}")

        self.object_points_world = list(self.transform_to_world(object_points_camera))
        self.publish_pointcloud(self.object_points_world, self.pointcloud_downsample_pub)

        # 3. Calculate object position from filtered points
        object_points_world_filtered = self.filter_outliers(self.object_points_world, threshold=2.0)        
        if object_points_world_filtered:
            self.publish_pointcloud(object_points_world_filtered, self.pointcloud_downsample_filtered_pub)
        
        # Update and publish tracking position
        if object_points_world_filtered:
            centroid_filtered_world = np.mean(object_points_world_filtered, axis=0)
            self.prev_world = centroid_filtered_world
            self.publish_tracking(centroid_filtered_world)
        elif self.object_points_world:
            centroid_world = np.mean(self.object_points_world, axis=0)
            self.prev_world = centroid_world
            self.publish_tracking(centroid_world)
        else:
            self.get_logger().error("No valid points found for position calculation")

    def publish_tracking(self, position, color="r"):
        # Publish the PointStamped message
        pos_msg = PointStamped()
        pos_msg.header.stamp = self.get_clock().now().to_msg()
        pos_msg.header.frame_id = "odom"
        pos_msg.point.x = float(position[0])
        pos_msg.point.y = float(position[1])
        pos_msg.point.z = float(position[2])

        self.position_pub.publish(pos_msg)

        # 1. Delete previous marker
        delete_marker = Marker()
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.header.frame_id = "odom"
        delete_marker.id = 0
        delete_marker.action = Marker.DELETE
        
        self.marker_pub.publish(delete_marker)

        # 2. Publish new marker
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = "odom"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        self.get_logger().info("4")
        marker.pose.position.z = position[2]
        self.get_logger().info("5")
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        if color == "r":
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        elif color == "g":
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        else:
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
        self.marker_pub.publish(marker)

        self.get_logger().info(
            f"Published tracking: pos=({pos_msg.point.x:.3f}, {pos_msg.point.y:.3f}, {pos_msg.point.z:.3f})"
        )

    def publish_pointcloud(self, object_points, publisher):
        """Publish object points as a PointCloud2 message."""
        if not object_points:
            return

        pointcloud_msg = self.create_pointcloud_msg(np.array(object_points), "odom")
        publisher.publish(pointcloud_msg)

        self.get_logger().info(f"Published point cloud with {len(object_points)} points.")

    def create_pointcloud_msg(self, points, frame_id):
        """Create a PointCloud2 message from an array of points."""
        pointcloud_msg = PointCloud2()
        pointcloud_msg.header.stamp = self.get_clock().now().to_msg()
        pointcloud_msg.header.frame_id = frame_id

        # Define PointField structure (XYZ)
        pointcloud_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        pointcloud_msg.point_step = 12  # Each point is 3 floats (4 bytes each)
        pointcloud_msg.row_step = pointcloud_msg.point_step * len(points)
        pointcloud_msg.height = 1
        pointcloud_msg.width = len(points)
        pointcloud_msg.is_dense = True

        # Convert to binary format
        pointcloud_data = []
        for point in points:
            pointcloud_data.append(struct.pack("fff", point[0], point[1], point[2]))

        pointcloud_msg.data = b"".join(pointcloud_data)
        return pointcloud_msg

    # ============== Utility Methods ===============

    def pixel_to_camera(self, u, v, depth):
        """Convert pixel coordinates and depth to camera frame coordinates."""
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return np.array([x, y, z])

    def transform_to_world(self, point_camera):
        """Transform a point from camera frame to world (odom) frame."""
        # Handle single point case        
        N = len(point_camera)        
        # Convert to 4xN matrix, where each column is a homogeneous point
        point_camera_homogeneous = np.vstack((np.array(point_camera).T, np.ones(N)))
        point_base = self.camera_to_base @ point_camera_homogeneous
        base_to_world = self.get_transform_matrix(self.latest_odom.pose.pose)
        point_world = base_to_world @ point_base
        return point_world[:3, :].T

    def transform_livox_to_world(self, points_livox):
        """Transform points from livox frame to world (odom) frame."""
        # Handle single point case        
        N = len(points_livox)        
        # Convert to 4xN matrix, where each column is a homogeneous point
        points_homogeneous = np.vstack((np.array(points_livox).T, np.ones(N)))
        # Transform from livox to base_link
        points_base = self.livox_to_base @ points_homogeneous
        # Transform from base_link to world frame
        base_to_world = self.get_transform_matrix(self.latest_odom.pose.pose)
        points_world = base_to_world @ points_base
        # Convert back to Nx3 matrix, where each row is a point
        return points_world[:3, :].T        

    def get_transform_matrix(self, pose):
        """Create transformation matrix from ROS pose."""
        transform = np.eye(4)
        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        transform[:3, :3] = quaternion_matrix(q)[:3, :3]
        transform[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
        return transform

    def filter_outliers(self, points, threshold=2.0):
        """Filter outliers based on distance from the mean."""
        if len(points) < 5:  # If too few points, don't filter
            return points

        points = np.array(points)

        # Calculate mean and standard deviation
        mean = np.mean(points, axis=0)
        std_dev = np.std(points, axis=0)

        # Calculate Euclidean distance to mean and filter points within threshold
        mask = np.linalg.norm(points - mean, axis=1) < (threshold * np.mean(std_dev))

        # Return filtered points
        return points[mask].tolist()

    def downsample_points(self, points, voxel_size=0.1):
        """Downsample point cloud using Open3D voxel grid filter."""
        points = np.asarray(points, dtype=np.float32)

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd = pcd.voxel_down_sample(voxel_size)

        return np.asarray(pcd.points, dtype=np.float32)

    def filter_points_kdtree(self, points_xyz, object_points_xy, object_center, threshold=0.2):
        """Filter points using KDTree to remove points close to the object."""
        # Ensure all data is float32 or float64 type
        points_xyz = np.array(points_xyz, dtype=np.float32)
        object_points_xy = np.array(object_points_xy, dtype=np.float32)
        object_center = np.array(object_center, dtype=np.float32)
        
        # Compute Euclidean distance from object_center (x, y)
        distances_from_center = np.linalg.norm(points_xyz[:, :2] - object_center, axis=1)

        # Select points within 1.5m of object_center (they need to be checked with KDTree)
        mask_nearby = distances_from_center <= 1.5
        points_nearby = points_xyz[mask_nearby]
        
        # Directly include points that are farther than 1.5m
        points_far_away = points_xyz[~mask_nearby]
        
        # Build KDTree only for nearby object points
        tree = cKDTree(object_points_xy)  
        distances, _ = tree.query(points_nearby[:, :2])  # Query only nearby points

        # Keep points that are either (1) beyond 1.0m from object_center or (2) meeting the threshold condition
        filtered_points = np.vstack([points_far_away, points_nearby[distances >= threshold]])

        # Ensure the output is float32 for ROS compatibility
        return filtered_points.astype(np.float32)

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveTrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()