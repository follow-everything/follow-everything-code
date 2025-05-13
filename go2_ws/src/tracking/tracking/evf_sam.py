import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import torch
import datetime
from efficient_track_anything.build_efficienttam import build_efficienttam_camera_predictor
from sensor_msgs.msg import Image  # Import the Image message type
from std_msgs.msg import Header  # Import the Header message type
from geometry_msgs.msg import Point, PointStamped  # 导入点消息类型用于发布中心点
from visualization_msgs.msg import Marker, MarkerArray  # 用于可视化
from message_filters import ApproximateTimeSynchronizer, Subscriber

class InteractiveSegmentationNode(Node):

    def __init__(self):
        super().__init__('interactive_segmentation_node')
        
        # Create CvBridge
        self.bridge = CvBridge()
        
        # Initialize tracking variables
        self.frame = None
        self.segmentation_done = False
        self.clicked_points = []
        self.clicked_labels = []
        self.current_obj_id = 1
        self.predictor = None  # EfficientTAM predictor
        self.ann_frame_idx = 0 #annotation frame index
        self.reset_pending = False
        self.new_frame = None
        self.new_mask = None

        # Camera parameters
        self.K = np.array([
            [607.4929809570312, 0.0, 320.42071533203125],
            [0.0, 607.2711791992188, 250.74566650390625],
            [0.0, 0.0, 1.0]
        ])

        # Load EfficientTAM (replace with your actual paths)
        checkpoint = "/home/newusername/etam_kalman/checkpoints/efficienttam_ti_512x512.pt"
        model_cfg = "configs/efficienttam/efficienttam_ti_512x512.yaml"
        self.model_cfg = model_cfg
        self.checkpoint = checkpoint

        # Regular camera subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            10)
            
        # Publishers
        self.mask_publisher = self.create_publisher(
            Image, 
            'segmentation_mask',
            10)
        
        # For each object ID, create a separate point publisher
        self.center_publishers = {}
        
        # Subscribe to EVF SAM outputs for automatic reset
        self.evf_image_sub = self.create_subscription(
            Image,
            'inference_original_image',
            self.evf_image_callback,
            10
        )
        
        self.evf_mask_sub = self.create_subscription(
            Image,
            'inference_segmentation_mask',
            self.evf_mask_callback,
            10
        )
            
        # Create a timer to periodically check if we need to reset tracking
        self.reset_timer = self.create_timer(0.1, self.check_reset_tracking)
        
        self.get_logger().info('Interactive Segmentation Node started.')

    def evf_image_callback(self, msg):
        """Callback for the EVF SAM original image"""
        try:
            self.new_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self.get_logger().info('Received new EVF original image')
        except Exception as e:
            self.get_logger().error(f'Error converting EVF image: {str(e)}')
    
    def evf_mask_callback(self, msg):
        """Callback for the EVF SAM segmentation mask"""
        try:
            self.new_mask = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            # Normalize mask to 0/1 values if it's in range 0-255
            if np.max(self.new_mask) > 1:
                self.new_mask = (self.new_mask > 127).astype(np.uint8)
            self.get_logger().info('Received new EVF segmentation mask')
            
            # Mark that a reset is pending if we have both frame and mask
            if self.new_frame is not None:
                self.reset_pending = True
                self.get_logger().info('Reset pending: New EVF data available')
        except Exception as e:
            self.get_logger().error(f'Error converting EVF mask: {str(e)}')
    
    def check_reset_tracking(self):
        """Periodically check if we need to reset tracking with new EVF data"""
        if self.reset_pending and self.new_frame is not None and self.new_mask is not None:
            self.reset_tracking_with_evf_data()
            
    def reset_tracking_with_evf_data(self):
        """Reset tracking with new EVF data"""
        self.get_logger().info('Resetting tracking with new EVF data')
        
        # Reset tracking state
        self.current_obj_id = 1
        self.clicked_points = []
        self.clicked_labels = []
        self.ann_frame_idx = 0
        
        # Initialize or reset predictor
        if self.predictor is None:
            self.predictor = build_efficienttam_camera_predictor(self.model_cfg, self.checkpoint)
        
        # Load the new frame
        self.predictor.load_first_frame(self.new_frame)
        
        # Add the mask as a prompt for the current object
        nonzero_coords = cv2.findNonZero(self.new_mask)
        if nonzero_coords is not None and len(nonzero_coords) > 0:
            # Get contour of the mask
            contours, _ = cv2.findContours(self.new_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Pick 5 points from the contour
                contour_points = []
                labels = []
                
                largest_contour = max(contours, key=cv2.contourArea)
                # Simplify contour to get fewer points
                epsilon = 0.01 * cv2.arcLength(largest_contour, True)
                approx_polygon = cv2.approxPolyDP(largest_contour, epsilon, True)
                
                # Get points from the approximate polygon
                for i in range(min(5, len(approx_polygon))):
                    point = approx_polygon[i][0]
                    contour_points.append(point)
                    labels.append(1)  # Mark as positive points
                
                # Add the points to the predictor
                if contour_points:
                    points = np.array(contour_points, dtype=np.float32)
                    labels = np.array(labels, dtype=np.int32)
                    self.predictor.add_new_prompt(
                        frame_idx=self.ann_frame_idx,
                        obj_id=self.current_obj_id,
                        points=points,
                        labels=labels
                    )
                    self.get_logger().info(f"Added EVF-provided object with ID: {self.current_obj_id}")
                    self.current_obj_id += 1
                    
                    # Set segmentation as done to start tracking
                    self.segmentation_done = True
                else:
                    self.get_logger().warn("No valid contour points found in the EVF mask.")
        else:
            self.get_logger().warn("EVF mask is empty, cannot use for tracking.")
        
        # Clear the reset flag
        self.reset_pending = False
        self.new_frame = None
        self.new_mask = None

    def mouse_callback(self, event, x, y, flags, param):
        if not self.segmentation_done:  # Only allow clicks during annotation phase
            if event == cv2.EVENT_LBUTTONDOWN:  # Left click for positive
                self.clicked_points.append([x, y])
                self.clicked_labels.append(1)
            elif event == cv2.EVENT_RBUTTONDOWN:  # Right click for negative
                self.clicked_points.append([x, y])
                self.clicked_labels.append(0)

    def image_callback(self, msg):
        try:
            self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return

        if self.frame is not None:
            if not self.segmentation_done:
                self.annotation_phase(self.frame)
            else:
                self.track_objects(self.frame, msg.header)  # Pass the header

    def track_objects(self, frame, header):
        # 跟踪阶段
        obj_ids, mask_logits = self.predictor.track(frame)

        # 创建叠加层
        overlay = frame.copy()
        alpha = 0.3  # 透明度

        # 预定义颜色表（BGR格式）
        colors = [
            (255, 0, 0),   # 红
            (0, 255, 0),   # 绿
            (0, 0, 255),   # 蓝
            (0, 255, 255), # 黄
            (255, 0, 255), # 品红
            (255, 255, 0), # 青
        ]

        combined_mask = np.zeros(frame.shape[:2], dtype=np.uint8)  # Initialize combined mask

        for idx, obj_id in enumerate(obj_ids):
            # 获取当前对象mask
            mask = (mask_logits[idx] > 0).squeeze().cpu().numpy().astype(np.uint8)
            color = colors[idx % len(colors)]

            # 绘制轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(overlay, contours, -1, color, 2)

            # 半透明填充
            mask_color = np.zeros_like(overlay)
            mask_color[mask > 0] = color
            overlay = cv2.addWeighted(overlay, 1, mask_color, 0.3, 0)

            # 计算中心点
            y, x = np.where(mask)
            if len(x) > 0:
                x_center, y_center = int(np.mean(x)), int(np.mean(y))
                cv2.circle(overlay, (x_center, y_center), 5, color, -1)
                cv2.putText(overlay, f'ID:{obj_id}', (x_center+5, y_center+5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                
                # 发布中心点在图像的像素位置
                self.publish_center_point(obj_id, x_center, y_center, header)
            
            # Combine masks
            combined_mask = cv2.bitwise_or(combined_mask, mask)

        # 融合叠加层
        frame = cv2.addWeighted(frame, 1-alpha, overlay, alpha, 0)

        cv2.imshow("Segmentation Result", cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)

        # Publish the combined mask
        self.publish_mask(combined_mask, header)

    def publish_center_point(self, obj_id, x, y, header):
        """为单个对象发布中心点像素坐标"""
        # 如果该对象ID没有发布器，创建一个
        if obj_id not in self.center_publishers:
            topic_name = f'object_{obj_id}_pixel_position'
            self.center_publishers[obj_id] = self.create_publisher(
                PointStamped,
                topic_name,
                10
            )
            self.get_logger().info(f'Created publisher for object {obj_id} pixel position')
        
        # 创建点消息，保留纯像素坐标
        center_point = PointStamped()
        center_point.header = header
        center_point.point.x = float(x)
        center_point.point.y = float(y)
        center_point.point.z = 0.0
        
        # 发布点坐标
        self.center_publishers[obj_id].publish(center_point)

    def publish_mask(self, mask, header):
        """Publishes the segmentation mask as a ROS2 Image message."""
        try:
            # Ensure mask values are in the correct range (0-255)
            if np.max(mask) <= 1:
                mask = (mask * 255).astype(np.uint8)
                
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")  # "mono8" for grayscale masks
            mask_msg.header = header  # Use the same header as the input image
            self.mask_publisher.publish(mask_msg)
        except Exception as e:
            self.get_logger().error('Error publishing mask: %s' % str(e))

    def annotation_phase(self, frame):
        if self.predictor is None:
            self.predictor = build_efficienttam_camera_predictor(self.model_cfg, self.checkpoint)
            self.predictor.load_first_frame(frame)

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


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveSegmentationNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()