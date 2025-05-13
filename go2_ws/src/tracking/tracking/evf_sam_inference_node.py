import argparse
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn.functional as F
from torchvision import transforms
from torchvision.transforms.functional import InterpolationMode
from transformers import AutoTokenizer, BitsAndBytesConfig
from evf_sam.model.segment_anything.utils.transforms import ResizeLongestSide
 
class InferencePublisherNode(Node):
    def __init__(self, args):
        super().__init__('inference_publisher_node')
        
        # ROS 2 subscriber for RealSense camera
        # 创建 ROS 2 订阅者，订阅 RealSense 相机的图像
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # RealSense color image topic
            self.image_callback,
            10
        )
        
        # ROS 2 publisher for segmentation masks
        # 创建 ROS 2 发布者，发布分割掩码
        self.mask_publisher = self.create_publisher(
            Image,
            'inference_segmentation_mask',
            10
        )
        
        # ROS 2 publisher for visualization
        # 创建 ROS 2 发布者，发布可视化结果
        self.viz_publisher = self.create_publisher(
            Image,
            'inference_visualization',
            10
        )
        
        # 初始化 CvBridge 对象
        self.bridge = CvBridge()
    
        # 保存传入的参数
        self.args = args
    
        # 初始化模型
        # Initialize models
        self.tokenizer, self.model = self.init_models(args)
        
        # 标志位，表示处理是否正在进行
        # Flag to indicate if processing is ongoing
        self.processing = False
        
        # 记录日志，表示推理节点已初始化，等待相机图像
        self.get_logger().info('Inference node initialized, waiting for camera images...')

    def init_models(self, args):
        tokenizer = AutoTokenizer.from_pretrained(
            args.version,
            padding_side="right",
            use_fast=False,
            trust_remote_code=True,
        )
 
        torch_dtype = torch.float32
        if args.precision == "bf16":
            torch_dtype = torch.bfloat16
        elif args.precision == "fp16":
            torch_dtype = torch.half
 
        kwargs = {"torch_dtype": torch_dtype}
        if args.load_in_4bit:
            kwargs.update(
                {
                    "torch_dtype": torch.half,
                    "quantization_config": BitsAndBytesConfig(
                        llm_int8_skip_modules=["visual_model"],
                        load_in_4bit=True,
                        bnb_4bit_compute_dtype=torch.float16,
                        bnb_4bit_use_double_quant=True,
                        bnb_4bit_quant_type="nf4",
                    ),
                }
            )
        elif args.load_in_8bit:
            kwargs.update(
                {
                    "torch_dtype": torch.half,
                    "quantization_config": BitsAndBytesConfig(
                        llm_int8_skip_modules=["visual_model"],
                        load_in_8bit=True,
                    ),
                }
            )
 
        if args.model_type=="ori":
            from model.evf_sam import EvfSamModel
            model = EvfSamModel.from_pretrained(
                args.version, low_cpu_mem_usage=True, device_map="auto", **kwargs
            )
        elif args.model_type=="effi":
            from model.evf_effisam import EvfEffiSamModel
            model = EvfEffiSamModel.from_pretrained(
                args.version, low_cpu_mem_usage=True, device_map="auto", **kwargs
            )
        elif args.model_type=="sam2":
            from model.evf_sam2 import EvfSam2Model
            model = EvfSam2Model.from_pretrained(
                args.version, low_cpu_mem_usage=True, device_map="auto", **kwargs
            )
 
        model.eval()
        self.get_logger().info(f"Model {args.model_type} loaded successfully")
        return tokenizer, model
 
    def sam_preprocess(self, x: np.ndarray, pixel_mean=torch.Tensor([123.675, 116.28, 103.53]).view(-1, 1, 1), 
                       pixel_std=torch.Tensor([58.395, 57.12, 57.375]).view(-1, 1, 1), 
                       img_size=1024, model_type="ori") -> torch.Tensor:
        assert img_size==1024, \
            "both SAM and Effi-SAM receive images of size 1024^2, don't change this setting unless you're sure that your employed model works well with another size."
        if model_type=="ori":
            x = ResizeLongestSide(img_size).apply_image(x)
            h, w = resize_shape = x.shape[:2]
            x = torch.from_numpy(x).permute(2,0,1).contiguous()
            x = (x - pixel_mean) / pixel_std
            padh = img_size - h
            padw = img_size - w
            x = F.pad(x, (0, padw, 0, padh))
        else:
            x = torch.from_numpy(x).permute(2,0,1).contiguous()
            x = F.interpolate(x.unsqueeze(0), (img_size, img_size), mode="bilinear", align_corners=False).squeeze(0)
            x = (x - pixel_mean) / pixel_std
            resize_shape = None
        return x, resize_shape
 
    def beit3_preprocess(self, x: np.ndarray, img_size=224) -> torch.Tensor:
        beit_preprocess = transforms.Compose([
            transforms.ToTensor(),
            transforms.Resize((img_size, img_size), interpolation=InterpolationMode.BICUBIC, antialias=None), 
            transforms.Normalize(mean=(0.5, 0.5, 0.5), std=(0.5, 0.5, 0.5))
        ])
        return beit_preprocess(x)
 
    def image_callback(self, msg):
        # Skip if we're already processing an image
        if self.processing:
            return
            
        self.processing = True
        self.get_logger().info('Received image, processing...')
        
        try:
            # Convert ROS Image message to OpenCV image
            image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
            
            # Process the image
            self.process_image(image_np)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
        finally:
            self.processing = False
    
    def process_image(self, image_np):
        # Start autocast for mixed precision
        torch.autocast(device_type="cuda", dtype=torch.float16).__enter__()
 
        if torch.cuda.get_device_properties(0).major >= 8:
            torch.backends.cuda.matmul.allow_tf32 = True
            torch.backends.cudnn.allow_tf32 = True
 
        prompt = self.args.prompt
        self.get_logger().info(f'Processing with prompt: "{prompt}"')
 
        # Prepare image
        original_size_list = [image_np.shape[:2]]
 
        image_beit = self.beit3_preprocess(image_np, self.args.image_size).to(dtype=self.model.dtype, device=self.model.device)
 
        image_sam, resize_shape = self.sam_preprocess(image_np, model_type=self.args.model_type)
        image_sam = image_sam.to(dtype=self.model.dtype, device=self.model.device)
 
        input_ids = self.tokenizer(prompt, return_tensors="pt")["input_ids"].to(device=self.model.device)
 
        # Inference
        pred_mask = self.model.inference(
            image_sam.unsqueeze(0),
            image_beit.unsqueeze(0),
            input_ids,
            resize_list=[resize_shape],
            original_size_list=original_size_list,
        )
        pred_mask = pred_mask.detach().cpu().numpy()[0]
        pred_mask = pred_mask > 0
 
        # Publish segmentation mask
        mask_msg = self.bridge.cv2_to_imgmsg(pred_mask.astype(np.uint8) * 255, encoding="mono8")
        self.mask_publisher.publish(mask_msg)
        self.get_logger().info('Published segmentation mask')
 
        # Create and publish visualization
        save_img = image_np.copy()
        save_img[pred_mask] = (
            image_np * 0.5
            + pred_mask[:, :, None].astype(np.uint8) * np.array([50, 120, 220]) * 0.5
        )[pred_mask]
        
        # Publish visualization
        viz_msg = self.bridge.cv2_to_imgmsg(cv2.cvtColor(save_img, cv2.COLOR_RGB2BGR), encoding="bgr8")
        self.viz_publisher.publish(viz_msg)
        self.get_logger().info('Published visualization')
        
        # Optionally save to disk if vis_save_path is provided
        if self.args.vis_save_path:
            os.makedirs(self.args.vis_save_path, exist_ok=True)
            timestamp = self.get_clock().now().to_msg().sec
            save_path = os.path.join(
                self.args.vis_save_path, 
                f"inference_result_{timestamp}.png"
            )
            cv2.imwrite(save_path, cv2.cvtColor(save_img, cv2.COLOR_RGB2BGR))
            self.get_logger().info(f'Saved visualization to {save_path}')
 
def parse_args(args):
    parser = argparse.ArgumentParser(description="EVF inference with ROS2")
    parser.add_argument("--version", required=True, help="Model version or path")
    parser.add_argument("--vis_save_path", default="", type=str, help="Path to save visualizations (empty to disable)")
    parser.add_argument(
        "--precision",
        default="fp16",
        type=str,
        choices=["fp32", "bf16", "fp16"],
        help="precision for inference",
    )
    parser.add_argument("--image_size", default=224, type=int, help="image size")
    parser.add_argument("--model_max_length", default=512, type=int)
 
    parser.add_argument("--local-rank", default=0, type=int, help="node rank")
    parser.add_argument("--load_in_8bit", action="store_true", default=False)
    parser.add_argument("--load_in_4bit", action="store_true", default=False)
    parser.add_argument("--model_type", default="ori", choices=["ori", "effi", "sam2"])
    parser.add_argument("--prompt", type=str, default="segment the main object", 
                        help="Text prompt for segmentation")
    return parser.parse_args(args)
 
def main(args):
    rclpy.init(args=args)
    # Parse command-line arguments
    parsed_args = parse_args(args)
    # Create and spin the node
    node = InferencePublisherNode(parsed_args)
    # Spin to keep the node alive and process incoming messages
    rclpy.spin(node)
    # Shutdown
    rclpy.shutdown()
 
if __name__ == "__main__":
    main(sys.argv[1:])