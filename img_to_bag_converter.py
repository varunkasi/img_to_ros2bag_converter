#!/usr/bin/env python3

import argparse
import os
import sys
import time
from datetime import datetime
from typing import List, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image

import rosbag2_py


def get_rosbag_options(path: str, storage_id: str = 'mcap') -> Tuple:
    """
    Get rosbag writer options
    
    Args:
        path: Path to the output bag file (without extension)
        storage_id: Storage ID (default: 'mcap')
        
    Returns:
        Tuple of rosbag options
    """
    storage_options = rosbag2_py.StorageOptions(
        uri=path,
        storage_id=storage_id
    )
    
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    
    return storage_options, converter_options


def get_image_files(image_dir: str) -> List[str]:
    """
    Get a list of image files in the specified directory, sorted alphabetically
    
    Args:
        image_dir: Path to the directory containing images
        
    Returns:
        List of image file paths
    """
    extensions = ['.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif']
    image_files = []
    
    print(f"Scanning directory: {image_dir}")
    try:
        for file in sorted(os.listdir(image_dir)):
            file_path = os.path.join(image_dir, file)
            if os.path.isfile(file_path) and any(file.lower().endswith(ext) for ext in extensions):
                image_files.append(file_path)
    except Exception as e:
        print(f"Error reading directory {image_dir}: {e}")
        sys.exit(1)
    
    if not image_files:
        print(f"No image files found in {image_dir}")
        sys.exit(1)
    
    print(f"Found {len(image_files)} images (first: {os.path.basename(image_files[0])}, last: {os.path.basename(image_files[-1])})")
    return image_files


def main():
    parser = argparse.ArgumentParser(description='Convert a folder of images to a ROS2 bag file')
    parser.add_argument('image_dir', help='Path to the directory containing images')
    parser.add_argument('--frame_rate', type=float, default=10.0, help='Frame rate (Hz)')
    parser.add_argument('--topic', type=str, default='/image', help='ROS2 topic name')
    parser.add_argument('--output', type=str, help='Output bag file path (without extension)')
    parser.add_argument('--encoding', type=str, default='bgr8', help='Image encoding (default: bgr8)')
    
    args = parser.parse_args()
    
    # Validate image directory
    if not os.path.isdir(args.image_dir):
        print(f"Error: {args.image_dir} is not a directory")
        sys.exit(1)
    
    # Set output path if not specified
    if args.output is None:
        output_dir = os.getcwd()
        args.output = os.path.join(output_dir, f"image_bag_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
    else:
        # Ensure the output directory exists
        output_dir = os.path.dirname(args.output)
        if output_dir and not os.path.exists(output_dir):
            try:
                os.makedirs(output_dir)
                print(f"Created output directory: {output_dir}")
            except Exception as e:
                print(f"Error creating output directory {output_dir}: {e}")
                sys.exit(1)
    
    print(f"Output bag file will be: {args.output}.mcap")
    print(f"Topic name: {args.topic}")
    print(f"Frame rate: {args.frame_rate} Hz")
    
    # Get image files
    image_files = get_image_files(args.image_dir)
    
    # Initialize rosbag writer
    storage_options, converter_options = get_rosbag_options(args.output)
    writer = rosbag2_py.SequentialWriter()
    
    try:
        writer.open(storage_options, converter_options)
        print("Bag file created successfully")
    except Exception as e:
        print(f"Error creating bag file: {e}")
        sys.exit(1)
    
    # Register topic
    topic_info = rosbag2_py.TopicMetadata(
        name=args.topic,
        type='sensor_msgs/msg/Image',
        serialization_format='cdr'
    )
    writer.create_topic(topic_info)
    
    # Initialize CV bridge
    cv_bridge = CvBridge()
    
    # Calculate time between frames based on frame rate
    time_between_frames = 1.0 / args.frame_rate
    
    # Initialize timestamp
    timestamp = int(time.time_ns())
    
    # Process images and write to bag
    image_count = 0
    start_time = time.time()
    
    print(f"Starting to process images at {args.frame_rate} Hz")
    
    for image_file in image_files:
        # Read image
        cv_image = cv2.imread(image_file)
        if cv_image is None:
            print(f"Warning: Could not read {image_file}, skipping")
            continue
        
        # Create ROS Image message
        try:
            ros_image = cv_bridge.cv2_to_imgmsg(cv_image, encoding=args.encoding)
            ros_image.header.stamp.sec = int(timestamp // 1000000000)
            ros_image.header.stamp.nanosec = int(timestamp % 1000000000)
            ros_image.header.frame_id = "camera"
        except Exception as e:
            print(f"Error converting image {image_file} to ROS message: {e}")
            continue
        
        # Serialize message
        try:
            serialized_image = serialize_message(ros_image)
        except Exception as e:
            print(f"Error serializing image {image_file}: {e}")
            continue
        
        # Write to bag
        try:
            writer.write(
                args.topic,
                serialized_image,
                timestamp
            )
        except Exception as e:
            print(f"Error writing image {image_file} to bag: {e}")
            continue
        
        # Increment timestamp for next frame
        timestamp += int(time_between_frames * 1000000000)
        image_count += 1
        
        if image_count % 100 == 0:
            elapsed = time.time() - start_time
            fps = image_count / elapsed if elapsed > 0 else 0
            print(f"Processed {image_count} images... ({fps:.1f} images/sec)")
    
    # Calculate statistics
    elapsed = time.time() - start_time
    fps = image_count / elapsed if elapsed > 0 else 0
    duration = image_count / args.frame_rate if args.frame_rate > 0 else 0
    
    print(f"\nConversion completed:")
    print(f"Successfully wrote {image_count} images to {args.output}.mcap")
    print(f"Topic name: {args.topic}")
    print(f"Frame rate: {args.frame_rate} Hz")
    print(f"Bag duration: {duration:.2f} seconds")
    print(f"Processing speed: {fps:.1f} images/sec")
    print(f"Total processing time: {elapsed:.2f} seconds")


if __name__ == '__main__':
    main()
