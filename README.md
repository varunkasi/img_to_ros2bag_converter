# Image to ROS2 Bag Converter

This utility converts a folder of images into a ROS2 bag file in MCAP format. It reads images from a directory, converts them to ROS2 sensor_msgs/Image messages, and writes them to a bag file with timestamps based on the specified frame rate.

## Overview

This tool uses Docker to run a ROS2 Humble environment that converts your images to ROS2 bag files. This allows you to create ROS2 bag files on any platform that supports Docker, including macOS and Windows, without having to install ROS2 directly.

## Requirements

- Docker (Docker Desktop for macOS or Windows)
- Bash-compatible shell

## Quick Start

1. Clone this repository:
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. Make the conversion script executable:
   ```bash
   chmod +x convert_images.sh
   ```

3. Run the conversion:
   ```bash
   ./convert_images.sh /path/to/image/folder --frame_rate 15.0 --topic /camera/image_raw
   ```

## Usage

The script handles the Docker image building and container execution for you. The first time you run it, it will build the Docker image, which may take a few minutes.

Basic usage:

```bash
./convert_images.sh /path/to/image/folder [OPTIONS]
```

### Arguments

- `/path/to/image/folder`: Path to the directory containing images (required)
- `--frame_rate RATE`: Frame rate in Hz (default: 10.0)
- `--topic NAME`: ROS2 topic name (default: /image)
- `--output PATH`: Output bag file path (default: ./output/bag)
- `--encoding TYPE`: Image encoding (default: bgr8)

### Examples

**Basic usage with default settings:**
```bash
./convert_images.sh ~/Pictures/dataset
```

**Specifying frame rate and topic:**
```bash
./convert_images.sh ~/Pictures/dataset --frame_rate 30.0 --topic /camera/image_raw
```

**Using thermal images with appropriate encoding:**
```bash
./convert_images.sh ~/thermal_images --encoding mono8 --topic /camera/thermal --frame_rate 15.0
```

## Output

The output bag file will be created in the `output` directory in the current directory. The bag file will be named after the folder containing the images (e.g., if your images are in a folder named "thermal_dataset", the bag file will be named "output/thermal_dataset.mcap").

The bag file will contain the images as ROS2 sensor_msgs/Image messages, published at the specified frame rate on the specified topic.

## How It Works

The tool uses Docker to:

1. Create a ROS2 Humble environment with all necessary dependencies
2. Mount your image directory and the output directory into the container
3. Run the Python script inside the container to convert the images to a ROS2 bag file
4. Save the ROS2 bag file to the output directory
5. Automatically remove the container after conversion is completed

The output bag file will be named after the folder containing the images. For example, if your images are in a folder named "thermal_dataset", the bag file will be named "thermal_dataset.mcap".

## Notes

- Images are sorted alphabetically before conversion
- Supported image formats: jpg, jpeg, png, bmp, tiff, tif
- The timestamp for each image is calculated based on the specified frame rate
- For large datasets, the conversion may take some time depending on the number and size of images
- The script provides progress updates during conversion
- The Docker image is only built the first time you run the tool, subsequent runs will reuse the existing image

## Technical Details

### Docker Image

The Docker image is based on the official ROS2 Humble image and includes the following additional components:

- Python 3
- OpenCV
- ROS2 packages: cv_bridge, rosbag2, rosbag2-storage-mcap, sensor_msgs

### Script Operation

The `img_to_bag_converter.py` script inside the Docker container:

1. Reads all images from the specified directory
2. Creates ROS2 sensor_msgs/Image messages with appropriate timestamps
3. Writes these messages to a ROS2 bag file in MCAP format

## Troubleshooting

- **Docker not installed**: Install Docker Desktop for your platform
- **Permission denied**: Make sure you've made the `convert_images.sh` script executable
- **Unable to mount directory**: Ensure Docker has permission to access the directory you're trying to convert
- **No images found**: Check that your image directory contains supported image formats
- **Slow conversion**: Large images or a large number of images may take longer to process

## Using the Bag File

Once generated, the bag file can be used with ROS2 tools on a system with ROS2 installed:

```bash
# List topics in the bag
ros2 bag info /path/to/your/bagfile.mcap

# Play back the bag file
ros2 bag play /path/to/your/bagfile.mcap

# In another terminal, view the images
ros2 run rqt_image_view rqt_image_view
