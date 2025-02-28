#!/bin/bash

# Check if at least one argument is provided
if [ $# -lt 1 ]; then
    echo "Usage: $0 /path/to/image/folder [OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  --frame_rate RATE     Frame rate in Hz (default: 10.0)"
    echo "  --topic NAME          ROS2 topic name (default: /image)"
    echo "  --encoding TYPE       Image encoding (default: bgr8)"
    echo ""
    echo "Example: $0 ~/Pictures/dataset --frame_rate 30.0 --topic /camera/image_raw"
    exit 1
fi

# Extract the image directory
IMAGE_DIR=$(realpath "$1")
shift  # Remove the first argument

# Check if the directory exists
if [ ! -d "$IMAGE_DIR" ]; then
    echo "Error: $IMAGE_DIR is not a directory"
    exit 1
fi

# Get the folder name from the image directory
FOLDER_NAME=$(basename "$IMAGE_DIR")
echo "Input folder name: $FOLDER_NAME"

# Get the current directory for output
CURRENT_DIR=$(pwd)
OUTPUT_DIR="$CURRENT_DIR/output"
mkdir -p "$OUTPUT_DIR"

# Build the Docker image if it doesn't exist
if ! docker image inspect img_to_bag_converter &>/dev/null; then
    echo "Building Docker image..."
    docker build -t img_to_bag_converter .
fi

# Run the container with the provided arguments
echo "Running image to bag conversion..."
echo "Container will be automatically removed after completion."
docker run --rm \
    -v "$IMAGE_DIR:/input_images" \
    -v "$OUTPUT_DIR:/output" \
    img_to_bag_converter /input_images --output /output/$FOLDER_NAME "$@"

echo ""
echo "Conversion completed. Bag file created as: $OUTPUT_DIR/$FOLDER_NAME.mcap"
