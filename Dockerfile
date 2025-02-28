FROM ros:humble

# Install Python and required dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-rclpy \
    ros-humble-cv-bridge \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Set up working directory
WORKDIR /app

# Copy the script and requirements
COPY img_to_bag_converter.py /app/
COPY requirements.txt /app/

# Install Python dependencies
RUN pip3 install -r requirements.txt

# Create a shell script to source ROS and run the converter
RUN echo '#!/bin/bash \n\
source /opt/ros/humble/setup.bash \n\
python3 /app/img_to_bag_converter.py "$@"' > /app/run.sh && \
    chmod +x /app/run.sh

# Set the entrypoint to the script
ENTRYPOINT ["/app/run.sh"]
