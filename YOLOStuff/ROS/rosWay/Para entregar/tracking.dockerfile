# FROM fleetman/ros:galactic

# # Install dependencies
# RUN apt update && \
#     apt install -y \
#       python3-pip \
#       libusb-1.0-0-dev\
#       libgtk2.0-dev \
#       pkg-config

# # Install pip requirements
# RUN pip3 install \
#         matplotlib \
#         haversine \
#         pyrealsense2\
#         numpy \
#         paho-mqtt \
#         opencv-python \
#         opencv-python-headless \
#         ultralytics 

# # Install rclpy
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
#     pip3 install -U ros2pkg && \
#     pip3 install -U ament_index_python && \
#     pip3 install -U rclpy

# # Create a directory for the sensor files
# RUN mkdir -p /sensors/configs

# # Create a directory for the yolo Model
# RUN mkdir -p /sensors/yolo_model

# COPY models/best.pt /sensors/yolo_model/best.pt
# COPY imagesTest/yolo.PNG /sensors/yolo_model/yolo.PNG
# COPY imagesTest/Full.mp4 /sensors/yolo_model/Full.mp4
# COPY imagesTest/Fullv2.mp4 /sensors/yolo_model/Fullv2.mp4



# # Copy source files
# COPY *.py /sensors
# COPY *.yml /sensors/configs

# # Set the working directory
# WORKDIR /sensors

# # Set the entrypoint command
# CMD ["/bin/bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.sh && python3 tracking.py"]


FROM fleetman/ros:galactic

# Install dependencies
RUN apt update && \
    apt install -y \
      python3-pip \
      libusb-1.0-0-dev \
      libgtk2.0-dev \
      pkg-config \
      libgl1 \
      libsm6 \
      libxrender1 \
      libxext6 \
      x11-xserver-utils

# Install pip requirements
RUN pip3 install \
        matplotlib \
        haversine \
        pyrealsense2 \
        numpy \
        paho-mqtt \
        ultralytics
        #opencv-python \

# Install rclpy
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    pip3 install -U ros2pkg && \
    pip3 install -U ament_index_python && \
    pip3 install -U rclpy

# Create a directory for the sensor files
RUN mkdir -p /sensors/configs /sensors/yolo_model

# Copy model and input data
COPY models/best.pt /sensors/yolo_model/best.pt
COPY imagesTest/yolo.PNG /sensors/yolo_model/yolo.PNG
COPY imagesTest/Full.mp4 /sensors/yolo_model/Full.mp4
COPY imagesTest/Fullv2.mp4 /sensors/yolo_model/Fullv2.mp4

# Copy source files
COPY *.py /sensors/

# Set the working directory
WORKDIR /sensors

# Set the entrypoint command
CMD ["/bin/bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.sh && python3 tracking.py"]
