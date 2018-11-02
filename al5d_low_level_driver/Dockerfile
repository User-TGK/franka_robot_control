FROM osrf/ros:lunar-desktop-full

# Copy the package to the image.
COPY ./ /catkin_ws/src/al5d_low_level_driver

# Move the updated ros_entrypoint.sh to the correct place
RUN mv /catkin_ws/src/al5d_low_level_driver/ros_entrypoint.sh / && \
    chmod +x /ros_entrypoint.sh && \
    chown -R root:root /ros_entrypoint.sh

# Run the package
CMD ["/bin/bash", "-c", "rosrun al5d_low_level_driver al5d_low_level_driver /home/dev/pts/99"]
