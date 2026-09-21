# syntax=docker/dockerfile:1
#
# Runtime image for the ec ROS2 package. Targets linux/arm64 so it can run
# on a 64-bit Raspberry Pi. Built on the Pi by deployment/build-images.yml
# into localhost:5000/ec:<sha>.
#
# One image, one node per Atlas Scientific I2C probe: the deployment's
# metric role runs it three times (ec/ph/temp) with different NODE_NAME and
# I2C_ADDRESS. Run it with the bus device, e.g.:
#   docker run -d --device /dev/i2c-1 \
#     -e NODE_NAME=ec -e I2C_ADDRESS=100 \
#     --network host \
#     localhost:5000/ec:<sha>

# ---- build stage ------------------------------------------------------
FROM ros:iron-ros-base AS builder

RUN apt-get update \
    && apt-get install -y --no-install-recommends python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/ec
# copy manifest first to leverage layer caching when only source changes
COPY package.xml setup.py setup.cfg ./
COPY resource ./resource
COPY ec ./ec

WORKDIR /ws
RUN . /opt/ros/iron/setup.sh && colcon build --packages-select ec

# ---- runtime stage ----------------------------------------------------
# No extra packages: the node talks to /dev/i2c-* directly via fcntl/io.
FROM ros:iron-ros-base AS runtime

COPY --from=builder /ws/install /ws/install

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "run", "ec", "start"]
