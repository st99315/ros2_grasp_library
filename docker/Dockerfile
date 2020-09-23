########################################################
# Based on Ubuntu 18.04
########################################################

# Set the base image to ubuntu 18.04

FROM ubuntu:bionic

MAINTAINER Liu Cong "congx.liu@intel.com"

# Change the sources to NCHC, Taiwan (20 Gbps)
RUN sed -i'' 's/archive\.ubuntu\.com/free\.nchc\.org\.tw/' /etc/apt/sources.list && \
    sed -i'' 's/security\.ubuntu\.com/free\.nchc\.org\.tw/' /etc/apt/sources.list && \
    # for fixing Undetermined Error of apt install
    echo "Acquire::http::Pipeline-Depth 0;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
    echo "Acquire::http::No-Cache true;" >> /etc/apt/apt.conf.d/99fixbadproxy && \
    echo "Acquire::BrokenProxy    true;" >> /etc/apt/apt.conf.d/99fixbadproxy

# Install basic packages
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y \
    software-properties-common \
    tzdata \
    unzip \
    wget \
    git \
    && rm -rf /var/lib/apt/lists/*

ARG DEPS_DIR=/root/deps
# WORKDIR $DEPS_DIR

# install ros2 grasp library deps
# COPY ./script/ $DEPS_DIR/script/
# RUN bash script/install_ros2_grasp_library_deps.sh /root/deps

RUN mkdir -p $DEPS_DIR/script
ENV DEPS_PATH $DEPS_DIR

# install ros2 dashing
COPY ./script/00_ros2_install.sh $DEPS_DIR/script/00_ros2_install.sh
RUN bash $DEPS_DIR/script/00_ros2_install.sh

# instal eigen 3.2
COPY ./script/10_eigen_install.sh $DEPS_DIR/script/10_eigen_install.sh
RUN bash $DEPS_DIR/script/10_eigen_install.sh

# install libpcl 1.8.1
COPY ./script/11_libpcl_install.sh $DEPS_DIR/script/11_libpcl_install.sh
RUN bash $DEPS_DIR/script/11_libpcl_install.sh

# install opencv 4.1.2
COPY ./script/12_opencv_install.sh $DEPS_DIR/script/12_opencv_install.sh
RUN bash $DEPS_DIR/script/12_opencv_install.sh

# install openvino 2019_R3.1
COPY ./script/13_openvino_install.sh $DEPS_DIR/script/13_openvino_install.sh
RUN bash $DEPS_DIR/script/13_openvino_install.sh

# install librealsense 2.31
COPY ./script/20_librealsense_install.sh $DEPS_DIR/script/20_librealsense_install.sh
RUN bash $DEPS_DIR/script/20_librealsense_install.sh

# install gpg
COPY ./script/30_gpg_install.sh $DEPS_DIR/script/30_gpg_install.sh
RUN bash $DEPS_DIR/script/30_gpg_install.sh

# install gpd
COPY ./script/31_gpd_install.sh $DEPS_DIR/script/31_gpd_install.sh
RUN bash $DEPS_DIR/script/31_gpd_install.sh

# install ur_modern_driver
COPY ./script/32_ur_modern_driver_install.sh $DEPS_DIR/script/32_ur_modern_driver_install.sh
RUN bash $DEPS_DIR/script/32_ur_modern_driver_install.sh

# build ros2 other deps
COPY ./script/50_ros2_deps.sh $DEPS_DIR/script/50_ros2_deps.sh
RUN bash $DEPS_DIR/script/50_ros2_deps.sh

COPY ./script/ros_entrypoint.sh /root/script/ros_entrypoint.sh

WORKDIR /root

RUN echo "export CPU_EXTENSION_LIB=/opt/openvino_toolkit/openvino/inference-engine/bin/intel64/Release/lib/libcpu_extension.so" >> $HOME/.bashrc && \
    echo "export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/mklml/lib" >> $HOME/.bashrc && \
    echo "source /opt/ros/dashing/setup.bash" >> $HOME/.bashrc && \
    echo "source /root/ros2_ws/install/setup.bash" >> $HOME/.bashrc

# on container initailize only
# ENTRYPOINT ["/root/script/ros_entrypoint.sh"]

CMD ["bash"]
