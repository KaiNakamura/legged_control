FROM nvidia/cuda:12.6.1-runtime-ubuntu22.04

# Set environment variables for CUDA
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all

ARG HSL_PATH=null

WORKDIR /home

ENV SHELL /bin/bash

# Set the default command to bash
CMD ["/bin/bash"]

RUN apt update && apt upgrade -y
RUN apt install -y --no-install-recommends --allow-change-held-packages \
    python3-dev \
    python3-pip \
    libglfw3-dev \
    libcublas-12-6 \
    libcublas-dev-12-6 \
    cuda-toolkit-12-6 \
    git && \
    apt clean && rm -rf /var/lib/apt/lists/*

RUN mkdir repos

# Eigen: pinocchio requirement
RUN cd repos && \
    git clone https://github.com/LeCar-Lab/dial-mpc.git --depth 1 && \
    cd dial-mpc && \
    pip3 install -e .