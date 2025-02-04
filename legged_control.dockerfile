FROM osrf/ros:noetic-desktop-full

ARG HSL_PATH=null

WORKDIR /home

# Set up the workspace environment
ENV CATKIN_WS catkin_ws
ENV ROS_PACKAGE_PATH $CATKIN_WS/src

# Create the catkin workspace
RUN mkdir -p $CATKIN_WS/src

# Set environment variables
ENV SHELL /bin/bash
ENV PATH="/usr/local/bin:$PATH"
ENV LD_LIBRARY_PATH="/usr/local/lib:$LD_LIBRARY_PATH"
ENV CMAKE_PREFIX_PATH="/usr/local:$CMAKE_PREFIX_PATH"

# Set the default command to bash
CMD ["/bin/bash"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gcc \
    g++ \
    gfortran \
    git \
    cmake \
    wget \
    make \
    liblapack-dev \
    pkg-config \
    gnuplot \
    libboost-all-dev \
    libassimp-dev \
    libclang-dev \
    llvm-dev \
    libblas-dev \
    libmetis-dev \
    libconsole-bridge-dev \
    liburdfdom-dev \
    liburdfdom-headers-dev \
    liboctomap-dev \
    python3-catkin-tools \
    ros-noetic-catkin \
    ros-noetic-interactive-markers \
    ros-noetic-grid-map-rviz-plugin \
    libglpk-dev \
    gnome-terminal \
    dbus-x11 \
    xterm
    # apt-get clean && rm -rf /var/lib/apt/lists/*

RUN mkdir repos

# # Eigen: pinocchio requirement
# RUN cd repos && \
#     curl -L https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz -o eigen-3.4.0.tar.gz && \
#     tar -xzf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz && \
#     cd eigen-3.4.0 && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/eigen3.4.0 && \
#     make -j$(nproc) && \
#     make install && \
#     cd /home && rm -rf repos/eigen-3.4.0

# # HSL: casadi requirement
# RUN cd repos && \
#     git clone https://github.com/coin-or-tools/ThirdParty-HSL.git
# ADD ${HSL_PATH} /home/repos/ThirdParty-HSL/coinhsl/
# RUN cd repos/ThirdParty-HSL && \
#     git checkout 4f8da75 && \
#     rm -r .git && \
#     ./configure && \
#     make && \   
#     make install && \
#     cd /home && rm -rf repos/ThirdParty-HSL

# # urdfdomm: pinocchio requirement
# RUN cd repos && \
#     git clone https://github.com/ros/urdfdom.git && \
#     cd urdfdom && git checkout 3f6bf9a && rm -r .git && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
#     cd /home && rm -rf repos/urdfdom

# # CppAD: CppADCodegen requirement
# RUN cd repos && \
#     git clone https://github.com/coin-or/CppAD.git && \
#     cd CppAD && git checkout 470c769 && rm -r .git && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && make install && \
#     cd /home && rm -rf repos/CppAD

# # CppADCodegen: pinocchio requirement
# RUN cd repos && \
#     git clone https://github.com/joaoleal/CppADCodeGen.git && \
#     cd CppADCodeGen && git checkout 656d23e && rm -r .git && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && make install && \
#     cd /home && rm -rf repos/CppADCodeGen

# # hpp-fcl: pinocchio requirement
# RUN cd repos && \
#     git clone https://github.com/humanoid-path-planner/hpp-fcl.git && \
#     cd hpp-fcl && git checkout 6b9f9c8 && rm -r .git && \
#     mkdir build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF -DHPP_FCL_HAS_OCTOMAP=ON && \
#     make -j$(nproc) install && \
#     cd /home && rm -rf repos/hpp-fcl

# # casadi: galileo, pinocchio requirement
# RUN cd repos && \
#     git clone https://github.com/casadi/casadi.git && \
#     cd casadi && git checkout 81bbcd3 && rm -r .git && \
#     mkdir build && cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DWITH_BUILD_IPOPT=ON -DWITH_IPOPT=ON -DWITH_HSL=ON -DWITH_OPENMP=ON -DWITH_MUMPS=ON -DWITH_BUILD_MUMPS=ON -DWITH_SNOPT=OFF -DWITH_CLANG=ON -DWITH_THREAD=ON -DWITH_OSQP=ON -DWITH_BUILD_OSQP=ON -DWITH_QPOASES=ON -DWITH_LAPACK=ON -DWITH_BUILD_LAPACK=ON -DWITH_BLOCKSQP=ON -DWITH_PYTHON=OFF -DWITH_PYTHON3=OFF -DWITH_BUILD_METIS=ON .. && \
#     make -j$(nproc) && make install && \
#     cd /home && rm -rf repos/casadi

# # pinocchio: galileo requirement
# RUN cd repos && \
#     git clone --recursive https://github.com/stack-of-tasks/pinocchio.git && \
#     cd pinocchio && \ 
#     git checkout 0b594a0 && \
#     rm -r .git && \
#     mkdir build && \ 
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local/pinocchio -DCMAKE_PREFIX_PATH=/usr/local/eigen3.4.0 -DBUILD_BENCHMARK=ON -DBUILD_UTILS=ON -DBUILD_PYTHON_INTERFACE=OFF -DGENERATE_PYTHON_STUBS=OFF -DBUILD_WITH_URDF_SUPPORT=ON -DBUILD_WITH_COLLISION_SUPPORT=ON -DBUILD_WITH_AUTODIFF_SUPPORT=ON -DBUILD_WITH_CASADI_SUPPORT=ON -DBUILD_WITH_CODEGEN_SUPPORT=ON -DBUILD_WITH_OPENMP_SUPPORT=ON .. && \
#     make -j$(nproc) && \
#     make install && \
#     cd /home && rm -rf repos/pinocchio

# Galileo
# RUN cd repos && \
#     git clone https://github.com/KaiNakamura/Galileo.git --depth=1 && \
#     cd Galileo && \ 
#     mkdir build && \ 
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local/pinocchio -DBUILD_WITH_OPENMP=ON .. && \
#     make && \
#     make install

# RUN cd repos && \
#     git clone https://github.com/KaiNakamura/Galileo.git --depth=1 && \
#     cd Galileo && \ 
#     mkdir build && \ 
#     cd build && \
#     cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_WITH_OPENMP=ON .. && \
#     make && \
#     make install

# Clone OCS2
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/JaredMorgan21/ocs2.git

# Eigen 3.3.9
RUN cd repos && \
    curl -L https://gitlab.com/libeigen/eigen/-/archive/3.3.9/eigen-3.3.9.tar.gz -o eigen-3.3.9.tar.gz && \
    tar -xzf eigen-3.3.9.tar.gz && rm eigen-3.3.9.tar.gz && \
    cd eigen-3.3.9 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && make install && \
    cd /home && rm -rf repos/eigen-3.3.9

# Clone pinocchio
RUN cd $CATKIN_WS/src && \
    git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git

# Clone hpp-fcl
RUN cd $CATKIN_WS/src && \
    git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git

# Clone ocs2_robotic_assets
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/leggedrobotics/ocs2_robotic_assets.git

# RUN /bin/bash -c "cd $CATKIN_WS && \
#     source /opt/ros/$ROS_DISTRO/setup.sh && \
#     catkin init && \
#     catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
#     catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization && \
#     source devel/setup.bash"

# RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /root/.bashrc
# RUN echo "source /home/$CATKIN_WS/devel/setup.bash" >> /root/.bashrc

RUN apt-get install -y --no-install-recommends \
    coinor-libipopt-dev \
    libncurses5-dev \
    ros-noetic-xpp \
    ros-noetic-pybind11-catkin \
    rsync \
    libmpfr-dev \
    libgmp-dev

# Clone Raisim
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/raisimTech/raisimlib.git

ENV raisim_DIR=/home/catkin_ws/src/raisimlib/raisim/linux/lib/cmake/raisim
ENV LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$CATKIN_WS/src/raisimlib/raisim/linux/lib
ENV PYTHONPATH=\$PYTHONPATH:\$CATKIN_WS/src/raisimlib/raisim/linux/lib
# RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:\$CATKIN_WS/src/raisimlib/raisim/linux/lib" >> /root/.bashrc
# RUN echo "export PYTHONPATH=\$PYTHONPATH:\$CATKIN_WS/src/raisimlib/raisim/linux/lib" >> /root/.bashrc
# RUN source /root/.bashrc

# Install ONNX Runtime
RUN cd /tmp && \
    wget https://github.com/microsoft/onnxruntime/releases/download/v1.7.0/onnxruntime-linux-x64-1.7.0.tgz && \
    tar xf onnxruntime-linux-x64-1.7.0.tgz && \
    mkdir -p ~/.local/bin ~/.local/include/onnxruntime ~/.local/lib ~/.local/share/cmake/onnxruntime && \
    rsync -a /tmp/onnxruntime-linux-x64-1.7.0/include/ ~/.local/include/onnxruntime && \
    rsync -a /tmp/onnxruntime-linux-x64-1.7.0/lib/ ~/.local/lib && \
    rsync -a /home/$CATKIN_WS/src/ocs2/ocs2_mpcnet/ocs2_mpcnet_core/misc/onnxruntime/cmake/ ~/.local/share/cmake/onnxruntime

ENV onnxruntime_DIR=~/.local/share/cmake/onnxruntime

# Clone grid_map
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/ANYbotics/grid_map.git

# Clone elevation_mapping_cupy
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/leggedrobotics/elevation_mapping_cupy.git

# Install dependencies
RUN cd $CATKIN_WS && \
    rosdep install --from-path src --ignore-src -ry

RUN apt-get install -y --no-install-recommends \
    portaudio19-dev \
    python3-pip

# Install sphinx
RUN pip install sphinx breathe sphinxcontrib-bibtex sphinx-rtd-theme

# Install ifopt
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/ethz-adrl/ifopt.git

# Install towr
RUN cd $CATKIN_WS/src && \
    git clone https://github.com/KaiNakamura/towr.git

RUN /bin/bash -c "cd $CATKIN_WS && \
    source /opt/ros/$ROS_DISTRO/setup.sh && \
    catkin init && \
    catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo && \
    catkin build ocs2_legged_robot_ros ocs2_self_collision_visualization towr_ros && \
    source devel/setup.bash"

RUN echo "source /opt/ros/$ROS_DISTRO/setup.sh" >> /root/.bashrc
RUN echo "source /home/$CATKIN_WS/devel/setup.bash" >> /root/.bashrc

# roslaunch ocs2_legged_robot_ros legged_robot_ddp.launch
# roslaunch ocs2_legged_robot_ros legged_robot_sqp.launch

# RUN catkin build legged_controllers legged_unitree_description

# RUN catkin build legged_gazebo

# RUN catkin build legged_unitree_hw
