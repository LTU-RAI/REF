FROM osrf/ros:noetic-desktop-full-focal
# The OSRF ROS Noetic containers use the root user.
# Therefore, the following commands are executed as root up until the
# USER user statement.

# We love UTF!
ENV LANG C.UTF-8

RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Set the nvidia container runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Install some handy tools.
RUN set -x \
	&& apt-get update \
	&& apt-get --with-new-pkgs upgrade -y \
	&& apt-get install -y mesa-utils \
  && apt-get install -y vim \
  && apt-get install -y tmuxinator \
  && apt-get install -y python3-catkin-tools \
  && apt-get install -y python3-osrf-pycommon \
  && apt-get install -y ros-noetic-octomap-server \
  && apt-get install -y ros-noetic-octomap-ros \
  && apt-get install -y ros-noetic-octomap-rviz-plugins \
  && apt-get install -y ros-noetic-octomap-mapping \
  && apt-get install -y libtool \
  && apt-get install -y libgoogle-glog-dev \
  && apt-get install -y libnlopt-dev \
  && apt-get install -y libsuitesparse-dev \
  && apt-get install -y ros-noetic-nlopt \
  && apt-get install -y liblapacke-dev \
  && apt-get install -y ros-noetic-gtsam \
	&& apt-get install -y iputils-ping \
	&& apt-get install -y apt-transport-https ca-certificates \
	&& apt-get install -y openssh-server python3-pip exuberant-ctags \
	&& apt-get install -y git vim tmux nano htop sudo curl wget gnupg2 \
	&& apt-get install -y bash-completion \
	&& apt-get install -y libcanberra-gtk3-0 \
    && apt-get install -y ros-noetic-gmapping ros-noetic-slam-gmapping ros-noetic-openslam-gmapping \
	&& pip3 install powerline-shell dearpygui \
	&& rm -rf /var/lib/apt/lists/* \
	&& useradd -ms /bin/bash user \
	&& echo "user:user" | chpasswd && adduser user sudo \
	&& echo "user ALL=(ALL) NOPASSWD: ALL " >> /etc/sudoers


RUN set -x \
	&& apt-get update \
    && apt-get install -y ros-noetic-multi-map-server ros-noetic-multirobot-map-merge \
	&& apt-get install -y ros-noetic-hector-sensors-gazebo \
	&& rm -rf /var/lib/apt/lists/* 

# The OSRF contianer didn't link python3 to python, causing ROS scripts to fail.
RUN ln -s /usr/bin/python3 /usr/bin/python



USER user
WORKDIR /home/user
RUN sudo usermod -a -G video user

RUN git clone https://github.com/jimeh/tmux-themepack.git ~/.tmux-themepack  \
        && git clone https://github.com/tmux-plugins/tmux-resurrect ~/.tmux-resurrect
#COPY --chown=user:user ./.tmux.conf /home/user/.tmux.conf
#COPY --chown=user:user ./.powerline.sh /home/user/.powerline.sh

#
# Set some decent colors if the container needs to be accessed via /bin/bash.
RUN echo LS_COLORS=$LS_COLORS:\'di=1\;33:ln=36\' >> ~/.bashrc \
        && echo export LS_COLORS >> ~/.bashrc \
        && echo 'source ~/.powerline.sh' >> ~/.bashrc \
        && echo 'alias tmux="tmux -2"' >> ~/.bashrc \
        && echo 'PATH=~/bin:$PATH' >> ~/.bashrc \
        && touch ~/.sudo_as_admin_successful # To surpress the sudo message at run.

RUN rosdep update \
        && echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc


# Adding the ardrone
RUN mkdir -p catkin_ws/src 
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws; catkin init'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws; catkin config --extend /opt/ros/noetic'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws; catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release'

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ethz-asl/eigen_checks.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/aakapatel/rotors_simulator.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/LTU-RAI/darpa_subt_worlds.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/catkin/catkin_simple.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ethz-asl/eigen_catkin.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ntnu-arl/lidar_simulator.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ethz-asl/mav_comm.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ros-planning/navigation_msgs.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ethz-asl/numpy_eigen.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ros-perception/perception_pcl.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ros/xacro.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/jhu-asco/dsl.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src/dsl; git checkout 61cf588668309e87de209cd95f03a0f792a16c33; mkdir build'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src/dsl/build; cmake -DCMAKE_BUILD_TYPE=Release ..'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src/dsl/build; sudo make install'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/LTU-RAI/Dsp.git'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git'

RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/src; git clone https://github.com/LTU-RAI/REF.git'




RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/; catkin build rotors_simulator'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/; catkin build dsp'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/; catkin build perception_pcl'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/; catkin build pcl_conversions'
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /home/user/catkin_ws/; catkin build ref'



RUN echo "source /home/user/catkin_ws/devel/setup.bash --extend" >> /home/user/.bashrc

STOPSIGNAL SIGTERM

CMD sudo service ssh start && /bin/bash
