FROM duckietown/rpi-ros-kinetic-roscore:master18

LABEL maintainer="ne@cegeka.com"

# REQUIRED ENVIRONMENT VARIABLES THAT HAVE TO BE PASSED WHEN RUNNING THE CONTAINER:
# ROS_MASTER_URI - the hostname and port of the roscore master, typically http://hostname:11311 - ALWAYS REQUIRED! 
# DUCKIEBOT_NAME - the hostname of the Duckiebot, e.g. duckiebot


RUN [ "cross-build-start" ]

ENV ROS_LANG_DISABLE=genlisp:gencpp:gennodejs:geneus

RUN git clone https://github.com/RPi-Distro/python-sense-hat /tmp/sense-hat && \
    pip install -e /tmp/sense-hat && \
    git clone https://github.com/RPi-Distro/RTIMULib/ /tmp/RTIMU && \
    cd /tmp/RTIMU/Linux/python && ls -la && python setup.py build && python setup.py install

RUN mkdir /node-ws 

COPY /src /node-ws/src
COPY .gitignore /node-ws
COPY .catkin_workspace /node-ws
COPY node_launch.sh /node-ws

ENV ROS_HOSTNAME duckie


RUN /bin/bash -c "cd /node-ws && source /opt/ros/kinetic/setup.bash && catkin_make -C /node-ws"

RUN /bin/bash -c "source /node-ws/devel/setup.bash"

RUN /bin/bash -c "chmod +x /node-ws/node_launch.sh"

RUN /bin/bash -c "pip install Adafruit_PCA9685 azure-iothub-device-client"

RUN [ "cross-build-end" ]

WORKDIR /node-ws

CMD [ "/node-ws/node_launch.sh" ]
