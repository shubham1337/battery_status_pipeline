FROM ros:melodic

RUN apt-get update && \
  apt-get install -y python3-pip && \
  pip3 install pyyaml rospkg

RUN mkdir -p /app/src

WORKDIR /app

#RUN /bin/bash -c "source /opt/ros/melodic/setup.bash ; catkin_make ; catkin_create_pkg battery_status_package std_msgs rospy"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash ; catkin_make"

COPY . /app

RUN mv /app/battery_status_package /app/src/

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash ; catkin_make"
