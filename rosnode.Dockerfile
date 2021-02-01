FROM ros:melodic

COPY . /app

WORKDIR /app

ENV PYTHONPATH=/app
ENV ROS_PYTHON_VERSION=3

# Install Dependencies and Build ROS Package
RUN apt-get update && \
  apt-get install -y python3-pip && \
  pip3 install pipenv && \
  pipenv install --system --deploy && \
  mkdir -p /app/src && \
  mv /app/battery_status_package /app/src/ && \
  . /opt/ros/melodic/setup.sh && catkin_make
