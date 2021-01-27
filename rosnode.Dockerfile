FROM ros:melodic

RUN mkdir -p /app/src

WORKDIR /app

#RUN . /opt/ros/melodic/setup.sh && catkin_make

COPY . /app

RUN apt-get update && \
  apt-get install -y python3-pip && \
  pip3 install pipenv && \
  pipenv install --system --deploy

RUN mv /app/battery_status_package /app/src/

ENV PYTHONPATH=/app

RUN . /opt/ros/melodic/setup.sh && catkin_make
