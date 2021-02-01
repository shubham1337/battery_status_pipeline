# Battery Management System

This code repository contains various components for monitoring battery state information for a ROS based application.

### Quick Usage
For quickly getting started with this, please clone this repo and run the following command at the root of this repository:

_Note: This will deploy both the cloud and rosnode components on your local machine_
_Please let me know if you need the PGP_PASSPHRASE, it is used to decrypt secrets which are used in some config files_
```
docker-compose build --build-arg PGP_PASSPHRASE=<passphrase>
docker-compose up
```
After the above command starts all the containers, you can visit http://localhost:3000/ on your web browser
to monitor the changes in battery state and check for alerts in Grafana.
To access Grafana you can use the following credentials:
```
username: admin
password: admin
```

## Running tests
There are 2 sets of tests, unit tests (_dir: tests/_) for the components and ROS node tests (_dir: battery_status_package/tests_), both of these are packaged into 
docker-compose.test.yaml file and can be executed using:
```
docker-compose -f docker-compose.test.yaml up
```

---
## Components

## Overview Diagram
![Overview Diagram](https://github.com/shubham1337/battery_status_pipeline/raw/master/screenshots/overview.png "Overview Diagram")

### Monitoring & Alerting (Grafana)
_dir: grafana/_

Grafana is quite a standard Monitoring and Alerting solution, hence it was chosen to be used here. 
The datasource used in Grafana is the cloud database (Postgres)

BMS dashboard in Grafana looks like the following:
![BMS Dashboard](https://github.com/shubham1337/battery_status_pipeline/raw/master/screenshots/dash.png "BMS Dashboard")
In the above screenshot you can see the Fake battery going through the cycles of NotCharging -> Charging -> FullyCharged -> Discharging and then back to NotCharging again.
Please note, The battery capacity and data sync time was reduced to 1 V and per minute respectively to showcase the monitoring capabilities in the above screenshot, The project on submission will have the values which were specified in the assignment, which were 24 V for battery capacity and hourly cloud data sync.

#### Alerting
The alert email is currently configured to send emails to bms.alert1337@gmail.com and shubh133@gmail.com but it can be easily changed in `grafana/notifiers.yaml` config file.
There are 2 types of alerts configured:
1. No Data alert

  If there is something wrong with the pipeline and there is no data in the cloud database, this will usually trigger once when deploying the whole pipeline for the first time, as there is no data initially.
  ![No Data Alert Email](https://github.com/shubham1337/battery_status_pipeline/raw/master/screenshots/nodata_alert.png "No Data Alert Email")

2. Current Spiked alert

  This is the alert that was required in the assignment, this will get triggered if there more than 10 current spikes in the last 4 hours.
  ![Current Spiked Alert Email](https://github.com/shubham1337/battery_status_pipeline/raw/master/screenshots/current_spike_alert.png "Current Spiked Alert Email")


Alerts can also be managed in Grafana's Alerting tab:

![Grafana Alerting](https://github.com/shubham1337/battery_status_pipeline/raw/master/screenshots/alerting.png "Grafana Alerting")


### Cloud Database (Postgres)
_dir: common/db_schema/_

Postgres is a general purpose and popular SQL based Database. It was chosen for this assignment because it works well with other tools, such as Grafana and the Python-based ORM SQLAlchemy which is used by the CLoudSync component to sync the local SQLite data to this cloud Postgres database. The same ORM models are also used for both the databases.

### Cloud Sync
_dir: cloud_sync/_

Cloud Syncing from the ROS nodes to the cloud database is done through a local SQLite database, the ROS Subscribers dumps all BatteryState messages to this database and this CloudSync component runs every 1h (Based on the `cloud_sync/crontab`) and syncs to data to the cloud database and then deletes all the synced data from the local database.

### Battery State Generation
_dir: common/fake_battery/_

A fake battery was needed to generate the current, voltage and power_supply_status information which can be further used in the pipeline, hence this module was introduced. It constantly oscillates between the following states:

__Not Charged__ (0 V, 0 A) -> __Charging__ (+ve V, 1 A) -> __Fully Charged__ (24 V, -ve Current) -> __Discharging__ (dropping Voltage and Current with a gaussian distribution with mean 0.25A and variance 0.8) -> __Not Charged__ (0 V, 0 A)

This way we can test with various power_supply_status values and follow the complete charge/discharge lifecycle. A visual representation of this cycle is visible in the above Grafana dashboard screenshot.

### ROS Nodes
_dir: battery_status_package/_

There are 2 ROS nodes which are using the message type `sensor_msgs/BatteryState` and 'fake_battery_state' topic for sharing the battery state with each other. They are the main components of the ROS package, which is built using catkin tool during deployment.

#### Battery State Publisher
_dir: battery_status_package/src/publisher_

This is the ROS Publisher node which publshes the fake battery state at a rate of 0.5 Hz on the 'fake_battery_state' topic.

#### Battery State Subscriber
_dir: battery_status_package/src/subscriber_

This is the ROS Subscriber node which listens to the 'fake_battery_state' topic and dumps the BatteryState messages to a local SQLite database.



### Cloud Deployment

First Deploy the cloud services on a cloud instance:
```
docker-compose -f docker-compose.cloud.yaml up
```
Then, Make sure to set `CLOUD_DB_PATH` for the `cloud_sync` service in docker-compose.rosnodes.yaml otherwise the Cloud syncing will NOT work.
Then, Deploy ROS nodes:
On local machine (or wherever you want to deploy the ROS Nodes):
```
docker-compose -f docker-compose.rosnodes.yaml up
```

### Questions
Please let me know (shubh133@gmail.com) if you have any questions
