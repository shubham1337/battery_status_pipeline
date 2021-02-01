# Battery Management System

This code repository contains various components for monitoring battery state information for any ROS based application.

### Quick Usage
For quickly getting started with this, please clone this repo and run the following command at the root of this repository:
_Note: This will deploy both the cloud and rosnode components on your local machine_
```
docker-compose -e PGP_PASSPHRASE=<passphrase> up
```
After the above command starts all the containers, you can visit http://localhost:3000/ on your web browser
to monitor the changes in battery state and check for alerts in Grafana.
To access Grafana you can use the following credentials:
```
username: admin
password: admin
```

---
## Components

### Monitoring & Alerting (Grafana)
_dir: grafana/_

### Cloud Database (Postgres)
_dir: common/db_schema/_

### Cloud Sync
_dir: cloud_sync/_


### Battery State Generation
_dir: common/fake_battery/_

### ROS Nodes
_dir: battery_status_package/_

#### Battery State Publisher
_dir: battery_status_package/src/publisher_

#### Battery State Subscriber
_dir: battery_status_package/src/subscriber_
