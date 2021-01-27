#!/bin/sh

# Setup cron for scheduled running of the task
cat /dev/null > /etc/cron.d/crontab
env >> /etc/cron.d/crontab
cat /app/cloud_sync/crontab >> /etc/cron.d/crontab
chmod 0644 /etc/cron.d/crontab
crontab /etc/cron.d/crontab
service cron start
touch /app/cron.logs
tail -f /app/cron.logs
