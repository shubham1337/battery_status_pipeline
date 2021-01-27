FROM python:3.7

COPY . /app

WORKDIR /app

# Install Dependencies
RUN apt-get update && \
  apt-get install -y cron && \
  pip install pipenv && \
  pipenv install --system --deploy

# Setup cron for scheduled running of the task
RUN cp cloud_sync/crontab /etc/cron.d/crontab && \
  chmod 0644 /etc/cron.d/crontab && \
  crontab /etc/cron.d/crontab

ENV PYTHONPATH=/app

#ENTRYPOINT ["cron", "-f"] 
