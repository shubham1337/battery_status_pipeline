FROM python:3.6

COPY . /app

WORKDIR /app

ENV PYTHONPATH=/app

# Install Dependencies
RUN apt-get update && \
  apt-get install -y cron && \
  pip install pipenv && \
  pipenv install

ENTRYPOINT ["./cloud_sync/run.sh"] 
