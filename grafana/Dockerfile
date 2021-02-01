FROM grafana/grafana

ARG PGP_PASSPHRASE

# Create config dirs
RUN mkdir -p /etc/grafana/provisioning/dashboards/ && \
  mkdir -p /etc/grafana/provisioning/notifiers/ && \
  mkdir -p /etc/grafana/provisioning/datasources/

# Copy config files to correct config dirs
COPY ./grafana.ini.asc /etc/grafana/
COPY ./home.json /etc/grafana/provisioning/dashboards/
COPY ./datasources.yaml /etc/grafana/provisioning/datasources/
COPY ./dashboards.yaml /etc/grafana/provisioning/dashboards/
COPY ./notifiers.yaml /etc/grafana/provisioning/notifiers/

# Decrypt config files with secrets
COPY ./private.gpg /etc/grafana/
USER root
RUN apk add gnupg && \
  echo $PGP_PASSPHRASE | gpg --batch --yes --passphrase-fd 0 --pinentry-mode loopback --import /etc/grafana/private.gpg && \
  echo $PGP_PASSPHRASE | gpg --batch --yes --passphrase-fd 0 --pinentry-mode loopback --decrypt /etc/grafana/grafana.ini.asc > /etc/grafana/grafana.ini
