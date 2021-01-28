FROM grafana/grafana

RUN mkdir -p /etc/grafana/provisioning/dashboards/ && \
  mkdir -p /etc/grafana/provisioning/notifiers/ && \
  mkdir -p /etc/grafana/provisioning/datasources/

COPY ./grafana.ini /etc/grafana/grafana.ini
COPY ./home.json /etc/grafana/provisioning/dashboards/home.json
COPY ./datasources.yaml /etc/grafana/provisioning/datasources/
COPY ./dashboards.yaml /etc/grafana/provisioning/dashboards/
COPY ./notifiers.yaml /etc/grafana/provisioning/notifiers/
