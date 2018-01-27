#ifndef MWIFI_H_
#define MWIFI_H_

esp_err_t esp32_wifi_eventHandler(void *ctx, system_event_t *event);
void wifiInitStation();
void wifiInitAccessPoint();
void serveLogData(struct netconn *conn);
void http_server_netconn_serve(struct netconn *conn);
void http_server(void *pvParameters);

#endif