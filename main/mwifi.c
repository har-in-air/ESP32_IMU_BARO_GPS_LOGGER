#include "common.h"
#include "config.h"
#include "spiflash.h"
#include "flashlog.h"
#include "mwifi.h"
	
#define TAG "wifi"


esp_err_t esp32_wifi_eventHandler(void *ctx, system_event_t *event) {
	tcpip_adapter_ip_info_t ip_info;
	switch(event->event_id) {
		case SYSTEM_EVENT_AP_START: // Handle the AP start event
			tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ip_info);
			ESP_LOGI(TAG, "Now an access point with SSID = ESP32Logger")
			ESP_LOGI(TAG, "Connect to http://" IPSTR, IP2STR(&ip_info.ip));
			break;

		case SYSTEM_EVENT_STA_DISCONNECTED: 
			ESP_LOGE(TAG, "sta_disconnected ? How did we get here ...");
			wifiInitAccessPoint();
			break;

		case SYSTEM_EVENT_STA_GOT_IP: 
			ESP_LOGE(TAG, "sta_got_IP ? How did we get here ...");
			break;

		default: 
			break;
		} 

	return ESP_OK;
	}

void serveLogData(struct netconn *conn) {  
	char szbuf[35];

	sprintf(szbuf,"HTTP/1.1 200 OK\r\n");
	netconn_write(conn, szbuf, strlen(szbuf), NETCONN_COPY);

	sprintf(szbuf,"Content-Type: application/octet-stream\r\n");
	netconn_write(conn, szbuf, strlen(szbuf), NETCONN_COPY);

	sprintf(szbuf,"Content-Length: %d\r\n", FlashLogFreeAddress);
	netconn_write(conn, szbuf, strlen(szbuf), NETCONN_COPY);

	sprintf(szbuf,"Connection: close\r\n");
	netconn_write(conn, szbuf, strlen(szbuf), NETCONN_COPY);

	sprintf(szbuf,"Access-Control-Allow-Origin: *\r\n");
	netconn_write(conn, szbuf, strlen(szbuf), NETCONN_COPY);

	sprintf(szbuf,"\r\n");
	netconn_write(conn, szbuf, strlen(szbuf), NETCONN_COPY);

	uint32_t flashAddr = 0;
   uint8_t buffer[256];
   int bytesRemaining = (int)(FlashLogFreeAddress - flashAddr);
   do {
      int numXmitBytes =  bytesRemaining > 256 ? 256 : bytesRemaining;  
		SpiFlash_ReadBuffer(flashAddr, buffer, numXmitBytes);
		netconn_write(conn, buffer, numXmitBytes, NETCONN_COPY);
      flashAddr += numXmitBytes;
      bytesRemaining = (int)(FlashLogFreeAddress - flashAddr);
		} while (bytesRemaining >= 0);
	}


void http_server_netconn_serve(struct netconn *conn) {
    struct netbuf *inbuf;
    char *buf;
    uint16_t buflen;
    err_t err;
    // Read the data from the port, blocking if nothing yet there.
    // We assume the request (the part we care about) is in one netbuf 
    err = netconn_recv(conn, &inbuf);
    if (err == ERR_OK) {
        netbuf_data(inbuf, (void**) &buf, &buflen);
		buf[buflen] = 0;
		ESP_LOGI(TAG,"%s", buf);

      // to save the binary log file as data<xyz>, e.g. dataThu9am, just
      // open the webpage at 192.168.4.1/dataThu9am
      // Only the first four chars are matched  
		if (strncmp(buf,"GET /data",9) == 0) {
			serveLogData(conn);
			}
        }
    
    // Close the connection (server closes in HTTP) 
    netconn_close(conn);
    // Delete the buffer (netconn_recv gives us ownership,
    // so we have to make sure to deallocate the buffer) 
    netbuf_delete(inbuf);
}


void http_server(void *pvParameters){
	ESP_LOGI(TAG, "Starting http_server task...");
    struct netconn *conn, *newconn;
    err_t err;
    conn = netconn_new(NETCONN_TCP);
    netconn_bind(conn, NULL, 80);
    netconn_listen(conn);
    do {
        err = netconn_accept(conn, &newconn);
        if (err == ERR_OK) {
            http_server_netconn_serve(newconn);
            netconn_delete(newconn);
        }
    } while (err == ERR_OK);
    netconn_close(conn);
    netconn_delete(conn);
}

#if 0
void wifiInitStation() {
	tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
	tcpip_adapter_ip_info_t ipInfo;

	inet_pton(AF_INET, "192.168.1.99", &ipInfo.ip); // no dhcp, use this static IP address
	inet_pton(AF_INET, "192.168.1.1", &ipInfo.gw);
	inet_pton(AF_INET, "255.255.255.0", &ipInfo.netmask);
	tcpip_adapter_set_ip_info(TCPIP_ADAPTER_IF_STA, &ipInfo);	

    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );
} 
#endif

void wifiInitAccessPoint() {    
	ESP_LOGI(TAG, "Starting access point \"ESP32Logger\"");
   ESP_LOGI(TAG, "To download binary log file as \"data<...>\", open http server web page at http://192.168.4.1/data<...>");
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
	wifi_config_t apConfig = {
		.ap = {
			.ssid="ESP32Logger",
			.ssid_len=0,
			.password="logger",
			.channel=0,
			.authmode=WIFI_AUTH_OPEN,
			.ssid_hidden=0,
			.max_connection=4,
			.beacon_interval=100
		}
	};
	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &apConfig));
	ESP_ERROR_CHECK(esp_wifi_start());
} 

