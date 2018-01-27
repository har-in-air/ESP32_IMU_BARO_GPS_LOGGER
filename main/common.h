#ifndef COMMON_H_
#define COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_attr.h"
#include "esp_intr_alloc.h"
#include "driver/timer.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/api.h"
#include "tcpip_adapter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/portmacro.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "hal/esp32-hal.h"  // has good stuff like NOP etc.

#define delayMs(ms)  vTaskDelay((ms) / portTICK_PERIOD_MS);

#define MIN(x,y)                 ((x) < (y) ? (x) : (y))
#define MAX(x,y)                 ((x) > (y) ? (x) : (y))
#define ABS(x)                 ((x) < 0 ? -(x) : (x))
#define CLAMP(x,mn,mx)       {if (x <= (mn)) x = (mn); else if (x >= (mx)) x = (mx);}
#define CORE(x,t)              {if (ABS(x) <= (t)) x = 0;}
#define MCORE(x,t)              {if (x > (t)) x -= (t); else if (x < -(t)) x += (t); else x = 0;}
#define CORRECT(x,mx,mn)  		(((float)(x-mn)/(float)(mx-mn)) - 0.5f)

//#define M_PI 		  3.1415927f
#define RAD2DEG(r)   ((r)*57.29577951f)
#define DEG2RAD(d)   ((d)*0.017453292f)

#define _180_DIV_PI         57.295779f
#define PI_DIV_180          0.017453292f

#endif