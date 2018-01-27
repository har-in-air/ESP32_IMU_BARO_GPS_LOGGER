#include "common.h"
#include "config.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "max21100.h"

#define TAG "nvs"


void nvs_Init(void) {
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
		}
    ESP_ERROR_CHECK( err );

    ESP_LOGI(TAG,"Opening Non-Volatile Storage (NVS) handle... ");
    nvs_handle my_handle;
    err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
		return;
		} 

	gxBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "gxBias", &gxBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"gxBias = %d", gxBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"gxBias is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading gxBias !", err);
		}
	gyBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "gyBias", &gyBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"gyBias = %d", gyBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"gyBias value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading gyBias!", err);
		}
	gzBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "gzBias", &gzBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"gzBias = %d", gzBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"gzBias value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading gzBias!", err);
		}

		
	axBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "axBias", &axBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"axBias = %d", axBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"axBias is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading axBias !", err);
		}
	ayBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "ayBias", &ayBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"ayBias = %d", ayBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"ayBias value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading ayBias!", err);
		}
	azBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "azBias", &azBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"azBias = %d", azBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"azBias value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading azBias!", err);
		}

	mxBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "mxBias", &mxBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"mxBias = %d", mxBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"mxBias is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading mxBias !", err);
		}
	myBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "myBias", &myBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"myBias = %d", myBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"myBias value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading myBias!", err);
		}
	mzBias = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "mzBias", &mzBias);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"mzBias = %d", mzBias);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"mzBias value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading mzBias!", err);
		}

	mxSensitivity = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "mxSensitivity", &mxSensitivity);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"mxSensitivity = %d", mxSensitivity);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"mxSensitivity is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading mxSensitivity !", err);
		}
	mySensitivity = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "mySensitivity", &mySensitivity);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"mySensitivity = %d", mySensitivity);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"mySensitivity value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading mySensitivity!", err);
		}
	mzSensitivity = 0; // value will default to 0, if not set yet in NVS
	err = nvs_get_i16(my_handle, "mzSensitivity", &mzSensitivity);
	switch (err) {
		case ESP_OK:
			ESP_LOGI(TAG,"mzSensitivity = %d", mzSensitivity);
			break;
		case ESP_ERR_NVS_NOT_FOUND:
			ESP_LOGI(TAG,"mzSensitivity value is not initialized yet!");
			break;
		default :
			ESP_LOGE(TAG,"Error (%d) reading mzSensitivity!", err);
		}
	}


void nvs_SaveGyro(void ) {	
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
		return;
		} 
	err = nvs_set_i16(my_handle, "gxBias", gxBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving gxBias");
		}
	err = nvs_set_i16(my_handle, "gyBias", gyBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving gyBias");
		}
	err = nvs_set_i16(my_handle, "gzBias", gzBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving gzBias");
		}
	err = nvs_commit(my_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"nvs_commit error");
		}
	nvs_close(my_handle);
    }


void nvs_SaveAccel(void ) {	
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
		return;
		} 
	err = nvs_set_i16(my_handle, "axBias", axBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving axBias");
		}
	err = nvs_set_i16(my_handle, "ayBias", ayBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving gyBias");
		}
	err = nvs_set_i16(my_handle, "azBias", azBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving azBias");
		}
	err = nvs_commit(my_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"nvs_commit error");
		}
	nvs_close(my_handle);
    }

void nvs_SaveMag(void ) {	
    nvs_handle my_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG,"Error (%d) opening NVS handle!", err);
		return;
		} 
	err = nvs_set_i16(my_handle, "mxBias", mxBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving mxBias");
		}
	err = nvs_set_i16(my_handle, "myBias", myBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving myBias");
		}
	err = nvs_set_i16(my_handle, "mzBias", mzBias);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving mzBias");
		}
	err = nvs_set_i16(my_handle, "mxSensitivity", mxSensitivity);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving mxSensitivity");
		}
	err = nvs_set_i16(my_handle, "mySensitivity", mySensitivity);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving mySensitivity");
		}
	err = nvs_set_i16(my_handle, "mzSensitivity", mzSensitivity);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"Error saving mzSensitivity");
		}
	err = nvs_commit(my_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG,"nvs_commit error");
		}
	nvs_close(my_handle);
    }
