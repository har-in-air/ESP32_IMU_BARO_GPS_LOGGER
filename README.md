# ESP32_IMU_BARO_GPS_LOGGER

Logs IMU (gyroscope, accelerometer and magnetometer) data samples at 500Hz

Logs Barometric pressure sensor derived altitude readings at 50Hz

Logs GPS readings at 10Hz

Data is logged to a 128Mbit spi serial flash, and can be uploaded via Wifi with the unit acting as a wifi access point and http web server.

Purpose is to log real-time data for offline analysis and algorithm development.

The /docs directory has some example data dumps, and code used to extract the data from the binary log dump, and process it. This was compiled and run on an AMDx64 Ubuntu 16.04LTS platform, and the resulting filtered data charted using LibreOffice.
