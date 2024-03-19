## What is this

This consists of two projects:

* Minimal Arduino raw and calibration data reader with output like this:
  ```
  === BME280 TEST PROGRAM ============
  ChipID: 58 -> BMP280
  BME280 Calibration Data (26 bytes): 36 6C 05 68 18 FC A1 8D 93 D6 D0 0B C3 06 3B 01 F9 FF 8C 3C F8 C6 70 17 00 00 
  BME280 Raw Data: 6B B7 00 7E 01 00 00 00 
  BME280 Raw Data: 6B B9 00 7E 02 00 00 00 
  ```  
* Decoder for Bosch BMP280 (and in future BME280) raw data output written in C++20 (only because of integrated fmt::lib).
  As of 2024-03-19 it contains hardcoded data.
  ```
  ./bosch-decode 
  Pressure: 99414.17Pa, Temperature: 23.45C, Humidity: 0
  ```
  
It's meant as a sandbox for a future data logger

