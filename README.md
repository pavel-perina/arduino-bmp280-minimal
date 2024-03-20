## What is this

This consists of three projects:

* Minimal Arduino raw and calibration data reader with output like this:
  ```
  === BME280 TEST PROGRAM ============
  ChipID: 58 -> BMP280
  BME280 Calibration Data (26 bytes): 36 6C 05 68 18 FC A1 8D 93 D6 D0 0B C3 06 3B 01 F9 FF 8C 3C F8 C6 70 17 00 00 
  BME280 Raw Data: 6B B7 00 7E 01 00 00 00 
  BME280 Raw Data: 6B B9 00 7E 02 00 00 00 
  ```  

* The same for BMP388 (based on previous project, datasheet and [Bosch SensorTec example](https://github.com/boschsensortec/BMP3_SensorAPI/tree/master)
  ```
  === Bmp388 TEST PROGRAM ============
  ChipID: 50 -> BMP388
  Bmp388 Calibration Data (21 bytes):     48 6B 17 49 F6 6F 02 0B F8 23 00 27 61 8A 78 F3 F6 EE 41 17 C4 
  Bmp388 Raw Data:        60 29 6D C0 07 7F 33 5F 00 
  Bmp388 Raw Data:        30 29 6D C0 07 7F C2 C2 00 
  Bmp388 Raw Data:        80 29 6D 10 08 7F 4B 26 01 
  Bmp388 Raw Data:        A0 29 6D 80 08 7F CA 89 01 
  Bmp388 Raw Data:        90 29 6D 80 08 7F 57 ED 01 
  ```

* Decoder for Bosch BMP280 (and in future BME280) raw data output written in C++20 (only because of integrated fmt::lib).
  As of 2024-03-19 it contains hardcoded data.
  ```
  ./bosch-decode 
  Pressure: 99414.17Pa, Temperature: 23.45C, Humidity: 0
  ```
  
It's meant as a sandbox for a future data logger

