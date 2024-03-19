## What is this

Decoder for Bosch BMP280 (and in future BME280) raw data output
written in C++20 (only because of integrated fmt::lib).

## Why it exists (motivation)
Attempt was to try how hard is to read data from the sensor.
Actually it's hard, if it does not work, there's no reference 
how calibration data should look like etc.

## References

Bosch datasheets and samples

Inspiration (for debugging and typical values) was 
https://github.com/mahfuz195/BMP280-arduino-library

## How to compile it

I used Visual Studio Code, very minimal CMakeLists.txt and clang++ on Fedora 39.

Something like this should work:
```sh
cmake -S . -B build && cd build && make && cd .. && build/bosch-decode
```
