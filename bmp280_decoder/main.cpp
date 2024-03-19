#include <cstdint>
#include <format>
#include <iostream>
#include <string>


// This is from BMP280 and BME280 datasheets,
// chapter Trimming parameter readout.
// NOTE: BME has humidity calibration separated
static const uint8_t calibration[26] = 
{
//   LSB, MSB    
    0x36, 0x6C,     // dig_T1 uint16_t expected 27702 0x6c36
    0x05, 0x68,     // dig_T2  int16_t expected 26629 0x6805
    0x18, 0xFC,     // dig_T3  int16_t expected -1000 0xFC18
    0xA1, 0x8D,     // dig_P1 uint16_t 36257
    0x93, 0xD6,     // dig_P2 -10605
    0xD0, 0x0B,     // dig_P3 3024
    0xC3, 0x06,     // dig_P4 1731
    0x3B, 0x01,     // dig_P5 315
    0xF9, 0xFF,     // dig_P6 -7
    0x8C, 0x3C,     // dig_P7 15500
    0xF8, 0xC6,     // dig_P8 -14600
    0x70, 0x17,     // dig_P9 6000
    0x00,           // BMP: reserved, BME: undocumented
    0x00            // BMP: reserved, BME: dig_H1 as uchar
};


static const uint8_t measurement[8] =
{
    0x6C, 0x07, 0x00, 0x7E, 0x4C, 0x00, 0x00, 0x00
};


struct Calibration
{
    uint16_t T1;
    int16_t T2, T3;
    uint16_t P1;
    int16_t P2, P3, P4, P5, P6, P7, P8, P9;
    uint8_t H1;
    int16_t H2;
    uint8_t H3;
    int16_t H4, H5;
};


struct Measurement 
{
    float pressure = 0.;
    float temp = 0.;
    float humidity = 0.;
};


int32_t decode20bit(const uint8_t *data) 
{

    int32_t i = static_cast<uint32_t>(data[0]) << 12 
              | static_cast<uint32_t>(data[1]) << 4
              | static_cast<uint32_t>(data[2]) >> 4;
    return i;

}


uint16_t decodeU16LE(const uint8_t *data) 
{
    const uint32_t result = 
          uint32_t(data[0]) 
        | uint32_t(data[1]) << 8;
    return result;
}


int16_t decodeS16LE(const uint8_t *data) 
{
    uint16_t u = decodeU16LE(data);
    return (int16_t)u;
}


Calibration decodeCalibration(const uint8_t *calibration) 
{
    Calibration result;
    result.T1 = decodeU16LE(calibration); 
    result.T2 = decodeS16LE(calibration+2);
    result.T3 = decodeS16LE(calibration+4);
    result.P1 = decodeU16LE(calibration+6);
    result.P2 = decodeS16LE(calibration+8);
    result.P3 = decodeS16LE(calibration+10);
    result.P4 = decodeS16LE(calibration+12);
    result.P5 = decodeS16LE(calibration+14);
    result.P6 = decodeS16LE(calibration+16);
    result.P7 = decodeS16LE(calibration+18);
    result.P8 = decodeS16LE(calibration+20);
    result.P9 = decodeS16LE(calibration+22);
    result.H1 = calibration[25];
    return result;   
}
        


Measurement decode(
    const uint8_t *calibration, 
    const uint8_t *measurement)
{
    Measurement result;

    Calibration dig = decodeCalibration(calibration);
  
    int32_t t_fine;
    // Temperature
    {
        // typical value is 520000
        int32_t temp_raw = decode20bit(measurement+3);
        int32_t var1, var2, T;

        var1 = ((((temp_raw >> 3) - ((int32_t)dig.T1 << 1))) * ((int32_t)dig.T2)) >> 11;
        var2 = (((((temp_raw >> 4) - ((int32_t)dig.T1)) * ((temp_raw >> 4) - ((int32_t)dig.T1))) >> 12) * ((int32_t)dig.T3)) >> 14;
        
        t_fine = var1 + var2;
        T = (t_fine * 5 + 128) >> 8;    
        result.temp = (float)T / 100;    
    }

    // Pressure
    {
        // typical might be 440000
        int32_t adc_P = decode20bit(measurement+0);
        int64_t var1, var2, p;

        var1 = ((int64_t)t_fine) - 128000;
        var2 = var1 * var1 * (int64_t)dig.P6;
        var2 = var2 + ((var1*(int64_t)dig.P5)<<17);
        var2 = var2 + (((int64_t)dig.P4)<<35);
        var1 = ((var1 * var1 * (int64_t)dig.P3)>>8) + ((var1 * (int64_t)dig.P2)<<12);
        var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig.P1)>>33;
        if (var1 != 0)
        {
            p = 1048576-adc_P;
            p = (((p<<31)-var2)*3125)/var1;
            var1 = (((int64_t)dig.P9) * (p>>13) * (p>>13)) >> 25;
            var2 = (((int64_t)dig.P8) * p) >> 19;
            p = ((p + var1 + var2) >> 8) + (((int64_t)dig.P7)<<4);
            result.pressure = p / 256.0;
        }
    }

    return result;
}

int main(int argc, char **argv) 
{
    Measurement m = decode(calibration, measurement);
    std::string s  = std::format("Pressure: {}Pa, Temperature: {}C, Humidity: {}\n", m.pressure, m.temp, m.humidity);
    std::cout << s;
    return 0;
}
