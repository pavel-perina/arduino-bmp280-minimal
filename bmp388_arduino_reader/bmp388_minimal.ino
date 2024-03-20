/// \file   bmp388_minimal.ino
/// \author Pavel Perina
/// \date   2024-03-20
/// \brief  Attempt to make minimal BMP388 sensor reader

// https://www.bosch-sensortec.com/products/environmental-sensors/pressure-sensors/
// https://github.com/boschsensortec/BMP3_SensorAPI

// Typical output (modified, without and with oversampling):
// Tested with Arduino Nano Every
// === Bmp388 TEST PROGRAM ============
// ChipID: 50 -> BMP388
// Address                              30|31 32 33 34 35 36 37 38 39 3A 3B 3C 3D 3E 3F 40 41 42 43 44 45|46 47 48 49.4A 4B 4C 4D|4E 4F 50 51 52 53 54 55 56 57
// Bmp388 Calibration Data (40 bytes):	2D 48 6B 17 49 F6 6F 02 0B F8 23 00 27 61 8A 78 F3 F6 EE 41 17 C4 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 
// Bmp388 Calibration Data (21 bytes):	   48 6B 17 49 F6 6F 02 0B F8 23 00 27 61 8A 78 F3 F6 EE 41 17 C4 
// Bmp388 Raw Data:	00 48 6D 00 68 7F 95 D6 00 
// Bmp388 Raw Data:	00 47 6D 00 68 7F 02 9F 01 
// Bmp388 Raw Data:	00 45 6D 00 68 7F AE 67 02 
// Bmp388 Raw Data:	B8 25 6D 90 F0 7E E8 24 08 
// Bmp388 Raw Data:	00 26 6D 68 F0 7E 5A 88 08 
// Bmp388 Raw Data:	E8 25 6D C0 F1 7E C8 EB 08 
// Bmp388 Raw Data:	80 26 6D 80 F2 7E 41 4F 09 
// Bmp388 Raw Data:	80 26 6D 60 F2 7E BD B2 09
#include <Wire.h>


/// Wrapper for I2C bus
class DeviceI2C
{
public:
  using uint = unsigned int;
  enum ErrCode {
    OK,
    TIMEOUT,
    END_TRANSMISSION_FAIL
  };
  /// Constructor
  DeviceI2C(uint8_t address, uint waitMinMs = 0);
  /// Write byte
  ErrCode writeByte(uint8_t) const;
  /// Write two bytes (e.g. addr, value)
  ErrCode writeBytes(uint8_t, uint8_t) const;
  /// Write byte array
  ErrCode writeArray(uint8_t addr, const uint8_t* data, size_t len) const;
  /// Read byte array
  ErrCode readArray(uint8_t addr, uint8_t* data, size_t len) const;
protected:
  ErrCode awaitData(size_t len) const;
  uint8_t m_i2cAddr;
  uint m_waitMin;
  uint m_waitMax;
};



DeviceI2C::DeviceI2C(uint8_t address, uint waitMinMs)
  : m_i2cAddr(address)
  , m_waitMin(waitMinMs)
  , m_waitMax(200)
{  
}


DeviceI2C::ErrCode DeviceI2C::writeByte(uint8_t b) const
{
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(b);
  return (Wire.endTransmission() == 0) ? ErrCode::OK : ErrCode::END_TRANSMISSION_FAIL;
}


DeviceI2C::ErrCode DeviceI2C::writeBytes(uint8_t b0, uint8_t b1) const
{
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(b0);
  Wire.write(b1);
  return (Wire.endTransmission() == 0) ? ErrCode::OK : ErrCode::END_TRANSMISSION_FAIL;
}


DeviceI2C::ErrCode DeviceI2C::writeArray(uint8_t addr, const uint8_t* data, size_t len) const
{
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(addr);
  for (size_t i = 0; i < len; ++i) {
    Wire.write(data[i]);
  }
  return (Wire.endTransmission() == 0) ? ErrCode::OK : ErrCode::END_TRANSMISSION_FAIL;
}


DeviceI2C::ErrCode DeviceI2C::awaitData(size_t len) const
{
  constexpr uint8_t retryMs = 5;
  // Wait for data to become available, up to 100ms
  uint counter = m_waitMin;
  delay(m_waitMin);
  while (Wire.available() < len) {
    counter += retryMs;
    delay(retryMs);
    if (counter > m_waitMax)
      return ErrCode::TIMEOUT;
  }
  return ErrCode::OK;
}


DeviceI2C::ErrCode DeviceI2C::readArray(uint8_t addr, uint8_t* data, size_t len) const
{
  DeviceI2C::ErrCode errCode;
  errCode = writeByte(addr);
  if (errCode != ErrCode::OK) {
    return errCode;
  }
  Wire.requestFrom(m_i2cAddr, len);
  errCode = awaitData(len);
  if (errCode != ErrCode::OK) {
    return errCode;
  }
  Wire.readBytes(data, len);
  return ErrCode::OK;
}

////////////////////////////////////////////////////////////////////
class Bmp388
{
public:  
  /// Constructor
  Bmp388();
  /// Initialize measurement
  void begin();
  /// Read chip id
  DeviceI2C::ErrCode readChipId(uint8_t& data) const;
  /// Read calibration data (as of 2024-03-19 only for temperature and pressure)
  DeviceI2C::ErrCode readCalibData(uint8_t* data) const;
  /// Read raw temperature value
  DeviceI2C::ErrCode readRawTemperature(uint8_t* data) const;
  /// Read raw pressure value
  DeviceI2C::ErrCode readRawPressure(uint8_t* data) const;
  /// Read raw time value
  DeviceI2C::ErrCode readRawTime(uint8_t* data) const;
  /// Read all at once (8 bytes)
  DeviceI2C::ErrCode readRawData(uint8_t* data) const;
  /// Resets chip
  DeviceI2C::ErrCode reset() const;
  static constexpr uint8_t RAW_TEMPERATURE_LEN = 3;
  static constexpr uint8_t RAW_PRESSURE_LEN    = 3;
  static constexpr uint8_t RAW_TIME_LEN        = 3;
  static constexpr uint8_t CALIB_DATA_LEN      = 0x45 - 0x31 + 1; // maybe 0x31-0x45, maybe 0x30-0x57
  /// Decode id as a string
  /// \returns pointer to static buffer
  static const char* decodeId(uint8_t); 
private:
  static constexpr uint8_t BMP388_I2C_ADDR = 0x77;

  static constexpr uint8_t REG_CHIP_ID    = 0x00;
  static constexpr uint8_t REG_REV_ID     = 0x01;
  
  // bit[5:4] 00 sleep (default), 01 or 10 forced, 11 normal, [1] temp_on, [0] press_on -> set to 0b00110011 = 0x33
  // in sleep mode chip id and compensation and be read, all regs accessible
  // forced is one-shot and then chip goes to sleep
  static constexpr uint8_t REG_POWER_CTL  = 0x1B; 
  // oversampling [5:3]osr_t [2:0]osr_p
  static constexpr uint8_t REG_OSR_CTL    = 0x1C; 
  // odr_sel [4:0]
  static constexpr uint8_t REG_ODR_CTL    = 0x1D;
  // [3:1] irr_filter, [0] short_in
  static constexpr uint8_t REG_CONFIG     = 0x1F;
  //static constexpr uint8_t REG_RESET      = 0x0;
  static constexpr uint8_t REG_CALIB_DATA = 0x31; // 0x31-0x45 21 bytes, other page says 0x30-0x57

    static constexpr uint8_t REG_PRESS_XLSB  = 0x04; ///< Starting address of pressure data // XSLB, LSB, MSB
  static constexpr uint8_t REG_TEMP_XLSB   = 0x07; ///< Starting address of temperature data
  static constexpr uint8_t REG_TIME_XLSB   = 0x0C; ///< Starting address of time data

  static constexpr uint8_t REG_CMD         = 0x7E;
  DeviceI2C m_bus;  ///< Simple bus driver
  uint8_t m_chipId; ///< ChipID 
};


Bmp388::Bmp388()
  : m_bus(BMP388_I2C_ADDR)
  , m_chipId(0)
{
}


void Bmp388::begin()
{
  // we want
  // reset
  // enable pressure & temp
  // filter no OS for both
  // normal mode
  readChipId(m_chipId);
  reset();
#if 0
  m_bus.writeBytes(REG_ODR_CTL, 0x06); // 3Hz
  m_bus.writeBytes(REG_OSR_CTL, 0b00000000); // no oversampling
  m_bus.writeBytes(REG_CONFIG, 0x00); // IIR bypass
#else
  m_bus.writeBytes(REG_ODR_CTL, 0x05); // ODR 12.5Hz
  m_bus.writeBytes(REG_OSR_CTL, 0b00101101); // 32x oversampling
  m_bus.writeBytes(REG_CONFIG, 0x00); // IIR bypass
#endif
  m_bus.writeBytes(REG_POWER_CTL, 0b00110011); // pressure&temp on, normal mode
  /*m_bus.writeBytes(REG_CTRL_MEAS, 0x27); // 0b0010.0111
  m_bus.writeBytes(REG_CONFIG,    0x00);*/

}

DeviceI2C::ErrCode Bmp388::reset() const 
{
  auto err = m_bus.writeBytes(REG_CMD, 0xB6);
  // Delay to ensure the reset process completes
  delay(10); 
  //return err;
  return DeviceI2C::ErrCode::OK;
}


DeviceI2C::ErrCode Bmp388::readChipId(uint8_t& data) const
{
  return m_bus.readArray(REG_CHIP_ID, &data, 1);
}


DeviceI2C::ErrCode Bmp388::readCalibData(uint8_t* data) const
{
  return m_bus.readArray(REG_CALIB_DATA, data, CALIB_DATA_LEN);
}


DeviceI2C::ErrCode Bmp388::readRawTemperature(uint8_t* data) const 
{
  return m_bus.readArray(REG_TEMP_XLSB, data, RAW_TEMPERATURE_LEN);
}


DeviceI2C::ErrCode Bmp388::readRawPressure(uint8_t* data) const 
{
  return m_bus.readArray(REG_PRESS_XLSB, data, RAW_PRESSURE_LEN);
}


DeviceI2C::ErrCode Bmp388::readRawTime(uint8_t* data) const 
{
  return m_bus.readArray(REG_TIME_XLSB, data, RAW_TIME_LEN);
}


DeviceI2C::ErrCode Bmp388::readRawData(uint8_t* data) const 
{
  /*if (m_chipId == 0x60) {
    return m_bus.readArray(REG_PRESS_MSB, data, RAW_PRESSURE_LEN + RAW_TEMPERATURE_LEN + RAW_HUMIDITY_LEN);
  } else {
    data[6] = data[7] = 0x00;
    return m_bus.readArray(REG_PRESS_MSB, data, RAW_PRESSURE_LEN + RAW_TEMPERATURE_LEN);
  }*/
  //return m_bus.readArray(REG_PRESS_XLSB, data, RAW_PRESSURE_LEN + RAW_TEMPERATURE_LEN + RAW_HUMIDITY_LEN);
  return DeviceI2C::ErrCode::OK;
}


const char* Bmp388::decodeId(uint8_t id)
{
  static const char *table[] = { "UNKNOWN", "BMP388", "BMP390" };
  switch(id) {
    case 0x50: return table[1];
    case 0x60: return table[2];
  }
  return table[0];

}

/// Auxiliary helpers
namespace aux {

void printByte(uint8_t b) 
{
  static const char table[] = "0123456789ABCDEF";
  Serial.print(table[(b & 0xF0) >> 4]);
  Serial.print(table[ b & 0x0F]);
};

void printArray(const uint8_t* data, uint8_t len)
{
  for (int i = 0; i < len; ++i) {
    printByte(data[i]);
    Serial.print(" ");
  }
  Serial.println("");
}

} // namespace aux


/// Setup code
void setup() 
{
  Wire.begin();

  Serial.begin(9600);
  while(!Serial)
    ;

  Serial.println("=== Bmp388 TEST PROGRAM ============");    

  Bmp388 Bmp388;
  Bmp388.begin();
  Serial.print("ChipID: ");
  uint8_t chipId;
  Bmp388.readChipId(chipId);
  Serial.print(chipId, HEX);
  Serial.print(" -> ");
  Serial.println(Bmp388::decodeId(chipId));

  Serial.print("Bmp388 Calibration Data (");
  Serial.print(Bmp388::CALIB_DATA_LEN);
  Serial.print(" bytes):\t");
  uint8_t calibData[Bmp388::CALIB_DATA_LEN];
  Bmp388.readCalibData(calibData);
  aux::printArray(calibData, Bmp388::CALIB_DATA_LEN);  
}


/// Main loop
void loop() 
{
  delay(1000);
  uint8_t measurement[Bmp388::RAW_PRESSURE_LEN + Bmp388::RAW_TEMPERATURE_LEN + Bmp388::RAW_TIME_LEN] = { 0 };

  Bmp388 Bmp388; 
  
  Bmp388.readRawPressure(measurement);
  Bmp388.readRawTemperature(measurement + Bmp388::RAW_PRESSURE_LEN);
  Bmp388.readRawTime(measurement + Bmp388::RAW_PRESSURE_LEN + Bmp388::RAW_TEMPERATURE_LEN);
  
  
  Serial.print("Bmp388 Raw Data:\t");  
  aux::printArray(measurement, sizeof(measurement));
}
