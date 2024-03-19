/// \file   bmp280_minimal.ino
/// \author Pavel Perina
/// \date   2024-03-18
/// \brief  Attempt to make minimal BMP280/BME280 sensor reader for a future data logger

// Typical output:
// === BME280 TEST PROGRAM ============
// ChipID: 58 -> BMP280
// BME280 Calibration Data (26 bytes): 36 6C 05 68 18 FC A1 8D 93 D6 D0 0B C3 06 3B 01 F9 FF 8C 3C F8 C6 70 17 00 00 
// BME280 Raw Data:	6B D1 00 7E 66 00 00 00 
// BME280 Raw Data:	6B CF 00 7E 61 00 00 00 

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
  ErrCode writeArray(uint8_t addr, const uint8_t* data, uint8_t len) const;
  /// Read byte array
  ErrCode readArray(uint8_t addr, uint8_t* data, uint8_t len) const;
protected:
  ErrCode awaitData(uint8_t len) const;
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


DeviceI2C::ErrCode DeviceI2C::writeArray(uint8_t addr, const uint8_t* data, uint8_t len) const
{
  Wire.beginTransmission(m_i2cAddr);
  Wire.write(addr);
  for (int i = 0; i < len; ++i) {
    Wire.write(data[i]);
  }
  return (Wire.endTransmission() == 0) ? ErrCode::OK : ErrCode::END_TRANSMISSION_FAIL;
}


DeviceI2C::ErrCode DeviceI2C::awaitData(uint8_t len) const
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


DeviceI2C::ErrCode DeviceI2C::readArray(uint8_t addr, uint8_t* data, uint8_t len) const
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
class Bme280
{
public:  
  /// Constructor
  Bme280();
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
  /// Read raw humidity value (BME only)
  DeviceI2C::ErrCode readRawHumidity(uint8_t* data) const;
  /// Read all at once (8 bytes)
  DeviceI2C::ErrCode readRawData(uint8_t* data) const;
  /// Resets chip
  DeviceI2C::ErrCode reset() const;
  static constexpr uint8_t RAW_TEMPERATURE_LEN = 3;   // 3 bytes (20bits)
  static constexpr uint8_t RAW_HUMIDITY_LEN    = 2;   // 2 bytes (16bits)
  static constexpr uint8_t RAW_PRESSURE_LEN    = 3;   // 3 bytes (20bits)
  static constexpr uint8_t CALIB_DATA_LEN      = 26;  // 26 bytes for temp and pressure cal.
  /// Decode id as a string
  /// \returns pointer to static buffer
  static const char* decodeId(uint8_t); 
private:
  static constexpr uint8_t BME280_I2C_ADDR = 0x76;

  static constexpr uint8_t REG_CHIP_ID    = 0xD0;
  static constexpr uint8_t REG_RESET      = 0xE0;
  static constexpr uint8_t REG_CALIB_DATA = 0x88; // 0x88-0xA1 26 bytes of clibration data

  static constexpr uint8_t REG_STATUS     = 0xF3;
  static constexpr uint8_t REG_CTRL_MEAS  = 0xF4;
  static constexpr uint8_t REG_CONFIG     = 0xF5;

  static constexpr uint8_t REG_PRESS_MSB  = 0xF7; ///< Starting address of pressure data
  static constexpr uint8_t REG_TEMP_MSB   = 0xFA; ///< Starting address of temperature data
  static constexpr uint8_t REG_HUM_MSB    = 0xFD; ///< Starting address of humidity data (BME280 only)

  DeviceI2C m_bus;  ///< Simple bus driver
  uint8_t m_chipId; ///< ChipID 
};


Bme280::Bme280()
  : m_bus(BME280_I2C_ADDR)
  , m_chipId(0)
{
}


void Bme280::begin()
{
  readChipId(m_chipId);
  reset();
  m_bus.writeBytes(REG_CTRL_MEAS, 0x27);
  m_bus.writeBytes(REG_CONFIG,    0x00);

}

DeviceI2C::ErrCode Bme280::reset() const 
{
  auto err = m_bus.writeBytes(REG_RESET, 0xB6);
  // Delay to ensure the reset process completes
  delay(10); 
  return err;
}


DeviceI2C::ErrCode Bme280::readChipId(uint8_t& data) const
{
  return m_bus.readArray(REG_CHIP_ID, &data, 1);
}


DeviceI2C::ErrCode Bme280::readCalibData(uint8_t* data) const
{
  return m_bus.readArray(REG_CALIB_DATA, data, CALIB_DATA_LEN);
}


DeviceI2C::ErrCode Bme280::readRawTemperature(uint8_t* data) const 
{
  return m_bus.readArray(REG_TEMP_MSB, data, RAW_TEMPERATURE_LEN);
}


DeviceI2C::ErrCode Bme280::readRawPressure(uint8_t* data) const 
{
  return m_bus.readArray(REG_PRESS_MSB, data, RAW_PRESSURE_LEN);
}


DeviceI2C::ErrCode Bme280::readRawHumidity(uint8_t* data) const 
{
  return m_bus.readArray(REG_HUM_MSB, data, RAW_HUMIDITY_LEN);
}


DeviceI2C::ErrCode Bme280::readRawData(uint8_t* data) const 
{
  if (m_chipId == 0x60) {
    return m_bus.readArray(REG_PRESS_MSB, data, RAW_PRESSURE_LEN + RAW_TEMPERATURE_LEN + RAW_HUMIDITY_LEN);
  } else {
    data[6] = data[7] = 0x00;
    return m_bus.readArray(REG_PRESS_MSB, data, RAW_PRESSURE_LEN + RAW_TEMPERATURE_LEN);
  }
}


const char* Bme280::decodeId(uint8_t id)
{
  static const char *table[] = { "UNKNOWN", "BMP180", "BMP280", "BME280" };
  switch(id) {
    case 0x55: return table[1];
    case 0x58: return table[2];
    case 0x60: return table[3];
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

  Serial.println("=== BME280 TEST PROGRAM ============");    

  Bme280 bme280;
  bme280.begin();
  Serial.print("ChipID: ");
  uint8_t chipId;
  bme280.readChipId(chipId);
  Serial.print(chipId, HEX);
  Serial.print(" -> ");
  Serial.println(Bme280::decodeId(chipId));

  Serial.print("BME280 Calibration Data (");
  Serial.print(Bme280::CALIB_DATA_LEN);
  Serial.print(" bytes):\t");
  uint8_t calibData[Bme280::CALIB_DATA_LEN];
  bme280.readCalibData(calibData);
  aux::printArray(calibData, Bme280::CALIB_DATA_LEN);

  bme280.begin();
}


/// Main loop
void loop() 
{
  delay(2000);
  uint8_t measurement[Bme280::RAW_PRESSURE_LEN + Bme280::RAW_TEMPERATURE_LEN + Bme280::RAW_HUMIDITY_LEN] = { 0 };

  Bme280 bme280; 
  /*
  bme280.readRawPressure(measurement);
  bme280.readRawTemperature(measurement + Bme280::RAW_PRESSURE_LEN);
  bme280.readRawHumidity(measurement + Bme280::RAW_PRESSURE_LEN + Bme280::RAW_TEMPERATURE_LEN);
  */
  bme280.readRawData(measurement);
  Serial.print("BME280 Raw Data:\t");  
  aux::printArray(measurement, sizeof(measurement));
}
