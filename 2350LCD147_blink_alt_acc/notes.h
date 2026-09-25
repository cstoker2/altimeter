/*
Pins:
SCL 5
SDA 4

SERVO_PIN 3
*/

/* 
have to add this to the library and call to disable lpf

//     @brief Sets the accelerometer's DLPF to 0.
    

void Adafruit_ICM20649::setAccelDLPF() {
  _setBank(2);

  Adafruit_BusIO_Register accel_config_1 = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, ICM20649_ACCEL_CONFIG_1);

  Adafruit_BusIO_RegisterBits accel_dlpf =
      Adafruit_BusIO_RegisterBits(&accel_config_1, 1, 0);

  accel_dlpf.write(0);
  _setBank(0);
}

encoder lib:
https://github.com/gbr1/rp2040-encoder-library

*/