/**
 * Author:  Elias Vestli
 * Project: Torque Estimation for Robotic Actuators Using Gearbox Torsional Compliance — Master's Thesis
 * Content: Arduino sketch that reads the AksIM-2 absolute magnetic encoder via SPI
 *          and streams position, angle, status, CRC, and raw bytes over USB serial
 *          as a CSV line at ~100 Hz.
 */

#include <SPI.h>

const int CS_PIN = 10;
const int FRAME_BYTES = 4;

SPISettings aksimSettings(1000000, MSBFIRST, SPI_MODE1);

/**
 * @brief Initialise serial port and SPI, then print the CSV header.
 */
void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  Serial.println("# AksIM-2 reader v3");
  Serial.println("# format: position,degrees,error,warning,crc,raw0,raw1,raw2,raw3");
}

/**
 * @brief Read one 4-byte SPI frame, decode position and status bits, and
 *        print a CSV line to Serial.
 */
void loop() {
  uint8_t rx[FRAME_BYTES];

  SPI.beginTransaction(aksimSettings);
  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(6);

  for (int i = 0; i < FRAME_BYTES; i++) {
    rx[i] = SPI.transfer(0x00);
  }

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();

  uint32_t raw =
      ((uint32_t)rx[0] << 24) |
      ((uint32_t)rx[1] << 16) |
      ((uint32_t)rx[2] << 8)  |
       (uint32_t)rx[3];

  uint32_t position = (raw >> 15) & 0x1FFFF;

  uint8_t err  = (((raw >> 9) & 0x01) == 0) ? 1 : 0;
  uint8_t warn = (((raw >> 8) & 0x01) == 0) ? 1 : 0;

  uint8_t crc = raw & 0xFF;

  float degrees = position * (360.0f / 131072.0f);

  Serial.print(position);
  Serial.print(',');
  Serial.print(degrees, 4);
  Serial.print(',');
  Serial.print(err);
  Serial.print(',');
  Serial.print(warn);
  Serial.print(',');
  Serial.print(crc);

  for (int i = 0; i < FRAME_BYTES; i++) {
    Serial.print(',');
    Serial.print(rx[i]);
  }

  Serial.println();

  delay(10);
}
