#include <SPI.h>

const int CS_PIN = 10;
const int FRAME_BYTES = 4;

// AksIM-2 SPI / EncoLink: CPOL=0, CPHA=1
SPISettings aksimSettings(1000000, MSBFIRST, SPI_MODE1);

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();

  Serial.println("# AksIM-2 reader v3");
  Serial.println("# format: position,degrees,error,warning,crc,raw0,raw1,raw2,raw3");
}

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

  // Pack 4 received bytes into a 32-bit word
  uint32_t raw =
      ((uint32_t)rx[0] << 24) |
      ((uint32_t)rx[1] << 16) |
      ((uint32_t)rx[2] << 8)  |
       (uint32_t)rx[3];

  /*
    17-bit single-turn decode:
    raw[31:15] = position (17 bits)
    raw[9]     = error bit   (active low)
    raw[8]     = warning bit (active low)
    raw[7:0]   = CRC byte
  */

  uint32_t position = (raw >> 15) & 0x1FFFF;

  // Active-low status bits
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

  delay(10);  // ~100 Hz
}
