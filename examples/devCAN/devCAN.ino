#include <CANDude.h>
#include <SPI.h>

CANDudeController(controller, 9, 2)

uint8_t buf[256];

void setup() {
  Serial.begin(115200);
  Serial.println("C'est parti");
  if (controller.begin(CANDudeSettings(mcp2515::CRYSTAL_16MHZ, 250000)) == CANDudeOk) {
    Serial.println("Connexion Ok");
    controller.read(0, 256, buf);
    for (uint16_t low = 0 ; low < 16 ; low++) {
      for (uint16_t high = 0; high < 8 ; high++) {
        uint16_t i = high << 4 | low ;
        for (uint8_t b = 7; b > 0; b--) {
          if ((buf[i] & (1 << b)) == 0) Serial.print('0');
         else break;
        }
        Serial.print(buf[i], BIN);
        Serial.print(' ');
      }
      Serial.println();
    }
  }
  else {
    Serial.println("Echec");
  }
  Serial.println();

  uint8_t j = 0;
  for (uint16_t i = 0; i < 256 ; i++) {
    Serial.print(j);
    Serial.print(' ');
    uint8_t inSize = j;
    uint8_t mask = 0xFF;
    uint8_t size = inSize > 16 ? inSize - 1 : 15;
    while (! (size & 0x80)) {
      mask >>= 1;
      size <<= 1;
    }
    Serial.println(mask, BIN);
    j++;
  }
//  CANDudeQueue queue(8);
//  queue.print();
//  queue.pushByte(3);
//  queue.pushByte(4);
//  queue.pushByte(5);
//  queue.pushByte(6);
//  queue.pushByte(7);
//  queue.print();
//  Serial.println(queue.popByte());
//  Serial.println(queue.popByte());
//  queue.print();
//  queue.pushByte(8);
//  queue.pushByte(9);
//  queue.pushByte(10);
//  queue.pushByte(11);
//  queue.pushByte(12);
//  queue.pushByte(13);
//  queue.print();
//  Serial.println(queue.popByte());
//  Serial.println(queue.popByte());
//  Serial.println(queue.popByte());
//  Serial.println(queue.popByte());
//  Serial.println(queue.popByte());
//  Serial.println(queue.popByte());
//  queue.print();
//  queue.pushByte(3);
//  queue.pushByte(4);
//  queue.pushByte(5);
//  queue.pushByte(6);
//  queue.pushByte(7);
//  queue.print();
  uint32_t numberOfConfigs = 0;
  uint32_t dureeMax = 0;
  for (uint32_t baud = 1; baud <= 1000000; baud++) {
    uint32_t date = micros();
    CANDudeSettings settings(mcp2515::CRYSTAL_16MHZ, baud);
    uint32_t duree = micros() - date;
    if (duree > dureeMax) dureeMax = duree;
    if (settings.configOk()) {
      if (settings.PPMError() == 0) {
          settings.print();
          numberOfConfigs++;
      }
    }
  }
  Serial.print("Configs = ");
  Serial.println(numberOfConfigs);
  Serial.print("duree max=");
  Serial.println(dureeMax);
}

void loop() {
  // put your main code here, to run repeatedly:

}
