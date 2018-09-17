#include <CANDude.h>
#include <SPI.h>

CANDude controller(9);

uint8_t buf[256];

void setup() {
  delay(5000);
  Serial.begin(115200);
  Serial.println("C'est parti");

  /* Teste la configuration des filtres */

  CANDudeFilters filters;

  filters.setMask(0, 0x3FF);        /* Buffer 0 mask */
  filters.setFilter(0, false, 0x20);
  //filters.setMask(1, 0x1FFFFFFF);   /* Buffer 1 mask */
  filters.setFilter(2, true, 0x1FFFFF00);
  filters.print();
  filters.finalize();
  filters.print();

  /* Démarre le controlleur CAN */
  if (controller.begin(CANDudeSettings(mcp2515::CRYSTAL_16MHZ, 250000), filters) == CANDudeOk) {
    Serial.println("Connexion Ok");
    controller.dumpRegisters();
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
