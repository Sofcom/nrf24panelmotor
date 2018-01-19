// этот скетч грузим в Мегу
// в мониторе порта ставим скорость 57600

#include <SerialFlow.h>                         // библиотека пакетной передачи данны
SerialFlow nrf24(9, 10);                        // пины модуля nrf24

int8_t nrfDataX, nrfDataY;

void setup(void){
    Serial.begin(57600);                        // скорость порта
    nrf24.setPacketFormat(1, 2);                // задаём формат пакетов
    nrf24.begin(0xF7F0F6F3D2LL,0xF7F0F6F3E1LL); // адреса 1 приемника 2 передатчика
}
void loop(void){
//    unsigned int v;
      int8_t nrfDataX, nrfDataY;
      
    if( nrf24.receivePacket() ){      // если получили пакет
        nrfDataX = nrf24.getPacketValue(0);  // извлечь данные из пакета 1
        nrfDataY = nrf24.getPacketValue(1);  // извлечь данные из пакета 2

        // распечатать полученные данные в порт
        Serial.print(nrfDataX);  Serial.print("  ");  Serial.print(nrfDataY);  Serial.println();
        
    }
}
