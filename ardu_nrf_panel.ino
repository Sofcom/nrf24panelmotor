// -------------------------
// ----- П У Л Ь Т (TX) ----
// -------------------------

// Подключаемые библиотеки
#include <SPI.h>                      // Подключаем библиотеку для работы с шиной SPI
#include <SerialFlow.h>                         // библиотека пакетной передачи данных
//#include "LowPower.h"                 // Библиотека сна

SerialFlow nrf24(9, 10);                        // пины модуля nrf24

// Для отладки программы
#include <SoftwareSerial.h>

// Джойстик: Пины для подключения
const uint8_t           pinX = A2; // pinX номер вывода Arduino к которому подключён вывод X джойстика (аналоговый).
const uint8_t           pinY = A1; // pinY номер вывода Arduino к которому подключён вывод Y джойстика (аналоговый).

// Джойстик: Задаём люфт для центра джойстика:
const uint16_t          gapX = 15;
const uint16_t          gapY = 15;

// Джойстик: переменные и массивы:
uint16_t                axisX, centerX; // axisX положения джойстика по оси X и centerX для центра по оси X.
uint16_t                axisY, centerY; // axisY положения джойстика по оси Y и centerY для центра по оси Y.
const uint8_t           HOLDTIME_NRF = 100; // 100 мс
//const uint8_t           HOLDTIME_JOY = 50;  // 50 мс

unsigned long tm, data_next;       // переменные для таймера

//int stb_time = 100000;   // время бездействия через которое пульт уйдёт в сон

// Джойстик: массив для передачи данных, координаты XY:
int8_t nrfData[2];

void setup() {

  // Джойстик: Устанавливаем режимы работы выводов:
  pinMode(pinX, INPUT);
  pinMode(pinY, INPUT);

  // Джойстик: Определяем центральные положения по осям X, Y:
  centerX = analogRead(pinX);
  centerY = analogRead(pinY);

  //  Для отладки - функция Serial
  Serial.begin(57600);            // скорость порта

  //NRF: Настройка радиоканала
  nrf24.setPacketFormat(1, 2);    // задаём формат пакетов
  nrf24.begin(0xF7F0F6F3E1LL,0xF7F0F6F3D2LL); // адреса 1 приемника 2 передатчика

}

void loop() {
  
  // Джойстик: Считываем показания:
  axisX = analogRead(pinX);
  axisY = analogRead(pinY);
  
  // Джойстик: Преобразуем считанные показания и добавляем люфт: 
  if (axisX < centerX - gapX) {
    axisX = map(axisX, centerX - gapX,   0, 0, -100);
  } else // Преобразуем значения axisX от диапазона centerX-люфт...0    к диапазону 0...-100
    if (axisX > centerX + gapX) {
      axisX = map(axisX, centerX + gapX, 1023, 0, 100);
    } else // Преобразуем значения axisX от диапазона centerX+люфт...1023 к диапазону 0...+100
    {
      axisX = 0; // Оставшиеся  значения centerX+-люфт преобразуем в 0
    }
  if (axisY < centerY - gapY) {
    axisY = map(axisY, centerY - gapY,   0, 0, -100);
  } else // Преобразуем значения axisY от диапазона centerY-люфт...0    к диапазону 0...-100
    if (axisY > centerY + gapY) {
      axisY = map(axisY, centerY + gapY, 1023, 0, 100);
    } else // Преобразуем значения axisY от диапазона centerY+люфт...1023 к диапазону 0...+100
    {
      axisY = 0;
    }
    
  // Джойстик: Сохраняем полученные значения в массив:
  nrfData[0] = int8_t(axisX);                           // Положение джойстика по оси X в диапазоне от -100 до +100 с люфтом
  nrfData[1] = int8_t(axisY);                           // Положение джойстика по оси Y в диапазоне от -100 до +100 с люфтом  

  nrfDataSend();


  // Выводим содержимое nrfData в Serial
  Serial.print(nrfData[0]);  Serial.print("  ");  Serial.print(nrfData[1]);  Serial.println();
  delay(50);
}

//=========
// ФУНКЦИИ
//=========

// -- ДЖОЙСТИК --

//void getJoysticXY() {
//
//  uint8_t nrfData[2];
//  // Джойстик: Считываем показания:
//  axisX = analogRead(pinX);
//  axisY = analogRead(pinY);
//  
//  // Джойстик: Преобразуем считанные показания и добавляем люфт: 
//  if (axisX < centerX - gapX) {
//    axisX = map(axisX, centerX - gapX,   0, 0, -100);
//  } else // Преобразуем значения axisX от диапазона centerX-люфт...0    к диапазону 0...-100
//    if (axisX > centerX + gapX) {
//      axisX = map(axisX, centerX + gapX, 1023, 0, 100);
//    } else // Преобразуем значения axisX от диапазона centerX+люфт...1023 к диапазону 0...+100
//    {
//      axisX = 0; // Оставшиеся  значения centerX+-люфт преобразуем в 0
//    }
//  if (axisY < centerY - gapY) {
//    axisY = map(axisY, centerY - gapY,   0, 0, -100);
//  } else // Преобразуем значения axisY от диапазона centerY-люфт...0    к диапазону 0...-100
//    if (axisY > centerY + gapY) {
//      axisY = map(axisY, centerY + gapY, 1023, 0, 100);
//    } else // Преобразуем значения axisY от диапазона centerY+люфт...1023 к диапазону 0...+100
//    {
//      axisY = 0;
//    }
//    
//  // Джойстик: Сохраняем полученные значения в массив:
//  nrfData[0] = uint8_t(axisX);                           // Положение джойстика по оси X в диапазоне от -100 до +100 с люфтом
//  nrfData[1] = uint8_t(axisY);                           // Положение джойстика по оси Y в диапазоне от -100 до +100 с люфтом
//  
//}

void nrfDataSend() {

    
     tm = millis();                  // получаем текущие мс
    if( tm > data_next ){           // если tm больше чем data_next
        data_next = tm + HOLDTIME_NRF;   // data_next это текущие мс + 100мс
        nrf24.setPacketValue( nrfData[0] ); // запечатываем в пакет значение 1
        nrf24.setPacketValue( nrfData[1] ); // запечатываем в пакет значение 2
        nrf24.sendPacket();         // отправляем пакет в радио-эфир 10 раз в сек
    }
}

// функция чтения внутреннего опорного напряжения, универсальная (для всех ардуин)
//long readVcc() { 
//#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
//  ADMUX = _BV(MUX5) | _BV(MUX0);
//#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
//  ADMUX = _BV(MUX3) | _BV(MUX2);
//#else
//  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
//#endif
//  delay(2); // Wait for Vref to settle
//  ADCSRA |= _BV(ADSC); // Start conversion
//  while (bit_is_set(ADCSRA, ADSC)); // measuring
//  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
//  uint8_t high = ADCH; // unlocks both
//  long result = (high << 8) | low;
//
//  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
//  return result; // возвращает VCC
//}

// Функция определения конца периода времени с учётом переполнения
// аппаратного счётчика (аппаратный счётчик микросекунд считает до 4294967295
// а потом сбрасывается в 0).
//bool isTimer(unsigned long startTime, unsigned long period )
//{
//   unsigned long currentTime;
//   currentTime = millis();
//   if (currentTime >= startTime) return (currentTime >= (startTime + period));
//   else return (currentTime >= (4294967295 - startTime + period));
//}


//TODO:
//1) Повесить джойстик на прерывание
//    Засыпание: если джойстик пусто - вкл прерывания и спать
//      если джойстик не пусто - просыпаемся и выкл прерывания
//2) Используем только 0, 1, 2 для управления движками
//3) Если передаём дынные -> моргаем светодиодом (облом, пин 13 занят под nrf)

