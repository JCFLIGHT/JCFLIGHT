#include "BLACKBOX.h"

struct Struct_PinMap
{
  volatile uint8_t *DDR;
  volatile uint8_t *PIN;
  volatile uint8_t *PORT;
  uint8_t BITSHIFTH;
};

static const Struct_PinMap DigitalPinMap[] = {
    {&DDRE, &PINE, &PORTE, 0}, //E0  0
    {&DDRE, &PINE, &PORTE, 1}, //E1  1
    {&DDRE, &PINE, &PORTE, 4}, //E4  2
    {&DDRE, &PINE, &PORTE, 5}, //E5  3
    {&DDRG, &PING, &PORTG, 5}, //G5  4
    {&DDRE, &PINE, &PORTE, 3}, //E3  5
    {&DDRH, &PINH, &PORTH, 3}, //H3  6
    {&DDRH, &PINH, &PORTH, 4}, //H4  7
    {&DDRH, &PINH, &PORTH, 5}, //H5  8
    {&DDRH, &PINH, &PORTH, 6}, //H6  9
    {&DDRB, &PINB, &PORTB, 4}, //B4 10
    {&DDRB, &PINB, &PORTB, 5}, //B5 11
    {&DDRB, &PINB, &PORTB, 6}, //B6 12
    {&DDRB, &PINB, &PORTB, 7}, //B7 13
    {&DDRJ, &PINJ, &PORTJ, 1}, //J1 14
    {&DDRJ, &PINJ, &PORTJ, 0}, //J0 15
    {&DDRH, &PINH, &PORTH, 1}, //H1 16
    {&DDRH, &PINH, &PORTH, 0}, //H0 17
    {&DDRD, &PIND, &PORTD, 3}, //D3 18
    {&DDRD, &PIND, &PORTD, 2}, //D2 19
    {&DDRD, &PIND, &PORTD, 1}, //D1 20
    {&DDRD, &PIND, &PORTD, 0}, //D0 21
    /********PINOS ANALOGICOS**********/
    {&DDRA, &PINA, &PORTA, 0}, //A0 22
    {&DDRA, &PINA, &PORTA, 1}, //A1 23
    {&DDRA, &PINA, &PORTA, 2}, //A2 24
    {&DDRA, &PINA, &PORTA, 3}, //A3 25
    {&DDRA, &PINA, &PORTA, 4}, //A4 26
    {&DDRA, &PINA, &PORTA, 5}, //A5 27
    {&DDRA, &PINA, &PORTA, 6}, //A6 28
    {&DDRA, &PINA, &PORTA, 7}, //A7 29
    /**********************************/
    {&DDRC, &PINC, &PORTC, 7}, //C7 30
    {&DDRC, &PINC, &PORTC, 6}, //C6 31
    {&DDRC, &PINC, &PORTC, 5}, //C5 32
    {&DDRC, &PINC, &PORTC, 4}, //C4 33
    {&DDRC, &PINC, &PORTC, 3}, //C3 34
    {&DDRC, &PINC, &PORTC, 2}, //C2 35
    {&DDRC, &PINC, &PORTC, 1}, //C1 36
    {&DDRC, &PINC, &PORTC, 0}, //C0 37
    {&DDRD, &PIND, &PORTD, 7}, //D7 38
    {&DDRG, &PING, &PORTG, 2}, //G2 39
    {&DDRG, &PING, &PORTG, 1}, //G1 40
    {&DDRG, &PING, &PORTG, 0}, //G0 41
    {&DDRL, &PINL, &PORTL, 7}, //L7 42
    {&DDRL, &PINL, &PORTL, 6}, //L6 43
    {&DDRL, &PINL, &PORTL, 5}, //L5 44
    {&DDRL, &PINL, &PORTL, 4}, //L4 45
    {&DDRL, &PINL, &PORTL, 3}, //L3 46
    {&DDRL, &PINL, &PORTL, 2}, //L2 47
    {&DDRL, &PINL, &PORTL, 1}, //L1 48
    {&DDRL, &PINL, &PORTL, 0}, //L0 49
    {&DDRB, &PINB, &PORTB, 3}, //B3 50
    {&DDRB, &PINB, &PORTB, 2}, //B2 51
    {&DDRB, &PINB, &PORTB, 1}, //B1 52
    {&DDRB, &PINB, &PORTB, 0}, //B0 53
    {&DDRF, &PINF, &PORTF, 0}, //F0 54
    {&DDRF, &PINF, &PORTF, 1}, //F1 55
    {&DDRF, &PINF, &PORTF, 2}, //F2 56
    {&DDRF, &PINF, &PORTF, 3}, //F3 57
    {&DDRF, &PINF, &PORTF, 4}, //F4 58
    {&DDRF, &PINF, &PORTF, 5}, //F5 59
    {&DDRF, &PINF, &PORTF, 6}, //F6 60
    {&DDRF, &PINF, &PORTF, 7}, //F7 61
    {&DDRK, &PINK, &PORTK, 0}, //K0 62
    {&DDRK, &PINK, &PORTK, 1}, //K1 63
    {&DDRK, &PINK, &PORTK, 2}, //K2 64
    {&DDRK, &PINK, &PORTK, 3}, //K3 65
    {&DDRK, &PINK, &PORTK, 4}, //K4 66
    {&DDRK, &PINK, &PORTK, 5}, //K5 67
    {&DDRK, &PINK, &PORTK, 6}, //K6 68
    {&DDRK, &PINK, &PORTK, 7}  //K7 69
};

static inline __attribute__((always_inline)) void SetPinMode(uint8_t Pin)
{
  if (__builtin_constant_p(Pin) && Pin < sizeof(DigitalPinMap) / sizeof(Struct_PinMap))
  {
    *DigitalPinMap[Pin].DDR |= 1 << DigitalPinMap[Pin].BITSHIFTH;
  }
}

void BlackBox::Init()
{

  _SPI3_SCK = SCK;
  _SPI3_MOSI = MOSI;
  _SPI3_MISO = MISO;
  CHIP_SELECT = ChipSelect;

  SetPinMode(ChipSelect);

  if (!SD.begin(ChipSelect))
  {
    return;
  }

  String DirectoryName = BlackBoxName;

  for (int i = 0; i <= 500; i++)
  {
    String DirectoryNamed = DirectoryName;
    DirectoryNamed.concat(i);
    char CharDirection[DirectoryNamed.length() + 1];
    DirectoryNamed.toCharArray(CharDirection, sizeof(CharDirection));
    if (!SD.exists(CharDirection))
    {
      if (SD.mkdir(CharDirection))
      {
        Directory = String(CharDirection);
        break;
      }
    }
  }
}

void BlackBox::IMU(int16_t AngleRoll, int16_t AnglePitch, int16_t Heading)
{

  String DirectoryCreate(Directory);
  DirectoryCreate.concat("/ATTITUDE.csv");
  char CharDirection[DirectoryCreate.length() + 1];
  DirectoryCreate.toCharArray(CharDirection, sizeof(CharDirection));

  File DataFileWrite = SD.open(CharDirection, FILE_WRITE);

  DataFileWrite.print(AngleRoll);

  DataFileWrite.print(Separator);
  DataFileWrite.print(AnglePitch);

  DataFileWrite.print(Separator);
  DataFileWrite.println(Heading);

  DataFileWrite.close();
}

void BlackBox::RC(uint16_t Throttle_CH, uint16_t Yaw_CH, uint16_t Pitch_CH, uint16_t Roll_CH)
{

  String DirectoryCreate(Directory);
  DirectoryCreate.concat("/RC.csv");
  char CharDirection[DirectoryCreate.length() + 1];
  DirectoryCreate.toCharArray(CharDirection, sizeof(CharDirection));

  File DataFileWrite = SD.open(CharDirection, FILE_WRITE);

  DataFileWrite.print(Throttle_CH);

  DataFileWrite.print(Separator);
  DataFileWrite.print(Yaw_CH);

  DataFileWrite.print(Separator);
  DataFileWrite.print(Pitch_CH);

  DataFileWrite.print(Separator);
  DataFileWrite.println(Roll_CH);

  DataFileWrite.close();
}

void BlackBox::GPS(int32_t Latitude, int32_t Longitude, uint8_t GPS_Num_Sat)
{

  String DirectoryCreate(Directory);
  DirectoryCreate.concat("/GPS.csv");
  char CharDirection[DirectoryCreate.length() + 1];
  DirectoryCreate.toCharArray(CharDirection, sizeof(CharDirection));

  File DataFileWrite = SD.open(CharDirection, FILE_WRITE);

  DataFileWrite.print(GPS_Num_Sat);

  DataFileWrite.print(Separator);
  DataFileWrite.print(Latitude);

  DataFileWrite.print(Separator);
  DataFileWrite.println(Longitude);

  DataFileWrite.close();
}
