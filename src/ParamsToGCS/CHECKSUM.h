#ifndef CHECKSUM_H_
#define CHECKSUM_H_
#include "Arduino.h"
class CheckSumClass
{
public:
    uint16_t GetFailSafeValue;
    uint8_t GetDevicesActived();
    void UpdateServosReverse();
    void UpdateChannelsReverse();
};
extern CheckSumClass CHECKSUM;
#endif