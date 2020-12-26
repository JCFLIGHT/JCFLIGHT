#ifndef GLITCH_H_
#define GLITCH_H_
#include "Arduino.h"
class GlitchClass
{
public:
    bool CheckGPS(void);
    bool CheckBarometer(void);
    bool CheckCompass(void);
};
extern GlitchClass GLITCH;
#endif