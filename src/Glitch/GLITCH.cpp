#include "GLITCH.h"
#include "Common/VARIABLES.h"

GlitchClass GLITCH;

bool GlitchClass::CheckGPS(void)
{
    if (!GPS_3DFIX && GPS_NumberOfSatellites < 5)
    {
        return false;
    }
    return true;
}

bool GlitchClass::CheckBarometer(void)
{
    return true;
}

bool GlitchClass::CheckCompass(void)
{
    return true;
}