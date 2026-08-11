
#include "src/FlightManager/FlightManager.h"


FlightManager LEONIDAS;


void setup()
{
    LEONIDAS.begin();
}


void loop()
{
    LEONIDAS.update();
}