/*
   Este arquivo faz parte da JCFLIGHT.

   JCFLIGHT é um software livre: você pode redistribuí-lo e/ou modificar
   sob os termos da GNU General Public License conforme publicada por
   a Free Software Foundation, seja a versão 3 da Licença, ou
   (à sua escolha) qualquer versão posterior.

  JCFLIGHT é distribuído na esperança de ser útil,
  mas SEM QUALQUER GARANTIA; sem mesmo a garantia implícita de
  COMERCIALIZAÇÃO ou ADEQUAÇÃO A UM DETERMINADO FIM. Veja o
  GNU General Public License para mais detalhes.

   Você deve ter recebido uma cópia da Licença Pública Geral GNU
  junto com a JCFLIGHT. Caso contrário, consulte <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include "DJINAZAGPS.h"

void setup()
{
    Serial.begin(115200);  //BAUD-RATE DO SERIAL MONITOR
    Serial1.begin(115200); //BAUD-RATE DO GPS
    pinMode(13, OUTPUT);
}

void loop()
{
    //PARA O MICROC ATMEGA 328:VOCÊ PODE SE USAR A SOFTWARE SERIAL PARA A LEITURA DO GPS,E ASSIM USAR A SERIAL PRINCIPAL PARA DEBUG
    int GPSSerialDataRead = Serial1.read(); //GPS CONECTADO NO TX1 E RX1 DO ATMEGA2560

    DjiNazaGpsNewFrame(GPSSerialDataRead);

    if (DJINaza_Num_Sat >= 5 && millis() % 1000 > 500)
    {
        digitalWrite(13, HIGH);
    }
    else
    {
        digitalWrite(13, LOW);
    }

    Serial.print(DJINaza_Num_Sat);
    Serial.print(" ");

    Serial.print(DJINaza_Latitude); //COORDENADA LATITUDE
    Serial.print(" ");

    Serial.print(DJINaza_Longitude); //COORDENADA LONGITUDE
    Serial.print(" ");

    Serial.print(DJINaza_Altitude); //ALTITUDE DADA PELO GPS EM CENTIMETROS
    Serial.print(" ");

    Serial.print(DJINaza_HDOP); //HDOP
    Serial.print(" ");

    Serial.print(DJINaza_Fix_State); //2D E 3D GPS FIX
    Serial.print(" ");

    Serial.print(DJINaza_Compass_Roll); //EIXO X DO MAG DO GPS
    Serial.print(" ");

    Serial.print(DJINaza_Compass_Pitch); //EIXO Y DO MAG DO GPS
    Serial.print(" ");

    Serial.print(DJINaza_Compass_Yaw); //EIXO Z DO MAG DO GPS
    Serial.print(" ");

    Serial.print(DJINaza_GroundCourse); //GROUND COURSE EM GRAUS
    Serial.print(" ");

    Serial.print(DJINaza_GroundSpeed); //GROUND SPEED EM CM/S
    Serial.print(" ");

    Serial.println();
}
