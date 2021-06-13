#include <Arduino.h>
#include <axp20x.h>
#include <RadioLib.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <MicroNMEA.h>

#include "TTGOMS0.h"

SFE_UBLOX_GPS myGPS;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void setup() {

    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    Serial.println("SparkFun Ublox Example");

    if (myGPS.begin(Serial1) == false) {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }
  
}

void loop() {
      myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

    if (nmea.isValid() == true) {
        long latitude_mdeg = nmea.getLatitude();
        long longitude_mdeg = nmea.getLongitude();
        
        long altitulde_mm;
        nmea.getAltitude(altitulde_mm);

        byte latitudeByte[4];
        byte longitudeByte[4];
        byte altituldeByte[4];

        latitudeByte[0] = (int)((latitude_mdeg >> 24) & 0xFF);
        latitudeByte[1] = (int)((latitude_mdeg >> 16) & 0xFF) ;
        latitudeByte[2] = (int)((latitude_mdeg>> 8) & 0XFF);
        latitudeByte[3] = (int)((latitude_mdeg & 0XFF));

        longitudeByte[0] = (int)((longitude_mdeg >> 24) & 0xFF);
        longitudeByte[1] = (int)((longitude_mdeg >> 16) & 0xFF) ;
        longitudeByte[2] = (int)((longitude_mdeg>> 8) & 0XFF);
        longitudeByte[3] = (int)((longitude_mdeg & 0XFF));

        altituldeByte[0] = (int)((altitulde_mm >> 24) & 0xFF);
        altituldeByte[1] = (int)((altitulde_mm>> 16) & 0xFF) ;
        altituldeByte[2] = (int)((altitulde_mm>> 8) & 0XFF);
        altituldeByte[3] = (int)((altitulde_mm & 0XFF));

        

        Serial.print("Latitude (deg): ");
        Serial.println(latitude_mdeg / 1000000., 6);
        Serial.print("Longitude (deg): ");
        Serial.println(longitude_mdeg / 1000000., 6);
        Serial.print("Altitude (m): ");
        Serial.println(altitulde_mm / 1000., 4);


    } else {
        Serial.print("No Fix - ");
        Serial.print("Num. satellites: ");
        Serial.println(nmea.getNumSatellites());
    }

    delay(250); //Don't pound too hard on the I2C bus
  
}

//This function gets called from the SparkFun Ublox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
    //Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
    //for sentence cracking
    nmea.process(incoming);
}
