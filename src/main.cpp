#include <Arduino.h>
#include <axp20x.h>
#include <RadioLib.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <MicroNMEA.h>

#include "TTGOMS0.h"
#include <RadioLib.h>

SFE_UBLOX_GPS myGPS;
SX1276 radio = new Module(RADIO_CS_PIN, RADIO_DI0_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void setup() {

    initBoard();
    // When the power is turned on, a delay is required.
    delay(1500);

    
    if (myGPS.begin(Serial1) == false) {
        Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
        while (1);
    }

    Serial.print(F("[SX1276] Initializing ... "));
    int state = radio.begin(868.0);
    //int state = radio.begin(868.0, 125, 6, 7, SX127X_SYNC_WORD, 10, 8, 0);
    //state = radio.setCurrentLimit(60);
    //state = radio.setCRC(true);

    if (state == ERR_NONE) {
        Serial.println(F("success!"));
    } 
    else {
        Serial.print(F("failed, code "));
        Serial.println(state);
        while (true);
    }
    
  
}

void loop() {
      myGPS.checkUblox(); //See if new data is available. Process bytes as they come in.

        long altitulde_mm = 600000;
        long latitude_mdeg = 0;
        long longitude_mdeg = 0;

    if (nmea.isValid() == true) {
        latitude_mdeg = nmea.getLatitude();
        longitude_mdeg = nmea.getLongitude();
        
        altitulde_mm;
        nmea.getAltitude(altitulde_mm);
    

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

    Serial.print(F("[SX1276] Transmitting packet ... "));

    byte byteArr[4];

    byteArr[0] = (int)((latitude_mdeg >> 24) & 0xFF);
    byteArr[1] = (int)((latitude_mdeg >> 16) & 0xFF) ;
    byteArr[2] = (int)((latitude_mdeg>> 8) & 0XFF);
    byteArr[3] = (int)((latitude_mdeg & 0XFF));

    byteArr[4] = (int)((longitude_mdeg >> 24) & 0xFF);
    byteArr[5] = (int)((longitude_mdeg >> 16) & 0xFF) ;
    byteArr[6] = (int)((longitude_mdeg>> 8) & 0XFF);
    byteArr[7] = (int)((longitude_mdeg & 0XFF));

    byteArr[8] = (int)((altitulde_mm >> 24) & 0xFF);
    byteArr[9] = (int)((altitulde_mm>> 16) & 0xFF) ;
    byteArr[10] = (int)((altitulde_mm>> 8) & 0XFF);
    byteArr[11] = (int)((altitulde_mm & 0XFF));

    
    int state = radio.transmit(byteArr, 12);

    delay(250); //Don't pound too hard on the I2C bus

    if (state == ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));
        Serial.print("Altitude (m): ");

        // print measured data rate
        Serial.print(F("[SX1276] Datarate:\t"));
        Serial.print(radio.getDataRate());
        Serial.println(F(" bps"));
    }

    else if (state == ERR_PACKET_TOO_LONG) {
        // the supplied packet was longer than 256 bytes
        Serial.println(F("too long!"));

    } else if (state == ERR_TX_TIMEOUT) {
        // timeout occurred while transmitting packet
        Serial.println(F("timeout!"));

    } else {
        // some other error occurred
        Serial.print(F("failed, code "));
        Serial.println(state);

    }

    // wait for a second before transmitting again
    delay(1000);
  
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
