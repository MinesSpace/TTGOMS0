#include <Arduino.h>
#include <axp20x.h>
#include <RadioLib.h>
#include <SparkFun_Ublox_Arduino_Library.h>
#include <MicroNMEA.h>
#include <RadioLib.h>

#include "TTGOMS0.h"

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
    int state = radio.begin(868.5);
    //int state = radio.begin(868.5, 125, 12, 7, SX127X_SYNC_WORD, 10, 8, 0);
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
    myGPS.checkUblox(); //See if new dataByte is available. Process bytes as they come in.

    AllData dataVar;

    //GPSTakeData

    if (nmea.isValid() == true) {

        dataVar.gpsVar.latitude_mdeg = nmea.getLatitude();
        dataVar.gpsVar.longitude_mdeg = nmea.getLongitude();
        dataVar.gpsVar.hourGPS = nmea.getHour();
        dataVar.gpsVar.minuteGPS = nmea.getMinute();
        dataVar.gpsVar.secondGPS = nmea.getSecond();
        dataVar.gpsVar.gpsRun = 1;

        nmea.getAltitude(dataVar.gpsVar.altitulde_mm);

        char time_Byte[11] = {0};
        sprintf(time_Byte, "%i:%i:%i", dataVar.gpsVar.hourGPS, dataVar.gpsVar.minuteGPS, dataVar.gpsVar.secondGPS);

        Serial.print("Gps Time (H:M:S): ");
        Serial.println(time_Byte);
        Serial.print("Latitude (deg): ");
        Serial.println(dataVar.gpsVar.latitude_mdeg / 1000000., 6);
        Serial.print("Longitude (deg): ");
        Serial.println(dataVar.gpsVar.longitude_mdeg / 1000000., 6);
        Serial.print("Altitude (m): ");
        Serial.println(dataVar.gpsVar.altitulde_mm / 1000., 4);

    } else {

        dataVar.gpsVar.gpsRun = 0;

        Serial.print("No Fix - ");
        Serial.print("Num. satellites: ");
        Serial.println(nmea.getNumSatellites());
    }

    //Send Telemeausre

    Serial.print(F("[SX1276] Transmitting packet ... "));

    byte dataByte[12];
    
    int state = radio.transmit(dataByte, 12);

    delay(250); //Don't pound too hard on the I2C bus

    if (state == ERR_NONE) {
        // the packet was successfully transmitted
        Serial.println(F(" success!"));
        Serial.print("Altitude (m): ");

        // print measured dataByte rate
        Serial.print(F("[SX1276] dataRate:\t"));
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
    delay(100);
  
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

bool initPMU()
{
    Wire.begin(I2C_SDA, I2C_SCL);

    if (PMU.begin(Wire, AXP192_SLAVE_ADDRESS) == AXP_FAIL) {
        return false;
    }
    /*
     * The charging indicator can be turned on or off
     * * * */
    // PMU.setChgLEDMode(LED_BLINK_4HZ);

    /*
    * The default ESP32 power supply has been turned on,
    * no need to set, please do not set it, if it is turned off,
    * it will not be able to program
    *
    *   PMU.setDCDC1Voltage(3300);
    *   PMU.setPowerOutPut(AXP192_DCDC1, AXP202_ON);
    *
    * * * */

    /*
     *   Turn off unused power sources to save power
     * **/
    PMU.setPowerOutPut(AXP192_DCDC2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO2, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_OFF);
    PMU.setPowerOutPut(AXP192_EXTEN, AXP202_OFF);

    /*
     * Set the power of LoRa and GPS module to 3.3V
     **/
    PMU.setLDO2Voltage(3300);   //LoRa VDD
    PMU.setLDO3Voltage(3300);   //GPS  VDD

    PMU.setPowerOutPut(AXP192_LDO2, AXP202_ON);
    PMU.setPowerOutPut(AXP192_LDO3, AXP202_ON);

    return true;
}

void initBoard()
{
    Serial.begin(115200);
    Serial.println("initBoard");
    Serial1.begin(GPS_BAND_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    SPI.begin(RADIO_SCLK_PIN, RADIO_MISO_PIN, RADIO_MOSI_PIN, RADIO_CS_PIN);
    initPMU();
}

void dataForm(byte * dataByte, const AllData data)
{
    dataByte[0] = (int)((data.gpsVar.latitude_mdeg >> 24) & 0xFF);
    dataByte[1] = (int)((data.gpsVar.latitude_mdeg >> 16) & 0xFF) ;
    dataByte[2] = (int)((data.gpsVar.latitude_mdeg >> 8) & 0XFF);
    dataByte[3] = (int)((data.gpsVar.latitude_mdeg & 0XFF));

    dataByte[4] = (int)((data.gpsVar.longitude_mdeg >> 24) & 0xFF);
    dataByte[5] = (int)((data.gpsVar.longitude_mdeg >> 16) & 0xFF) ;
    dataByte[6] = (int)((data.gpsVar.longitude_mdeg>> 8) & 0XFF);
    dataByte[7] = (int)((data.gpsVar.longitude_mdeg & 0XFF));

    dataByte[8] = (int)((data.gpsVar.altitulde_mm >> 24) & 0xFF);
    dataByte[9] = (int)((data.gpsVar.altitulde_mm>> 16) & 0xFF) ;
    dataByte[10] = (int)((data.gpsVar.altitulde_mm>> 8) & 0XFF);
    dataByte[11] = (int)((data.gpsVar.altitulde_mm & 0XFF));
}