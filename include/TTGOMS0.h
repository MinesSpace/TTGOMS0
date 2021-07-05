#include <SPI.h>

// Pin design

#define GPS_RX_PIN 34
#define GPS_TX_PIN 12
#define BUTTON_PIN 38
#define BUTTON_PIN_MASK GPIO_SEL_38

#define I2C_SDA                     21
#define I2C_SCL                     22
#define PMU_IRQ                     35

#define RADIO_SCLK_PIN               5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DI0_PIN               26
#define RADIO_RST_PIN               23
#define RADIO_DIO1_PIN              33
#define RADIO_BUSY_PIN              32

#define GPS_BAND_RATE      9600

#include <axp20x.h>
AXP20X_Class PMU;

// All struct 

typedef struct GpsData{

    long altitulde_mm = 0;
    long latitude_mdeg = 0;
    long longitude_mdeg = 0;

    uint8_t hourGPS = 0;
    uint8_t minuteGPS = 0;
    uint8_t secondGPS = 0;

    uint8_t gpsRun = 0;

}GpsData; 

typedef struct Bmp280{

    float temp = 0;
    float pres = 0;

    uint8_t bmp280Run = 0;

}Bmp280;

typedef struct Ds18b20{

    float tempMotor = 0;
    uint8_t ds18B20Run = 0; 

}Ds18b20;

typedef struct Ds3231S{

    uint8_t secondRTC;
    uint8_t minuteRTC;
    uint8_t hourRTC;

    uint8_t ds3231sRun = 0;

}Ds3231;

typedef struct Mpu6050{

    double gyroX = 0;
    double gyroY = 0;
    double gyroZ = 0;
    double AccX = 0;
    double AccY = 0;
    double AccZ = 0;

    uint8_t mpu6050Run = 0;
    
}Mpu6050;

typedef struct AllData{

    GpsData gpsVar;
    Bmp280 bmp280Var;
    Ds18b20 ds18b20Var;
    Ds3231 ds3231Var;

}AllData;

// Prototype

bool initPMU();
void initBoard();
void dataForm(byte * dataByte, const AllData data);