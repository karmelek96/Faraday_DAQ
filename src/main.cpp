#include <Arduino.h>
#include <SPI.h>
#include <SPIFlash.h>
#include "Arduino_AD7190.h"
#include "DataPoint.h"

#define FLASH_CS 26
#define THRUST_CS 25

#define LOADCELL_VREF 5.0 //V
#define LOADCELL_FULL_SCALE_OUTPUT 0.095 //V
#define LOADCELL_GAIN 32.0
#define LOADCELL_RANGE 8896.443 //N

#define PTAP_VREF 5 //V
#define PTAP_FULL_SCALE_OUTPUT 4.5 //V
#define PTAP_GAIN 1.0
#define PTAP_RANGE 100 //Bar

enum TestState {
    STANDBY,
    AUTOSTART,
    BURN,
    DATA_TRANSFER,
    END
};

SPIFlash flash(FLASH_CS);
AD7190 thrustADC(THRUST_CS);

TestState state = STANDBY;

uint32_t startTime = 0;
uint32_t prevAddress = 0;

double getValue(uint32_t _rawADCReading, double _vcc, double _adcGain, double _fullScaleOutput, double _range) {
    double voltage = _rawADCReading * (_vcc / ((double)pow(2, 24) - 1));
    double value = voltage * (_range / (_fullScaleOutput * _adcGain));
    return value;
}

void setup(){
    Serial.begin(115200);
    SPI.begin();
    flash.begin();
    
    //Setup loadcell
    if(thrustADC.init()){
        Serial.println("AD7190 Initialized");
    }
    else{
        Serial.println("AD7190 Error");
    }
    delay(1000);
    thrustADC.calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M);
    //thrustADC.calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AINCOM);
    thrustADC.rangeSetup(1, AD7190_CONF_GAIN_32);
    thrustADC.continuousReadAvg(1);
}

void loop(){
    //loop through states
    switch(state){
        case STANDBY:
            //wait for autostart signal
            DataPoint dataPoint;
            dataPoint.data.time = millis();
            dataPoint.data.state = state;
            dataPoint.data.thrust = getValue(thrustADC.continuousSingleRead(), LOADCELL_VREF, LOADCELL_GAIN, LOADCELL_FULL_SCALE_OUTPUT, LOADCELL_RANGE);
            //dataPoint.data.pressure = getValue(thrustADC.continuousSingleRead(), PTAP_VREF, PTAP_GAIN, PTAP_FULL_SCALE_OUTPUT, PTAP_RANGE);
            Serial.println(dataPoint.data.thrust);
            break;
        case AUTOSTART:
            //start the test
            state = BURN;
            break;
        case BURN:
            //read data from loadcell
            break;
        case DATA_TRANSFER:
            //transfer data to internal flash
            break;
        case END:
            //end the test
            break;
    }
}
