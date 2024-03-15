#include <Arduino.h>
#include <SPI.h>
#include <SPIFlash.h>
#include <SDFat.h>
#include "Arduino_AD7190.h"
#include "DataPoint.h"

#define CTRLSERIAL Serial

#define SD_CS 5
#define FLASH_CS 26
#define THRUST_CS 25
#define PRESSURE_CS 27

#define LOADCELL_VREF 5.0 //V
#define LOADCELL_FULL_SCALE_OUTPUT 0.095 //V
#define LOADCELL_GAIN 32.0
#define LOADCELL_RANGE 8896.443 //N
#define LOADCELL_OFFSET 0.0 //V

#define PTAP_VREF 5 //V
#define PTAP_FULL_SCALE_OUTPUT 4.5 //V
#define PTAP_GAIN 1.0
#define PTAP_RANGE 100 //Bar
#define PTAP_OFFSET 0.5 //V

enum TestState {
    STANDBY,
    AUTOSTART,
    BURN,
    DATA_TRANSFER,
    END
};
SdFat SD;
SPIFlash flash(FLASH_CS);
AD7190 thrustADC(THRUST_CS);
AD7190 pressureADC(PRESSURE_CS);

TestState state = STANDBY;

uint32_t startTime = 0;
uint32_t ignitionTime = 0;
uint32_t prevTelemetry = 0;

uint32_t prevAddress = 0;

uint8_t globalFlags = 0;

double getValue(uint32_t _rawADCReading, double _vcc, double _adcGain, double _fullScaleOutput, double _range, double _offset = 0) {
    double voltage = _rawADCReading * (_vcc / ((double)pow(2, 24) - 1));
    double value = (voltage - _offset) * (_range / (_fullScaleOutput * _adcGain));
    return value;
}

void setup(){
    Serial.begin(115200);
    Serial2.begin(57600);
    SPI.begin();
    Serial.println("SPI Initialized");
    flash.begin();
    SD.begin(SD_CS);
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE3));
    Serial.println("SPI Settings Set");
    
    //Setup loadcell
    if(thrustADC.init()){
        Serial.println("Loadcell ADC Initialized");
    }
    else{
        Serial.println("Loadcell ADC Error");
    }

    //Setup pressure sensor
    if(pressureADC.init()){
        Serial.println("Pressure ADC Initialized");
    }
    else{
        Serial.println("Pressure ADC Error");
    }

    delay(1000);
    thrustADC.calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AIN2M);
    thrustADC.calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AIN2M);
    thrustADC.rangeSetup(1, AD7190_CONF_GAIN_32);
    thrustADC.continuousReadAvg(1);

    pressureADC.calibrate(AD7190_MODE_CAL_INT_ZERO, AD7190_CH_AIN1P_AINCOM);
    pressureADC.calibrate(AD7190_MODE_CAL_INT_FULL, AD7190_CH_AIN1P_AINCOM);
    pressureADC.rangeSetup(1, AD7190_CONF_GAIN_1);
    pressureADC.continuousReadAvg(1);
}

void sendTelemetry(DataPoint _dataPoint) {
    if(millis() - prevTelemetry > 100) {
        _dataPoint.data.time = millis();
        DataPointCastrated _dataPointCastrated;
        _dataPointCastrated.data.time = _dataPoint.data.time;
        _dataPointCastrated.data.state = _dataPoint.data.state;
        _dataPointCastrated.data.flags = _dataPoint.data.flags;
        //_dataPointCastrated.data.thrust = (uint16_t)(_dataPoint.data.thrust);
        //_dataPointCastrated.data.pressure = (uint8_t)(_dataPoint.data.pressure * 10);
        //CTRLSERIAL.write(_dataPointCastrated.bytes, sizeof(_dataPointCastrated.data));
        CTRLSERIAL.print(_dataPoint.data.time);
        CTRLSERIAL.print(",");
        CTRLSERIAL.print(_dataPoint.data.state);
        CTRLSERIAL.print(",");
        CTRLSERIAL.print(_dataPoint.data.flags);
        CTRLSERIAL.print(",");
        CTRLSERIAL.print(_dataPoint.data.thrust);
        CTRLSERIAL.print(",");
        CTRLSERIAL.print(_dataPoint.data.pressure);
        CTRLSERIAL.print("\n");
        prevTelemetry = millis();
    }
}

void ignitionStart() {
    //digitalWrite HIGH
    globalFlags |= 0x01;
}

void ignitionStop() {
    //digitalWrite LOW
    globalFlags &= 0xFE;
}

void transferLog() {
  String newLine ="";
  const String fileName = "LOGFILE";
  uint8_t fileNumber = 1;
  uint32_t maxAddress = prevAddress;
  File logFile;
  prevAddress = 0;
  while(SD.exists(fileName + fileNumber + ".csv")) {
    fileNumber++;
    delayMicroseconds(200);
  }
  logFile = SD.open(fileName + fileNumber + ".csv", FILE_WRITE);
  logFile.println("time,state,flags,thrust,pressure");
  logFile.close();
  while(prevAddress<maxAddress) {
    logFile = SD.open(fileName + fileNumber + ".csv", FILE_WRITE);
    DataPoint temp;
    flash.readByteArray(prevAddress+1, temp.bytes, sizeof(temp.data));
    logFile.println(temp.data.toString());
    logFile.close();
    prevAddress += sizeof(temp.data);
  }
}

DataPoint dataPoint;

void loop(){
    while(false) {
        uint8_t command = CTRLSERIAL.read();
        switch(command) {
            case 'a':
                state = AUTOSTART;
                startTime = millis();
                break;
            case 's':
                state = DATA_TRANSFER;
                break;
                break;
            case 'z':
                //Tare the thing
                break;
        }
    }
    //loop through states
    switch(state){
        case STANDBY:
            dataPoint.data.time = millis();
            dataPoint.data.state = state;
            dataPoint.data.flags = globalFlags;
            dataPoint.data.thrust = getValue(thrustADC.continuousSingleRead(), LOADCELL_VREF, LOADCELL_GAIN, LOADCELL_FULL_SCALE_OUTPUT, LOADCELL_RANGE);
            dataPoint.data.pressure = getValue(pressureADC.continuousSingleRead(), PTAP_VREF, PTAP_GAIN, PTAP_FULL_SCALE_OUTPUT, PTAP_RANGE, PTAP_OFFSET);
            sendTelemetry(dataPoint);
            break;
        case AUTOSTART:
            dataPoint.data.time = millis() - startTime;
            dataPoint.data.state = state;
            dataPoint.data.flags = globalFlags;
            dataPoint.data.thrust = getValue(thrustADC.continuousSingleRead(), LOADCELL_VREF, LOADCELL_GAIN, LOADCELL_FULL_SCALE_OUTPUT, LOADCELL_RANGE);
            dataPoint.data.pressure = getValue(pressureADC.continuousSingleRead(), PTAP_VREF, PTAP_GAIN, PTAP_FULL_SCALE_OUTPUT, PTAP_RANGE, PTAP_OFFSET);
            sendTelemetry(dataPoint);
            flash.writeByteArray(prevAddress, dataPoint.bytes, sizeof(dataPoint.data));
            prevAddress += sizeof(dataPoint.data);
            if (millis() - startTime > 1000) {
                ignitionStart();
                ignitionTime = millis();
                state = BURN;
            }
            break;
        case BURN:
            if(((millis() - ignitionTime) > 500) && (globalFlags & 0x01)) {
                ignitionStop();
            }
            dataPoint.data.time = millis() - startTime;
            dataPoint.data.state = state;
            dataPoint.data.flags = globalFlags;
            dataPoint.data.thrust = getValue(thrustADC.continuousSingleRead(), LOADCELL_VREF, LOADCELL_GAIN, LOADCELL_FULL_SCALE_OUTPUT, LOADCELL_RANGE);
            dataPoint.data.pressure = getValue(pressureADC.continuousSingleRead(), PTAP_VREF, PTAP_GAIN, PTAP_FULL_SCALE_OUTPUT, PTAP_RANGE, PTAP_OFFSET);
            sendTelemetry(dataPoint);
            flash.writeByteArray(prevAddress, dataPoint.bytes, sizeof(dataPoint.data));
            prevAddress += sizeof(dataPoint.data);
            break;
        case DATA_TRANSFER:
            transferLog();
            state = END;
            break;
        case END:
            //Same as standby
            dataPoint.data.time = millis();
            dataPoint.data.state = state;
            dataPoint.data.flags = globalFlags;
            dataPoint.data.thrust = getValue(thrustADC.continuousSingleRead(), LOADCELL_VREF, LOADCELL_GAIN, LOADCELL_FULL_SCALE_OUTPUT, LOADCELL_RANGE);
            dataPoint.data.pressure = getValue(pressureADC.continuousSingleRead(), PTAP_VREF, PTAP_GAIN, PTAP_FULL_SCALE_OUTPUT, PTAP_RANGE, PTAP_OFFSET);
            sendTelemetry(dataPoint);
            break;
    }
}
