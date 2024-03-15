#include "Arduino_AD7190.h"

AD7190::AD7190(uint8_t _CS_PIN) {
    this->cs_pin = _CS_PIN;
}

void AD7190::setRegisterValue(uint8_t _registerAddress, uint32_t _registerValue, uint8_t _bytesNumber, bool _modifyCS) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint8_t* dataPointer = (uint8_t*)&_registerValue;
    uint8_t bytesNr = _bytesNumber + 1;

    data[0] = 0x01 * _modifyCS;
    data[1] = AD7190_COMM_WRITE | AD7190_COMM_ADDR(_registerAddress);
    while(bytesNr > 1) {
        data[bytesNr] = *dataPointer;
        dataPointer++;
        bytesNr--;
    }
    spi_write(data, (_bytesNumber + 1));
}

uint32_t AD7190::getRegisterValue(uint8_t _registerAddress, uint8_t _bytesNumber, bool _modifyCS) {
    uint8_t data[5] = {0, 0, 0, 0, 0};
    uint32_t receivedData = 0;
    uint8_t i = 0;

    data[0] = 0x01 * _modifyCS;
    data[1] = AD7190_COMM_READ | AD7190_COMM_ADDR(_registerAddress);
    spi_read(data, (_bytesNumber + 1));
    for(i = 1; i < _bytesNumber + 1; i++) {
        receivedData = (receivedData << 8) + data[i];
    }
    return receivedData;
}

uint8_t AD7190::init() {
    uint8_t status = 1;
    uint8_t regVal = 0;

    spi_init();
    reset();
    regVal = getRegisterValue(AD7190_REG_ID, 1, 1);
    //do some logic to check ID
    return(status);
}

void AD7190::reset() {
    digitalWrite(this->cs_pin, LOW);
    delay(100);
    for(int i = 0; i < 8; i++) {
        SPI.transfer(0xFF);
    }
    digitalWrite(this->cs_pin, HIGH);
    delay(100);
}

void AD7190::setPower(uint8_t _powerMode) {
    uint32_t oldRegValue = 0;
    uint32_t newRegValue = 0;

    oldRegValue = getRegisterValue(AD7190_REG_MODE, 3, 1);
    oldRegValue &= ~AD7190_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7190_MODE_SEL((_powerMode * (AD7190_MODE_IDLE)) | (!_powerMode * (AD7190_MODE_PWRDN)));
    setRegisterValue(AD7190_REG_MODE, newRegValue, 3, 1);
}

void AD7190::waitRdyGoLow() {
    while(digitalRead(MISO) == HIGH);
}

void AD7190::channelSelect(uint8_t _channel) {
    uint32_t oldRegValue = 0;
    uint32_t newRegValue = 0;

    oldRegValue = getRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_CHAN(0xFF));
    newRegValue = oldRegValue | AD7190_CONF_CHAN(1 << _channel);
    setRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

void AD7190::calibrate(uint8_t _mode, uint8_t _channel) {
    uint32_t oldRegValue = 0;
    uint32_t newRegValue = 0;

    channelSelect(_channel);
    oldRegValue = getRegisterValue(AD7190_REG_MODE, 3, 1);
    oldRegValue &= ~AD7190_MODE_SEL(0x7);
    newRegValue = oldRegValue | AD7190_MODE_SEL(_mode);
    digitalWrite(this->cs_pin, LOW);
    setRegisterValue(AD7190_REG_MODE, newRegValue, 3, 0); //CS is not modified
    waitRdyGoLow();
    digitalWrite(this->cs_pin, HIGH);
}

void AD7190::rangeSetup(uint8_t _polarity, uint8_t _range) {
    uint32_t oldRegValue = 0;
    uint32_t newRegValue = 0;

    oldRegValue = getRegisterValue(AD7190_REG_CONF, 3, 1);
    oldRegValue &= ~(AD7190_CONF_UNIPOLAR |
                     AD7190_CONF_GAIN(0x7));
    newRegValue = oldRegValue | (_polarity * AD7190_CONF_UNIPOLAR) |
                  AD7190_CONF_GAIN(_range);
    setRegisterValue(AD7190_REG_CONF, newRegValue, 3, 1);
}

uint32_t AD7190::singleConversion() {
    uint32_t command = 0;
    uint32_t regData = 0;

    command = AD7190_MODE_SEL(AD7190_MODE_SINGLE) | AD7190_MODE_CLKSRC(AD7190_CLK_INT) | AD7190_MODE_RATE(0x060);
    digitalWrite(this->cs_pin, LOW);
    setRegisterValue(AD7190_REG_MODE, command, 3, 0); //CS is not modified
    waitRdyGoLow();
    regData = getRegisterValue(AD7190_REG_DATA, 3, 0); //CS is not modified
    digitalWrite(this->cs_pin, HIGH);
    return regData;
}

uint32_t AD7190::continuousReadAvg(unsigned char _sampleNumber)
{
    unsigned long samplesAverage = 0x0;
    unsigned char count = 0x0;
    unsigned long command = 0x0;
    
    command = AD7190_MODE_SEL(AD7190_MODE_CONT) | 
              AD7190_MODE_CLKSRC(AD7190_CLK_INT) |
              AD7190_MODE_SINC3 |
              AD7190_MODE_RATE(0x10);
    digitalWrite(this->cs_pin, LOW);
    setRegisterValue(AD7190_REG_MODE, command, 3, 0); // CS is not modified.
    for(count = 0;count < _sampleNumber;count ++)
    {
        waitRdyGoLow();
        samplesAverage += getRegisterValue(AD7190_REG_DATA, 3, 0); // CS is not modified.
    }
    digitalWrite(this->cs_pin, HIGH);
    samplesAverage = samplesAverage / _sampleNumber;
    
    return(samplesAverage);
}

uint32_t AD7190::continuousSingleRead() {
    uint32_t regData = 0;
    digitalWrite(this->cs_pin, LOW);
    waitRdyGoLow();
    regData = getRegisterValue(AD7190_REG_DATA, 3, 1); //CS is modified
    digitalWrite(this->cs_pin, HIGH);
    return regData;
}

uint8_t AD7190::spi_write(uint8_t* _data, uint8_t _bytesNumber) {
    int SCNumber = _data[0];
    digitalWrite(this->cs_pin, LOW);
    SPI.transfer(&_data[1], _bytesNumber);
    if(SCNumber == 0x1) {
        digitalWrite(this->cs_pin, HIGH);
    }
    return _bytesNumber;
}

uint8_t AD7190::spi_read(uint8_t* _data, uint8_t _bytesNumber) {
    int i = 0;
    digitalWrite(this->cs_pin, LOW);
    int SCNumber = _data[0];
    for(i=1;i<_bytesNumber+1;i++) {
        _data[i-1] = SPI.transfer(_data[i]);
    }
    if(SCNumber == 0x1) {
        digitalWrite(this->cs_pin, HIGH);
    }
    return _bytesNumber;
}

void AD7190::spi_init() {
    pinMode(this->cs_pin, OUTPUT);
    digitalWrite(this->cs_pin, HIGH);
}