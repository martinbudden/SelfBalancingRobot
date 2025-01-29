#pragma once

#include <I2C.h>
#include <IMU_Base.h>


class  IMU_BNO085 : public IMU_Base {
public:
    enum {CHANNEL_COMMAND = 0, CHANNEL_EXECUTABLE = 1, CHANNEL_CONTROL = 2, CHANNEL_REPORTS = 3, CHANNEL_WAKE_REPORTS = 4, CHANNEL_GYRO = 5, CHANNEL_COUNT = 6};
    enum {MAX_PACKET_SIZE = 128}; //Packets can be up to 32k.
    enum {MAX_METADATA_SIZE = 9}; //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

public:
    static constexpr uint8_t I2C_ADDRESS=0x4A;

public:
    IMU_BNO085(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_BNO085(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) :  IMU_BNO085(axisOrder, SDA_pin, SCL_pin, nullptr) {}
    void init();
    virtual xyz_int32_t readGyroRaw() const override;
    virtual xyz_int32_t readAccRaw() const override;
public:
    bool receivePacket();
    bool getData(int bytesRemaining);
    bool sendPacket(uint8_t channelNumber, uint8_t dataLength);
    uint16_t parseInputReport();
    uint16_t parseCommandReport();
private:
    I2C _bus; //!< Serial Communication Bus interface, can be either I2C or SPI
    // SHTP (Sensor Hub Transport Protocol) t
    uint8_t _shtpHeader[4]; //Each packet has a header of 4 bytes
    uint8_t _shtpData[MAX_PACKET_SIZE];
    uint8_t _sequenceNumber[CHANNEL_COUNT] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
    uint8_t _commandSequenceNumber = 0;                //Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
    uint32_t _metaData[MAX_METADATA_SIZE];            //There is more than 10 words in a metadata record but we'll stop at Q point 3

    bool _hasReset = false; // Keeps track of any Reset Complete packets we receive. 
    uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
    uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
    uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
    uint16_t rawUncalibratedGyroX, rawUncalibratedGyroY, rawUncalibratedGyroZ, rawBiasX, rawBiasY, rawBiasZ, uncalibratedGyroAccuracy;
    uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
    uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
    uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
    uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
    uint8_t tapDetector;
    uint16_t stepCount;
    uint32_t _timeStamp;
    uint8_t stabilityClassifier;
    uint8_t activityClassifier;
    uint8_t *_activityConfidences;                          //Array that store the confidences of the 9 possible activities
    uint8_t _calibrationStatus;                              //Byte R0 of ME Calibration Response
    uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
    uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;    //Raw readings from MEMS sensor
    uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;          //Raw readings from MEMS sensor

};
