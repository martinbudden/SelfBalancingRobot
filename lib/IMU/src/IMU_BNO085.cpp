#if defined(USE_IMU_BNO085)

#include "IMU_BNO085.h"
#include <array>
#include <cassert>
#include <cmath>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
} // end namespace

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_UNCALIBRATED_GYRO 0x07
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

// Reset complete packet (BNO08X Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_NOW 0
#define TARE_PERSIST 1
#define TARE_SET_REORIENTATION 2

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z   0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5


IMU_BNO085::IMU_BNO085(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex) :
    IMU_Base(axisOrder, i2cMutex),
    _bus(I2C_ADDRESS, SDA_pin, SCL_pin)
{
}

void IMU_BNO085::init()
{
    _gyroResolutionDPS = GYRO_2000DPS_RES;
    _gyroResolutionRPS = GYRO_2000DPS_RES * degreesToRadians;
    _accResolution = ACC_16G_RES;
}

IMU_Base::xyz_int32_t IMU_BNO085::readGyroRaw() const
{
    IMU_Base::xyz_int32_t ret {};
    return ret;
}

IMU_Base::xyz_int32_t IMU_BNO085::readAccRaw() const
{
    IMU_Base::xyz_int32_t ret {};
    return ret;
}

//This function pulls the data from the command response report

//Unit responds with packet that contains the following:
//_shtpHeader[0:3]: First, a 4 byte header
//_shtpData[0]: The Report ID
//_shtpData[1]: Sequence number (See 6.5.18.2)
//_shtpData[2]: Command
//_shtpData[3]: Command Sequence Number
//_shtpData[4]: Response Sequence Number
//_shtpData[5 + 0]: R0
//_shtpData[5 + 1]: R1
//_shtpData[5 + 2]: R2
//_shtpData[5 + 3]: R3
//_shtpData[5 + 4]: R4
//_shtpData[5 + 5]: R5
//_shtpData[5 + 6]: R6
//_shtpData[5 + 7]: R7
//_shtpData[5 + 8]: R8
uint16_t IMU_BNO085::parseCommandReport()
{
    if (_shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE) {
        //The BNO085 responds with this report to command requests. It's up to use to remember which command we issued.
        const uint8_t command = _shtpData[2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE) {
            _calibrationStatus = _shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
        }
        return _shtpData[0];
    } else {
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
    }

    //TODO additional feature reports may be strung together. Parse them all.
    return 0;
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//_shtpHeader[0:3]: First, a 4 byte header
//_shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//_shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//_shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//_shtpData[5 + 2]: Status
//_shtpData[3]: Delay
//_shtpData[4:5]: i/accel x/gyro x/etc
//_shtpData[6:7]: j/accel y/gyro y/etc
//_shtpData[8:9]: k/accel z/gyro z/etc
//_shtpData[10:11]: real/gyro temp/etc
//_shtpData[12:13]: Accuracy estimate
uint16_t IMU_BNO085::parseInputReport(void)
{
    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)_shtpHeader[1] << 8 | _shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
    //Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; //Remove the header bytes from the data count

    _timeStamp = ((uint32_t)_shtpData[4] << (8 * 3)) | ((uint32_t)_shtpData[3] << (8 * 2)) | ((uint32_t)_shtpData[2] << (8 * 1)) | ((uint32_t)_shtpData[1] << (8 * 0));

    // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
    if(_shtpHeader[2] == CHANNEL_GYRO) {
        rawQuatI = (uint16_t)_shtpData[1] << 8 | _shtpData[0];
        rawQuatJ = (uint16_t)_shtpData[3] << 8 | _shtpData[2];
        rawQuatK = (uint16_t)_shtpData[5] << 8 | _shtpData[4];
        rawQuatReal = (uint16_t)_shtpData[7] << 8 | _shtpData[6];
        rawFastGyroX = (uint16_t)_shtpData[9] << 8 | _shtpData[8];
        rawFastGyroY = (uint16_t)_shtpData[11] << 8 | _shtpData[10];
        rawFastGyroZ = (uint16_t)_shtpData[13] << 8 | _shtpData[12];

        return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
    }

    const uint8_t status = _shtpData[5 + 2] & 0x03; //Get status bits
    const uint16_t data1 = (uint16_t)_shtpData[5 + 5] << 8 | _shtpData[5 + 4];
    const uint16_t data2 = (uint16_t)_shtpData[5 + 7] << 8 | _shtpData[5 + 6];
    const uint16_t data3 = (uint16_t)_shtpData[5 + 9] << 8 | _shtpData[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports
    uint16_t data6 = 0;

    if (dataLength - 5 > 9) {
        data4 = (uint16_t)_shtpData[5 + 11] << 8 | _shtpData[5 + 10];
    }
    if (dataLength - 5 > 11) {
        data5 = (uint16_t)_shtpData[5 + 13] << 8 | _shtpData[5 + 12];
    }
    if (dataLength - 5 > 13) {
        data6 = (uint16_t)_shtpData[5 + 15] << 8 | _shtpData[5 + 14];
    }


    //Store these generic values to their proper global variable
    switch (_shtpData[5]) {
    case SENSOR_REPORTID_ACCELEROMETER:
        accelAccuracy = status;
        rawAccelX = data1;
        rawAccelY = data2;
        rawAccelZ = data3;
        break;
    case SENSOR_REPORTID_LINEAR_ACCELERATION:
        accelLinAccuracy = status;
        rawLinAccelX = data1;
        rawLinAccelY = data2;
        rawLinAccelZ = data3;
        break;
    case SENSOR_REPORTID_GYROSCOPE:
        gyroAccuracy = status;
        rawGyroX = data1;
        rawGyroY = data2;
        rawGyroZ = data3;
        break;
    case SENSOR_REPORTID_UNCALIBRATED_GYRO:
        uncalibratedGyroAccuracy = status;
        rawUncalibratedGyroX = data1;
        rawUncalibratedGyroY = data2;
        rawUncalibratedGyroZ = data3;
        rawBiasX  = data4;
        rawBiasY  = data5;
        rawBiasZ  = data6;
        break;
    case SENSOR_REPORTID_MAGNETIC_FIELD:
        magAccuracy = status;
        rawMagX = data1;
        rawMagY = data2;
        rawMagZ = data3;
        break;
    case SENSOR_REPORTID_ROTATION_VECTOR:
        [[fallthrough]]
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
        [[fallthrough]]
    case SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR:
        [[fallthrough]]
    case SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR:
        quatAccuracy = status;
        rawQuatI = data1;
        rawQuatJ = data2;
        rawQuatK = data3;
        rawQuatReal = data4;

        //Only available on rotation vector and ar/vr stabilized rotation vector,
        // not game rot vector and not ar/vr stabilized rotation vector
        rawQuatRadianAccuracy = data5;
        break;
    case SENSOR_REPORTID_TAP_DETECTOR:
        tapDetector = _shtpData[5 + 4]; //Byte 4 only
        break;
    case SENSOR_REPORTID_STEP_COUNTER:
        stepCount = data3; //Bytes 8/9
        break;
    case SENSOR_REPORTID_STABILITY_CLASSIFIER:
        stabilityClassifier = _shtpData[5 + 4]; //Byte 4 only
        break;
    case SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER:
        activityClassifier = _shtpData[5 + 5]; //Most likely state

        //Load activity classification confidences into the array
        for (uint8_t x = 0; x < 9; x++)                       //Hardcoded to max of 9. TODO - bring in array size
            _activityConfidences[x] = _shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
        break;
    case SENSOR_REPORTID_RAW_ACCELEROMETER:
        memsRawAccelX = data1;
        memsRawAccelY = data2;
        memsRawAccelZ = data3;
        break;
    case SENSOR_REPORTID_RAW_GYROSCOPE:
        memsRawGyroX = data1;
        memsRawGyroY = data2;
        memsRawGyroZ = data3;
        break;
    case SENSOR_REPORTID_RAW_MAGNETOMETER:
        memsRawMagX = data1;
        memsRawMagY = data2;
        memsRawMagZ = data3;
        break;
    case SHTP_REPORT_COMMAND_RESPONSE:
        //The BNO085 responds with this report to command requests. It's up to use to remember which command we issued.
        if (_shtpData[5 + 2] == COMMAND_ME_CALIBRATE) {
            _calibrationStatus = _shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
        }
        break;
    case SENSOR_REPORTID_GRAVITY:
        gravityAccuracy = status;
        gravityX = data1;
        gravityY = data2;
        gravityZ = data3;
        break;
    default:
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
        return 0;
    }

    //TODO additional feature reports may be strung together. Parse them all.
    return _shtpData[5];
}

bool IMU_BNO085::receivePacket(void)
{
#if false
    _i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)4); //Ask for four bytes to find out how much data we need to read
    if (waitForI2C() == false)
        return (false); //Error

    //Get the first four bytes, aka the packet header
    const uint8_t packetLSB = _bus.readByte();
    const uint8_t packetMSB = _bus.readByte();
    const uint8_t channelNumber = _bus.readByte();
    const uint8_t sequenceNumber = _bus.readByte(); //Not sure if we need to store this or not

    //Store the header info.
    _shtpHeader[0] = packetLSB;
    _shtpHeader[1] = packetMSB;
    _shtpHeader[2] = channelNumber;
    _shtpHeader[3] = sequenceNumber;

    //Calculate the number of data bytes in this packet
    uint16_t dataLength = ((static_cast<uint16_t>(packetMSB)) << 8) | (static_cast<uint16_t>(packetLSB));
    dataLength &= ~(1 << 15); //Clear the MSbit.
    //This bit indicates if this package is a continuation of the last. Ignore it for now.
    //TODO catch this as an error and exit

    // if (_printDebug == true)
    // {
    //     _debugPort->print(F("receivePacket (I2C): dataLength is: "));
    //     _debugPort->println(dataLength);
    // }

    if (dataLength == 0) {
        //Packet is empty
        return false;
    }
    dataLength -= 4; //Remove the header bytes from the data count

    getData(dataLength);

    // Quickly check for reset complete packet. No need for a separate parser.
    // This function is also called after soft reset, so we need to catch this
    // packet here otherwise we need to check for the reset packet in multiple
    // places.
    if (_shtpHeader[2] == CHANNEL_EXECUTABLE && _shtpData[0] == EXECUTABLE_RESET_COMPLETE) {
        _hasReset = true;
    } 
#endif
    return true;
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The _shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool IMU_BNO085::getData(int bytesRemaining)
{
#if false
    int dataSpot = 0; //Start at the beginning of _shtpData array

    //Setup a series of chunked 32 byte reads
    while (bytesRemaining > 0) {
        uint16_t numberOfBytesToRead = bytesRemaining;
        if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
            numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

        //_i2cPort->requestFrom((uint8_t)_deviceAddress, (size_t)(numberOfBytesToRead + 4));
        //if (waitForI2C() == false)
        //    return (0); //Error

        //The first four bytes are header bytes and are throw away
        _bus.readByte();
        _bus.readByte();
        _bus.readByte();
        _bus.readByte();

        for (uint8_t x = 0; x < numberOfBytesToRead; x++) {
            uint8_t incoming = _bus.readByte();
            if (dataSpot < MAX_PACKET_SIZE) {
                _shtpData[dataSpot++] = incoming; //Store data into the _shtpData array
            } else {
                //Do nothing with the data
            }
        }

        bytesRemaining -= numberOfBytesToRead;
    }
#endif
    return true;
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool IMU_BNO085::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
#if false
    const uint8_t packetLength = dataLength + 4; //Add four bytes for the header

    if(packetLength > I2C_BUFFER_LENGTH) return(false); //You are trying to send too much. Break into smaller packets.

    _i2cPort->beginTransmission(_deviceAddress);

    //Send the 4 byte packet header
    _bus.writeByte(packetLength & 0xFF);
    _bus.writeByte(packetLength >> 8);
    _bus.writeByte(channelNumber);
    _bus.writeByte(sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel

    //Send the user's data packet
    for (uint8_t i = 0; i < dataLength; i++) {
        _bus.writeByte(_shtpData[i]);
    }

    int i2cResult = _i2cPort->endTransmission();
    if (i2cResult != 0) {
        return false;
    }
#endif
    return true;
}


#endif
