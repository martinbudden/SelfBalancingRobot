#if defined(USE_IMU_BNO085)

#include "IMU_BNO085.h"
#include <array>
#include <cassert>
#include <cmath>

namespace { // use anonymous namespace to make items local to this translation unit
constexpr float GYRO_2000DPS_RES { 2000.0 / 32768.0 };
constexpr float ACC_16G_RES { 16.0 / 32768.0 };
} // end namespace

enum {
    REPORT_ID_COMMAND_RESPONSE = 0xF1,
    REPORT_ID_COMMAND_REQUEST = 0xF2,
    REPORT_ID_FRS_READ_RESPONSE = 0xF3,
    REPORT_ID_FRS_READ_REQUEST = 0xF4,
    REPORT_ID_FRS_WRITE_RESPONSE = 0xF5,
    REPORT_ID_FRS_WRITE_DATA_REQUEST = 0xF6,
    REPORT_ID_FRS_WRITE_REQUEST = 0xF7,
    REPORT_ID_PRODUCT_ID_RESPONSE = 0xF8,
    REPORT_ID_PRODUCT_ID_REQUEST = 0xF9,
    REPORT_ID_TIMESTAMP_REBASE = 0xFA,
    REPORT_ID_BASE_TIMESTAMP_REFERENCE= 0xFB
};

enum { EXECUTABLE_RESET_COMPLETE = 0x1 };

enum { 
    COMMAND_REPORT_ERRORS = 0x01,
    COMMAND_COUNTER_COMMANDS = 0x02, // sub-commands specified in P0
    COMMAND_TARE = 0x03,
    COMMAND_INITIALIZE = 0x04,
    COMMAND_SAVE_DYNAMIC_CALIBRATION_DATA = 0x06, // DCD
    COMMAND_CALIBRATE_MOTION_ENGINE = 0x07,
    COMMAND_CONFIGURE_PERIODIC_DCD_SAVE = 0x09,
    COMMAND_GET_OSCILLATOR_TYPE = 0x0A,
    COMMAND_CLEAR_DCD_AND_RESET = 0x0B
};


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
    setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, 5000, 0); //Send data update every 5000us, 200Hz. Highest rate supported is 400Hz
}

void IMU_BNO085::setFeatureCommand(uint8_t reportID, uint32_t timeBetweenReportsUs, uint32_t specificConfig)
{
    _shtpPacket.data[0] = SENSOR_REPORTID_SET_FEATURE_COMMAND; // Set feature command. Reference page 55
    _shtpPacket.data[1] = reportID;                            // Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    _shtpPacket.data[2] = 0;                                   // Feature flags
    _shtpPacket.data[3] = 0;                                   // Change sensitivity (LSB)
    _shtpPacket.data[4] = 0;                                   // Change sensitivity (MSB)
    _shtpPacket.data[5] = (timeBetweenReportsUs >> 0) & 0xFF;  // Report interval (LSB) in microseconds. 0x7A120 = 500ms
    _shtpPacket.data[6] = (timeBetweenReportsUs >> 8) & 0xFF;  // Report interval
    _shtpPacket.data[7] = (timeBetweenReportsUs >> 16) & 0xFF; // Report interval
    _shtpPacket.data[8] = (timeBetweenReportsUs >> 24) & 0xFF; // Report interval (MSB)
    _shtpPacket.data[9] = 0;                                   // Batch Interval (LSB)
    _shtpPacket.data[10] = 0;                                  // Batch Interval
    _shtpPacket.data[11] = 0;                                  // Batch Interval
    _shtpPacket.data[12] = 0;                                  // Batch Interval (MSB)
    _shtpPacket.data[13] = (specificConfig >> 0) & 0xFF;       // Sensor-specific config (LSB)
    _shtpPacket.data[14] = (specificConfig >> 8) & 0xFF;       // Sensor-specific config
    _shtpPacket.data[15] = (specificConfig >> 16) & 0xFF;      // Sensor-specific config
    _shtpPacket.data[16] = (specificConfig >> 24) & 0xFF;      // Sensor-specific config (MSB)

    sendPacket(CHANNEL_SENSOR_HUB_CONTROL, 17);
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

Quaternion IMU_BNO085::readOrientation() const
{
    while (!_orientationAvailable) {
        const_cast<IMU_BNO085*>(this)->readPacketAndParse(); //!!TODO, for now
    }
    _orientationAvailable = false;
    // for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A // Q point = 14 for orientation, Q point = 10 for gyro
    constexpr int Q_point = 14;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return Quaternion(
        static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.real)) * multiplier,
        static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.i)) * multiplier,
        static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.j)) * multiplier,
        static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.k)) * multiplier
    );
}

xyz_t IMU_BNO085::readGyroRPS() const
{
    while (!_gyroAvailable) {
        const_cast<IMU_BNO085*>(this)->readPacketAndParse(); //!!TODO, for now
    }
    _gyroAvailable = false;
    // for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A // Q point = 14 for orientation, Q point = 10 for gyro
    constexpr int Q_point = 10;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return xyz_t {
        .x = static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.x)) * multiplier,
        .y = static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.y)) * multiplier,
        .z = static_cast<float>(static_cast<int16_t>(_gyroIntegratedRotationVector.z)) * multiplier
    };
}

xyz_t IMU_BNO085::getAcc() const
{
    constexpr int Q_point = 8;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return xyz_t {
        .x = static_cast<float>(static_cast<int16_t>(_acc.x)) * multiplier,
        .y = static_cast<float>(static_cast<int16_t>(_acc.y)) * multiplier,
        .z = static_cast<float>(static_cast<int16_t>(_acc.z)) * multiplier
    };
}


xyz_t IMU_BNO085::getAccLinear() const
{
    constexpr int Q_point = 8;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return xyz_t {
        .x = static_cast<float>(static_cast<int16_t>(_accLinear.x)) * multiplier,
        .y = static_cast<float>(static_cast<int16_t>(_accLinear.y)) * multiplier,
        .z = static_cast<float>(static_cast<int16_t>(_accLinear.z)) * multiplier
    };
}

xyz_t IMU_BNO085::getGyroRPS() const
{
    constexpr int Q_point = 9;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return xyz_t {
        .x = static_cast<float>(static_cast<int16_t>(_gyroRPS.x)) * multiplier,
        .y = static_cast<float>(static_cast<int16_t>(_gyroRPS.y)) * multiplier,
        .z = static_cast<float>(static_cast<int16_t>(_gyroRPS.z)) * multiplier
    };
}

xyz_t IMU_BNO085::getMag() const
{
    constexpr int Q_point = 4;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return xyz_t {
        .x = static_cast<float>(static_cast<int16_t>(_mag.x)) * multiplier,
        .y = static_cast<float>(static_cast<int16_t>(_mag.y)) * multiplier,
        .z = static_cast<float>(static_cast<int16_t>(_mag.z)) * multiplier
    };
}

xyz_t IMU_BNO085::getGravity() const
{
    constexpr int Q_point = 8;
    constexpr float multiplier = 1.0F / (1 << Q_point);
    return xyz_t {
        .x = static_cast<float>(static_cast<int16_t>(_gravity.x)) * multiplier,
        .y = static_cast<float>(static_cast<int16_t>(_gravity.y)) * multiplier,
        .z = static_cast<float>(static_cast<int16_t>(_gravity.z)) * multiplier
    };
}

uint16_t IMU_BNO085::parseCommandResponse(const SHTP_Packet& packet)
{
    const uint8_t reportID = packet.data[0];
    if (reportID == REPORT_ID_COMMAND_RESPONSE) {
        //The BNO085 responds with this report to command requests.
        const uint8_t command = packet.data[2];
        if (command == COMMAND_CALIBRATE_MOTION_ENGINE) {
            _calibrationStatus = packet.data[0]; //R0 - Status (0 = success, non-zero = fail)
        }
        return reportID;
    }

    return 0;
}


/*!
The gyro integrated rotation vector input reports are sent via the special gyro channel.
They DO NOT INCLUDE the usual reportID, sequenceNumber, status, and delay fields
Rates of up to 400 Hz are supported.
*/
uint16_t IMU_BNO085::parseGyroIntegratedRotationVectorReport(const SHTP_Packet& packet)
{
    _orientationAvailable = true;
    _gyroAvailable = true;
    _gyroIntegratedRotationVector.i    = static_cast<uint16_t>(packet.data[1])  << 8 | packet.data[0];
    _gyroIntegratedRotationVector.j    = static_cast<uint16_t>(packet.data[3])  << 8 | packet.data[2];
    _gyroIntegratedRotationVector.k    = static_cast<uint16_t>(packet.data[5])  << 8 | packet.data[4];
    _gyroIntegratedRotationVector.real = static_cast<uint16_t>(packet.data[7])  << 8 | packet.data[6];
    _gyroIntegratedRotationVector.x    = static_cast<uint16_t>(packet.data[9])  << 8 | packet.data[8];
    _gyroIntegratedRotationVector.y    = static_cast<uint16_t>(packet.data[11]) << 8 | packet.data[10];
    _gyroIntegratedRotationVector.z    = static_cast<uint16_t>(packet.data[13]) << 8 | packet.data[12];

    return SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR;
}

/*!
Parse the input sensor report packet.
Packet format is:
packet.header           4 byte header
packet.timestamp[0:5]   5 byte timestamp
packet.data[0:3]        4 bytes of data containing reportID, sequenceNumber, status, and delay.
packet.date[4..]        sensor data starts
*/
uint16_t IMU_BNO085::parseInputSensorReport(const SHTP_Packet& packet)
{
    if (packet.timestamp[0] == REPORT_ID_BASE_TIMESTAMP_REFERENCE) {
        _timestamp = (static_cast<uint32_t>(packet.timestamp[4]) << 24)
            | (static_cast<uint32_t>(packet.timestamp[3]) << 16) 
            | (static_cast<uint32_t>(packet.timestamp[2]) << 8) 
            | static_cast<uint32_t>(packet.timestamp[1]);
    }

    /*
    Bits 1:0 - indicate the status of a sensor. 
    0 - Unreliable 
    1 - Accuracy low 
    2 - Accuracy medium 
    3 - Accuracy high 
    Bits 7:2 - Delay upper bits: 6 most-significant bits of report delay
    */
    const uint8_t reportID = packet.data[0];
    const uint8_t sequenceNumber = packet.data[1];
    const uint8_t status = packet.data[2]; //Get status bits
    const uint8_t accuracy = status & 0x03;
    const uint16_t delay = static_cast<uint16_t>(status & 0xFC) << 6 | packet.data[3];

    const uint16_t dataX = static_cast<uint16_t>(packet.data[5]) << 8 | packet.data[4];
    const uint16_t dataY = static_cast<uint16_t>(packet.data[7]) << 8 | packet.data[6];
    const uint16_t dataZ = static_cast<uint16_t>(packet.data[9]) << 8 | packet.data[8];

    switch (reportID) {
    case SENSOR_REPORTID_ACCELEROMETER:
        _acc.x = dataX;
        _acc.y = dataY;
        _acc.z = dataZ;
        _acc.accuracy = accuracy;
        _acc.sequenceNumber = sequenceNumber;
        _acc.delay = delay;
        break;
    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
        _gyroRPS.x = dataX;
        _gyroRPS.y = dataY;
        _gyroRPS.z = dataZ;
        _gyroRPS.accuracy = accuracy;
        _gyroRPS.sequenceNumber = sequenceNumber;
        _gyroRPS.delay = delay;
        break;
    case SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED:
        _mag.x = dataX;
        _mag.y = dataY;
        _mag.z = dataZ;
        _mag.accuracy = accuracy;
        _mag.sequenceNumber = sequenceNumber;
        _mag.delay = delay;
        break;
    case SENSOR_REPORTID_LINEAR_ACCELERATION:
        _accLinear.x = dataX;
        _accLinear.y = dataY;
        _accLinear.z = dataZ;
        _accLinear.accuracy = accuracy;
        _accLinear.sequenceNumber = sequenceNumber;
        _accLinear.delay = delay;
        break;
    case SENSOR_REPORTID_GRAVITY:
        _gravity.x = dataX;
        _gravity.y = dataY;
        _gravity.z = dataZ;
        _gravity.accuracy = accuracy;
        _gravity.sequenceNumber = sequenceNumber;
        _gravity.delay = delay;
        break;
    case SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED:
        _gyroUncalibratedRPS.x = dataX;
        _gyroUncalibratedRPS.y = dataY;
        _gyroUncalibratedRPS.z = dataZ;
        _gyroUncalibratedRPS.accuracy = accuracy;
        _gyroUncalibratedRPS.sequenceNumber = sequenceNumber;
        _gyroUncalibratedRPS.delay = delay;
        _gyroUncalibratedBiasX  = static_cast<uint16_t>(packet.data[11]) << 8 | packet.data[10];
        _gyroUncalibratedBiasY  = static_cast<uint16_t>(packet.data[13]) << 8 | packet.data[12];
        _gyroUncalibratedBiasZ  = static_cast<uint16_t>(packet.data[15]) << 8 | packet.data[14];
        break;
    case SENSOR_REPORTID_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR:
        _rotationVector.radianAccuracy = static_cast<uint16_t>(packet.data[13]) << 8 | packet.data[12];
        [[fallthrough]];
    // the GAME rotation vectors do not report radianAccuracy
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
        [[fallthrough]];
    case SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR:
        _rotationVector.i = dataX;
        _rotationVector.j = dataY;
        _rotationVector.k = dataZ;
        _rotationVector.real = static_cast<uint16_t>(packet.data[11]) << 8 | packet.data[10];
        _rotationVector.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_ACCELEROMETER:
        _accRaw.x = dataX;
        _accRaw.y = dataY;
        _accRaw.z = dataZ;
        _accRaw.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_GYROSCOPE:
        _gyroRaw.x = dataX;
        _gyroRaw.y = dataY;
        _gyroRaw.z = dataZ;
        _gyroRaw.accuracy = accuracy;
        break;
    case SENSOR_REPORTID_RAW_MAGNETOMETER:
        _magRaw.x = dataX;
        _magRaw.y = dataY;
        _magRaw.z = dataZ;
        _magRaw.accuracy = accuracy;
        break;
    case REPORT_ID_COMMAND_RESPONSE:
        //The BNO085 responds with this report to command requests. It's up to use to remember which command we issued.
        if (packet.data[2] == COMMAND_CALIBRATE_MOTION_ENGINE) {
            _calibrationStatus = packet.data[5]; //R0 - Status (0 = success, non-zero = fail)
        }
        break;
    default:
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
        return 0;
    }

    return reportID;
}

uint16_t IMU_BNO085::readPacketAndParse()
{
    static_assert(sizeof(SHTP_Header) == 4);
    if (readPacket() == true) {
        //Check to see if this packet is a sensor reporting its data to us
        switch (_shtpPacket.header.channel) {
        case CHANNEL_SENSOR_HUB_CONTROL:
            return parseCommandResponse(_shtpPacket);
            break;
        case CHANNEL_INPUT_SENSOR_REPORTS:
            return parseInputSensorReport(_shtpPacket);
            break;
        case CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT:
            return parseGyroIntegratedRotationVectorReport(_shtpPacket);
            break;
        }
    }
    return 0;
}

bool IMU_BNO085::readPacket()
{
    _bus.readBytes(reinterpret_cast<uint8_t*>(&_shtpPacket.header), sizeof(SHTP_Header));

    uint16_t dataLength = ((static_cast<uint16_t>(_shtpPacket.header.lengthMSB)) << 8) | (static_cast<uint16_t>(_shtpPacket.header.lengthLSB));
    dataLength &= ~(1 << 15); //Clear the MSbit.
    if (dataLength == 0) {
        //Packet is empty
        return false;
    }
    dataLength -= sizeof(SHTP_Header);
    assert(dataLength < 28);

    _bus.readBytes(&_shtpPacket.timestamp[0], dataLength);

    // Check for a reset complete packet
    if (_shtpPacket.header.channel == CHANNEL_EXECUTABLE && _shtpPacket.data[0] == EXECUTABLE_RESET_COMPLETE) {
        _resetCompleteReceived = true;
    } 
    return true;
}

// NOTE: Arduino has a maximum 32 byte send.
bool IMU_BNO085::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    const uint8_t packetLength = dataLength + sizeof(_shtpPacket.header); // Add four bytes for the header
    _shtpPacket.header.lengthLSB = packetLength & 0xFF;
    _shtpPacket.header.lengthMSB = packetLength >> 8;
    _shtpPacket.header.channel = channelNumber;
    _shtpPacket.header.sequenceNumber = _sequenceNumber[channelNumber]++;
    _bus.writeBytes(reinterpret_cast<uint8_t*>(&_shtpPacket), packetLength);

    return true;
}

bool IMU_BNO085::sendCommandCalibrateMotionEngine()
{
    _commandMessage.P.fill(0);
    _commandMessage.P[0] = 1; // acc calibration enabled
    _commandMessage.P[1] = 1; // gyro calibration enabled
    // all other calibration disabled
    return sendCommand(COMMAND_CALIBRATE_MOTION_ENGINE);
}

bool IMU_BNO085::sendCommandSaveDynamicCalibrationData()
{
    _commandMessage.P.fill(0);
    return sendCommand(COMMAND_SAVE_DYNAMIC_CALIBRATION_DATA);
}

bool IMU_BNO085::sendCommand(uint8_t command)
{
    _commandMessage.reportID = REPORT_ID_COMMAND_REQUEST;
    _commandMessage.command = command;
    _commandMessage.sequenceNumber++;
    _bus.writeBytes(reinterpret_cast<uint8_t*>(&_commandMessage), sizeof(_commandMessage));

    return true;
}


#endif
