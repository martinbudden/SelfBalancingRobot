#pragma once

#include <I2C.h>
#include <IMU_Base.h>
#include <Quaternion.h>
#include <array>


class IMU_BNO085 : public IMU_Base {
public:
    enum {
        CHANNEL_COMMAND = 0,
        CHANNEL_EXECUTABLE = 1,
        CHANNEL_SENSOR_HUB_CONTROL = 2,
        CHANNEL_INPUT_SENSOR_REPORTS = 3,
        CHANNEL_WAKE_UP_INPUT_SENSOR_REPORTS = 4,
        CHANNEL_GYRO_INTEGRATED_ROTATION_VECTOR_REPORT = 5,
        CHANNEL_COUNT = 6
    };
    enum {
        SENSOR_REPORTID_ACCELEROMETER = 0x01, // Q point = 8
        SENSOR_REPORTID_GYROSCOPE_CALIBRATED = 0x02, // Q point = 9, radians/second
        SENSOR_REPORTID_MAGNETIC_FIELD_CALIBRATED = 0x03, // Q point = 4
        SENSOR_REPORTID_LINEAR_ACCELERATION = 0x04, // Q point = 8
        //quaternion referenced to magnetic north and gravity. It is produced by fusing the outputs of the accelerometer, gyroscope and magnetometer.
        SENSOR_REPORTID_ROTATION_VECTOR = 0x05, // Q point = 14
        SENSOR_REPORTID_GRAVITY = 0x06, // Q point = 8
        SENSOR_REPORTID_GYROSCOPE_UNCALIBRATED = 0x07, // Q point = 9
        // produced by fusing the outputs of the accelerometer and the gyroscope (ie no magnetometer).
        SENSOR_REPORTID_GAME_ROTATION_VECTOR = 0x08, // Q point = 14
        SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
        SENSOR_REPORTID_TAP_DETECTOR = 0x10,
        SENSOR_REPORTID_STEP_COUNTER = 0x11,
        SENSOR_REPORTID_STABILITY_CLASSIFIER = 0x13,
        SENSOR_REPORTID_RAW_ACCELEROMETER = 0x14,
        SENSOR_REPORTID_RAW_GYROSCOPE = 0x15,
        SENSOR_REPORTID_RAW_MAGNETOMETER = 0x16,
        SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER = 0x1E,
        SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR = 0x28,
        SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR = 0x29,
        // supports higher data rates than the more accurate Rotation Vector
        SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR = 0x2A, // Q point =14 for orientation, Q point = 10 for gyro
        SENSOR_REPORTID_GET_FEATURE_REQUEST = 0xFE,
        SENSOR_REPORTID_SET_FEATURE_COMMAND = 0xFD,
        SENSOR_REPORTID_GET_FEATURE_RESPONSE = 0xFC,
        SENSOR_REPORTID_FORCE_SENSOR_FLUSH = 0xF0,
        SENSOR_REPORTID_FLUSH_COMPLETED = 0xEF
    };

#pragma pack(push, 1)
    struct SHTP_Header {
        uint8_t lengthLSB;
        uint8_t lengthMSB;
        uint8_t channel;
        uint8_t sequenceNumber;
    };
    struct SHTP_Packet {
        SHTP_Header header;
        // Note that the BNO08X adds a timestamp reference report to sensor input reports.
        std::array<uint8_t, 5> timestamp;
        std::array<uint8_t, 23> data;
    };
    struct command_message_t {
        uint8_t reportID; // 0xF2
        uint8_t sequenceNumber;
        uint8_t command;
        std::array<uint8_t, 9> P;
    };
    struct command_response_t {
        uint8_t reportID; // 0xF1
        uint8_t sequenceNumber;
        uint8_t command;
        uint8_t commandSequenceNumber;
        uint8_t responseSequenceNumber;
        std::array<uint8_t, 11> R;
    };
    struct sensor_output_t {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t accuracy;
        uint8_t sequenceNumber;
        uint16_t delay;
    };
    struct sensor_output_uncalibrated_gyro_t {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t accuracy;
        uint8_t sequenceNumber;
        uint16_t delay;
        int16_t biasX;
        int16_t biasY;
        int16_t biasZ;
    };
    // The BNO085 data sheet uses the term "rotation vector" rather than "orientation quaternion" used in the Stabilized Vehicle software
    struct rotation_vector_t {
        int16_t i;
        int16_t j;
        int16_t k;
        int16_t real;
        uint16_t accuracy;
        uint16_t radianAccuracy;
    };
    struct gyro_integrated_rotation_vector_t {
        int16_t i;
        int16_t j;
        int16_t k;
        int16_t real;
        int16_t x;
        int16_t y;
        int16_t z;
    };
#pragma pack(pop)
public:
    static constexpr uint8_t I2C_ADDRESS=0x4A;
public:
    IMU_BNO085(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin, void* i2cMutex);
    IMU_BNO085(axis_order_t axisOrder, uint8_t SDA_pin, uint8_t SCL_pin) : IMU_BNO085(axisOrder, SDA_pin, SCL_pin, nullptr) {}
    void init();
    void setFeatureCommand(uint8_t reportID, uint32_t timeBetweenReportsUs, uint32_t specificConfig);
    virtual xyz_int32_t readGyroRaw() override;
    virtual xyz_int32_t readAccRaw() override;
    virtual xyz_t readGyroRPS() override;
    virtual Quaternion readOrientation() override;
public:
    xyz_t getAcc() const;
    xyz_t getAccLinear() const;
    xyz_t getGyroRPS() const;
    xyz_t getMag() const;
    xyz_t getGravity() const;
    uint16_t parseInputSensorReport(const SHTP_Packet& packet);
    uint16_t parseGyroIntegratedRotationVectorReport(const SHTP_Packet& packet);
    uint16_t parseCommandResponse(const SHTP_Packet& packet);
    // for unit testing
    inline gyro_integrated_rotation_vector_t getGyroIntegratedRotationVectorData() const { return _gyroIntegratedRotationVector; }
    inline rotation_vector_t getRotationVectorData() const { return _rotationVector; }
    inline sensor_output_t getAccData() const { return _acc; }
    inline sensor_output_t getAccLinearData() const { return _accLinear; }
    inline sensor_output_t getGyroRPS_Data() const { return _gyroRPS; }
    inline sensor_output_t getMagData() const { return _mag; }
    inline sensor_output_t getGravityData() const { return _gravity; }
    inline sensor_output_t getAccRawData() const { return _accRaw; }
    inline sensor_output_t getGyroRawData() const { return _gyroRaw; }
    inline sensor_output_t getMagRawData() const { return _magRaw; }
    inline sensor_output_uncalibrated_gyro_t getGyroUncalibratedRPS_Data() const { return _gyroUncalibratedRPS; }
private:
    uint16_t readPacketAndParse();
    bool readPacket();
    bool sendPacket(uint8_t channelNumber, uint8_t dataLength);
    bool sendCommandCalibrateMotionEngine();
    bool sendCommandSaveDynamicCalibrationData();
    bool sendCommand(uint8_t command);
protected:
    I2C _bus; //!< Serial Communication Bus interface, can be either I2C or SPI
    uint32_t _timestamp {};
    uint32_t _orientationAvailable {false};
    uint32_t _gyroAvailable {false};
    // SHTP (Sensor Hub Transport Protocol)
    SHTP_Packet _shtpPacket {};
    std::array<uint8_t, CHANNEL_COUNT> _sequenceNumber {}; //There are 6 com channels. Each channel has its own sequence number

    uint8_t _resetCompleteReceived = false; // set true when Reset Complete packet received.
    uint8_t _calibrationStatus {}; //R0 of COMMAND_CALIBRATE_MOTION_ENGINE Response

    // combined gyro and rotation for SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR
    gyro_integrated_rotation_vector_t _gyroIntegratedRotationVector {};
    rotation_vector_t _rotationVector {};

    command_message_t _commandMessage {};

    sensor_output_t _acc {};
    sensor_output_t _accLinear {}; // Acceleration of the device with gravity removed
    sensor_output_t _gyroRPS {};
    sensor_output_t _mag {};
    sensor_output_t _gravity {};
    sensor_output_t _accRaw {};
    sensor_output_t _gyroRaw {};
    sensor_output_t _magRaw {};
    sensor_output_uncalibrated_gyro_t _gyroUncalibratedRPS {};
};
