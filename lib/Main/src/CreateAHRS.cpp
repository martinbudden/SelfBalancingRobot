#include "Main.h"

#include <AHRS.h>
#include <IMU_BMI270.h>
#include <IMU_BNO085.h>
#include <IMU_Filters.h>
#include <IMU_LSM6DS3TR_C.h>
#include <IMU_M5Stack.h>
#include <IMU_M5Unified.h>
#include <IMU_MPU6886.h>

#if defined(USE_IMU_MPU6886) && !defined(LIBRARY_IMU_USE_SPI_BUS)
#if defined(M5_STACK)
#include <M5Stack.h>
#else
#include <M5Unified.h>
#endif
#endif

#include <SensorFusion.h>


// NOLINTBEGIN(misc-const-correctness)
IMU_Base& Main::createIMU(void* i2cMutex)
{
    // Statically allocate the IMU according the the build flags
#if defined(LIBRARY_IMU_USE_SPI_BUS)

    enum { SPI_FREQUENCY_20_MHZ = 20000000 };
    const BUS_SPI::spi_pins_t pins = IMU_SPI_PINS;
#if defined(USE_IMU_MPU6886)
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, SPI_FREQUENCY_20_MHZ, BUS_SPI::IMU_SPI_INDEX, pins);
#elif defined(USE_IMU_BMI270)
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, SPI_FREQUENCY_20_MHZ, BUS_SPI::IMU_SPI_INDEX, pins);
#elif defined(USE_IMU_BNO085)
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, SPI_FREQUENCY_20_MHZ, IMU_SPI_CS_PIN);
#elif defined(USE_IMU_LSM6DS3TR_C) || defined(USE_IMU_ISM330DHCX) || defined(USE_LSM6DSOX)
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, SPI_FREQUENCY_20_MHZ, BUS_SPI::IMU_SPI_INDEX, pins);
#endif

#else

#if defined(USE_IMU_MPU6886)
#if defined(M5_STACK)
    const BUS_I2C::i2c_pins_t pins = IMU_I2C_PINS;
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, pins);
#else
    const BUS_I2C::i2c_pins_t pins = BUS_I2C::i2c_pins_t {
        .sda=static_cast<uint8_t>(M5.In_I2C.getSDA()),
        .scl=static_cast<uint8_t>(M5.In_I2C.getSCL()),
        .irq=BUS_I2C::IRQ_NOT_SET
    };
    static IMU_MPU6886 imuSensor(IMU_AXIS_ORDER, pins);
#endif
#elif defined(USE_IMU_BMI270)
    const BUS_I2C::i2c_pins_t pins = IMU_I2C_PINS;
    static IMU_BMI270 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_BNO085)
    const BUS_I2C::i2c_pins_t pins = IMU_I2C_PINS;
    static IMU_BNO085 imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_LSM6DS3TR_C) || defined(USE_IMU_ISM330DHCX) || defined(USE_LSM6DSOX)
    const BUS_I2C::i2c_pins_t pins = IMU_I2C_PINS;
    static IMU_LSM6DS3TR_C imuSensor(IMU_AXIS_ORDER, pins);
#elif defined(USE_IMU_M5_STACK)
    static IMU_M5_STACK imuSensor(IMU_AXIS_ORDER);
#elif defined(USE_IMU_M5_UNIFIED)
    static IMU_M5_UNIFIED imuSensor(IMU_AXIS_ORDER);
#elif !defined(FRAMEWORK_TEST)
    static_assert(false && "IMU type not specified");
#endif

#endif

#if !defined(FRAMEWORK_TEST)
    //static_cast<IMU_Base&>(imuSensor).init(1000000 / AHRS_TASK_INTERVAL_MICROSECONDS, i2cMutex);
    static_cast<IMU_Base&>(imuSensor).init(i2cMutex);
#endif

    return imuSensor;
}

AHRS& Main::createAHRS(uint32_t AHRS_taskIntervalMicroseconds, IMU_Base& imuSensor, IMU_FiltersBase& imuFilters)
{
    // Statically allocate the Sensor Fusion Filter
    // Timings are for 240MHz ESP32-S3
#if defined(USE_COMPLEMENTARY_FILTER)
    // approx 130 microseconds per update
    static ComplementaryFilter sensorFusionFilter;
#elif defined(USE_MAHONY_FILTER)
    // approx 10 microseconds per update
    static MahonyFilter sensorFusionFilter;
#elif defined(USE_VQF)
    const float deltaT = static_cast<float>(AHRS_taskIntervalMicroseconds) / 1000000.0F;
    static VQF sensorFusionFilter(deltaT, deltaT, deltaT, true, false, false);
#elif defined(USE_VQF_BASIC)
    static BasicVQF sensorFusionFilter(static_cast<float>(AHRS_taskIntervalMicroseconds) / 1000000.0F);
#else
    // approx 16 microseconds per update
    static MadgwickFilter sensorFusionFilter;
#endif

    static AHRS ahrs(AHRS_taskIntervalMicroseconds, sensorFusionFilter, imuSensor, imuFilters);
    return ahrs;
}

AHRS& Main::createAHRS(void* i2cMutex)
{
    const uint32_t AHRS_taskIntervalMicroseconds = AHRS_TASK_INTERVAL_MICROSECONDS;

    IMU_Base& imuSensor = createIMU(i2cMutex);

    // statically allocate the IMU_Filters
    static constexpr float cutoffFrequency = 100.0F;
    static IMU_Filters imuFilters(cutoffFrequency, static_cast<float>(AHRS_taskIntervalMicroseconds) / 1000000.0F);

    // Statically allocate the AHRS object
    return createAHRS(AHRS_taskIntervalMicroseconds, imuSensor, imuFilters);
}
// NOLINTEND(misc-const-correctness)
