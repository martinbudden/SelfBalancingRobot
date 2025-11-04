#pragma once

/*!
Targets
*/

#define USE_MADGWICK_FILTER

#if defined(TARGET_CODECELL)
    #define BOARD_IDENTIFIER    "CodeCel_ESP32C3"

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BNO085
    #define IMU_I2C_PINS        {.sda=8,.scl=9,.irq=BUS_I2C::IRQ_NOT_SET}

    #define MOTORS_GPIO
    #define MOTOR_GPIO_PINS     {.motorLeft=0,.motorRight=0,.servoLeft=0,.servoRight=0}
    #define USE_IMU_BNO085
#endif

#if defined(TARGET_M5STACK_STICKC_BALAC)
    #define IMU_AXIS_ORDER      IMU_Base::XPOS_ZPOS_YNEG
    #define USE_IMU_M5_UNIFIED
    #define OUTPUT_TO_MOTORS_DENOMINATOR 2
    //#define USE_IMU_MPU6886
    //#define IMU_I2C_SDA_PIN 21
    //#define IMU_I2C_SCL_PIN 22
    //#define IMU_INTERRUPT_PIN 35
    //#define IMU_I2C_PINS        {.sda=21,.scl=22,.irq=35}

    #define MOTORS_BALA_C
    #define MOTOR_DEADBAND_POWER (0.11F)
    #define MOTOR_SDA_PIN       0
    #define MOTOR_SCL_PIN       26
    //#define MOTOR_I2C_PINS      {.sda=0,.scl=26}

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_FIRE_BALA2)
    //#define USE_RECEIVER_TASK_TIME_BASED_SCHEDULING

    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_MPU6886
    #define IMU_I2C_PINS        {.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    #define MOTORS_BALA_2
    //#define MOTORS_HAVE_ENCODERS
    #define MOTOR_SDA_PIN       21
    #define MOTOR_SCL_PIN       22
    //#define MOTOR_I2C_PINS      {.sda=21,.scl=22}
    #define I2C_MUTEX_REQUIRED
    #define OUTPUT_TO_MOTORS_DENOMINATOR 2

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_GREY_4ENCODERMOTOR)
    #define IMU_AXIS_ORDER      IMU_Base::YNEG_XPOS_ZPOS
    #define USE_IMU_MPU6886
    //#define USE_VQF
    //#define VQF_MOTION_BIAS_ESTIMATION
    //#define IMU_I2C_SDA_PIN     21
    //#define IMU_I2C_SCL_PIN     22
    #define IMU_I2C_PINS        {.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    #define MOTORS_4_ENCODER_MOTOR
    //#define MOTORS_HAVE_ENCODERS
    #define MOTOR_SDA_PIN       21
    #define MOTOR_SCL_PIN       22
    //#define MOTOR_I2C_PINS      {.sda=21,.scl=22}
    #define I2C_MUTEX_REQUIRED
    #define OUTPUT_TO_MOTORS_DENOMINATOR 2

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_FIRE_GOPLUS2)
    #define IMU_AXIS_ORDER      IMU_Base::YNEG_XPOS_ZPOS
    #define USE_IMU_MPU6886
    //#define IMU_I2C_SDA_PIN     21
    //#define IMU_I2C_SCL_PIN     22
    #define IMU_I2C_PINS        {.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    #define MOTORS_GO_PLUS_2
    #define MOTOR_SDA_PIN       21
    #define MOTOR_SCL_PIN       22
    //#define MOTOR_I2C_PINS      {.sda=21,.scl=22}
    #define I2C_MUTEX_REQUIRED
    #define OUTPUT_TO_MOTORS_DENOMINATOR 2

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_FIRE_ODRIVE_CAN)
    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BMI270
    #define IMU_SPI_PINS        {.cs=0,.sck=0,.cipo=0,.copi=0,.irq=BUS_I2C::IRQ_NOT_SET}
    //#define IMU_I2C_SDA_PIN     21
    //#define IMU_I2C_SCL_PIN     22
    #define IMU_I2C_PINS        {.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    //#define MOTORS_HAVE_ENCODERS
    #define MOTORS_CAN_ACCURATELY_ESTIMATE_SPEED

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_FIRE_ODRIVE_TWAI)
    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_MPU6886
    //#define USE_IMU_M5_UNIFIED
    //#define IMU_I2C_SDA_PIN     21
    //#define IMU_I2C_SCL_PIN     22
    #define IMU_I2C_PINS        {.sda=21,.scl=22,.irq=BUS_I2C::IRQ_NOT_SET}

    //#define MOTORS_HAVE_ENCODERS
    #define MOTORS_CAN_ACCURATELY_ESTIMATE_SPEED

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_ATOMS3R_MOTION_BASE)
    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_BMI270
    //#define IMU_I2C_SDA_PIN     45
    //#define IMU_I2C_SCL_PIN     0
    //#define IMU_INTERRUPT_PIN   16 // pin is pulled high
    #define IMU_I2C_PINS        {.sda=45,.scl=0,.irq=16}

    #define MOTORS_ATOMIC_MOTION_BASE
    #define MOTOR_DEADBAND_POWER    (0.2F)
    #define MOTOR_SDA_PIN       38
    #define MOTOR_SCL_PIN       39
    //#define MOTOR_I2C_PINS      {.sda=38,.scl=39}

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_M5STACK_CORE2)
    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
    //#define IMU_I2C_SDA_PIN     32
    //#define IMU_I2C_SCL_PIN     33
    #define IMU_I2C_PINS        {.sda=32,.scl=33,.irq=BUS_I2C::IRQ_NOT_SET}

    #define MOTORS_GPIO
    #define MOTOR_GPIO_PINS     {.motorLeft=0,.motorRight=0,.servoLeft=0,.servoRight=0}

    #define SDCARD_SPI_PINS     spi_pins_t{.cs=4,.sck=18,.cipo=38,.copi=23,.irq=0xFF}
    #define USE_BLACKBOX

    #define USE_SCREEN
    #define USE_BUTTONS
#endif

#if defined(TARGET_PICO)
    #define IMU_AXIS_ORDER      IMU_Base::XPOS_YPOS_ZPOS
    #define USE_IMU_LSM6DS3TR_C
#if defined(LIBRARY_SENSORS_IMU_USE_SPI_BUS)
    #define IMU_SPI_INDEX       BUS_INDEX_0
    #define IMU_SPI_PINS        {.cs=17,.sck=18,.cipo=16,.copi=19,.irq=20}
#else
    #define IMU_I2C_PINS        {.sda=07,.scl=27,.irq=BUS_I2C::IRQ_NOT_SET}
#endif

    #define MOTORS_GPIO
    #define MOTOR_GPIO_PINS     {.motorLeft=0,.motorRight=0,.servoLeft=0,.servoRight=0}
#endif
