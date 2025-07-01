# TODOs

List of potential TODOs for Self Balancing Robot. Not all of these will be implemented.

1. Implement variant that uses a semaphore that is set in IMU data ready interrupt.
2. Get ODrive versions going.
3. Get TWAI versions going.
4. Add setGain function to Madgwick filter.
5. Get AtomS3R working in interrupt driven mode
6. Get SPI bus working

## Done

1. Write driver for BMI270 IMU.
2. Implement AtomS3R variant, uses BMI270.
    AtomS3R has IMU using pins G0 and G45, whereas
    AtomMotionBase attaches to pins G38 and G39, so no I2C mutex is required.
    It also has the interrupt pin connected.
3. Write driver for LSM6DSOX IMU.
4. Implement and test single core version.
5. Implement version for Raspberry Pi Pico.
6. Investigate using microsecond timer for calculating `deltaT`.

## Don't Do

1. Implement and test variant using IMU FIFO with increased update rate.
2. Rename MotorPairController to VehicleController
3. Ramp up beta on Madgwick filter.

## IMUs

| Board                          | Processor               | Clock | RAM   | Flash     | IMU         |
| ------------------------------ | ----------------------- | ----- | ----- | --------- | ----------- |
| Seeed XIAO MG24 Sense          | ARM Cortex-M33          | 78MHz | 256kB | 1.5MB/4MB | LSM6DS3TR-C |
| Seeed XIAO nRF52840 Sense/Plus | nRF52840 ARM Cortex-M4F | 64MHz | 256KB | 1MB/2MB   | LSM6DS3TR-C |
| BBC micro:bit                  | nRF52833 ARM Cortex M4F | 64MHz | 128KB | 512 KB    | LSM303AGR - acc and magnetometer only |

## Seeed XIAO nRF52840 Sense/Plus

```cpp
// Wire Interfaces
#define WIRE_INTERFACES_COUNT   (2)

#define PIN_WIRE_SDA            (4)
#define PIN_WIRE_SCL            (5)

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_WIRE1_SDA           (17)
#define PIN_WIRE1_SCL           (16)
#define PIN_LSM6DS3TR_C_POWER   (15)
#define PIN_LSM6DS3TR_C_INT1    (18)
```

## Library release order

### Add to registry.platformio.org

None

### Moved to github

| Library                | On github | tag   | release | platformio | Arduino |
| ---------------------- | --------- | ----- | ------- | ---------- | ------- |
| IMU                    | yes       | 0.8.3 | 0.8.3   | yes        | no      |
| VectorQuaternionMatrix | yes       | 0.3.0 | 0.3.0   | yes        | no      |
| Filters                | yes       | 0.5.0 | 0.5.0   | yes        | no      |
| PIDF                   | yes       | 0.2.1 | 0.2.1   | yes        | no      |
| SensorFusion           | yes       | 0.2.1 | 0.2.0   | yes        | no      |
| Receiver               | yes       | 0.3.4 | 0.3.4   | yes        | no      |
| StabilizedVehicle      | yes       | 0.0.2 | 0.0.2   | yes        | no      |
| StreamBuf              | yes       | 0.0.1 | 0.0.1   | yes        | no      |
| TaskBase               | yes       | 0.0.2 | 0.0.2   | yes        | no      |
| MultiWiiSerialProtocol | no        | 0.0.1 | 0.0.1   | no         | no      |
| AtomJoyStickReceiver   | yes       | 0.0.1 | 0.0.1   | no         | no      |

Libraries
├── Filters @ 0.5.0
├── PIDF @ 0.2.1
├── Receiver @ 0.3.4
├── VectorQuaternionMatrix @ 0.3.0
├── IMU @ 0.8.3
│   └── VectorQuaternionMatrix @ 0.3.0
├── SensorFusion @ 0.2.1
│   └── VectorQuaternionMatrix @ 0.3.0
├── StabilizedVehicle @ 0.0.2
│   ├── TaskBase @ 0.0.2
│   ├── Filters @ 0.5.0
│   ├── PIDF @ 0.2.1
│   ├── Receiver @ 0.3.2
│   ├── VectorQuaternionMatrix @ 0.3.0
│   ├── IMU @ 0.8.3
│   │   └── VectorQuaternionMatrix @ 0.3.0
│   └── SensorFusion @ 0.2.1
│       └── VectorQuaternionMatrix @ 0.3.0
└── SelfBalancingRobot @ 0.0.1
    └── StabilizedVehicle @ 0.0.2

├── MultiWiiSerialProtocol @ 0.0.1
    ├── StreamBuf @ 0.0.1
    └── VectorQuaternionMatrix @ 0.3.0

├── Blackbox @ 0.0.1
    └── StreamBuf @ 0.0.1

To add a library to the Arduino Library manager, make a pull request [here](https://github.com/arduino/library-registry)

pio pkg list - checks has been published
To add a library to platformio, type "pio pkg publish", once logged into platformio account

```sh
git tag -a 0.0.1 -m"Initial release, 0.0.1"
git tag -a 0.0.2 -m"Release 0.0.2"
git tag -a 0.1.1 -m"Release 0.1.1"
git push origin 0.1.1

git tag -d 0.0.1
git push origin --delete 0.0.1
```

### To remain in this repository

1. MotorPairs
2. SBR_Main
3. SelfBalancingRobot

### Memory usage

M5Stack-Fire-Bala2-SpeedOptimized\firmware.elf
RAM:   [          ]   1.3% (used 57892 bytes from 4521984 bytes)
Flash: [=         ]  14.3% (used 939753 bytes from 6553600 bytes)

M5Stack-Fire-Bala2-SizeOptimized\firmware.elf
RAM:   [          ]   1.3% (used 57892 bytes from 4521984 bytes)
Flash: [=         ]  14.2% (used 928317 bytes from 6553600 bytes)
B

```text
[env:pico]
platform = raspberrypi
framework = arduino
board = pico
check_src_filters =
    ${env.check_src_filters}
lib_deps =
    ${env.lib_deps}
build_unflags =
    -D USE_I2C_BEGIN_2_PARAMETERS
    -D USE_FREERTOS
    -D USE_ARDUINO_ESP32
    -D USE_ESPNOW
    -D USE_ARDUINO_ESP32_PREFERENCES
    -D USE_FAST_RECIPROCAL_SQUARE_ROOT
    -D USE_AHRS_DATA_CRITICAL_SECTION
    -D AHRS_RECORD_TIMES_CHECKS
    -D BACKCHANNEL_MAC_ADDRESS={0xC0,0x4E,0x30,0x11,0x9D,0x60}
    -D USE_SCREEN
    -D USE_BUTTONS
build_flags =
    ${env.build_flags}
    -Wno-error
    -Wno-attributes
    -Wno-implicit-fallthrough
    -Wno-sign-compare
    ;-Wcast-align
    ;-Wconversion
    ;-Wfloat-conversion
    ;-Winline
    ;-Wpacked
    ;-Wpadded
    ;-Wshadow
    ;-D FRAMEWORK_RPI_PICO
    -D BUILD_PLATFORM_${PIOPLATFORM}
    -D MOTORS_GPIO
    -D MOTOR_GPIO_PINS={.motorLeft=0,.motorRight=0,.servoLeft=0,.servoRight=0}
    -D IMU_AXIS_ORDER=IMU_Base::XPOS_YPOS_ZPOS
    -D IMU_BUILD_XPOS_YPOS_ZPOS
    -D USE_IMU_LSM6DS3TR_C_I2C
    -D IMU_I2C_SDA_PIN=PICO_DEFAULT_I2C_SDA_PIN
    -D IMU_I2C_SCL_PIN=PICO_DEFAULT_I2C_SCL_PIN
    -D IMU_SPI_CS_PIN=PICO_DEFAULT_SPI_CSN_PIN
    -D IMU_SPI_CIPO_PIN=PICO_DEFAULT_SPI_RX_PIN
    -D IMU_SPI_COPI_PIN=PICO_DEFAULT_SPI_TX_PIN
    -D IMU_SPI_SCK_PIN=PICO_DEFAULT_SPI_SCK_PIN
```

```text
[env:Pico]
platform = raspberrypi
framework = arduino
board = pico
board_build.f_cpu = 200000000L ; max certified value
lib_deps =
    ${env.lib_deps}
build_unflags =
    -Os
    -D AHRS_RECORD_TIMES_CHECKS
    -D USE_FAST_RECIPROCAL_SQUARE_ROOT
build_flags =
    -O2
    ;-save-temps=obj
    -Werror
    -Wall
    -Wextra
    -Wattributes
    -Wdisabled-optimization
    -Wdouble-promotion
    -Wimplicit-fallthrough
    -Winline
    -Wnarrowing
    -Wsign-compare
    -Wtrampolines
    -Wunreachable-code
    -Wno-attributes
    -Wno-cast-align
    -Wno-conversion
    -Wno-float-conversion
    -Wno-ignored-qualifiers
    -Wno-implicit-fallthrough
    -Wno-missing-noreturn
    -Wno-packed
    -Wno-padded
    -Wno-shadow
    -Wno-sign-conversion
    -Wno-sign-compare
    -Wno-vla
    -D FRAMEWORK_ARDUINO
    -D USE_MADGWICK_FILTER
    -D USE_AHRS_DATA_CRITICAL_SECTION
    -D AHRS_RECORD_TIMES_CHECKS
    -D PICO_USE_FASTEST_SUPPORTED_CLOCK=1
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D BUILD_PLATFORM_${PIOPLATFORM}
    -D MOTORS_GPIO
    -D MOTOR_GPIO_PINS={.motorLeft=0,.motorRight=0,.servoLeft=0,.servoRight=0}
    -D IMU_AXIS_ORDER=IMU_Base::XPOS_YPOS_ZPOS
    -D IMU_BUILD_XPOS_YPOS_ZPOS
    -D USE_IMU_LSM6DS3TR_C_SPI
    -D IMU_SPI_CS_PIN=17
    -D IMU_SPI_SCK_PIN=18
    -D IMU_SPI_CIPO_PIN=16
    -D IMU_SPI_COPI_PIN=19
    -D IMU_SPI1_CS_PIN=13
    -D IMU_SPI1_SCK_PIN=14
    -D IMU_SPI1_CIPO_PIN=12
    -D IMU_SPI1_COPI_PIN=15
```

```text
[env:CodeCellEspIdf]
platform = espressif32
framework = espidf
board = seeed_xiao_esp32c3
check_src_filters =
    ${env.check_src_filters}
lib_deps =
    ${env.lib_deps}
build_unflags =
    -D FRAMEWORK_ARDUINO
    -D USE_ARDUINO_ESP32_PREFERENCES
    -D USE_ARDUINO_ESP32
    -D BACKCHANNEL_MAC_ADDRESS
    -D USE_ESPNOW
    -D USE_SCREEN
    -D USE_BUTTONS
build_flags =
    ${env.build_flags}
    -Wno-attributes
    -Wno-double-promotion
    ;-Wcast-align
    ;-Wconversion
    ;-Wfloat-conversion
    ;-Winline
    ;-Wpacked
    ;-Wpadded
    ;-Wshadow
    -D FRAMEWORK_ESPIDF
    -D ARDUINO_USB_MODE=1
    -D ARDUINO_USB_CDC_ON_BOOT=1
    -D MOTORS_GPIO
    -D MOTOR_GPIO_PINS={.motorLeft=0,.motorRight=0,.servoLeft=0,.servoRight=0}
    -D IMU_AXIS_ORDER=IMU_Base::XPOS_YPOS_ZPOS
    -D IMU_BUILD_XPOS_YPOS_ZPOS
    -D USE_IMU_BNO085_I2C
    -D IMU_DOES_SENSOR_FUSION
    -D IMU_I2C_SDA_PIN=8
    -D IMU_I2C_SCL_PIN=9
```

```text
# Axis mapping

NO0,8X_physical_axis_aligned_Mapping_quaternion};
X_Y_Z_Qw_Qx_Qy_Qz};

XPOS_YPOS_ZPOS {1,0,0,0};
YPOS_XNEG_ZPOS {(√2)/2,0,0,(√2)/2};
XNEG_YNEG_ZPOS {0,0,0,1};
YNEG_XPOS_ZPOS {(√2)/2,0,0,-(√2)/2};
XPOS_YNEG_ZNEG {0,0,-1,0};
YPOS_XPOS_ZNEG {0,-(√2)/2,-(√2)/2,0};
XNEG_YPOS_ZNEG {0,-1,0,0};
YNEG_XNEG_ZNEG {0,-(√2)/2,(√2)/2,0};
ZPOS_YNEG_XPOS {0,0,-(√2)/2,(√2)/2};
YPOS_ZPOS_XPOS {1/2,-1/2,-1/2,1/2};
ZNEG_YPOS_XPOS {(√2)/2,-(√2)/2,0,0};
YNEG_ZNEG_XPOS {1/2,-1/2,1/2,-1/2};
ZPOS_YPOS_XNEG {(√2)/2,-(√2)/2,0,0};
YPOS_ZNEG_XNEG {-1/2,-1/2,-1/2,-1/2};
ZNEG_YNEG_XNEG {0,0,-(√2)/2,-(√2)/2};
YNEG_ZPOS_XNEG {1/2,1/2,-1/2,-1/2};
ZPOS_XPOS_YPOS {-1/2,-1/2,-1/2,1/2};
XNEG_ZPOS_YPOS {0,-(√2)/2,0,(√2)/2};
ZNEG_XNEG_YPOS {1/2,-1/2,1/2,1/2};
XPOS_ZNEG_YPOS {-(√2)/2,0,-(√2)/2,0};
ZPOS_XNEG_YNEG {1/2,1/2,-1/2,1/2};
XNEG_ZNEG_YNEG {0,-(√2)/2,0,-(√2)/2};
ZNEG_XPOS_YNEG {1/2,-1/2,-1/2,-1/2};
XPOS_ZPOS_YNEG {(√2)/2,0,-(√2)/2,0};
```

See https://github.com/daveythacher/RP2040_SKELETON/blob/main/Code/CMakeLists.txt
