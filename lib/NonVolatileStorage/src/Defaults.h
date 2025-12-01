#pragma once

#include <Cockpit.h>
#include <MotorPairController.h>


namespace DEFAULTS {

#if defined(TARGET_M5STACK_FIRE_BALA2)
static const float balanceAngle = 2.5F;
#elif defined(TARGET_M5STACK_STICKC_BALAC)
static const float balanceAngle = 13.0F;
#else
static const float balanceAngle = 0.0F;
#endif

static constexpr MotorPairController::pidf_uint16_array_t motorPairControllerPIDs = {{
    {   0,   0,   0,   0,   0 }, // roll angle degrees
    {  80,   0,   0,   0,   0 }, // pitch angle degrees
    {   0,   0,   0, 100,   0 }, // yaw rate dps
    {   0,   0,   0,   0,   0 }, // speed serial dps
    {   0,   0,   0,   0,   0 }, // speed parallel dps
    {   0,   0,   0,   0,   0 }, // position degrees
}};

static const Cockpit::failsafe_config_t failsafeConfig = {
    .throttle_pwm = 1000,
    .throttle_low_delay_deciseconds = 100,
    .recovery_delay_deciseconds = 5,
    .delay_deciseconds = 15,
    .landing_time_seconds = 60,
    .procedure = 0,
    .switch_mode = 0,
};

} // END namespace
