#pragma once

#include <MotorPairController.h>
#include <RadioController.h>


namespace DEFAULTS {


static constexpr MotorPairController::pidf_uint16_array_t motorPairControllerPIDs = {{
    {   0,   0,   0,   0,   0 }, // roll angle degrees
    { 100,   0,   0,   0,   0 }, // pitch angle degrees
    {   0,   0,   0,   0, 100 }, // yaw rate dps
    {   0,   0,   0,   0,   0 }, // speed serial dps
    { 100,   0,   0,   0,   0 }, // speed parallel dps
    {   0,   0,   0,   0,   0 }, // position degrees
}};

static const float balanceAngle = 0.0F;

static const RadioController::failsafe_t radioControllerFailsafe = {
    .delay = 15,
    .landing_time = 60,
    .switch_mode = 0,
    .procedure = 0,
    .throttle = 1000,
    .throttle_low_delay = 100
};


} // END namespace
