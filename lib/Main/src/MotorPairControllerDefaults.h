#pragma once

#include "Targets.h"
#include <MotorPairController.h>
#include <PIDF.h>

// MOTORS_BALA_2
const MotorPairController::vehicle_t vehicleBala2 = {
    .maxMotorRPM                =  200.0F, // this is an estimate of max RPM under load
    .wheelDiameterMM            =  45.0F,
    .wheelTrackMM               =  75.0F,
    .pitchBalanceAngleDegrees   =  -2.5F,
    .motorSwitchOffAngleDegrees =  70.0F,
    .encoderStepsPerRevolution  = 420.0F
};

// MOTORS_BALA_C
const MotorPairController::vehicle_t vehicleBalaC = {
    .maxMotorRPM                = 80.0F,
    .wheelDiameterMM            = 66.5F,
    .wheelTrackMM               = 70.0F,
    .pitchBalanceAngleDegrees   = 13.0F,
    .motorSwitchOffAngleDegrees = 70.0F,
    .encoderStepsPerRevolution  =  0.0F
};

// MOTORS_4_ENCODER_MOTOR
const MotorPairController::vehicle_t vehicle4EncoderMotor = {
    .maxMotorRPM                = 170.0F,
    .wheelDiameterMM            =  68.0F,
    .wheelTrackMM               = 180.0F,
    .pitchBalanceAngleDegrees   =  -0.5F,
    .motorSwitchOffAngleDegrees =  65.0F,
    .encoderStepsPerRevolution  = 750.0F
};

// MOTORS_GO_PLUS_2 and MOTORS_ATOMIC_MOTION_BASE
const MotorPairController::vehicle_t vehicleGoPlus2 = {
    .maxMotorRPM                 = 100.0F,
    .wheelDiameterMM             =  56.0F,
    .wheelTrackMM                = 155.0F,
    .pitchBalanceAngleDegrees    =   0.0F,
    .motorSwitchOffAngleDegrees  =  70.0F,
    .encoderStepsPerRevolution   =   0.0F
};

// OTHER
const MotorPairController::vehicle_t vehicleOther = {
    .maxMotorRPM                 =  100.0F,
    .wheelDiameterMM             =   68.0F,
    .wheelTrackMM                =  170.0F,
    .pitchBalanceAngleDegrees    =    0.0F,
    .motorSwitchOffAngleDegrees  =   70.0F,
    .encoderStepsPerRevolution   = 1000.0F
};

#if defined(MOTORS_BALA_2)

const MotorPairController::vehicle_t& gVehicle = vehicleBala2;

#elif defined(MOTORS_BALA_C)

const MotorPairController::vehicle_t& gVehicle = vehicleBalaC;

#elif defined(MOTORS_4_ENCODER_MOTOR)

const MotorPairController::vehicle_t& gVehicle = vehicle4EncoderMotor;

#elif defined(MOTORS_GO_PLUS_2) || defined(MOTORS_ATOMIC_MOTION_BASE)

const MotorPairController::vehicle_t& gVehicle = vehicleGoPlus2;

#elif defined(MOTORS_ROLLER_CAN) || defined(MOTORS_PWR_CAN) || defined(MOTORS_O_DRIVE_CAN) || defined(MOTORS_O_DRIVE_TWAI) || defined(MOTORS_GPIO)

const MotorPairController::vehicle_t& gVehicle = vehicleOther;

#endif
