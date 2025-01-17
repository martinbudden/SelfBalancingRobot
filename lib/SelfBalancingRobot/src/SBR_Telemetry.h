# pragma once

#include "MotorPairController.h"
#include "TelemetryScaleFactors.h"


int packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController, const TelemetryScaleFactors& scaleFactors); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive

