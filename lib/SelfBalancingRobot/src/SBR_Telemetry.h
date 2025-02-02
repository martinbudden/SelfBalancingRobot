# pragma once

#include "MotorPairController.h"
#include "TelemetryScaleFactors.h"


size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController, const TelemetryScaleFactors& scaleFactors); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive

