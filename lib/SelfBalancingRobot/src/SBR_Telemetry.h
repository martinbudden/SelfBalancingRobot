# pragma once

#include "MotorPairController.h"


size_t packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
    const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive
