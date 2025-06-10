# pragma once

#include "MotorPairController.h"
#include "TelemetryScaleFactors.h"
#include <SV_TelemetryData.h>
#include <VehicleControllerTask.h>

#include <cstddef>
#include <cstdint>


size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const MotorPairController& motorPairController, const TelemetryScaleFactors& scaleFactors); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber, const VehicleControllerTask& vehicleControllerTask, const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive
