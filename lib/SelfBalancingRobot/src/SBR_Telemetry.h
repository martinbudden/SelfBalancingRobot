# pragma once

#include <SV_TelemetryData.h>

#include "AHRS.h"
#include "MotorPairController.h"
#include "ReceiverBase.h"
#include "TelemetryScaleFactors.h"


int packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id);

int packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        const AHRS& ahrs, // NOLINT(readability-avoid-const-params-in-decls)
        const MotorControllerBase& motorController,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController, const TelemetryScaleFactors& scaleFactors); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS& ahrs, const MotorControllerBase& motorController); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, const ReceiverBase& receiver); // NOLINT(readability-avoid-const-params-in-decls) false positive
