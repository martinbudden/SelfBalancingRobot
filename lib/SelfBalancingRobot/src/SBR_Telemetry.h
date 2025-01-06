# pragma once

#include <SBR_TelemetryData.h>

#include "AHRS_Base.h"
#include "MotorPairController.h"
#include "ReceiverBase.h"


int packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id);

int packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        uint32_t ahrsTaskTickCountDelta,
        uint32_t ahrsTaskFifoCount,
        uint32_t mpcTaskTickCountDelta,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount);

int packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS_Base& ahrs, const MotorControllerBase& motorController); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_MPC(uint8_t* telemetryDataPtr, uint32_t id, const MotorPairController& motorPairController); // NOLINT(readability-avoid-const-params-in-decls) false positive

int packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, const ReceiverBase& receiver); // NOLINT(readability-avoid-const-params-in-decls) false positive
