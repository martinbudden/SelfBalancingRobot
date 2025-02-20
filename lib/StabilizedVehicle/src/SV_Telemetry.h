#pragma once

#include <cstddef>
#include <cstdint>

class AHRS;
class ReceiverBase;


size_t packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id);

size_t packTelemetryData_TickIntervals(uint8_t* telemetryDataPtr, uint32_t id,
        const AHRS& ahrs, // NOLINT(readability-avoid-const-params-in-decls)
        const MotorControllerBase& motorController,
        uint32_t mcOutputPowerTimeMicroSeconds,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, const AHRS& ahrs, const MotorControllerBase& motorController); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_Receiver(uint8_t* telemetryDataPtr, uint32_t id, const ReceiverBase& receiver); // NOLINT(readability-avoid-const-params-in-decls) false positive

