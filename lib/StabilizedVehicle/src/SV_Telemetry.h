#pragma once

#include <cstddef>
#include <cstdint>

class AHRS;
class AHRS_Task;
class VehicleControllerBase;


size_t packTelemetryData_Minimal(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber);

size_t packTelemetryData_TaskIntervals(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const TaskBase& ahrsTask, // NOLINT(readability-avoid-const-params-in-decls) false positive
        const TaskBase& vehicleControllerTask,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_TaskIntervalsExtended(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
        const AHRS& ahrs, // NOLINT(readability-avoid-const-params-in-decls) false positive
        const VehicleControllerBase& vehicleController,
        uint32_t mainTaskTickCountDelta,
        uint32_t transceiverTickCountDelta,
        uint32_t receiverDroppedPacketCount); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_AHRS(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
    const AHRS& ahrs,
    const VehicleControllerBase& vehicleController); // NOLINT(readability-avoid-const-params-in-decls) false positive

size_t packTelemetryData_PID(uint8_t* telemetryDataPtr, uint32_t id, uint32_t sequenceNumber,
    const VehicleControllerBase& vehicleController,
    uint8_t controlMode,
    float f0,
    float f1); // NOLINT(readability-avoid-const-params-in-decls) false positive
