#pragma once

namespace zaber_stage
{
// HyperFusion scan stage — fixed hardware (X-MCC2 + 2× NMS23-E08P1T3A on LC40B).
// After lockstep is enabled, command motion through the primary axis only (axis 1).
// Secondary axis 2 follows via lockstep group 1.
constexpr int kDeviceAddress = 1;
constexpr int kLockstepGroupId = 1;
constexpr int kPrimaryAxisNumber = 1;
constexpr int kSecondaryAxisNumber = 2;
constexpr int kAxisCount = 2;

constexpr double kTravelLengthMm = 2000.0;
constexpr double kTravelMinimumMm = 0.0;
// All HyperFusion commanded motion uses this speed except Capture scanning (user-set)
// and the final homing creep (kHomingApproachSpeedMmPerSec).
constexpr double kMaxSpeedMmPerSec = 100.0;
/// Axis accel + per-move Lockstep acceleration (mm/s²). Lower = gentler start/stop.
/// ZML: accel setting; move options acceleration=0 falls back to this value.
constexpr double kDefaultMotionAccelerationMmPerSec2 = 30.0;
/// When the axis is already referenced, cruise at kMaxSpeedMmPerSec to this
/// standoff, then search the home sensor at kHomingApproachSpeedMmPerSec.
constexpr double kHomingApproachDistanceMm = 80.0;
/// Firmware lockstep.home() follows axis maxspeed; this is the sensor-approach cap.
constexpr double kHomingApproachSpeedMmPerSec = 20.0;
constexpr double kHomeOffsetMm = 0.0;
constexpr double kLockstepSecondaryOffsetMm = 0.0;

constexpr const char *kExpectedControllerPrefix = "X-MCC2";
constexpr const char *kExpectedPeripheralName = "NMS23-E08P1T3A";
constexpr const char *kStageType = "LC40B";
} // namespace zaber_stage
