#pragma once

namespace zaber_stage
{
// HyperFusion scan stage — fixed hardware (X-MCC2 + 2× NMS23-E08P1T3A on LC40B).
constexpr int kDeviceAddress = 1;
constexpr int kLockstepGroupId = 1;
constexpr int kPrimaryAxisNumber = 1;
constexpr int kSecondaryAxisNumber = 2;
constexpr int kAxisCount = 2;

constexpr double kTravelLengthMm = 2000.0;
constexpr double kTravelMinimumMm = 0.0;

constexpr const char *kExpectedControllerPrefix = "X-MCC2";
constexpr const char *kExpectedPeripheralName = "NMS23-E08P1T3A";
constexpr const char *kStageType = "LC40B";
} // namespace zaber_stage
