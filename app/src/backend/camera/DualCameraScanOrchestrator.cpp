// Dual-camera scan timing sync calculations (FX10e reference, SWIR3 follower).
// backend layer \u2014 used by MainWindow before preview/record and on FX10e frame-rate changes.
#include "backend/camera/DualCameraScanOrchestrator.hpp"

#include "backend/HyperFusionConfig.hpp"

namespace hf
{
bool dualCameraScanSyncEligible(const bool fx10eConnected,
                                const bool swir3Connected,
                                const bool fx10eSelected,
                                const bool swir3Selected,
                                const bool userEnabled)
{
    return userEnabled && fx10eConnected && swir3Connected && fx10eSelected && swir3Selected;
}

DualCameraScanSyncResult computeDualCameraScanSync(const DualCameraScanSyncInput &input)
{
    DualCameraScanSyncResult result;
    result.scanSpeedMmPerSec =
        recordScanSpeedMmPerSec(input.fx10eFrameRateHz, input.fx10eSpatialMmPerPixel);
    if (result.scanSpeedMmPerSec <= 0.0 || input.swir3SpatialMmPerPixel <= 0.0)
        return result;

    result.syncedSwir3FrameRateHz = result.scanSpeedMmPerSec / input.swir3SpatialMmPerPixel;
    if (result.syncedSwir3FrameRateHz <= 0.0)
        return result;

    result.valid = true;
    return result;
}
} // namespace hf
