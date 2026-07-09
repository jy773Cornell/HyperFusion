// Dual-camera scan timing sync: FX10e is reference; SWIR3 frame rate follows scan speed.
// backend layer — pure calculations, no hardware or UI dependencies.
#pragma once

namespace hf
{
struct DualCameraScanSyncInput
{
    double fx10eFrameRateHz = 0.0;
    double fx10eSpatialMmPerPixel = 0.0;
    double swir3SpatialMmPerPixel = 0.0;
};

struct DualCameraScanSyncResult
{
    bool valid = false;
    double scanSpeedMmPerSec = 0.0;
    double syncedSwir3FrameRateHz = 0.0;
};

/// True when FX10e + SWIR3 are connected, both selected for capture, and user enabled auto-sync.
bool dualCameraScanSyncEligible(bool fx10eConnected,
                                bool swir3Connected,
                                bool fx10eSelected,
                                bool swir3Selected,
                                bool userEnabled);

DualCameraScanSyncResult computeDualCameraScanSync(const DualCameraScanSyncInput &input);
} // namespace hf
