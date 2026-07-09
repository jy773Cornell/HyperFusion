// Extracts one spatial RGB scan line from a BIL hyperspectral frame (band × width).
#pragma once

#include "backend/camera/CameraTypes.hpp"

#include <cstdint>
#include <vector>

namespace ui
{
struct RgbBandIndices
{
    int red = 0;
    int green = 0;
    int blue = 0;
};

/// Maps a calibration-pack band index to a BIL row (accounts for spectral binning and frame height).
int mapCalpackBandToBilRow(int calpackBandIndex, int frameBandCount, int spectralBinning);

/// Builds one RGB888 row (width × 3 bytes) from the given band indices. Returns false if invalid.
bool extractRgbLineFromBilFrame(const FramePacket &frame,
                                const RgbBandIndices &bands,
                                std::vector<std::uint8_t> &rgbRowOut);
} // namespace ui
