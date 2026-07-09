// In-place BIL bad-pixel replacement for SWIR3 frames (backend/processing).
#pragma once

#include "backend/camera/CameraTypes.hpp"

#include <cstdint>
#include <vector>

namespace hf::processing
{
void applySwirBilBprInPlace(FramePacket &frame,
                            int spatialBinning,
                            int spectralBinning,
                            const std::vector<std::uint8_t> &badMaskBinned);
} // namespace hf::processing
