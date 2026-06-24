// SWIR3 software bad-pixel replacement from Specim calpack bpr/bprmap.bpr (backend/processing).
// Loads a BIL uint16 mask (1 = bad) at full sensor resolution and replaces bad pixels per frame
// using spatial neighbors at the same spectral band. SDK Camera.BPR stays off when this is active.
#pragma once

#include "backend/CameraTypes.hpp"

#include <QString>

#include <cstddef>
#include <utility>
#include <vector>

namespace hf::processing
{
class SwirBprCorrector
{
public:
    [[nodiscard]] bool isLoaded() const { return loaded_; }
    [[nodiscard]] std::size_t badPixelCount() const { return badPixelsFullRes_.size(); }
    [[nodiscard]] int fullBandCount() const { return fullBands_; }
    [[nodiscard]] int fullSampleCount() const { return fullSamples_; }

    bool loadFromCalpack(const QString &calpackPath, QString *errorMessage = nullptr);

    /// In-place BPR on a BIL frame (width = spatial, height = spectral bands).
    void apply(FramePacket &frame, int spatialBinning, int spectralBinning) const;

private:
    bool loaded_ = false;
    int fullBands_ = 0;
    int fullSamples_ = 0;
    std::vector<std::pair<int, int>> badPixelsFullRes_;
};
} // namespace hf::processing
