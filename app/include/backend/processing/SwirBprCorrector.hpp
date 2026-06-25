// SWIR3 software bad-pixel replacement from Specim calpack bpr/bprmap.bpr (backend/processing).
// Static calpack mask only. For SWIR3 column destripe see SwirColumnProfileCorrector.
#pragma once

#include "backend/CameraTypes.hpp"
#include "backend/processing/SwirBprCalpackMap.hpp"

#include <QString>

#include <cstddef>
#include <vector>

namespace hf::processing
{
class SwirBprCorrector
{
public:
    [[nodiscard]] bool isLoaded() const { return map_.isLoaded(); }
    [[nodiscard]] std::size_t badPixelCount() const { return map_.badPixelCount(); }
    [[nodiscard]] int fullBandCount() const { return map_.bands; }
    [[nodiscard]] int fullSampleCount() const { return map_.samples; }

    bool loadFromCalpack(const QString &calpackPath, QString *errorMessage = nullptr);

    /// In-place BPR on a BIL frame (width = spatial, height = spectral bands).
    void apply(FramePacket &frame, int spatialBinning, int spectralBinning) const;

private:
    SwirBprCalpackMap map_;
    mutable std::vector<std::uint8_t> binnedMaskScratch_;
};
} // namespace hf::processing
