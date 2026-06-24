// SWIR3 stream-adaptive BPR: calpack baseline + runtime outlier detection (backend/processing).
// When enabled, replaces SDK Camera.BPR. Detects spatial gain outliers per band and grows the
// bad-pixel mask after consecutive hits. Calpack pixels are always kept.
#pragma once

#include "backend/CameraTypes.hpp"
#include "backend/processing/SwirBprCalpackMap.hpp"

#include <QString>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace hf::processing
{
struct SwirAdaptiveBprSettings
{
    /// Low/high neighbor ratio thresholds (Specim BPR.CreateMap defaults).
    double gainMin = 0.3;
    double gainMax = 1.5;
    /// Minimum neighbor mean DN to trust ratio test.
    double minNeighborMeanDn = 64.0;
    /// Consecutive outlier frames before adding to adaptive mask.
    int minConsecutiveHits = 3;
    /// Cap on adaptive-only pixels (safety).
    int maxAdaptivePixels = 4096;
};

class SwirAdaptiveBprCorrector
{
public:
    void setSettings(const SwirAdaptiveBprSettings &settings) { settings_ = settings; }

    [[nodiscard]] bool isLoaded() const { return baseline_.isLoaded(); }
    [[nodiscard]] std::size_t baselineBadPixelCount() const { return baseline_.badPixelCount(); }
    [[nodiscard]] std::size_t adaptiveBadPixelCount() const { return adaptiveBadPixelCount_; }
    [[nodiscard]] std::size_t activeBadPixelCount() const
    {
        return baselineBadPixelCount() + adaptiveBadPixelCount_;
    }

    bool loadFromCalpack(const QString &calpackPath, QString *errorMessage = nullptr);

    /// Detect outliers, update adaptive mask, apply in-place BPR.
    void processFrame(FramePacket &frame, int spatialBinning, int spectralBinning);

    void resetAdaptiveState();

private:
    [[nodiscard]] bool isBaselineBadFullRes(int fullBand, int fullSample) const;
    [[nodiscard]] bool isAdaptiveBadFullRes(int fullBand, int fullSample) const;
    void markAdaptiveFullRes(int fullBand, int fullSample);
    void markAdaptiveBinCell(int band, int sample, int spatialBinning, int spectralBinning);
    void detectAndUpdate(const FramePacket &frame, int spatialBinning, int spectralBinning);
    void buildActiveMaskBinned(int width, int bands, int spatialBinning, int spectralBinning);

    SwirAdaptiveBprSettings settings_;
    SwirBprCalpackMap baseline_;
    std::vector<std::uint8_t> adaptiveMask_;
    std::vector<std::uint8_t> hitCounts_;
    std::vector<std::uint8_t> activeMaskBinned_;
    std::size_t adaptiveBadPixelCount_ = 0;
};
} // namespace hf::processing
