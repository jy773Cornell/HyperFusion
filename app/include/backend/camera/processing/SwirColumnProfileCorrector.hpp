// SWIR3 column destripe: median band profile along spatial axis, valley detect, column replace.
// Backend/processing layer; replaces per-pixel adaptive BPR for vertical comb artifacts.
#pragma once

#include "backend/CameraTypes.hpp"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace hf::processing
{
struct SwirColumnProfileSettings
{
    /// Median baseline window along spatial axis (±columns).
    int baselineRadius = 12;
    /// Valley when profile[x] / baseline[x] is below this (e.g. 0.88 = 12% dip).
    double valleyGainMin = 0.88;
    /// Bands below this DN are ignored when building the spatial profile.
    double minBandDn = 64.0;
    /// Consecutive valley frames before marking a column bad.
    int minConsecutiveHits = 1;
    /// Optional absolute valley depth (baseline - profile); 0 = disabled.
    double minValleyDn = 0.0;
    /// Minimum bands above minBandDn required per column.
    int minBandsPerColumn = 8;
};

class SwirColumnProfileCorrector
{
public:
    void setSettings(const SwirColumnProfileSettings &settings) { settings_ = settings; }

    [[nodiscard]] std::size_t badColumnCount() const { return badColumnCount_; }

    void resetState();

    /// Build band-median spatial profile, detect valleys, replace bad columns in-place.
    void processFrame(FramePacket &frame);

private:
    void buildSpatialProfile(const FramePacket &frame);
    void detectValleyColumns();
    void correctBadColumns(FramePacket &frame) const;

    [[nodiscard]] std::optional<double> spatialBaselineMedian(int sample, int width) const;

    SwirColumnProfileSettings settings_;
    std::vector<double> profile_;
    std::vector<std::uint8_t> columnBad_;
    std::vector<std::uint8_t> columnHits_;
    std::size_t badColumnCount_ = 0;
};
} // namespace hf::processing
