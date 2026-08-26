// SWIR3 offline residual-column BPR from white/dark refs (backend/processing).
// Detects comb columns on uniform refs, then interpolates sample + refs before FFC.
#pragma once

#include "backend/camera/processing/ReferenceBuilder.hpp"

#include <QString>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace hf::processing
{
struct SwirRefBprSettings
{
    int baselineRadius = 2;
    double whiteRatioMin = 0.88;
    double whiteRatioMax = 1.12;
    double darkAbsMinDn = 40.0;
    double darkAbsScale = 4.0;
    double columnPromoteFrac = 0.25;
};

class SwirRefBprCorrector
{
public:
    void setSettings(const SwirRefBprSettings &settings) { settings_ = settings; }

    [[nodiscard]] bool isReady() const { return ready_; }
    [[nodiscard]] std::size_t badPixelCount() const { return badPixelCount_; }
    [[nodiscard]] std::size_t badColumnCount() const { return badColumnCount_; }
    [[nodiscard]] int samples() const { return samples_; }
    [[nodiscard]] int bands() const { return bands_; }

    /// Build (band, sample) mask from averaged white/dark BIL rows. Does not use sample data.
    bool buildFromReferences(const BilRowReference &whiteRow,
                             const BilRowReference &darkRow,
                             int samples,
                             int bands,
                             QString *errorMessage = nullptr);

    void applyToFloatRow(BilRowReference &row) const;
    void applyToUint16Line(std::uint16_t *linePixels) const;

private:
    void interpolateSpatial(float *bandMajorLine) const;
    void interpolateSpatial(std::uint16_t *bandMajorLine) const;

    SwirRefBprSettings settings_;
    int samples_ = 0;
    int bands_ = 0;
    std::vector<std::uint8_t> badMask_;
    std::size_t badPixelCount_ = 0;
    std::size_t badColumnCount_ = 0;
    bool ready_ = false;
};
} // namespace hf::processing
