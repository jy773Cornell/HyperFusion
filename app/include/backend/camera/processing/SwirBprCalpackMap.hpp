// Loads SWIR3 BPR mask (bpr/bprmap.bpr) from a Specim .scp calpack (backend/processing).
#pragma once

#include <QString>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace hf::processing
{
struct SwirBprCalpackMap
{
    int bands = 0;
    int samples = 0;
    std::vector<std::uint8_t> mask;

    [[nodiscard]] bool isLoaded() const { return bands > 0 && samples > 0 && !mask.empty(); }
    [[nodiscard]] bool hasGeometry() const { return bands > 0 && samples > 0; }
    [[nodiscard]] std::size_t badPixelCount() const;
    [[nodiscard]] std::size_t pixelCount() const
    {
        return static_cast<std::size_t>(bands) * static_cast<std::size_t>(samples);
    }

    bool loadFromCalpack(const QString &calpackPath, QString *errorMessage = nullptr);
    /// Bands/samples from calpack BPR header only (no static mask).
    bool loadGeometryFromCalpack(const QString &calpackPath, QString *errorMessage = nullptr);
};
} // namespace hf::processing
