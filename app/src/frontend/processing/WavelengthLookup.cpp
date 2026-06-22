// Wavelength lookup table builder for profile plots.
#include "frontend/processing/WavelengthLookup.hpp"

namespace ui
{
std::vector<double> buildWavelengthNmLookup(const std::vector<SpectralBand> &bands, const int bandCount)
{
    if (bandCount <= 0)
        return {};

    std::vector<double> lookup(static_cast<std::size_t>(bandCount), 0.0);
    for (const SpectralBand &band : bands)
    {
        if (band.index >= 0 && band.index < bandCount)
            lookup[static_cast<std::size_t>(band.index)] = band.wavelengthNm;
    }

    int firstKnown = -1;
    int lastKnown = -1;
    for (int i = 0; i < bandCount; ++i)
    {
        if (lookup[static_cast<std::size_t>(i)] > 0.0)
        {
            if (firstKnown < 0)
                firstKnown = i;
            lastKnown = i;
        }
    }

    if (firstKnown < 0)
    {
        for (int i = 0; i < bandCount; ++i)
            lookup[static_cast<std::size_t>(i)] = static_cast<double>(i);
        return lookup;
    }

    const double wlStart = lookup[static_cast<std::size_t>(firstKnown)];
    const double wlEnd = lookup[static_cast<std::size_t>(lastKnown)];
    for (int i = 0; i < bandCount; ++i)
    {
        if (lookup[static_cast<std::size_t>(i)] > 0.0)
            continue;

        const double t = (bandCount > 1)
                             ? static_cast<double>(i) / static_cast<double>(bandCount - 1)
                             : 0.0;
        lookup[static_cast<std::size_t>(i)] = wlStart + (wlEnd - wlStart) * t;
    }

    return lookup;
}

const std::vector<double> &cachedWavelengthNmLookup(const std::vector<SpectralBand> &bands,
                                                    const int frameBandCount,
                                                    std::vector<double> &cacheStorage,
                                                    int &cacheBandCount)
{
    if (frameBandCount != cacheBandCount || cacheStorage.empty())
    {
        cacheStorage = buildWavelengthNmLookup(bands, frameBandCount);
        cacheBandCount = frameBandCount;
    }
    return cacheStorage;
}
} // namespace ui
