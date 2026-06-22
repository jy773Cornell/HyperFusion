// Builds and caches wavelength (nm) lookup tables for profile plot X axes.
#pragma once

#include "adapters/lumo/CalpackBandCatalog.hpp"

#include <vector>

namespace ui
{
std::vector<double> buildWavelengthNmLookup(const std::vector<SpectralBand> &bands, int bandCount);

/// Returns cached lookup; rebuilds only when band metadata or frame band count changes.
const std::vector<double> &cachedWavelengthNmLookup(const std::vector<SpectralBand> &bands,
                                                    int frameBandCount,
                                                    std::vector<double> &cacheStorage,
                                                    int &cacheBandCount);
} // namespace ui
