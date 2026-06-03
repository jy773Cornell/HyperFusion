// Extracts 16-bit DN profiles from BIL hyperspectral frames at a spatial/band cursor.
#pragma once

#include "core/CameraTypes.hpp"

#include <cstdint>
#include <vector>

namespace ui
{
struct ProfileCursor
{
    int spatialX = 0;
    int bandY = 0;
};

struct ProfileExtraction
{
    bool valid = false;
    ProfileCursor cursor;
    int frameWidth = 0;
    int frameBands = 0;
    std::vector<std::uint16_t> wavelengthDn;
    std::vector<std::uint16_t> spatialDn;
};

bool extractProfilesFromBilFrame(const FramePacket &frame,
                                 const ProfileCursor &cursor,
                                 ProfileExtraction &out);
} // namespace ui
