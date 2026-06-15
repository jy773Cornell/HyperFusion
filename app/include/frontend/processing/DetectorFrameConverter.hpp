// Converts camera frame packets into Qt images for detector views (RGB with saturation marks).
#pragma once

#include "backend/CameraTypes.hpp"

class QImage;

namespace ui
{
QImage framePacketToQImage(const FramePacket &frame);
} // namespace ui
