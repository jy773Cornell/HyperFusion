// Converts camera frame packets into Qt images for detector views.
#pragma once

#include "backend/CameraTypes.hpp"

class QImage;

namespace ui
{
QImage framePacketToQImage(const FramePacket &frame);
} // namespace ui
