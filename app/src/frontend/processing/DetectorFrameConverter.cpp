// Converts 16-bit camera frame packets to Qt images for detector panes.
// Saturated Mono12 pixels are shown in red; other pixels use fixed DN scaling.
#include "frontend/processing/DetectorFrameConverter.hpp"

#include "frontend/processing/Overexposure.hpp"

#include <QImage>

#include <cstdint>

namespace ui
{
QImage framePacketToQImage(const FramePacket &frame)
{
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return {};

    const int width = frame.width;
    const int height = frame.height;
    const std::size_t pixelCount = static_cast<std::size_t>(width * height);
    if (frame.pixels.size() < pixelCount)
        return {};

    QImage image(width, height, QImage::Format_RGB888);
    for (int y = 0; y < height; ++y)
    {
        auto *scanLine = reinterpret_cast<std::uint8_t *>(image.scanLine(y));
        for (int x = 0; x < width; ++x)
        {
            const std::uint16_t value =
                frame.pixels[static_cast<std::size_t>(y * width + x)];
            const std::size_t base = static_cast<std::size_t>(x) * 3;
            if (isOverexposedDn(value))
            {
                scanLine[base + 0] = 255;
                scanLine[base + 1] = 0;
                scanLine[base + 2] = 0;
            }
            else
            {
                const std::uint8_t gray = dnToDisplayGray(value);
                scanLine[base + 0] = gray;
                scanLine[base + 1] = gray;
                scanLine[base + 2] = gray;
            }
        }
    }

    return image;
}
} // namespace ui
