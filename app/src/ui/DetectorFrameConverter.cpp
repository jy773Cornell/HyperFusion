// Converts 16-bit camera frame packets to 8-bit grayscale QImages for Qt detector panes.
// Applies per-frame min–max scaling; does not touch hardware or SDK state.
#include "ui/DetectorFrameConverter.hpp"

#include <QImage>

#include <algorithm>
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

    std::uint16_t minValue = frame.pixels[0];
    std::uint16_t maxValue = frame.pixels[0];
    for (std::size_t i = 1; i < pixelCount; ++i)
    {
        minValue = std::min(minValue, frame.pixels[i]);
        maxValue = std::max(maxValue, frame.pixels[i]);
    }

    QImage image(width, height, QImage::Format_Grayscale8);
    if (maxValue == minValue)
    {
        image.fill(0);
        return image;
    }

    const double scale = 255.0 / static_cast<double>(maxValue - minValue);
    for (int y = 0; y < height; ++y)
    {
        auto *scanLine = image.scanLine(y);
        for (int x = 0; x < width; ++x)
        {
            const std::uint16_t value = frame.pixels[static_cast<std::size_t>(y * width + x)];
            scanLine[x] = static_cast<unsigned char>((value - minValue) * scale);
        }
    }

    return image;
}
} // namespace ui
