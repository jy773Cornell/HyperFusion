// Write uncompressed RGB8 TIFF (backend/multiview). No Qt TIFF plugin required.
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

class QString;

namespace hf::bfs
{
/// Writes a little-endian uncompressed RGB TIFF (SamplesPerPixel=3, 8 bits).
/// @return empty string on success, otherwise an error message.
[[nodiscard]] std::string saveRgb8AsTiff(const QString &path,
                                         int width,
                                         int height,
                                         const std::uint8_t *rgb,
                                         std::size_t rgbBytes);
} // namespace hf::bfs
