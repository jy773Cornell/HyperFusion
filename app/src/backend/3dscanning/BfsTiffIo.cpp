// Write uncompressed RGB8 TIFF without depending on Qt's optional TIFF plugin.
#include "backend/3dscanning/BfsTiffIo.hpp"

#include <QFile>
#include <QString>

namespace hf::bfs
{
namespace
{
void writeU16(QFile &file, const quint16 value)
{
    const char bytes[2] = {static_cast<char>(value & 0xFF),
                           static_cast<char>((value >> 8) & 0xFF)};
    file.write(bytes, 2);
}

void writeU32(QFile &file, const quint32 value)
{
    const char bytes[4] = {static_cast<char>(value & 0xFF),
                           static_cast<char>((value >> 8) & 0xFF),
                           static_cast<char>((value >> 16) & 0xFF),
                           static_cast<char>((value >> 24) & 0xFF)};
    file.write(bytes, 4);
}

void writeIfdEntry(QFile &file,
                   const quint16 tag,
                   const quint16 type,
                   const quint32 count,
                   const quint32 valueOrOffset)
{
    writeU16(file, tag);
    writeU16(file, type);
    writeU32(file, count);
    writeU32(file, valueOrOffset);
}
} // namespace

std::string saveRgb8AsTiff(const QString &path,
                           const int width,
                           const int height,
                           const std::uint8_t *rgb,
                           const std::size_t rgbBytes)
{
    if (path.isEmpty())
        return "empty path";
    if (rgb == nullptr || width <= 0 || height <= 0)
        return "invalid image dimensions";

    const auto expected =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3u;
    if (rgbBytes < expected)
        return "RGB buffer too small";

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return ("cannot open file: " + file.errorString()).toStdString();

    // Layout:
    //   0..7   TIFF header (IFD at 8)
    //   8..    IFD (12 entries) + next-IFD=0
    //   bits   BitsPerSample array (3 x SHORT)
    //   data   contiguous RGB strip
    constexpr quint16 kEntryCount = 12;
    constexpr quint32 kHeaderSize = 8;
    constexpr quint32 kIfdSize = 2 + (kEntryCount * 12) + 4;
    constexpr quint32 kBitsPerSampleOffset = kHeaderSize + kIfdSize;
    constexpr quint32 kBitsPerSampleBytes = 6;
    constexpr quint32 kStripOffset = kBitsPerSampleOffset + kBitsPerSampleBytes;
    const quint32 stripBytes = static_cast<quint32>(expected);

    // Header: little-endian TIFF
    file.write("II", 2);
    writeU16(file, 42);
    writeU32(file, kHeaderSize);

    writeU16(file, kEntryCount);
    // IFD entries must be sorted by tag number.
    writeIfdEntry(file, 256, 4, 1, static_cast<quint32>(width));  // ImageWidth
    writeIfdEntry(file, 257, 4, 1, static_cast<quint32>(height)); // ImageLength
    writeIfdEntry(file, 258, 3, 3, kBitsPerSampleOffset);         // BitsPerSample
    writeIfdEntry(file, 259, 3, 1, 1);                             // Compression=none
    writeIfdEntry(file, 262, 3, 1, 2);                             // Photometric=RGB
    writeIfdEntry(file, 273, 4, 1, kStripOffset);                  // StripOffsets
    writeIfdEntry(file, 274, 3, 1, 1);                             // Orientation
    writeIfdEntry(file, 277, 3, 1, 3);                             // SamplesPerPixel
    writeIfdEntry(file, 278, 4, 1, static_cast<quint32>(height)); // RowsPerStrip
    writeIfdEntry(file, 279, 4, 1, stripBytes);                   // StripByteCounts
    writeIfdEntry(file, 284, 3, 1, 1);                             // PlanarConfiguration
    writeIfdEntry(file, 296, 3, 1, 1);                             // ResolutionUnit
    writeU32(file, 0); // next IFD

    writeU16(file, 8);
    writeU16(file, 8);
    writeU16(file, 8);

    if (file.write(reinterpret_cast<const char *>(rgb), static_cast<qint64>(expected))
        != static_cast<qint64>(expected))
    {
        return ("write failed: " + file.errorString()).toStdString();
    }

    file.close();
    return {};
}
} // namespace hf::bfs
