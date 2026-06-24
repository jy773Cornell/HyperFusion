// SWIR3 software bad-pixel replacement from calpack bpr/bprmap.bpr (backend/processing layer).
#include "backend/processing/SwirBprCorrector.hpp"

#include <QFile>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <optional>
#include <string>

namespace hf::processing
{
namespace
{
constexpr std::uint32_t kZipLocalFileHeaderSignature = 0x04034b50;
constexpr const char *kBprMapEntry = "bpr/bprmap.bpr";
constexpr const char *kBprHeaderEntry = "bpr/bprmap.hdr";

std::uint16_t readU16(const std::uint8_t *data)
{
    return static_cast<std::uint16_t>(data[0] | (static_cast<std::uint16_t>(data[1]) << 8));
}

std::uint32_t readU32(const std::uint8_t *data)
{
    return static_cast<std::uint32_t>(data[0])
           | (static_cast<std::uint32_t>(data[1]) << 8)
           | (static_cast<std::uint32_t>(data[2]) << 16)
           | (static_cast<std::uint32_t>(data[3]) << 24);
}

bool extractStoredZipEntry(const QByteArray &zipBytes,
                           const std::string &entryName,
                           std::string &entryBytes,
                           QString &errorMessage)
{
    std::size_t offset = 0;
    while (offset + 30 <= static_cast<std::size_t>(zipBytes.size()))
    {
        const auto *header = reinterpret_cast<const std::uint8_t *>(zipBytes.constData() + offset);
        if (readU32(header) != kZipLocalFileHeaderSignature)
        {
            errorMessage = QStringLiteral("Unsupported calibration pack format (ZIP header not found).");
            return false;
        }

        const std::uint16_t compressionMethod = readU16(header + 8);
        const std::uint32_t compressedSize = readU32(header + 18);
        const std::uint16_t fileNameLength = readU16(header + 26);
        const std::uint16_t extraFieldLength = readU16(header + 28);

        offset += 30;
        if (offset + fileNameLength > static_cast<std::size_t>(zipBytes.size()))
        {
            errorMessage = QStringLiteral("Calibration pack ZIP entry name truncated.");
            return false;
        }

        const std::string fileName(zipBytes.constData() + static_cast<int>(offset), fileNameLength);
        offset += fileNameLength;
        offset += extraFieldLength;

        if (fileName == entryName)
        {
            if (compressionMethod != 0)
            {
                errorMessage = QStringLiteral("BPR map entry uses unsupported compression.");
                return false;
            }

            if (offset + compressedSize > static_cast<std::size_t>(zipBytes.size()))
            {
                errorMessage = QStringLiteral("BPR map entry truncated inside calibration pack.");
                return false;
            }

            entryBytes.assign(zipBytes.constData() + static_cast<int>(offset),
                              zipBytes.constData() + static_cast<int>(offset + compressedSize));
            return true;
        }

        offset += compressedSize;
    }

    errorMessage = QStringLiteral("Entry %1 not found inside calibration pack.")
                       .arg(QString::fromStdString(entryName));
    return false;
}

bool parseEnviHeaderInt(const std::string &headerText, const std::string &key, int &valueOut)
{
    const std::string needle = key + " =";
    std::size_t pos = 0;
    while (pos < headerText.size())
    {
        const std::size_t lineEnd = headerText.find('\n', pos);
        const std::string line = headerText.substr(pos, lineEnd == std::string::npos ? std::string::npos
                                                                                    : lineEnd - pos);
        pos = lineEnd == std::string::npos ? headerText.size() : lineEnd + 1;

        const std::size_t keyPos = line.find(needle);
        if (keyPos == std::string::npos)
            continue;

        const std::size_t valueStart = keyPos + needle.size();
        try
        {
            valueOut = std::stoi(line.substr(valueStart));
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    return false;
}
} // namespace

bool SwirBprCorrector::loadFromCalpack(const QString &calpackPath, QString *errorMessage)
{
    loaded_ = false;
    fullBands_ = 0;
    fullSamples_ = 0;
    badPixelsFullRes_.clear();

    QFile file(calpackPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open calibration pack: %1").arg(calpackPath);
        return false;
    }

    const QByteArray zipBytes = file.readAll();
    if (zipBytes.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Calibration pack is empty: %1").arg(calpackPath);
        return false;
    }

    QString localError;
    std::string headerText;
    std::string mapBytes;
    if (!extractStoredZipEntry(zipBytes, kBprHeaderEntry, headerText, localError)
        || !extractStoredZipEntry(zipBytes, kBprMapEntry, mapBytes, localError))
    {
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return false;
    }

    int bands = 0;
    int samples = 0;
    int headerBadPixelCount = -1;
    if (!parseEnviHeaderInt(headerText, "bands", bands) || bands <= 0
        || !parseEnviHeaderInt(headerText, "samples", samples) || samples <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid BPR ENVI header in calibration pack.");
        return false;
    }

  parseEnviHeaderInt(headerText, "BadPixelCount", headerBadPixelCount);

    const std::size_t expectedBytes =
        static_cast<std::size_t>(bands) * static_cast<std::size_t>(samples) * sizeof(std::uint16_t);
    if (mapBytes.size() != expectedBytes)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("BPR map size mismatch (expected %1 bytes, got %2).")
                    .arg(static_cast<qulonglong>(expectedBytes))
                    .arg(static_cast<qulonglong>(mapBytes.size()));
        }
        return false;
    }

    badPixelsFullRes_.reserve(static_cast<std::size_t>(std::max(0, headerBadPixelCount)));
    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < samples; ++sample)
        {
            const std::size_t index =
                static_cast<std::size_t>(band) * static_cast<std::size_t>(samples)
                + static_cast<std::size_t>(sample);
            const std::uint16_t value =
                mapBytes[index * sizeof(std::uint16_t)]
                | (static_cast<std::uint16_t>(mapBytes[index * sizeof(std::uint16_t) + 1]) << 8);
            if (value != 0)
                badPixelsFullRes_.emplace_back(band, sample);
        }
    }

    if (headerBadPixelCount >= 0
        && static_cast<int>(badPixelsFullRes_.size()) != headerBadPixelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("BPR map bad-pixel count mismatch (header %1, mask %2).")
                                .arg(headerBadPixelCount)
                                .arg(static_cast<int>(badPixelsFullRes_.size()));
        }
        return false;
    }

    fullBands_ = bands;
    fullSamples_ = samples;
    loaded_ = true;
    return true;
}

void SwirBprCorrector::apply(FramePacket &frame, int spatialBinning, int spectralBinning) const
{
    if (!loaded_ || frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return;

    const int width = frame.width;
    const int bands = frame.height;
    const int spatBin = std::max(1, spatialBinning);
    const int specBin = std::max(1, spectralBinning);

    const std::size_t planeSize = static_cast<std::size_t>(width) * static_cast<std::size_t>(bands);
    if (frame.pixels.size() < planeSize)
        return;

    std::vector<std::uint8_t> badMask(planeSize, 0);
    for (const auto &[fullBand, fullSample] : badPixelsFullRes_)
    {
        const int band = fullBand / specBin;
        const int sample = fullSample / spatBin;
        if (band < 0 || band >= bands || sample < 0 || sample >= width)
            continue;

        badMask[static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
                + static_cast<std::size_t>(sample)] = 1;
    }

    const auto indexOf = [width](const int band, const int sample) {
        return static_cast<std::size_t>(band) * static_cast<std::size_t>(width)
               + static_cast<std::size_t>(sample);
    };

    const auto isBad = [&](const int band, const int sample) {
        if (band < 0 || band >= bands || sample < 0 || sample >= width)
            return true;
        return badMask[indexOf(band, sample)] != 0;
    };

    const auto nearestGoodNeighbor = [&](const int band, const int sample, const int direction)
        -> std::optional<std::uint16_t> {
        for (int delta = 1; delta < width; ++delta)
        {
            const int neighborSample = sample + direction * delta;
            if (neighborSample < 0 || neighborSample >= width)
                return std::nullopt;
            if (isBad(band, neighborSample))
                continue;
            return frame.pixels[indexOf(band, neighborSample)];
        }
        return std::nullopt;
    };

    for (int band = 0; band < bands; ++band)
    {
        for (int sample = 0; sample < width; ++sample)
        {
            if (!isBad(band, sample))
                continue;

            const std::optional<std::uint16_t> left = nearestGoodNeighbor(band, sample, -1);
            const std::optional<std::uint16_t> right = nearestGoodNeighbor(band, sample, 1);

            std::uint32_t sum = 0;
            int count = 0;
            if (left)
            {
                sum += *left;
                ++count;
            }
            if (right)
            {
                sum += *right;
                ++count;
            }

            if (count > 0)
                frame.pixels[indexOf(band, sample)] = static_cast<std::uint16_t>(sum / count);
        }
    }
}

} // namespace hf::processing
