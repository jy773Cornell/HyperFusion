// Loads SWIR3 BPR mask from Specim calpack ZIP entries (backend/processing).
#include "backend/camera/processing/SwirBprCalpackMap.hpp"

#include <QFile>

#include <algorithm>
#include <cstdint>
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

std::size_t SwirBprCalpackMap::badPixelCount() const
{
    return static_cast<std::size_t>(
        std::count(mask.begin(), mask.end(), static_cast<std::uint8_t>(1)));
}

bool SwirBprCalpackMap::loadGeometryFromCalpack(const QString &calpackPath, QString *errorMessage)
{
    bands = 0;
    samples = 0;
    mask.clear();

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
    if (!extractStoredZipEntry(zipBytes, kBprHeaderEntry, headerText, localError))
    {
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return false;
    }

    int headerBands = 0;
    int headerSamples = 0;
    if (!parseEnviHeaderInt(headerText, "bands", headerBands) || headerBands <= 0
        || !parseEnviHeaderInt(headerText, "samples", headerSamples) || headerSamples <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid BPR ENVI header in calibration pack.");
        return false;
    }

    bands = headerBands;
    samples = headerSamples;
    return true;
}

bool SwirBprCalpackMap::loadFromCalpack(const QString &calpackPath, QString *errorMessage)
{
    bands = 0;
    samples = 0;
    mask.clear();

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

    int headerBands = 0;
    int headerSamples = 0;
    int headerBadPixelCount = -1;
    if (!parseEnviHeaderInt(headerText, "bands", headerBands) || headerBands <= 0
        || !parseEnviHeaderInt(headerText, "samples", headerSamples) || headerSamples <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid BPR ENVI header in calibration pack.");
        return false;
    }

    parseEnviHeaderInt(headerText, "BadPixelCount", headerBadPixelCount);

    const std::size_t expectedBytes =
        static_cast<std::size_t>(headerBands) * static_cast<std::size_t>(headerSamples)
        * sizeof(std::uint16_t);
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

    mask.assign(static_cast<std::size_t>(headerBands) * static_cast<std::size_t>(headerSamples), 0);
    for (int band = 0; band < headerBands; ++band)
    {
        for (int sample = 0; sample < headerSamples; ++sample)
        {
            const std::size_t index =
                static_cast<std::size_t>(band) * static_cast<std::size_t>(headerSamples)
                + static_cast<std::size_t>(sample);
            const std::uint16_t value =
                static_cast<std::uint8_t>(mapBytes[index * 2])
                | (static_cast<std::uint16_t>(static_cast<std::uint8_t>(mapBytes[index * 2 + 1])) << 8);
            if (value != 0)
                mask[index] = 1;
        }
    }

    if (headerBadPixelCount >= 0
        && static_cast<int>(badPixelCount()) != headerBadPixelCount)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("BPR map bad-pixel count mismatch (header %1, mask %2).")
                                .arg(headerBadPixelCount)
                                .arg(static_cast<int>(badPixelCount()));
        }
        return false;
    }

    bands = headerBands;
    samples = headerSamples;
    return true;
}

} // namespace hf::processing
