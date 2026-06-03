// Parses Specim .scp calibration packs (ZIP) to build spectral band dropdown lists.
// Extracts spectral/wlcal1b.wls (stored, uncompressed) and maps band index → wavelength (nm).
#include "adapters/lumo/CalpackBandCatalog.hpp"

#include <QFile>

#include <cstdint>
#include <sstream>
#include <string>

namespace
{
constexpr std::uint32_t kZipLocalFileHeaderSignature = 0x04034b50;
const char *wavelengthTableEntryForBinning(const int spectralBinning)
{
    switch (spectralBinning)
    {
    case 2:
        return "spectral/wlcal2b.wls";
    case 4:
        return "spectral/wlcal4b.wls";
    case 8:
        return "spectral/wlcal8b.wls";
    case 1:
    default:
        return "spectral/wlcal1b.wls";
    }
}

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
                           std::string &errorMessage)
{
    std::size_t offset = 0;
    while (offset + 30 <= static_cast<std::size_t>(zipBytes.size()))
    {
        const auto *header = reinterpret_cast<const std::uint8_t *>(zipBytes.constData() + offset);
        if (readU32(header) != kZipLocalFileHeaderSignature)
        {
            errorMessage = "Unsupported calibration pack format (ZIP header not found).";
            return false;
        }

        const std::uint16_t compressionMethod = readU16(header + 8);
        const std::uint32_t compressedSize = readU32(header + 18);
        const std::uint16_t fileNameLength = readU16(header + 26);
        const std::uint16_t extraFieldLength = readU16(header + 28);

        offset += 30;
        if (offset + fileNameLength > static_cast<std::size_t>(zipBytes.size()))
        {
            errorMessage = "Calibration pack ZIP entry name truncated.";
            return false;
        }

        const std::string fileName(zipBytes.constData() + static_cast<int>(offset), fileNameLength);
        offset += fileNameLength;
        offset += extraFieldLength;

        if (fileName == entryName)
        {
            if (compressionMethod != 0)
            {
                errorMessage = "Calibration pack entry uses unsupported compression.";
                return false;
            }

            if (offset + compressedSize > static_cast<std::size_t>(zipBytes.size()))
            {
                errorMessage = "Calibration pack wavelength table truncated.";
                return false;
            }

            entryBytes.assign(zipBytes.constData() + static_cast<int>(offset),
                              zipBytes.constData() + static_cast<int>(offset + compressedSize));
            return true;
        }

        offset += compressedSize;
    }

    errorMessage = "Wavelength table not found inside calibration pack.";
    return false;
}

bool parseWavelengthTable(const std::string &tableBytes, std::vector<SpectralBand> &bands, std::string &errorMessage)
{
    bands.clear();
    std::istringstream stream(tableBytes);
    std::string line;
    int bandIndex = 0;

    while (std::getline(stream, line))
    {
        if (line.empty())
            continue;

        double wavelengthNm = 0.0;
        double fwhmNm = 0.0;
        std::istringstream lineStream(line);
        if (!(lineStream >> wavelengthNm >> fwhmNm))
            continue;

        SpectralBand band;
        band.index = bandIndex++;
        band.wavelengthNm = wavelengthNm;
        band.fwhmNm = fwhmNm;
        bands.push_back(band);
    }

    if (bands.empty())
    {
        errorMessage = "No wavelength entries found in calibration pack.";
        return false;
    }

    return true;
}
} // namespace

bool CalpackBandCatalog::loadFromCalpack(const QString &calpackPath,
                                          const int spectralBinning,
                                          std::vector<SpectralBand> &bands,
                                          std::string &errorMessage)
{
    bands.clear();

    QFile file(calpackPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        errorMessage = "Cannot open calibration pack: " + calpackPath.toStdString();
        return false;
    }

    const QByteArray zipBytes = file.readAll();
    if (zipBytes.isEmpty())
    {
        errorMessage = "Calibration pack file is empty.";
        return false;
    }

    const int bin = (spectralBinning == 2 || spectralBinning == 4 || spectralBinning == 8) ? spectralBinning
                                                                                        : 1;
    const char *entry = wavelengthTableEntryForBinning(bin);
    std::string tableBytes;
    if (!extractStoredZipEntry(zipBytes, entry, tableBytes, errorMessage))
        return false;

    if (!parseWavelengthTable(tableBytes, bands, errorMessage))
        return false;

    return true;
}

QString CalpackBandCatalog::formatBandLabel(const SpectralBand &band)
{
    return QStringLiteral("Band %1: %2nm")
        .arg(band.index)
        .arg(band.wavelengthNm, 0, 'f', 3);
}

int CalpackBandCatalog::comboIndexForBand(const std::vector<SpectralBand> &bands, const int bandIndex)
{
    for (std::size_t i = 0; i < bands.size(); ++i)
    {
        if (bands[i].index == bandIndex)
            return static_cast<int>(i);
    }

    return -1;
}
