#include "backend/camera/processing/EnviBilReader.hpp"

#include <QFile>
#include <QFileInfo>
#include <QRegularExpression>
#include <QTextStream>

#include <algorithm>
#include <cmath>
#include <vector>

namespace hf::processing
{
namespace
{
QString readKeyValue(const QString &line, const QString &key)
{
    const QString trimmed = line.trimmed();
    if (!trimmed.startsWith(key, Qt::CaseInsensitive))
        return {};

    const int eq = trimmed.indexOf(QLatin1Char('='));
    if (eq < 0)
        return {};

    return trimmed.mid(eq + 1).trimmed();
}

bool parseNumericListBlock(QTextStream &in, const QString &firstLine, std::vector<double> &valuesOut)
{
    QString block = firstLine;
    while (!in.atEnd())
    {
        const qint64 pos = in.pos();
        const QString next = in.readLine();
        if (next.trimmed().endsWith(QLatin1Char('}')))
        {
            block += QLatin1Char('\n') + next;
            break;
        }
        if (next.contains(QLatin1Char('=')) && !next.trimmed().startsWith(QLatin1Char('#')))
        {
            in.seek(pos);
            break;
        }
        block += QLatin1Char('\n') + next;
    }

    static const QRegularExpression numberPattern(QStringLiteral("[-+]?\\d*\\.?\\d+(?:[eE][-+]?\\d+)?"));
    auto it = numberPattern.globalMatch(block);
    while (it.hasNext())
        valuesOut.push_back(it.next().captured().toDouble());

    return !valuesOut.empty();
}

int parsePositiveInt(const QString &text, const int fallback)
{
    bool ok = false;
    const int value = text.toInt(&ok);
    return ok && value > 0 ? value : fallback;
}
} // namespace

bool parseEnviHdr(const QString &hdrPath, EnviBilMetadata &metadataOut, QString *errorMessage)
{
    metadataOut = EnviBilMetadata{};

    QFile hdrFile(hdrPath);
    if (!hdrFile.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open ENVI header: %1").arg(hdrPath);
        return false;
    }

    QTextStream in(&hdrFile);
    while (!in.atEnd())
    {
        const QString line = in.readLine();
        const QString lower = line.trimmed().toLower();

        if (lower.startsWith(QStringLiteral("samples")))
            metadataOut.samples = parsePositiveInt(readKeyValue(line, QStringLiteral("samples")), 0);
        else if (lower.startsWith(QStringLiteral("bands")))
            metadataOut.bands = parsePositiveInt(readKeyValue(line, QStringLiteral("bands")), 0);
        else if (lower.startsWith(QStringLiteral("lines")))
            metadataOut.lines = parsePositiveInt(readKeyValue(line, QStringLiteral("lines")), 0);
        else if (lower.startsWith(QStringLiteral("data type")))
            metadataOut.dataType = parsePositiveInt(readKeyValue(line, QStringLiteral("data type")), 12);
        else if (lower.startsWith(QStringLiteral("interleave")))
            metadataOut.interleave = readKeyValue(line, QStringLiteral("interleave")).toLower();
        else if (lower.startsWith(QStringLiteral("wavelength")))
            parseNumericListBlock(in, line, metadataOut.wavelengthsNm);
    }

    if (metadataOut.samples <= 0 || metadataOut.bands <= 0 || metadataOut.lines <= 0)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage = QStringLiteral("Invalid ENVI dimensions in %1 (samples=%2, bands=%3, lines=%4).")
                                .arg(hdrPath)
                                .arg(metadataOut.samples)
                                .arg(metadataOut.bands)
                                .arg(metadataOut.lines);
        }
        return false;
    }

    if (metadataOut.dataType != 12 && metadataOut.dataType != 4)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Unsupported ENVI data type %1 in %2 (expected 4=float32 or 12=uint16).")
                                 .arg(metadataOut.dataType)
                                 .arg(hdrPath);
        return false;
    }

    if (metadataOut.interleave != QStringLiteral("bil"))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Unsupported interleave '%1' in %2 (expected bil).")
                                 .arg(metadataOut.interleave, hdrPath);
        return false;
    }

    if (metadataOut.wavelengthsNm.size() < static_cast<std::size_t>(metadataOut.bands))
    {
        metadataOut.wavelengthsNm.resize(static_cast<std::size_t>(metadataOut.bands));
        for (int band = 0; band < metadataOut.bands; ++band)
            metadataOut.wavelengthsNm[static_cast<std::size_t>(band)] = static_cast<double>(band);
    }
    else if (metadataOut.wavelengthsNm.size() > static_cast<std::size_t>(metadataOut.bands))
    {
        metadataOut.wavelengthsNm.resize(static_cast<std::size_t>(metadataOut.bands));
    }

    const QFileInfo hdrInfo(hdrPath);
    metadataOut.rawPath = hdrInfo.absolutePath() + QLatin1Char('/')
                          + hdrInfo.completeBaseName() + QStringLiteral(".raw");
    if (!QFileInfo::exists(metadataOut.rawPath))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("ENVI raw file not found: %1").arg(metadataOut.rawPath);
        return false;
    }

    return true;
}

bool readEnviBilLines(const EnviBilMetadata &metadata,
                    const std::function<bool(const std::uint16_t *linePixels)> &onEachLine,
                    QString *errorMessage)
{
    if (metadata.samples <= 0 || metadata.bands <= 0 || metadata.lines <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("ENVI metadata dimensions are invalid.");
        return false;
    }

    QFile rawFile(metadata.rawPath);
    if (!rawFile.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open ENVI raw file: %1").arg(metadata.rawPath);
        return false;
    }

    const std::size_t linePixels =
        static_cast<std::size_t>(metadata.samples) * static_cast<std::size_t>(metadata.bands);
    const qint64 lineBytes = static_cast<qint64>(linePixels * sizeof(std::uint16_t));

    std::vector<std::uint16_t> lineBuffer(linePixels);
    for (int line = 0; line < metadata.lines; ++line)
    {
        if (rawFile.read(reinterpret_cast<char *>(lineBuffer.data()), lineBytes) != lineBytes)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("Unexpected end of ENVI raw file at line %1/%2: %3")
                                    .arg(line + 1)
                                    .arg(metadata.lines)
                                    .arg(metadata.rawPath);
            }
            return false;
        }

        if (!onEachLine(lineBuffer.data()))
            return true;
    }

    return true;
}

bool readEnviFloatBilLines(const EnviBilMetadata &metadata,
                           const QString &rawPathOverride,
                           const std::function<bool(const float *linePixels)> &onEachLine,
                           QString *errorMessage)
{
    if (metadata.samples <= 0 || metadata.bands <= 0 || metadata.lines <= 0)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("ENVI metadata dimensions are invalid.");
        return false;
    }

    if (metadata.dataType != 4)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("ENVI float reader requires data type 4 (float32 FFC), got %1.")
                    .arg(metadata.dataType);
        }
        return false;
    }

    const QString rawPath = rawPathOverride.isEmpty() ? metadata.rawPath : rawPathOverride;
    QFile rawFile(rawPath);
    if (!rawFile.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open ENVI float raw file: %1").arg(rawPath);
        return false;
    }

    const std::size_t linePixels =
        static_cast<std::size_t>(metadata.samples) * static_cast<std::size_t>(metadata.bands);
    const qint64 lineBytes = static_cast<qint64>(linePixels * sizeof(float));

    std::vector<float> lineBuffer(linePixels);
    for (int line = 0; line < metadata.lines; ++line)
    {
        if (rawFile.read(reinterpret_cast<char *>(lineBuffer.data()), lineBytes) != lineBytes)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage = QStringLiteral("Unexpected end of ENVI float raw at line %1/%2: %3")
                                    .arg(line + 1)
                                    .arg(metadata.lines)
                                    .arg(rawPath);
            }
            return false;
        }

        if (!onEachLine(lineBuffer.data()))
            return true;
    }

    return true;
}

} // namespace hf::processing
