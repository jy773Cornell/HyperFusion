#include "backend/processing/EnviBilWriter.hpp"

#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>

namespace hf::processing
{
namespace
{
bool writeFloatEnviHdr(const QString &hdrPath,
                       const EnviBilMetadata &metadata,
                       const int lineCount,
                       const QString &sensorTypeLabel,
                       QString *errorMessage)
{
    QSaveFile file(hdrPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write ENVI header: %1").arg(hdrPath);
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out << "ENVI\n";
    out << "description = {HyperFusion preprocessed reflectance}\n";
    out << "file type = ENVI\n\n";
    out << "sensor type = " << sensorTypeLabel << "\n\n";
    out << "samples = " << metadata.samples << "\n";
    out << "bands = " << metadata.bands << "\n";
    out << "lines = " << lineCount << "\n\n";
    out << "interleave = bil\n";
    out << "data type = 4\n";
    out << "header offset = 0\n";
    out << "byte order = 0\n\n";

    out << "Wavelength = {\n";
    for (int band = 0; band < metadata.bands; ++band)
    {
        out << metadata.wavelengthsNm[static_cast<std::size_t>(band)];
        if (band + 1 < metadata.bands)
            out << ",\n";
    }
    out << "\n}\n";

    if (!file.commit())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not save ENVI header: %1").arg(hdrPath);
        return false;
    }

    return true;
}
} // namespace

bool beginEnviFloatWriter(EnviFloatWriter &writer,
                          const QString &hdrPath,
                          const EnviBilMetadata &templateMetadata,
                          const QString &sensorTypeLabel,
                          QString *errorMessage)
{
    writer = EnviFloatWriter{};
    writer.hdrPath = hdrPath;
    writer.metadata = templateMetadata;
    writer.metadata.lines = 0;
    writer.metadata.dataType = 4;

    const QFileInfo hdrInfo(hdrPath);
    writer.rawPath = hdrInfo.absolutePath() + QLatin1Char('/')
                     + hdrInfo.completeBaseName() + QStringLiteral(".raw");

    QFile rawFile(writer.rawPath);
    if (!rawFile.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open ENVI raw for writing: %1").arg(writer.rawPath);
        return false;
    }

    rawFile.close();
    return writeFloatEnviHdr(writer.hdrPath, writer.metadata, 0, sensorTypeLabel, errorMessage);
}

bool appendEnviFloatLine(EnviFloatWriter &writer,
                         const float *linePixels,
                         QString *errorMessage)
{
    if (linePixels == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("ENVI float line pointer is null.");
        return false;
    }

    QFile rawFile(writer.rawPath);
    if (!rawFile.open(QIODevice::WriteOnly | QIODevice::Append))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not append ENVI raw: %1").arg(writer.rawPath);
        return false;
    }

    const std::size_t pixelsPerLine =
        static_cast<std::size_t>(writer.metadata.samples) * static_cast<std::size_t>(writer.metadata.bands);
    const qint64 byteCount = static_cast<qint64>(pixelsPerLine * sizeof(float));
    if (rawFile.write(reinterpret_cast<const char *>(linePixels), byteCount) != byteCount)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Disk write failed for %1").arg(writer.rawPath);
        return false;
    }

    ++writer.linesWritten;
    return true;
}

bool finalizeEnviFloatWriter(EnviFloatWriter &writer, QString *errorMessage)
{
    writer.metadata.lines = writer.linesWritten;
    return writeFloatEnviHdr(writer.hdrPath,
                             writer.metadata,
                             writer.linesWritten,
                             QStringLiteral("HyperFusion preprocessed"),
                             errorMessage);
}

} // namespace hf::processing
