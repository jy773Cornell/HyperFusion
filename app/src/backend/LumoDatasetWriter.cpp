#include "backend/LumoDatasetWriter.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSaveFile>
#include <QTextStream>
#include <QXmlStreamWriter>

#include <cstring>

namespace
{
QString sanitizePathSegment(QString value)
{
    value = value.trimmed();
    for (const QChar ch : QStringLiteral("<>:\"/\\|?*"))
        value.replace(ch, QLatin1Char('_'));
    return value;
}

QString streamSlug(const QString &streamName)
{
    QString slug = sanitizePathSegment(streamName);
    slug.replace(QLatin1Char(' '), QLatin1Char('_'));
    return slug;
}

QString streamBaseName(const QString &datasetName, const QString &streamName, const int streamCount)
{
    if (streamCount <= 1)
        return datasetName;
    return datasetName + QStringLiteral("_") + streamSlug(streamName);
}

QString sensorTypeLabel(const CaptureWriterStreamConfig &config)
{
    return config.streamName + QStringLiteral(" , HyperFusion");
}

void appendWavelengthBlock(QTextStream &out, const std::vector<SpectralBand> &bands, const int bandCount)
{
    out << "Wavelength = {\n";
    for (int band = 0; band < bandCount; ++band)
    {
        double wavelengthNm = static_cast<double>(band);
        for (const SpectralBand &entry : bands)
        {
            if (entry.index == band)
            {
                wavelengthNm = entry.wavelengthNm;
                break;
            }
        }
        out << wavelengthNm;
        if (band + 1 < bandCount)
            out << ",\n";
    }
    out << "\n}\n\n";
}

void appendFwhmBlock(QTextStream &out, const std::vector<SpectralBand> &bands, const int bandCount)
{
    out << "fwhm = {\n";
    for (int band = 0; band < bandCount; ++band)
    {
        double fwhmNm = 0.0;
        for (const SpectralBand &entry : bands)
        {
            if (entry.index == band)
            {
                fwhmNm = entry.fwhmNm;
                break;
            }
        }
        out << fwhmNm;
        if (band + 1 < bandCount)
            out << ",\n";
    }
    out << "\n}\n";
}
} // namespace

LumoDatasetWriter::~LumoDatasetWriter()
{
    end();
}

bool LumoDatasetWriter::begin(const CaptureWriterSessionConfig &config, QString *errorMessage)
{
    end();

    if (config.saveFolder.trimmed().isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Save folder is empty.");
        return false;
    }

    if (config.streams.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("No capture streams configured.");
        return false;
    }

    datasetName_ = sanitizePathSegment(config.datasetName);
    if (datasetName_.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Dataset name is empty.");
        return false;
    }

    operatorName_ = config.operatorName.trimmed();
    description_ = config.description.trimmed();
    sessionDirectory_ = QDir(config.saveFolder.trimmed()).filePath(datasetName_);
    sessionStartedUtc_ = QDateTime::currentDateTimeUtc();

    QDir root;
    if (!root.mkpath(QDir(sessionDirectory_).filePath(QStringLiteral("capture")))
        || !root.mkpath(QDir(sessionDirectory_).filePath(QStringLiteral("metadata"))))
    {
        if (errorMessage != nullptr)
            *errorMessage =
                QStringLiteral("Could not create dataset directories under %1.").arg(sessionDirectory_);
        sessionDirectory_.clear();
        datasetName_.clear();
        return false;
    }

    if (!copyMetadataStylesheet(errorMessage))
        return false;

    const int streamCount = static_cast<int>(config.streams.size());
    for (const CaptureWriterStreamConfig &streamConfig : config.streams)
    {
        StreamState state;
        state.config = streamConfig;
        state.baseName = streamBaseName(datasetName_, streamConfig.streamName, streamCount);
        state.summary.baseName = state.baseName;

        const QString rawPath =
            QDir(sessionDirectory_).filePath(QStringLiteral("capture/%1.raw").arg(state.baseName));
        state.summary.rawPath = rawPath;
        state.summary.hdrPath =
            QDir(sessionDirectory_).filePath(QStringLiteral("capture/%1.hdr").arg(state.baseName));
        state.summary.logPath =
            QDir(sessionDirectory_).filePath(QStringLiteral("capture/%1.log").arg(state.baseName));

        state.rawFile = std::make_unique<QFile>(rawPath);
        if (!state.rawFile->open(QIODevice::WriteOnly | QIODevice::Truncate))
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Could not open %1 for writing.").arg(rawPath);
            end();
            return false;
        }

        streams_.emplace(streamConfig.source, std::move(state));
    }

    writePropertiesXml();
    active_ = true;
    return true;
}

bool LumoDatasetWriter::copyMetadataStylesheet(QString *errorMessage)
{
    const QString destination =
        QDir(sessionDirectory_).filePath(QStringLiteral("metadata/%1.xsl").arg(datasetName_));

#ifdef HF_APP_SOURCE_DIR
    const QString templatePath = QStringLiteral(HF_APP_SOURCE_DIR)
                                 + QStringLiteral("/examples/gras_ia10/metadata/gras_ia10.xsl");
    if (QFileInfo::exists(templatePath))
    {
        if (QFile::exists(destination))
            QFile::remove(destination);
        if (QFile::copy(templatePath, destination))
            return true;
    }
#else
    Q_UNUSED(errorMessage);
#endif

    QSaveFile fallback(destination);
    if (!fallback.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write metadata stylesheet.");
        return false;
    }

    fallback.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                   "<xsl:stylesheet version=\"1.0\" xmlns:xsl=\"http://www.w3.org/1999/XSL/Transform\">\n"
                   "<xsl:template match=\"/\">\n"
                   "<html><body><h1>HyperFusion capture report</h1>"
                   "<pre><xsl:value-of select=\"/\"/></pre></body></html>\n"
                   "</xsl:template></xsl:stylesheet>\n");
    if (!fallback.commit())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write metadata stylesheet.");
        return false;
    }

    return true;
}

bool LumoDatasetWriter::ensureStream(const FramePacket &frame, QString *errorMessage)
{
    if (!active_)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Capture writer session is not active.");
        return false;
    }

    const auto streamIt = streams_.find(frame.source);
    if (streamIt == streams_.end())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Unexpected frame source for this capture session.");
        return false;
    }

    StreamState &state = streamIt->second;
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Frame geometry is empty.");
        return false;
    }

    const std::size_t required =
        static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    if (frame.pixels.size() < required)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Frame buffer is smaller than %1 × %2.")
                                 .arg(frame.width)
                                 .arg(frame.height);
        return false;
    }

    if (state.summary.frameCount == 0)
    {
        state.summary.width = frame.width;
        state.summary.bands = frame.height;
        state.firstFrameUtc = QDateTime::currentDateTimeUtc();
    }
    else if (state.summary.width != frame.width || state.summary.bands != frame.height)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("%1 frame size changed (%2×%3 → %4×%5).")
                    .arg(state.config.streamName)
                    .arg(state.summary.width)
                    .arg(state.summary.bands)
                    .arg(frame.width)
                    .arg(frame.height);
        }
        return false;
    }

    return true;
}

bool LumoDatasetWriter::writeFramePayload(QFile &file, const FramePacket &frame, QString *errorMessage)
{
    const std::size_t sampleCount =
        static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    const std::size_t byteCount = sampleCount * sizeof(std::uint16_t);

    if (file.write(reinterpret_cast<const char *>(frame.pixels.data()),
                   static_cast<qint64>(byteCount))
        != static_cast<qint64>(byteCount))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Disk write failed for %1.").arg(file.fileName());
        return false;
    }

    return true;
}

bool LumoDatasetWriter::appendFrame(const FramePacket &frame, QString *errorMessage)
{
    if (!ensureStream(frame, errorMessage))
        return false;

    StreamState &state = streams_.at(frame.source);
    if (state.rawFile == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("No output file for %1.").arg(state.config.streamName);
        return false;
    }

    if (!writeFramePayload(*state.rawFile, frame, errorMessage))
        return false;

    state.lastFrameUtc = QDateTime::currentDateTimeUtc();
    ++state.summary.frameCount;
    state.summary.bytesWritten +=
        static_cast<std::uint64_t>(frame.width) * static_cast<std::uint64_t>(frame.height)
        * sizeof(std::uint16_t);
    return true;
}

void LumoDatasetWriter::writePropertiesXml() const
{
    const QString path = QDir(sessionDirectory_).filePath(QStringLiteral("properties.xml"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QXmlStreamWriter xml(&file);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    xml.writeStartElement(QStringLiteral("properties"));

    const auto writeProperty = [&xml](const QString &name, const QString &value) {
        xml.writeStartElement(QStringLiteral("property"));
        xml.writeAttribute(QStringLiteral("name"), name);
        xml.writeCharacters(value);
        xml.writeEndElement();
    };

    writeProperty(QStringLiteral("countervalue"), QStringLiteral("0000"));
    writeProperty(QStringLiteral("prefix"), QString());
    writeProperty(QStringLiteral("name"), datasetName_);
    writeProperty(QStringLiteral("timestampformat"), QStringLiteral("!%Y-%m-%d_%H-%M-%S"));
    writeProperty(QStringLiteral("timestamp"),
                  sessionStartedUtc_.toLocalTime().toString(QStringLiteral("yyyy-MM-dd_HH-mm-ss")));
    writeProperty(QStringLiteral("namingformat"), QStringLiteral("only_name"));

    xml.writeEndElement();
    xml.writeEndDocument();
    file.commit();
}

void LumoDatasetWriter::writeMetadataXml() const
{
    const QString path = QDir(sessionDirectory_).filePath(QStringLiteral("metadata/%1.xml").arg(datasetName_));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QXmlStreamWriter xml(&file);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    xml.writeProcessingInstruction("xml-stylesheet",
                                   QStringLiteral("type=\"text/xsl\" href=\"%1.xsl\"").arg(datasetName_));
    xml.writeStartElement(QStringLiteral("properties"));

    if (!streams_.empty())
    {
        const StreamState &primary = streams_.cbegin()->second;
        xml.writeStartElement(QStringLiteral("header"));
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("sensor type"));
        xml.writeCharacters(primary.config.streamName);
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("frame rate"));
        xml.writeCharacters(QString::number(primary.config.settings.frameRateHz, 'f', 0));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("integration time"));
        xml.writeCharacters(QString::number(primary.config.settings.exposureMs, 'f', 0));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("samples"));
        xml.writeCharacters(QString::number(primary.summary.width));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("active bands"));
        xml.writeCharacters(QString::number(primary.summary.bands));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("spatial binning"));
        xml.writeCharacters(QString::number(primary.config.settings.spatialBinning));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("spectral binning"));
        xml.writeCharacters(QString::number(primary.config.settings.spectralBinning));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("frames recorded"));
        xml.writeCharacters(QString::number(primary.summary.frameCount));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("frames dropped"));
        xml.writeCharacters(QStringLiteral("0"));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("data format"));
        xml.writeCharacters(QStringLiteral("BIL"));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("date"));
        xml.writeCharacters(sessionStartedUtc_.toLocalTime().toString(Qt::ISODate));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("time"));
        xml.writeCharacters(sessionStartedUtc_.toLocalTime().toString(QStringLiteral("HH:mm:ss")));
        xml.writeEndElement();
        if (!primary.config.calibrationPackPath.isEmpty())
        {
            xml.writeStartElement(QStringLiteral("key"));
            xml.writeAttribute(QStringLiteral("field"), QStringLiteral("calibration pack"));
            xml.writeCharacters(primary.config.calibrationPackPath);
            xml.writeEndElement();
        }
        xml.writeEndElement();
    }

    xml.writeStartElement(QStringLiteral("userdefined"));
    const auto writeUserKey = [&xml](const QString &field, const QString &value) {
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), field);
        xml.writeCharacters(value);
        xml.writeEndElement();
    };
    writeUserKey(QStringLiteral("sample name"), datasetName_);
    writeUserKey(QStringLiteral("operator"), operatorName_);
    writeUserKey(QStringLiteral("description"), description_);
    xml.writeEndElement();

    xml.writeEndElement();
    xml.writeEndDocument();
    file.commit();
}

void LumoDatasetWriter::writeManifestXml() const
{
    const QString path = QDir(sessionDirectory_).filePath(QStringLiteral("manifest.xml"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QXmlStreamWriter xml(&file);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    xml.writeStartElement(QStringLiteral("manifest"));

    const auto writeEntry = [&xml](const QString &extension, const QString &type, const QString &relativePath) {
        xml.writeStartElement(QStringLiteral("file"));
        xml.writeAttribute(QStringLiteral("extension"), extension);
        xml.writeAttribute(QStringLiteral("type"), type);
        xml.writeCharacters(relativePath);
        xml.writeEndElement();
    };

    for (const auto &entry : streams_)
    {
        const StreamState &state = entry.second;
        const QString rawRel = QStringLiteral("capture/%1.raw").arg(state.baseName);
        const QString hdrRel = QStringLiteral("capture/%1.hdr").arg(state.baseName);
        writeEntry(QStringLiteral("raw"), QStringLiteral("capture"), rawRel);
        writeEntry(QStringLiteral("hdr"), QStringLiteral("capture"), hdrRel);
    }

    writeEntry(QStringLiteral("xml"), QStringLiteral("properties"),
               QStringLiteral("metadata/%1.xml").arg(datasetName_));
    writeEntry(QStringLiteral("xsl"), QStringLiteral("properties"),
               QStringLiteral("metadata/%1.xsl").arg(datasetName_));

    xml.writeEndElement();
    xml.writeEndDocument();
    file.commit();
}

void LumoDatasetWriter::writeStreamHdr(const StreamState &stream) const
{
    QSaveFile file(stream.summary.hdrPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);

    const QDateTime startUtc =
        stream.firstFrameUtc.isValid() ? stream.firstFrameUtc : sessionStartedUtc_;
    const QDateTime stopUtc = stream.lastFrameUtc.isValid() ? stream.lastFrameUtc : startUtc;

    out << "ENVI\n";
    out << "description = {\nFile Imported into ENVI}\n";
    out << "file type = ENVI\n\n";
    out << "sensor type = " << sensorTypeLabel(stream.config) << "\n";
    out << "acquisition date = DATE(yyyy-mm-dd): "
        << startUtc.toLocalTime().toString(QStringLiteral("yyyy-MM-dd")) << "\n";
    out << "Start Time = UTC TIME: "
        << startUtc.toLocalTime().toString(QStringLiteral("HH:mm:ss")) << "\n";
    out << "Stop Time = UTC TIME: "
        << stopUtc.toLocalTime().toString(QStringLiteral("HH:mm:ss")) << "\n\n";
    out << "samples = " << stream.summary.width << "\n";
    out << "bands = " << stream.summary.bands << "\n";
    out << "lines = " << stream.summary.frameCount << "\n\n";
    out << "errors = {none}\n\n";
    out << "interleave = bil\n";
    out << "data type = 12\n";
    out << "header offset = 0\n";
    out << "byte order = 0\n";
    out << "x start = 0\n";
    out << "y start = 0\n";
    out << "default bands = {227, 118, 42}\n\n";
    out << "himg = {1, " << stream.summary.width << "}\n";
    out << "vimg = {1, " << stream.summary.bands << "}\n";
    out << "hroi = {1, " << stream.summary.width << "}\n";
    out << "vroi = {1, " << stream.summary.bands << "}\n\n";
    out << "fps = " << QString::number(stream.config.settings.frameRateHz, 'f', 2) << "\n";
    out << "tint = " << QString::number(stream.config.settings.exposureMs, 'f', 6) << "\n";
    out << "binning = {" << stream.config.settings.spatialBinning << ", "
        << stream.config.settings.spectralBinning << "}\n";
    out << "trigger mode = "
        << (stream.config.settings.externalTrigger ? QStringLiteral("External")
                                                   : QStringLiteral("Internal"))
        << "\n";
    if (!stream.config.calibrationPackPath.isEmpty())
        out << "calibration pack = " << stream.config.calibrationPackPath << "\n";
    out << "\n";
    appendWavelengthBlock(out, stream.config.spectralBands, stream.summary.bands);
    appendFwhmBlock(out, stream.config.spectralBands, stream.summary.bands);
    file.commit();
}

void LumoDatasetWriter::writeStreamLog(const StreamState &stream) const
{
    QSaveFile file(stream.summary.logPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QTextStream out(&file);
    out << "HyperFusion Capture - Dropped Frame Log\n";
    out << "  0 dropped frame incidents, 0 dropped frames\n\n";
    out << "  " << stream.summary.frameCount << " frames recorded\n";
    file.commit();
}

CaptureWriterSessionSummary LumoDatasetWriter::end()
{
    CaptureWriterSessionSummary summary;
    summary.sessionDirectory = sessionDirectory_;
    summary.active = active_;

    if (!active_)
        return summary;

    for (auto &entry : streams_)
    {
        StreamState &state = entry.second;
        if (state.rawFile != nullptr)
            state.rawFile->close();
        state.rawFile.reset();

        writeStreamHdr(state);
        writeStreamLog(state);
        summary.streams.insert(entry.first, state.summary);
    }

    writeMetadataXml();
    writeManifestXml();

    active_ = false;
    streams_.clear();
    sessionDirectory_.clear();
    datasetName_.clear();
    operatorName_.clear();
    description_.clear();
    return summary;
}
