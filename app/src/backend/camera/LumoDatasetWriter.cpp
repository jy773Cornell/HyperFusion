#include "backend/camera/LumoDatasetWriter.hpp"

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

QString illuminationModeLabel(const CaptureIlluminationMode mode)
{
    return mode == CaptureIlluminationMode::Reflectance ? QStringLiteral("reflectance")
                                                        : QStringLiteral("transmittance");
}

QString sampleCaptureBaseName(const QString &datasetName, const CaptureIlluminationMode mode)
{
    return QStringLiteral("%1_%2").arg(datasetName, illuminationModeLabel(mode));
}

QString captureMetadataDir(const QString &streamRoot)
{
    return QDir(streamRoot).filePath(QStringLiteral("capture/metadata"));
}

QString captureMetadataRelativePrefix()
{
    return QStringLiteral("capture/metadata/");
}

QString referenceCaptureBaseName(const QString &prefix,
                                 const QString &datasetName,
                                 const CaptureIlluminationMode mode)
{
    return QStringLiteral("%1_%2").arg(prefix, sampleCaptureBaseName(datasetName, mode));
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

bool writeEnviHdrFile(const QString &hdrPath,
                      const CaptureWriterStreamConfig &config,
                      const int width,
                      const int bands,
                      const std::uint64_t lineCount,
                      const QDateTime &sessionStartedUtc,
                      const QDateTime &startUtc,
                      const QDateTime &stopUtc)
{
    QSaveFile file(hdrPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;

    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);

    const QDateTime start = startUtc.isValid() ? startUtc : sessionStartedUtc;
    const QDateTime stop = stopUtc.isValid() ? stopUtc : start;

    out << "ENVI\n";
    out << "description = {\nFile Imported into ENVI}\n";
    out << "file type = ENVI\n\n";
    out << "sensor type = " << sensorTypeLabel(config) << "\n";
    out << "acquisition date = DATE(yyyy-mm-dd): "
        << start.toLocalTime().toString(QStringLiteral("yyyy-MM-dd")) << "\n";
    out << "Start Time = UTC TIME: " << start.toLocalTime().toString(QStringLiteral("HH:mm:ss")) << "\n";
    out << "Stop Time = UTC TIME: " << stop.toLocalTime().toString(QStringLiteral("HH:mm:ss")) << "\n\n";
    out << "samples = " << width << "\n";
    out << "bands = " << bands << "\n";
    out << "lines = " << lineCount << "\n\n";
    out << "errors = {none}\n\n";
    out << "interleave = bil\n";
    out << "data type = 12\n";
    out << "header offset = 0\n";
    out << "byte order = 0\n";
    out << "x start = 0\n";
    out << "y start = 0\n";
    out << "default bands = {227, 118, 42}\n\n";
    out << "himg = {1, " << width << "}\n";
    out << "vimg = {1, " << bands << "}\n";
    out << "hroi = {1, " << width << "}\n";
    out << "vroi = {1, " << bands << "}\n\n";
    out << "fps = " << QString::number(config.settings.frameRateHz, 'f', 2) << "\n";
    out << "tint = " << QString::number(config.settings.exposureMs, 'f', 6) << "\n";
    out << "binning = {" << config.settings.spatialBinning << ", " << config.settings.spectralBinning
        << "}\n";
    out << "trigger mode = "
        << (config.settings.externalTrigger ? QStringLiteral("External") : QStringLiteral("Internal"))
        << "\n";
    if (!config.calibrationPackPath.isEmpty())
        out << "calibration pack = " << config.calibrationPackPath << "\n";
    out << "\n";
    appendWavelengthBlock(out, config.spectralBands, bands);
    appendFwhmBlock(out, config.spectralBands, bands);
    return file.commit();
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

    const QFileInfo saveFolderInfo(config.saveFolder.trimmed());
    if (!saveFolderInfo.exists() || !saveFolderInfo.isDir())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Save folder does not exist: %1").arg(config.saveFolder.trimmed());
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

    const QDir sessionDir(sessionDirectory_);
    if (sessionDir.exists())
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("Dataset folder already exists \u2014 choose a different dataset name: %1")
                    .arg(sessionDirectory_);
        }
        sessionDirectory_.clear();
        datasetName_.clear();
        return false;
    }

    QDir root;
    if (!root.mkpath(sessionDirectory_))
    {
        if (errorMessage != nullptr)
            *errorMessage =
                QStringLiteral("Could not create dataset directory %1.").arg(sessionDirectory_);
        sessionDirectory_.clear();
        datasetName_.clear();
        return false;
    }

    writePropertiesXml();

    const int streamCount = static_cast<int>(config.streams.size());
    for (const CaptureWriterStreamConfig &streamConfig : config.streams)
    {
        if (streamConfig.relativeRoot.trimmed().isEmpty())
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Capture stream relative path is empty.");
            end();
            return false;
        }

        StreamState state;
        state.config = streamConfig;
        state.streamRoot = QDir(sessionDirectory_).filePath(streamConfig.relativeRoot);
        state.baseName = sampleCaptureBaseName(datasetName_, streamConfig.illuminationMode);
        state.summary.relativeRoot = streamConfig.relativeRoot;
        state.summary.baseName = state.baseName;

        if (!root.mkpath(QDir(state.streamRoot).filePath(QStringLiteral("capture")))
            || !root.mkpath(captureMetadataDir(state.streamRoot)))
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Could not create capture directories under %1.")
                                     .arg(state.streamRoot);
            end();
            return false;
        }

        if (!copyMetadataStylesheet(state.streamRoot, errorMessage))
        {
            end();
            return false;
        }

        const QString rawPath =
            QDir(state.streamRoot).filePath(QStringLiteral("capture/%1.raw").arg(state.baseName));
        state.summary.rawPath = rawPath;
        state.summary.hdrPath =
            QDir(state.streamRoot).filePath(QStringLiteral("capture/%1.hdr").arg(state.baseName));
        state.summary.logPath =
            QDir(captureMetadataDir(state.streamRoot)).filePath(state.baseName + QStringLiteral(".log"));

        streams_.emplace(streamConfig.relativeRoot, std::move(state));
    }

    active_ = true;
    return true;
}

bool LumoDatasetWriter::copyMetadataStylesheet(const QString &streamRoot, QString *errorMessage)
{
    const QString destination =
        QDir(captureMetadataDir(streamRoot)).filePath(datasetName_ + QStringLiteral(".xsl"));

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

bool LumoDatasetWriter::ensureStream(const FramePacket &frame, StreamState *&stateOut, QString *errorMessage)
{
    stateOut = nullptr;

    if (!active_)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Capture writer session is not active.");
        return false;
    }

    const auto streamIt = [&]() {
        const QString streamKey = frame.captureStreamKey.trimmed();
        if (!streamKey.isEmpty())
            return streams_.find(streamKey);

        for (auto it = streams_.begin(); it != streams_.end(); ++it)
        {
            if (it->second.config.source == frame.source)
                return it;
        }
        return streams_.end();
    }();

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
            *errorMessage = QStringLiteral("Frame buffer is smaller than %1 \u00D7 %2.")
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
                QStringLiteral("%1 frame size changed (%2\u00D7%3 → %4\u00D7%5).")
                    .arg(state.config.streamName)
                    .arg(state.summary.width)
                    .arg(state.summary.bands)
                    .arg(frame.width)
                    .arg(frame.height);
        }
        return false;
    }

    stateOut = &state;
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

bool LumoDatasetWriter::appendReferenceFrame(ReferenceCaptureState &reference,
                                             const QString &fileBaseName,
                                             StreamState &stream,
                                             const FramePacket &frame,
                                             QString *errorMessage)
{
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
            *errorMessage = QStringLiteral("Frame buffer is smaller than %1 \u00D7 %2.")
                                 .arg(frame.width)
                                 .arg(frame.height);
        return false;
    }

    if (reference.frameCount == 0)
    {
        reference.width = frame.width;
        reference.bands = frame.height;
        reference.firstFrameUtc = QDateTime::currentDateTimeUtc();
        const QString rawPath =
            QDir(stream.streamRoot).filePath(QStringLiteral("capture/%1.raw").arg(fileBaseName));
        reference.rawPath = rawPath;
        reference.hdrPath =
            QDir(stream.streamRoot).filePath(QStringLiteral("capture/%1.hdr").arg(fileBaseName));
        reference.rawFile = std::make_unique<QFile>(rawPath);
        if (!reference.rawFile->open(QIODevice::WriteOnly | QIODevice::Truncate))
        {
            if (errorMessage != nullptr)
                *errorMessage = QStringLiteral("Could not open %1 for writing.").arg(rawPath);
            reference.rawFile.reset();
            return false;
        }
    }
    else if (reference.width != frame.width || reference.bands != frame.height)
    {
        if (errorMessage != nullptr)
        {
            *errorMessage =
                QStringLiteral("%1 frame size changed (%2\u00D7%3 → %4\u00D7%5).")
                    .arg(fileBaseName)
                    .arg(reference.width)
                    .arg(reference.bands)
                    .arg(frame.width)
                    .arg(frame.height);
        }
        return false;
    }

    if (!writeFramePayload(*reference.rawFile, frame, errorMessage))
        return false;

    reference.lastFrameUtc = QDateTime::currentDateTimeUtc();
    ++reference.frameCount;
    return true;
}

bool LumoDatasetWriter::openSampleRawFile(StreamState &state, QString *errorMessage)
{
    if (state.rawFile != nullptr)
        return true;

    if (state.summary.rawPath.isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Sample raw path is empty for %1.").arg(state.config.streamName);
        return false;
    }

    state.rawFile = std::make_unique<QFile>(state.summary.rawPath);
    if (!state.rawFile->open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage =
                QStringLiteral("Could not open %1 for writing.").arg(state.summary.rawPath);
        state.rawFile.reset();
        return false;
    }

    return true;
}

bool LumoDatasetWriter::appendFrame(const FramePacket &frame, QString *errorMessage)
{
    StreamState *state = nullptr;
    if (!ensureStream(frame, state, errorMessage))
        return false;

    if (frame.captureDestination == CaptureFrameDestination::BlackReference)
    {
        const QString fileBaseName =
            referenceCaptureBaseName(QStringLiteral("DARKREF"), datasetName_, state->config.illuminationMode);
        const bool ok =
            appendReferenceFrame(state->blackReference, fileBaseName, *state, frame, errorMessage);
        if (ok)
        {
            state->summary.blackReferenceRawPath = state->blackReference.rawPath;
            state->summary.blackReferenceHdrPath = state->blackReference.hdrPath;
            state->summary.blackReferenceFrameCount = state->blackReference.frameCount;
        }
        return ok;
    }

    if (frame.captureDestination == CaptureFrameDestination::WhiteReference)
    {
        const QString fileBaseName =
            referenceCaptureBaseName(QStringLiteral("WHITEREF"), datasetName_, state->config.illuminationMode);
        const bool ok =
            appendReferenceFrame(state->whiteReference, fileBaseName, *state, frame, errorMessage);
        if (ok)
        {
            state->summary.whiteReferenceRawPath = state->whiteReference.rawPath;
            state->summary.whiteReferenceHdrPath = state->whiteReference.hdrPath;
            state->summary.whiteReferenceFrameCount = state->whiteReference.frameCount;
        }
        return ok;
    }

    if (state->rawFile == nullptr)
    {
        if (!openSampleRawFile(*state, errorMessage))
            return false;
    }

    if (!writeFramePayload(*state->rawFile, frame, errorMessage))
        return false;

    state->lastFrameUtc = QDateTime::currentDateTimeUtc();
    ++state->summary.frameCount;
    state->summary.bytesWritten +=
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

void LumoDatasetWriter::writeMetadataXml(const StreamState &stream) const
{
    const QString path =
        QDir(captureMetadataDir(stream.streamRoot)).filePath(datasetName_ + QStringLiteral(".xml"));
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QXmlStreamWriter xml(&file);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    xml.writeProcessingInstruction("xml-stylesheet",
                                   QStringLiteral("type=\"text/xsl\" href=\"%1.xsl\"").arg(datasetName_));
    xml.writeStartElement(QStringLiteral("properties"));

    if (stream.summary.frameCount > 0 || stream.summary.width > 0)
    {
        xml.writeStartElement(QStringLiteral("header"));
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("sensor type"));
        xml.writeCharacters(stream.config.streamName);
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("illumination mode"));
        xml.writeCharacters(stream.config.illuminationMode == CaptureIlluminationMode::Reflectance
                                ? QStringLiteral("reflectance")
                                : QStringLiteral("transmittance"));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("frame rate"));
        xml.writeCharacters(QString::number(stream.config.settings.frameRateHz, 'f', 0));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("integration time"));
        xml.writeCharacters(QString::number(stream.config.settings.exposureMs, 'f', 0));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("samples"));
        xml.writeCharacters(QString::number(stream.summary.width));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("active bands"));
        xml.writeCharacters(QString::number(stream.summary.bands));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("spatial binning"));
        xml.writeCharacters(QString::number(stream.config.settings.spatialBinning));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("spectral binning"));
        xml.writeCharacters(QString::number(stream.config.settings.spectralBinning));
        xml.writeEndElement();
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), QStringLiteral("frames recorded"));
        xml.writeCharacters(QString::number(stream.summary.frameCount));
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
        if (!stream.config.calibrationPackPath.isEmpty())
        {
            xml.writeStartElement(QStringLiteral("key"));
            xml.writeAttribute(QStringLiteral("field"), QStringLiteral("calibration pack"));
            xml.writeCharacters(stream.config.calibrationPackPath);
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
        const QString prefix =
            state.config.relativeRoot.isEmpty() ? QString() : state.config.relativeRoot + QLatin1Char('/');

        if (state.blackReference.frameCount > 0 && !state.blackReference.rawPath.isEmpty())
        {
            const QString rawName = QFileInfo(state.blackReference.rawPath).fileName();
            const QString hdrName = QFileInfo(state.blackReference.hdrPath).fileName();
            writeEntry(QStringLiteral("raw"), QStringLiteral("darkref"), prefix + QStringLiteral("capture/") + rawName);
            writeEntry(QStringLiteral("hdr"), QStringLiteral("darkref"), prefix + QStringLiteral("capture/") + hdrName);
        }
        if (state.whiteReference.frameCount > 0 && !state.whiteReference.rawPath.isEmpty())
        {
            const QString rawName = QFileInfo(state.whiteReference.rawPath).fileName();
            const QString hdrName = QFileInfo(state.whiteReference.hdrPath).fileName();
            writeEntry(QStringLiteral("raw"), QStringLiteral("whiteref"), prefix + QStringLiteral("capture/") + rawName);
            writeEntry(QStringLiteral("hdr"), QStringLiteral("whiteref"), prefix + QStringLiteral("capture/") + hdrName);
        }

        const QString rawRel = prefix + QStringLiteral("capture/%1.raw").arg(state.baseName);
        const QString hdrRel = prefix + QStringLiteral("capture/%1.hdr").arg(state.baseName);
        writeEntry(QStringLiteral("raw"), QStringLiteral("capture"), rawRel);
        writeEntry(QStringLiteral("hdr"), QStringLiteral("capture"), hdrRel);

        if (!state.summary.logPath.isEmpty())
        {
            const QString logName = QFileInfo(state.summary.logPath).fileName();
            writeEntry(QStringLiteral("log"), QStringLiteral("capture"),
                       prefix + captureMetadataRelativePrefix() + logName);
        }
    }

    for (const auto &entry : streams_)
    {
        const StreamState &state = entry.second;
        const QString prefix =
            state.config.relativeRoot.isEmpty() ? QString() : state.config.relativeRoot + QLatin1Char('/');
        writeEntry(QStringLiteral("xml"), QStringLiteral("properties"),
                   prefix + captureMetadataRelativePrefix() + datasetName_ + QStringLiteral(".xml"));
        writeEntry(QStringLiteral("xsl"), QStringLiteral("properties"),
                   prefix + captureMetadataRelativePrefix() + datasetName_ + QStringLiteral(".xsl"));
    }

    xml.writeEndElement();
    xml.writeEndDocument();
    file.commit();
}

void LumoDatasetWriter::writeStreamHdr(const StreamState &stream) const
{
    if (stream.summary.frameCount == 0 || stream.summary.hdrPath.isEmpty())
        return;

    writeEnviHdrFile(stream.summary.hdrPath,
                     stream.config,
                     stream.summary.width,
                     stream.summary.bands,
                     stream.summary.frameCount,
                     sessionStartedUtc_,
                     stream.firstFrameUtc,
                     stream.lastFrameUtc);
}

void LumoDatasetWriter::writeReferenceHdr(const StreamState &stream,
                                          const ReferenceCaptureState &reference) const
{
    if (reference.frameCount == 0 || reference.hdrPath.isEmpty())
        return;

    writeEnviHdrFile(reference.hdrPath,
                     stream.config,
                     reference.width,
                     reference.bands,
                     reference.frameCount,
                     sessionStartedUtc_,
                     reference.firstFrameUtc,
                     reference.lastFrameUtc);
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
        if (state.blackReference.rawFile != nullptr)
            state.blackReference.rawFile->close();
        state.blackReference.rawFile.reset();
        if (state.whiteReference.rawFile != nullptr)
            state.whiteReference.rawFile->close();
        state.whiteReference.rawFile.reset();

        writeStreamHdr(state);
        writeReferenceHdr(state, state.blackReference);
        writeReferenceHdr(state, state.whiteReference);
        writeStreamLog(state);
        writeMetadataXml(state);
        summary.streams.insert(entry.first, state.summary);
    }

    writeManifestXml();

    active_ = false;
    streams_.clear();
    sessionDirectory_.clear();
    datasetName_.clear();
    operatorName_.clear();
    description_.clear();
    return summary;
}
