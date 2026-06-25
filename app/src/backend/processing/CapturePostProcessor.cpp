#include "backend/processing/CapturePostProcessor.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/processing/FlatFieldCorrector.hpp"
#include "backend/processing/Gsam2RoiAnalysis.hpp"
#include "backend/processing/Gsam2SegmentationClient.hpp"
#include "backend/processing/HsiToColor.hpp"
#include "backend/processing/IlluminantTables.hpp"
#include "backend/processing/ReferenceBuilder.hpp"
#include "backend/processing/ReferenceSpectrumPlot.hpp"
#include "backend/processing/ReferenceSpectrumStats.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QXmlStreamWriter>

#include <algorithm>

namespace hf::processing
{
namespace
{
struct StreamProcessReport
{
    QString relativeRoot;
    bool darkPlotOk = false;
    bool whitePlotOk = false;
    bool ffcOk = false;
    bool rgbOk = false;
    bool segmentationOk = false;
    QString darkPlotPath;
    QString whitePlotPath;
    QString ffcHdrPath;
    QString ffcRawPath;
    QString rgbPath;
    QString segmentationDir;
    QString segmentationCsvPath;
    QString segmentationPlotPath;
    QString rgbExportMode;
    QString errorMessage;
};

QString preprocessedDirectoryForStream(const CaptureWriterSessionSummary &summary,
                                       const CaptureWriterStreamSummary &stream)
{
    if (!stream.relativeRoot.isEmpty())
        return QDir(summary.sessionDirectory).filePath(stream.relativeRoot + QStringLiteral("/preprocessed"));

    return QDir(summary.sessionDirectory).filePath(QStringLiteral("preprocessed"));
}

QString datasetStemFromStreamBaseName(const QString &baseName)
{
    static const QString kReflectanceSuffix = QStringLiteral("_reflectance");
    static const QString kTransmittanceSuffix = QStringLiteral("_transmittance");

    if (baseName.endsWith(kReflectanceSuffix, Qt::CaseInsensitive))
        return baseName.left(baseName.size() - kReflectanceSuffix.size());
    if (baseName.endsWith(kTransmittanceSuffix, Qt::CaseInsensitive))
        return baseName.left(baseName.size() - kTransmittanceSuffix.size());

    return baseName;
}

bool streamIsTransmittance(const CaptureWriterStreamSummary &stream)
{
    static const QString kTransmittanceSuffix = QStringLiteral("_transmittance");
    if (stream.baseName.endsWith(kTransmittanceSuffix, Qt::CaseInsensitive))
        return true;

    return stream.relativeRoot.startsWith(QStringLiteral("transmittance/"), Qt::CaseInsensitive);
}

struct CaptureStreamTraits
{
    bool isTransmittance = false;
    bool isSwir3 = false;
};

CaptureStreamTraits captureStreamTraits(const CaptureWriterStreamSummary &stream)
{
    CaptureStreamTraits traits;
    const QStringList parts =
        stream.relativeRoot.split(QLatin1Char('/'), Qt::SkipEmptyParts);

    if (parts.size() >= 2)
    {
        traits.isTransmittance =
            parts[0].compare(QStringLiteral("transmittance"), Qt::CaseInsensitive) == 0;
        traits.isSwir3 = parts[1].compare(QStringLiteral("swir3"), Qt::CaseInsensitive) == 0;
        return traits;
    }

    traits.isTransmittance = streamIsTransmittance(stream);
    const QString streamHint = stream.relativeRoot + QLatin1Char('/') + stream.baseName;
    traits.isSwir3 = streamHint.contains(QStringLiteral("swir3"), Qt::CaseInsensitive);
    return traits;
}

QString spectrumYAxisLabelForStream(const CaptureStreamTraits &traits)
{
    return traits.isTransmittance ? QStringLiteral("Transmittance") : QStringLiteral("Reflectance");
}

QString preprocessedEnviDescriptionForStream(const CaptureStreamTraits &traits)
{
    return traits.isTransmittance ? QStringLiteral("HyperFusion preprocessed transmittance")
                                  : QStringLiteral("HyperFusion preprocessed reflectance");
}

double referencePlotDnAxisMaxForStream(const CaptureStreamTraits &traits)
{
    return traits.isSwir3 ? 65535.0 : 4096.0;
}

ReflectanceRgbExportMode rgbExportModeForStream(const CaptureStreamTraits &traits)
{
    // reflectance/fx10e + transmittance/fx10e → D-illuminant sRGB (λ ≤ truncate_nm)
    // reflectance/swir3 + transmittance/swir3 → SWIR false-color wavelength ranges
    return traits.isSwir3 ? ReflectanceRgbExportMode::SwirFalseColorRanges
                          : ReflectanceRgbExportMode::SpectralToSrgb;
}

QString rgbExportModeLabel(const ReflectanceRgbExportMode mode)
{
    return mode == ReflectanceRgbExportMode::SwirFalseColorRanges ? QStringLiteral("swir_false_color")
                                                                  : QStringLiteral("srgb");
}

SwirFalseColorConfig swirFalseColorConfigFromHardware(const hf::HardwareConfig::PreprocessingConfig &cfg)
{
    SwirFalseColorConfig falseColor;
    falseColor.red = cfg.swirFalseColorRed;
    falseColor.green = cfg.swirFalseColorGreen;
    falseColor.blue = cfg.swirFalseColorBlue;
    return falseColor;
}

QString referencePlotPath(const QString &preprocessedDir,
                          const QString &referencePrefix,
                          const QString &datasetStem)
{
    return QDir(preprocessedDir).filePath(referencePrefix + QLatin1Char('_') + datasetStem
                                        + QStringLiteral("_ref_plot.png"));
}

FlatFieldParams flatFieldParamsFromConfig()
{
    const hf::HardwareConfig::PreprocessingConfig &cfg = hf::hardwareConfig().preprocessing;
    FlatFieldParams params;
    params.epsilon = cfg.ffcEpsilon;
    params.clampMin = cfg.ffcClampMin;
    params.clampMax = cfg.ffcClampMax;
    return params;
}

QString manifestRelativePath(const QString &sessionDirectory, const QString &absolutePath)
{
    QString relative = QDir(sessionDirectory).relativeFilePath(absolutePath);
    return relative.replace(QLatin1Char('\\'), QLatin1Char('/'));
}

QString streamPathPrefix(const QString &relativeRoot)
{
    return relativeRoot.isEmpty() ? QString() : relativeRoot + QLatin1Char('/');
}

bool copyCaptureMetadataStylesheet(const QString &sessionDirectory,
                                   const QString &relativeRoot,
                                   const QString &datasetName,
                                   const QString &destinationPath)
{
    const QString captureStylesheet =
        QDir(sessionDirectory).filePath(streamPathPrefix(relativeRoot)
                                      + QStringLiteral("capture/metadata/%1.xsl").arg(datasetName));
    if (QFileInfo::exists(captureStylesheet))
    {
        if (QFile::exists(destinationPath))
            QFile::remove(destinationPath);
        return QFile::copy(captureStylesheet, destinationPath);
    }

    QSaveFile fallback(destinationPath);
    if (!fallback.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;

    fallback.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                   "<xsl:stylesheet version=\"1.0\" xmlns:xsl=\"http://www.w3.org/1999/XSL/Transform\">\n"
                   "<xsl:template match=\"/\">\n"
                   "<html><body><h1>HyperFusion preprocessing report</h1>"
                   "<pre><xsl:value-of select=\"/\"/></pre></body></html>\n"
                   "</xsl:template></xsl:stylesheet>\n");
    return fallback.commit();
}

void writePreprocessedMetadataXml(const QString &metadataXmlPath,
                                  const QString &datasetName,
                                  const StreamProcessReport &report,
                                  const CapturePostProcessOptions &options)
{
    QSaveFile file(metadataXmlPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    QXmlStreamWriter xml(&file);
    xml.setAutoFormatting(true);
    xml.writeStartDocument();
    xml.writeProcessingInstruction("xml-stylesheet",
                                   QStringLiteral("type=\"text/xsl\" href=\"%1.xsl\"").arg(datasetName));
    xml.writeStartElement(QStringLiteral("properties"));

    const auto writeKey = [&xml](const QString &field, const QString &value) {
        xml.writeStartElement(QStringLiteral("key"));
        xml.writeAttribute(QStringLiteral("field"), field);
        xml.writeCharacters(value);
        xml.writeEndElement();
    };

    xml.writeStartElement(QStringLiteral("header"));
    writeKey(QStringLiteral("generated utc"),
             QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    writeKey(QStringLiteral("save ffc image"), options.saveFfcImage ? QStringLiteral("yes")
                                                                    : QStringLiteral("no"));
    writeKey(QStringLiteral("run gsam segmentation"),
             options.runGsamSegmentation ? QStringLiteral("yes") : QStringLiteral("no"));
    if (!options.gsamPrompt.trimmed().isEmpty())
        writeKey(QStringLiteral("gsam prompt"), options.gsamPrompt.trimmed());
    if (options.runGsamSegmentation)
        writeKey(QStringLiteral("gsam sample count"), QString::number(options.gsamSampleCount));
    if (!report.rgbExportMode.isEmpty())
        writeKey(QStringLiteral("rgb export mode"), report.rgbExportMode);

    const auto writeArtifact = [&writeKey](const QString &label, const bool ok, const QString &path) {
        writeKey(label + QStringLiteral(" status"), ok ? QStringLiteral("ok") : QStringLiteral("missing"));
        if (!path.isEmpty())
            writeKey(label, QFileInfo(path).fileName());
    };

    writeArtifact(QStringLiteral("dark reference plot"), report.darkPlotOk, report.darkPlotPath);
    writeArtifact(QStringLiteral("white reference plot"), report.whitePlotOk, report.whitePlotPath);
    writeArtifact(QStringLiteral("flat-field corrected cube"), report.ffcOk, report.ffcHdrPath);
    writeArtifact(QStringLiteral("rgb preview"), report.rgbOk, report.rgbPath);
    writeArtifact(QStringLiteral("segmentation"), report.segmentationOk, report.segmentationCsvPath);
    if (!report.segmentationPlotPath.isEmpty())
        writeKey(QStringLiteral("segmentation spectrum plot"),
                 QFileInfo(report.segmentationPlotPath).fileName());
    xml.writeEndElement();

    xml.writeEndElement();
    xml.writeEndDocument();
    file.commit();
}

bool writePreprocessedPropertiesFiles(const QString &sessionDirectory,
                                      const QString &datasetName,
                                      const StreamProcessReport &report,
                                      const CapturePostProcessOptions &options)
{
    if (!report.darkPlotOk && !report.whitePlotOk && !report.ffcOk && !report.rgbOk
        && !report.segmentationOk)
        return false;

    const QString prefix = streamPathPrefix(report.relativeRoot);
    const QString metadataDir =
        QDir(sessionDirectory).filePath(prefix + QStringLiteral("preprocessed/metadata"));
    if (!QDir().mkpath(metadataDir))
        return false;

    const QString xmlPath = QDir(metadataDir).filePath(datasetName + QStringLiteral(".xml"));
    const QString xslPath = QDir(metadataDir).filePath(datasetName + QStringLiteral(".xsl"));
    writePreprocessedMetadataXml(xmlPath, datasetName, report, options);
    copyCaptureMetadataStylesheet(sessionDirectory, report.relativeRoot, datasetName, xslPath);
    return QFileInfo::exists(xmlPath) && QFileInfo::exists(xslPath);
}

struct ManifestFileEntry
{
    QString extension;
    QString type;
    QString relativePath;
};

bool appendManifestFileEntries(const QString &manifestPath,
                               const std::vector<ManifestFileEntry> &entries)
{
    if (entries.empty())
        return true;

    QFile file(manifestPath);
    if (!file.open(QIODevice::ReadOnly))
        return false;

    QString content = QString::fromUtf8(file.readAll());
    file.close();

    QStringList lines;
    for (const ManifestFileEntry &entry : entries)
    {
        if (content.contains(QStringLiteral(">") + entry.relativePath + QStringLiteral("<")))
            continue;

        lines.push_back(QStringLiteral("    <file extension=\"%1\" type=\"%2\">%3</file>")
                            .arg(entry.extension, entry.type, entry.relativePath));
    }

    if (lines.isEmpty())
        return true;

    const int closingTag = content.lastIndexOf(QStringLiteral("</manifest>"));
    if (closingTag < 0)
        return false;

    content.insert(closingTag, lines.join(QLatin1Char('\n')) + QLatin1Char('\n'));

    QSaveFile out(manifestPath);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return false;

    out.write(content.toUtf8());
    return out.commit();
}

std::vector<ManifestFileEntry>
preprocessedManifestEntries(const QString &sessionDirectory,
                            const QString &datasetName,
                            const StreamProcessReport &report)
{
    std::vector<ManifestFileEntry> entries;
    const QString prefix = streamPathPrefix(report.relativeRoot);

    const auto addEntry = [&entries, &sessionDirectory](const QString &type,
                                                        const QString &absolutePath) {
        if (absolutePath.isEmpty() || !QFileInfo::exists(absolutePath))
            return;

        const QFileInfo info(absolutePath);
        entries.push_back({info.suffix().toLower(), type,
                           manifestRelativePath(sessionDirectory, absolutePath)});
    };

    entries.push_back({QStringLiteral("xml"), QStringLiteral("properties"),
                       prefix + QStringLiteral("preprocessed/metadata/%1.xml").arg(datasetName)});
    entries.push_back({QStringLiteral("xsl"), QStringLiteral("properties"),
                       prefix + QStringLiteral("preprocessed/metadata/%1.xsl").arg(datasetName)});

    addEntry(QStringLiteral("preprocessed"), report.darkPlotPath);
    addEntry(QStringLiteral("preprocessed"), report.whitePlotPath);
    addEntry(QStringLiteral("preprocessed"), report.ffcRawPath);
    addEntry(QStringLiteral("preprocessed"), report.ffcHdrPath);
    addEntry(QStringLiteral("preprocessed"), report.rgbPath);
    addEntry(QStringLiteral("properties"),
             QDir(sessionDirectory)
                 .filePath(prefix + QStringLiteral("preprocessed/processing_manifest.json")));

    if (!report.segmentationDir.isEmpty() && QFileInfo::exists(report.segmentationDir))
    {
        const QDir segmentationDir(report.segmentationDir);
        const QStringList segmentationFiles =
            segmentationDir.entryList(QDir::Files | QDir::NoDotAndDotDot);
        for (const QString &name : segmentationFiles)
            addEntry(QStringLiteral("preprocessed"), segmentationDir.filePath(name));

        const QStringList subdirs = segmentationDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
        for (const QString &subdir : subdirs)
        {
            const QDir nestedDir(segmentationDir.filePath(subdir));
            const QStringList nestedFiles =
                nestedDir.entryList(QDir::Files | QDir::NoDotAndDotDot);
            for (const QString &name : nestedFiles)
                addEntry(QStringLiteral("preprocessed"), nestedDir.filePath(name));
        }
    }

    return entries;
}

void updateDatasetManifestForPreprocessing(const QString &sessionDirectory,
                                           const QString &datasetName,
                                           const std::vector<StreamProcessReport> &reports,
                                           const CapturePostProcessOptions &options)
{
    const QString manifestPath = QDir(sessionDirectory).filePath(QStringLiteral("manifest.xml"));
    if (!QFileInfo::exists(manifestPath))
        return;

    std::vector<ManifestFileEntry> entries;
    for (const StreamProcessReport &report : reports)
    {
        if (!writePreprocessedPropertiesFiles(sessionDirectory, datasetName, report, options))
            continue;

        const std::vector<ManifestFileEntry> streamEntries =
            preprocessedManifestEntries(sessionDirectory, datasetName, report);
        entries.insert(entries.end(), streamEntries.begin(), streamEntries.end());
    }

    appendManifestFileEntries(manifestPath, entries);
}

bool writeProcessingManifestJson(const QString &preprocessedDir,
                   const CaptureWriterSessionSummary &summary,
                   const std::vector<StreamProcessReport> &reports,
                   const CapturePostProcessOptions &options,
                   QString *errorMessage)
{
    QJsonObject root;
    root.insert(QStringLiteral("generatedUtc"),
                QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    root.insert(QStringLiteral("sessionDirectory"), summary.sessionDirectory);
    root.insert(QStringLiteral("saveFfcImage"), options.saveFfcImage);
    root.insert(QStringLiteral("runGsamSegmentation"), options.runGsamSegmentation);
    root.insert(QStringLiteral("gsamPrompt"), options.gsamPrompt);
    root.insert(QStringLiteral("gsamSampleCount"), options.gsamSampleCount);

    const hf::HardwareConfig::PreprocessingConfig &cfg = hf::hardwareConfig().preprocessing;
    QJsonObject configObject;
    configObject.insert(QStringLiteral("illuminantD"), cfg.illuminantD);
    configObject.insert(QStringLiteral("ffcEpsilon"), cfg.ffcEpsilon);
    configObject.insert(QStringLiteral("ffcClampMin"), cfg.ffcClampMin);
    configObject.insert(QStringLiteral("ffcClampMax"), cfg.ffcClampMax);
    configObject.insert(QStringLiteral("truncateNm"), cfg.truncateNm);
    configObject.insert(QStringLiteral("illuminantsJson"), defaultIlluminantsJsonPath());
    configObject.insert(QStringLiteral("swirFalseColorRedNmMin"), cfg.swirFalseColorRed.minNm);
    configObject.insert(QStringLiteral("swirFalseColorRedNmMax"), cfg.swirFalseColorRed.maxNm);
    configObject.insert(QStringLiteral("swirFalseColorGreenNmMin"), cfg.swirFalseColorGreen.minNm);
    configObject.insert(QStringLiteral("swirFalseColorGreenNmMax"), cfg.swirFalseColorGreen.maxNm);
    configObject.insert(QStringLiteral("swirFalseColorBlueNmMin"), cfg.swirFalseColorBlue.minNm);
    configObject.insert(QStringLiteral("swirFalseColorBlueNmMax"), cfg.swirFalseColorBlue.maxNm);
    root.insert(QStringLiteral("config"), configObject);

    QJsonArray streamsArray;
    for (const StreamProcessReport &report : reports)
    {
        QJsonObject streamObject;
        streamObject.insert(QStringLiteral("relativeRoot"), report.relativeRoot);
        streamObject.insert(QStringLiteral("darkPlotOk"), report.darkPlotOk);
        streamObject.insert(QStringLiteral("whitePlotOk"), report.whitePlotOk);
        streamObject.insert(QStringLiteral("ffcOk"), report.ffcOk);
        streamObject.insert(QStringLiteral("rgbOk"), report.rgbOk);
        streamObject.insert(QStringLiteral("segmentationOk"), report.segmentationOk);
        streamObject.insert(QStringLiteral("darkPlotPath"), report.darkPlotPath);
        streamObject.insert(QStringLiteral("whitePlotPath"), report.whitePlotPath);
        streamObject.insert(QStringLiteral("ffcHdrPath"), report.ffcHdrPath);
        streamObject.insert(QStringLiteral("ffcRawPath"), report.ffcRawPath);
        streamObject.insert(QStringLiteral("rgbPath"), report.rgbPath);
        if (!report.rgbExportMode.isEmpty())
            streamObject.insert(QStringLiteral("rgbExportMode"), report.rgbExportMode);
        streamObject.insert(QStringLiteral("segmentationDir"), report.segmentationDir);
        streamObject.insert(QStringLiteral("segmentationCsvPath"), report.segmentationCsvPath);
        streamObject.insert(QStringLiteral("segmentationPlotPath"), report.segmentationPlotPath);
        if (!report.errorMessage.isEmpty())
            streamObject.insert(QStringLiteral("error"), report.errorMessage);
        streamsArray.push_back(streamObject);
    }
    root.insert(QStringLiteral("streams"), streamsArray);

    const QString manifestPath = QDir(preprocessedDir).filePath(QStringLiteral("processing_manifest.json"));
    QSaveFile file(manifestPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write manifest: %1").arg(manifestPath);
        return false;
    }

    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    if (!file.commit())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not save manifest: %1").arg(manifestPath);
        return false;
    }

    return true;
}

StreamProcessReport processStream(const CaptureWriterSessionSummary &summary,
                                  const CaptureWriterStreamSummary &stream,
                                  const CapturePostProcessOptions &options,
                                  QStringList &logLines)
{
    StreamProcessReport report;
    report.relativeRoot = stream.relativeRoot;

    const QString preprocessedDir = preprocessedDirectoryForStream(summary, stream);
    if (!QDir().mkpath(preprocessedDir))
    {
        report.errorMessage = QStringLiteral("Could not create %1").arg(preprocessedDir);
        logLines.push_back(QStringLiteral("Capture post-process (%1): %2")
                               .arg(report.relativeRoot, report.errorMessage));
        return report;
    }

    const QString streamLabel =
        stream.relativeRoot.isEmpty() ? stream.baseName : stream.relativeRoot;
    const QString datasetStem = datasetStemFromStreamBaseName(stream.baseName);
    const CaptureStreamTraits streamTraits = captureStreamTraits(stream);
    const QString illuminantsJsonPath = defaultIlluminantsJsonPath();
    const hf::HardwareConfig::PreprocessingConfig &cfg = hf::hardwareConfig().preprocessing;

    BilRowReference darkRow;
    BilRowReference whiteRow;
    EnviBilMetadata darkMeta;
    EnviBilMetadata whiteMeta;
    BandMeanStd darkPlotStats;
    BandMeanStd whitePlotStats;
    bool darkRowReady = false;
    bool whiteRowReady = false;
    QString error;

    if (!stream.blackReferenceHdrPath.isEmpty() && stream.blackReferenceFrameCount > 0)
    {
        report.darkPlotPath =
            referencePlotPath(preprocessedDir, QStringLiteral("DARKREF"), datasetStem);
        if (!buildRowMeanReferenceAndFrameStats(stream.blackReferenceHdrPath,
                                                darkRow,
                                                darkMeta,
                                                darkPlotStats,
                                                &error))
        {
            report.errorMessage = QStringLiteral("Dark row reference failed: %1").arg(error);
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2")
                                   .arg(streamLabel, report.errorMessage));
        }
        else
        {
            darkRowReady = true;
            report.darkPlotOk =
                saveReferenceMeanStdPlotPng(darkPlotStats.wavelengthsNm,
                                            darkPlotStats.meanDn,
                                            darkPlotStats.stdDn,
                                            QStringLiteral("Dark Reference Mean ±1σ"),
                                            report.darkPlotPath,
                                            referencePlotDnAxisMaxForStream(streamTraits),
                                            &error);
            if (report.darkPlotOk)
            {
                logLines.push_back(QStringLiteral("Capture post-process (%1): wrote %2")
                                       .arg(streamLabel, QFileInfo(report.darkPlotPath).fileName()));
            }
            else
            {
                report.errorMessage = error;
                logLines.push_back(
                    QStringLiteral("Capture post-process (%1): dark reference plot failed \u2014 %2")
                        .arg(streamLabel, error));
            }
        }
    }

    if (!stream.whiteReferenceHdrPath.isEmpty() && stream.whiteReferenceFrameCount > 0)
    {
        report.whitePlotPath =
            referencePlotPath(preprocessedDir, QStringLiteral("WHITEREF"), datasetStem);
        if (!buildRowMeanReferenceAndFrameStats(stream.whiteReferenceHdrPath,
                                                whiteRow,
                                                whiteMeta,
                                                whitePlotStats,
                                                &error))
        {
            if (report.errorMessage.isEmpty())
                report.errorMessage = QStringLiteral("White row reference failed: %1").arg(error);
            logLines.push_back(QStringLiteral("Capture post-process (%1): white row reference failed \u2014 %2")
                                   .arg(streamLabel, error));
        }
        else
        {
            whiteRowReady = true;
            report.whitePlotOk =
                saveReferenceMeanStdPlotPng(whitePlotStats.wavelengthsNm,
                                            whitePlotStats.meanDn,
                                            whitePlotStats.stdDn,
                                            QStringLiteral("White Reference Mean ±1σ"),
                                            report.whitePlotPath,
                                            referencePlotDnAxisMaxForStream(streamTraits),
                                            &error);
            if (report.whitePlotOk)
            {
                logLines.push_back(QStringLiteral("Capture post-process (%1): wrote %2")
                                       .arg(streamLabel, QFileInfo(report.whitePlotPath).fileName()));
            }
            else
            {
                if (report.errorMessage.isEmpty())
                    report.errorMessage = error;
                logLines.push_back(
                    QStringLiteral("Capture post-process (%1): white reference plot failed \u2014 %2")
                        .arg(streamLabel, error));
            }
        }
    }

    const bool canProcessSample = !stream.hdrPath.isEmpty() && stream.frameCount > 0 && darkRowReady
                                  && whiteRowReady;

    if (!canProcessSample)
    {
        QString manifestError;
        writeProcessingManifestJson(preprocessedDir, summary, {report}, options, &manifestError);
        return report;
    }

    const QString ffcBaseName = datasetStem + QStringLiteral("_ffc");
    report.ffcHdrPath = QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral(".hdr"));
    report.ffcRawPath = QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral(".raw"));
    report.rgbPath = QDir(preprocessedDir).filePath(datasetStem + QStringLiteral("_rgb.png"));

    const QString sensorLabel = stream.baseName;
    const FlatFieldParams ffcParams = flatFieldParamsFromConfig();

    QString ffcHdrForRgb = report.ffcHdrPath;
    const bool keepFfcForSegmentation = options.runGsamSegmentation && !options.saveFfcImage;
    if (!options.saveFfcImage)
    {
        ffcHdrForRgb = QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.hdr"));
        QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
        QFile::remove(ffcHdrForRgb);
    }
    else
    {
        QFile::remove(report.ffcRawPath);
        QFile::remove(report.ffcHdrPath);
    }

    if (!writeFlatFieldCorrectedEnvi(stream.hdrPath,
                                   darkRow,
                                   whiteRow,
                                   ffcHdrForRgb,
                                   sensorLabel,
                                   preprocessedEnviDescriptionForStream(streamTraits),
                                   ffcParams,
                                   &error))
    {
        report.errorMessage = QStringLiteral("FFC failed: %1").arg(error);
        logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
        return report;
    }

    report.ffcOk = options.saveFfcImage;
    if (options.saveFfcImage)
    {
        logLines.push_back(QStringLiteral("Capture post-process (%1): wrote %2")
                               .arg(streamLabel, QFileInfo(report.ffcRawPath).fileName()));
    }

    EnviBilMetadata ffcMetadata;
    if (!parseEnviHdr(ffcHdrForRgb, ffcMetadata, &error))
    {
        report.errorMessage = QStringLiteral("FFC metadata read failed: %1").arg(error);
        logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
        return report;
    }

    const QString ffcRawPath = QFileInfo(ffcHdrForRgb).absolutePath() + QLatin1Char('/')
                               + QFileInfo(ffcHdrForRgb).completeBaseName() + QStringLiteral(".raw");
    const ReflectanceRgbExportMode rgbMode = rgbExportModeForStream(streamTraits);
    const SwirFalseColorConfig swirFalseColor = swirFalseColorConfigFromHardware(cfg);

    if (!writeFfcCubeRgbPng(ffcMetadata,
                            ffcRawPath,
                            rgbMode,
                            swirFalseColor,
                            cfg.illuminantD,
                            cfg.truncateNm,
                            illuminantsJsonPath,
                            report.rgbPath,
                            &error))
    {
        report.errorMessage = QStringLiteral("RGB export failed: %1").arg(error);
        logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
        return report;
    }

    report.rgbOk = true;
    report.rgbExportMode = rgbExportModeLabel(rgbMode);
    logLines.push_back(
        QStringLiteral("Capture post-process (%1): RGB from FFC float raw (%2/%3)")
            .arg(streamLabel)
            .arg(streamTraits.isTransmittance ? QStringLiteral("transmittance")
                                              : QStringLiteral("reflectance"))
            .arg(streamTraits.isSwir3 ? QStringLiteral("swir3") : QStringLiteral("fx10e")));
    if (rgbMode == ReflectanceRgbExportMode::SwirFalseColorRanges)
    {
        logLines.push_back(
            QStringLiteral("Capture post-process (%1): wrote %2 (SWIR false-color, R %3–%4 nm, G %5–%6 nm, B %7–%8 nm)")
                .arg(streamLabel)
                .arg(QFileInfo(report.rgbPath).fileName())
                .arg(cfg.swirFalseColorRed.minNm, 0, 'f', 0)
                .arg(cfg.swirFalseColorRed.maxNm, 0, 'f', 0)
                .arg(cfg.swirFalseColorGreen.minNm, 0, 'f', 0)
                .arg(cfg.swirFalseColorGreen.maxNm, 0, 'f', 0)
                .arg(cfg.swirFalseColorBlue.minNm, 0, 'f', 0)
                .arg(cfg.swirFalseColorBlue.maxNm, 0, 'f', 0));
    }
    else
    {
        logLines.push_back(QStringLiteral("Capture post-process (%1): wrote %2 (sRGB)")
                               .arg(streamLabel, QFileInfo(report.rgbPath).fileName()));
    }

    if (options.runGsamSegmentation)
    {
        const QString segDir = QDir(preprocessedDir).filePath(QStringLiteral("segmentation"));
        if (!QDir().mkpath(segDir))
        {
            report.errorMessage = QStringLiteral("Could not create segmentation directory.");
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
            return report;
        }
        report.segmentationDir = segDir;

        const hf::HardwareConfig::SegmentationConfig &segCfg = hf::hardwareConfig().segmentation;
        Gsam2SegmentationRequest segRequest;
        segRequest.inputRgbPath = report.rgbPath;
        segRequest.outputDirectory = segDir;
        segRequest.imageName = datasetStem + QStringLiteral("_rgb.png");
        segRequest.prompt = options.gsamPrompt.trimmed().isEmpty() ? QStringLiteral("sample.")
                                                                    : options.gsamPrompt.trimmed();
        segRequest.maxDetections = std::max(1, options.gsamSampleCount);
        segRequest.boxThreshold = segCfg.boxThreshold;
        segRequest.serverUrl = options.gsamServerUrl;

        QString segError;
        const Gsam2SegmentationResponse segResponse = requestGsam2Segmentation(segRequest, &segError);
        if (!segResponse.ok)
        {
            report.errorMessage =
                segError.isEmpty() ? QStringLiteral("GSAM2 segmentation failed.") : segError;
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
            if (!options.saveFfcImage && keepFfcForSegmentation)
            {
                QFile::remove(ffcHdrForRgb);
                QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
            }
            return report;
        }

        logLines.push_back(QStringLiteral("Capture post-process (%1): GSAM2 found %2 ROI(s)")
                               .arg(streamLabel)
                               .arg(segResponse.detectionCount));
        logLines.push_back(QStringLiteral("Capture post-process (%1): per-ROI segmented RGB in preprocessed/segmentation/segmented_rgb/")
                               .arg(streamLabel));

        const Gsam2RoiAnalysisResult roiResult = analyzeGsam2SegmentationRois(
            ffcHdrForRgb,
            segDir,
            datasetStem + QStringLiteral("_rgb.png"),
            segResponse.manifestJsonPath,
            spectrumYAxisLabelForStream(streamTraits),
            &segError);
        if (!roiResult.success)
        {
            report.errorMessage = roiResult.errorMessage.isEmpty()
                                      ? QStringLiteral("GSAM2 ROI analysis failed.")
                                      : roiResult.errorMessage;
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
            if (!options.saveFfcImage && keepFfcForSegmentation)
            {
                QFile::remove(ffcHdrForRgb);
                QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
            }
            return report;
        }

        report.segmentationOk = true;
        report.segmentationCsvPath = roiResult.csvPath;
        report.segmentationPlotPath = roiResult.spectrumPlotPath;
        logLines.push_back(QStringLiteral("Capture post-process (%1): wrote segmentation ROI CSV and spectrum plot")
                               .arg(streamLabel));
    }

    if (!options.saveFfcImage)
    {
        QFile::remove(ffcHdrForRgb);
        QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
    }

    QString manifestError;
    writeProcessingManifestJson(preprocessedDir, summary, {report}, options, &manifestError);

    return report;
}
} // namespace

CapturePostProcessResult processCaptureSession(const CaptureWriterSessionSummary &summary,
                                               const CapturePostProcessOptions &options)
{
    CapturePostProcessResult result;

    if (summary.sessionDirectory.isEmpty() || summary.streams.isEmpty())
    {
        result.logLines.push_back(
            QStringLiteral("Capture post-process: no capture session data to process."));
        return result;
    }

    std::vector<StreamProcessReport> reports;
    bool allOk = true;

    for (auto it = summary.streams.cbegin(); it != summary.streams.cend(); ++it)
    {
        StreamProcessReport report = processStream(summary, it.value(), options, result.logLines);
        reports.push_back(report);

        const bool streamOk = (report.darkPlotOk || report.whitePlotOk || report.ffcOk || report.rgbOk)
                              && report.errorMessage.isEmpty();
        if (!streamOk && !report.errorMessage.isEmpty())
            allOk = false;
        if (!report.darkPlotOk && !report.whitePlotOk && !report.ffcOk && !report.rgbOk
            && report.errorMessage.isEmpty())
        {
            result.logLines.push_back(
                QStringLiteral("Capture post-process (%1): nothing to process (missing refs/sample).")
                    .arg(report.relativeRoot.isEmpty() ? it.value().baseName : report.relativeRoot));
        }
    }

    result.success = allOk;
    if (allOk)
        result.logLines.push_front(QStringLiteral("Capture post-process: completed successfully."));
    else
        result.logLines.push_front(QStringLiteral("Capture post-process: completed with errors."));

    if (!reports.empty())
    {
        QString datasetName;
        for (auto it = summary.streams.cbegin(); it != summary.streams.cend(); ++it)
        {
            datasetName = datasetStemFromStreamBaseName(it.value().baseName);
            break;
        }

        if (!datasetName.isEmpty())
            updateDatasetManifestForPreprocessing(summary.sessionDirectory, datasetName, reports,
                                                  options);
    }

    return result;
}

} // namespace hf::processing
