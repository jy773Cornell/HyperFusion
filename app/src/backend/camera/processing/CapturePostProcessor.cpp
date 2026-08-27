// Offline post-processing for completed stage-scan capture sessions (backend).
#include "backend/camera/processing/CapturePostProcessor.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/EnviBilReader.hpp"
#include "backend/camera/processing/FlatFieldCorrector.hpp"
#include "backend/camera/processing/Gsam2RoiAnalysis.hpp"
#include "backend/camera/processing/Gsam2SegmentationClient.hpp"
#include "backend/camera/processing/GsamPlan.hpp"
#include "backend/camera/processing/GsamPlanSegmentRunner.hpp"
#include "backend/camera/processing/HfFusionRunner.hpp"
#include "backend/camera/processing/HsiToColor.hpp"
#include "backend/camera/processing/IlluminantTables.hpp"
#include "backend/camera/processing/ReferenceBuilder.hpp"
#include "backend/camera/processing/ReferenceSpectrumPlot.hpp"
#include "backend/camera/processing/ReferenceSpectrumStats.hpp"
#include "backend/camera/processing/SwirRefBprCorrector.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

#include <algorithm>
#include <cstdint>

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
    QString gsamPromptApplied;
    bool rgbInverted = false;
    bool swirRefBprApplied = false;
    int swirRefBprBadColumns = 0;
    int swirRefBprBadPixels = 0;
    double gsamBoxThreshold = 0.0;
    double gsamMaxBoxAreaFrac = 0.0;
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
    if (traits.isSwir3)
        return ReflectanceRgbExportMode::SwirFalseColorRanges;
    return ReflectanceRgbExportMode::SpectralToSrgb;
}

QString rgbExportModeLabel(const ReflectanceRgbExportMode mode)
{
    if (mode == ReflectanceRgbExportMode::SwirFalseColorRanges)
        return QStringLiteral("swir_false_color");
    return QStringLiteral("srgb");
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

SwirRefBprSettings swirRefBprSettingsFromConfig()
{
    const hf::HardwareConfig::PreprocessingConfig &cfg = hf::hardwareConfig().preprocessing;
    SwirRefBprSettings settings;
    settings.baselineRadius = cfg.swir3RefBprBaselineRadius;
    settings.whiteRatioMin = cfg.swir3RefBprWhiteRatioMin;
    settings.whiteRatioMax = cfg.swir3RefBprWhiteRatioMax;
    settings.darkAbsMinDn = cfg.swir3RefBprDarkAbsMinDn;
    settings.darkAbsScale = cfg.swir3RefBprDarkAbsScale;
    settings.columnPromoteFrac = cfg.swir3RefBprColumnPromoteFrac;
    return settings;
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
    writeKey(QStringLiteral("run hf fusion"),
             options.runHfFusion ? QStringLiteral("yes") : QStringLiteral("no"));
    if (!options.gsamPrompt.trimmed().isEmpty())
        writeKey(QStringLiteral("gsam prompt"), options.gsamPrompt.trimmed());
    if (options.runGsamSegmentation)
        writeKey(QStringLiteral("gsam sample count"), QString::number(options.gsamSampleCount));
    if (!report.rgbExportMode.isEmpty())
        writeKey(QStringLiteral("rgb export mode"), report.rgbExportMode);
    if (report.swirRefBprApplied)
    {
        writeKey(QStringLiteral("swir ref bpr"), QStringLiteral("yes"));
        writeKey(QStringLiteral("swir ref bpr bad columns"),
                 QString::number(report.swirRefBprBadColumns));
        writeKey(QStringLiteral("swir ref bpr bad pixels"),
                 QString::number(report.swirRefBprBadPixels));
    }

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

void appendPreviewSheetEntries(const QString &sessionDirectory, const QStringList &sheetPaths)
{
    const QString manifestPath = QDir(sessionDirectory).filePath(QStringLiteral("manifest.xml"));
    if (!QFileInfo::exists(manifestPath) || sheetPaths.isEmpty())
        return;

    std::vector<ManifestFileEntry> entries;
    for (const QString &path : sheetPaths)
    {
        if (path.isEmpty() || !QFileInfo::exists(path))
            continue;
        const QFileInfo info(path);
        entries.push_back({info.suffix().toLower(), QStringLiteral("preview"),
                           manifestRelativePath(sessionDirectory, path)});
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
    root.insert(QStringLiteral("runHfFusion"), options.runHfFusion);
    root.insert(QStringLiteral("hfFusionMode"), options.hfFusionMode);
    root.insert(QStringLiteral("gsamPrompt"), options.gsamPrompt);
    root.insert(QStringLiteral("gsamSampleCount"), options.gsamSampleCount);
    if (!options.gsamPlanPath.isEmpty())
    {
        root.insert(QStringLiteral("gsamPlanPath"), options.gsamPlanPath);
        root.insert(QStringLiteral("gsamPlanId"), QFileInfo(options.gsamPlanPath).completeBaseName());
    }

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
    configObject.insert(QStringLiteral("swir3RefBpr"), cfg.swir3RefBprCorrect);
    configObject.insert(QStringLiteral("swir3RefBprBaselineRadius"), cfg.swir3RefBprBaselineRadius);
    configObject.insert(QStringLiteral("swir3RefBprWhiteRatioMin"), cfg.swir3RefBprWhiteRatioMin);
    configObject.insert(QStringLiteral("swir3RefBprWhiteRatioMax"), cfg.swir3RefBprWhiteRatioMax);
    configObject.insert(QStringLiteral("swir3RefBprDarkAbsMinDn"), cfg.swir3RefBprDarkAbsMinDn);
    configObject.insert(QStringLiteral("swir3RefBprDarkAbsScale"), cfg.swir3RefBprDarkAbsScale);
    configObject.insert(QStringLiteral("swir3RefBprColumnPromoteFrac"), cfg.swir3RefBprColumnPromoteFrac);
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
        if (report.swirRefBprApplied)
        {
            streamObject.insert(QStringLiteral("swirRefBprApplied"), true);
            streamObject.insert(QStringLiteral("swirRefBprBadColumns"), report.swirRefBprBadColumns);
            streamObject.insert(QStringLiteral("swirRefBprBadPixels"), report.swirRefBprBadPixels);
        }
        if (!report.rgbExportMode.isEmpty())
            streamObject.insert(QStringLiteral("rgbExportMode"), report.rgbExportMode);
        if (!report.gsamPromptApplied.isEmpty())
            streamObject.insert(QStringLiteral("gsamPrompt"), report.gsamPromptApplied);
        if (report.rgbInverted)
            streamObject.insert(QStringLiteral("rgbInverted"), true);
        if (report.gsamBoxThreshold > 0.0)
            streamObject.insert(QStringLiteral("gsamBoxThreshold"), report.gsamBoxThreshold);
        if (report.gsamMaxBoxAreaFrac > 0.0)
            streamObject.insert(QStringLiteral("gsamMaxBoxAreaFrac"), report.gsamMaxBoxAreaFrac);
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
    SampleLineMutator sampleLineMutator;
    SwirRefBprCorrector swirRefBpr;

    if (streamTraits.isSwir3 && cfg.swir3RefBprCorrect)
    {
        swirRefBpr.setSettings(swirRefBprSettingsFromConfig());
        QString bprError;
        if (!swirRefBpr.buildFromReferences(whiteRow, darkRow, whiteMeta.samples, whiteMeta.bands,
                                            &bprError))
        {
            logLines.push_back(
                QStringLiteral("Capture post-process (%1): SWIR ref BPR skipped \u2014 %2")
                    .arg(streamLabel, bprError));
        }
        else
        {
            swirRefBpr.applyToFloatRow(whiteRow);
            swirRefBpr.applyToFloatRow(darkRow);
            sampleLineMutator = [&swirRefBpr](std::uint16_t *linePixels) {
                swirRefBpr.applyToUint16Line(linePixels);
            };
            report.swirRefBprApplied = true;
            report.swirRefBprBadColumns = static_cast<int>(swirRefBpr.badColumnCount());
            report.swirRefBprBadPixels = static_cast<int>(swirRefBpr.badPixelCount());
            logLines.push_back(
                QStringLiteral("Capture post-process (%1): SWIR ref BPR interpolated %2 columns (%3 pixels)")
                    .arg(streamLabel)
                    .arg(report.swirRefBprBadColumns)
                    .arg(report.swirRefBprBadPixels));
        }
    }

    QString ffcHdrForRgb = report.ffcHdrPath;
    // Plan-driven GSAM (Python CLI) reads `{stem}_ffc.hdr` from preprocessed/. Keep the cube.
    const bool writeFfcToSession =
        options.saveFfcImage || options.runHfFusion || options.runGsamSegmentation;
    const bool keepFfcForSegmentation = options.runGsamSegmentation && !writeFfcToSession;
    if (!writeFfcToSession)
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
                                   &error,
                                   sampleLineMutator))
    {
        report.errorMessage = QStringLiteral("FFC failed: %1").arg(error);
        logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
        return report;
    }

    report.ffcOk = writeFfcToSession;
    if (writeFfcToSession)
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

    GsamPlanStreamSettings planStream;
    bool usePlan = false;
    GsamPlan loadedPlan;
    if (!options.gsamPlanPath.isEmpty())
    {
        QString planError;
        if (!loadGsamPlan(options.gsamPlanPath, &loadedPlan, &planError))
        {
            report.errorMessage = planError;
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, planError));
            return report;
        }
        planStream = resolveGsamPlanStream(loadedPlan, stream.relativeRoot, streamTraits.isTransmittance,
                                           streamTraits.isSwir3);
        usePlan = true;
    }

    if (usePlan && planStream.hasInvertRgb && planStream.invertRgb)
    {
        QImage rgbImage(report.rgbPath);
        if (rgbImage.isNull())
        {
            report.errorMessage = QStringLiteral("Could not reload RGB for invert: %1").arg(report.rgbPath);
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
            return report;
        }
        rgbImage.invertPixels(QImage::InvertRgb);
        if (!rgbImage.save(report.rgbPath))
        {
            report.errorMessage = QStringLiteral("Could not save inverted RGB: %1").arg(report.rgbPath);
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
            return report;
        }
        report.rgbInverted = true;
        logLines.push_back(QStringLiteral("Capture post-process (%1): inverted RGB for GSAM plan")
                               .arg(streamLabel));
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
        segRequest.imageName = QFileInfo(report.rgbPath).fileName();
        segRequest.serverUrl = options.gsamServerUrl;

        if (usePlan)
        {
            if (planStream.hasPrompt && !planStream.prompt.isEmpty())
                segRequest.prompt = planStream.prompt;
            else
                segRequest.prompt = options.gsamPrompt.trimmed().isEmpty() ? QStringLiteral("sample.")
                                                                            : options.gsamPrompt.trimmed();
            segRequest.maxDetections =
                loadedPlan.hasSampleCount ? std::max(1, loadedPlan.sampleCount)
                                          : std::max(1, options.gsamSampleCount);
            segRequest.boxThreshold =
                planStream.hasBoxThreshold ? planStream.boxThreshold : segCfg.boxThreshold;
            if (planStream.hasMaxBoxAreaFrac && planStream.maxBoxAreaFrac > 0.0)
                segRequest.maxBoxAreaFrac = planStream.maxBoxAreaFrac;
        }
        else
        {
            segRequest.prompt = options.gsamPrompt.trimmed().isEmpty() ? QStringLiteral("sample.")
                                                                        : options.gsamPrompt.trimmed();
            segRequest.maxDetections = std::max(1, options.gsamSampleCount);
            segRequest.boxThreshold = segCfg.boxThreshold;
        }

        report.gsamPromptApplied = segRequest.prompt;
        report.gsamBoxThreshold = segRequest.boxThreshold;
        report.gsamMaxBoxAreaFrac = segRequest.maxBoxAreaFrac;

        QString segError;
        int detectionCount = 0;
        QString manifestJsonPath;
        if (usePlan)
        {
            if (planStream.hasReuseMasksFrom && !planStream.reuseMasksFrom.isEmpty())
            {
                report.gsamPromptApplied =
                    QStringLiteral("reused from %1").arg(planStream.reuseMasksFrom);
            }
            else if (planStream.twoStage.enabled)
            {
                const QString stage2Prompt = planStream.twoStage.stage2Prompt.isEmpty()
                                                 ? segRequest.prompt
                                                 : planStream.twoStage.stage2Prompt;
                report.gsamPromptApplied =
                    QStringLiteral("%1 box -> %2").arg(planStream.twoStage.stage1Prompt, stage2Prompt);
            }

            GsamPlanSegmentRunRequest planSegRequest;
            planSegRequest.sessionDirectory = summary.sessionDirectory;
            planSegRequest.planPath = options.gsamPlanPath;
            planSegRequest.stream = stream.relativeRoot.trimmed().toLower();
            if (planSegRequest.stream.isEmpty())
            {
                planSegRequest.stream =
                    gsamPlanStreamKey(streamTraits.isTransmittance, streamTraits.isSwir3);
            }
            planSegRequest.stem = datasetStem;
            planSegRequest.gsamServerUrl = options.gsamServerUrl;

            logLines.push_back(
                QStringLiteral("Capture post-process (%1): GSAM from plan (%2)")
                    .arg(streamLabel, report.gsamPromptApplied));

            const GsamPlanSegmentRunResult planSegResult = runGsamPlanSegment(planSegRequest);
            for (const QString &line : planSegResult.logLines)
                logLines.push_back(line);
            if (!planSegResult.success)
            {
                report.errorMessage = planSegResult.errorMessage.isEmpty()
                                          ? QStringLiteral("GSAM plan segmentation failed.")
                                          : planSegResult.errorMessage;
                logLines.push_back(
                    QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
                if (!writeFfcToSession && keepFfcForSegmentation)
                {
                    QFile::remove(ffcHdrForRgb);
                    QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
                }
                return report;
            }

            detectionCount = planSegResult.detectionCount;
            manifestJsonPath = QDir(segDir).filePath(QStringLiteral("segmentation_results.json"));
            if (!planSegResult.prompt.isEmpty())
                report.gsamPromptApplied = planSegResult.prompt;
        }
        else
        {
            const Gsam2SegmentationResponse segResponse = requestGsam2Segmentation(segRequest, &segError);
            if (!segResponse.ok)
            {
                report.errorMessage =
                    segError.isEmpty() ? QStringLiteral("GSAM2 segmentation failed.") : segError;
                logLines.push_back(
                    QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
                if (!writeFfcToSession && keepFfcForSegmentation)
                {
                    QFile::remove(ffcHdrForRgb);
                    QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
                }
                return report;
            }
            detectionCount = segResponse.detectionCount;
            manifestJsonPath = segResponse.manifestJsonPath;
        }

        logLines.push_back(QStringLiteral("Capture post-process (%1): GSAM2 found %2 ROI(s)")
                               .arg(streamLabel)
                               .arg(detectionCount));
        logLines.push_back(QStringLiteral("Capture post-process (%1): per-ROI segmented RGB in preprocessed/segmentation/segmented_rgb/")
                               .arg(streamLabel));

        const Gsam2RoiAnalysisResult roiResult = analyzeGsam2SegmentationRois(
            ffcHdrForRgb,
            segDir,
            QFileInfo(report.rgbPath).fileName(),
            manifestJsonPath,
            spectrumYAxisLabelForStream(streamTraits),
            &segError);
        if (!roiResult.success)
        {
            report.errorMessage = roiResult.errorMessage.isEmpty()
                                      ? QStringLiteral("GSAM2 ROI analysis failed.")
                                      : roiResult.errorMessage;
            logLines.push_back(QStringLiteral("Capture post-process (%1): %2").arg(streamLabel, report.errorMessage));
            if (!writeFfcToSession && keepFfcForSegmentation)
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

    if (!writeFfcToSession)
    {
        QFile::remove(ffcHdrForRgb);
        QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
    }

    QString manifestError;
    writeProcessingManifestJson(preprocessedDir, summary, {report}, options, &manifestError);

    return report;
}

QString streamRootFromManifestRelativePath(const QString &relativePath)
{
    const int captureIdx = relativePath.indexOf(QStringLiteral("/capture/"), Qt::CaseInsensitive);
    if (captureIdx < 0)
        return {};

    return relativePath.left(captureIdx);
}

std::uint64_t frameCountFromHdrPath(const QString &hdrPath)
{
    EnviBilMetadata metadata;
    if (!parseEnviHdr(hdrPath, metadata, nullptr))
        return 0;

    return metadata.lines > 0 ? static_cast<std::uint64_t>(metadata.lines) : 0;
}

void applyHdrPathsToStream(CaptureWriterStreamSummary &stream,
                           const QString &sessionDirectory,
                           const QString &relativeHdrPath,
                           const QString &type)
{
    const QString absoluteHdr = QDir(sessionDirectory).filePath(relativeHdrPath);
    const QString absoluteRaw =
        QFileInfo(absoluteHdr).absolutePath() + QLatin1Char('/')
        + QFileInfo(absoluteHdr).completeBaseName() + QStringLiteral(".raw");
    const std::uint64_t frameCount = frameCountFromHdrPath(absoluteHdr);

    if (type.compare(QStringLiteral("capture"), Qt::CaseInsensitive) == 0)
    {
        stream.hdrPath = absoluteHdr;
        stream.rawPath = absoluteRaw;
        stream.baseName = QFileInfo(absoluteHdr).completeBaseName();
        stream.frameCount = frameCount;
        return;
    }

    if (type.compare(QStringLiteral("darkref"), Qt::CaseInsensitive) == 0)
    {
        stream.blackReferenceHdrPath = absoluteHdr;
        stream.blackReferenceRawPath = absoluteRaw;
        stream.blackReferenceFrameCount = frameCount;
        return;
    }

    if (type.compare(QStringLiteral("whiteref"), Qt::CaseInsensitive) == 0)
    {
        stream.whiteReferenceHdrPath = absoluteHdr;
        stream.whiteReferenceRawPath = absoluteRaw;
        stream.whiteReferenceFrameCount = frameCount;
    }
}

bool discoverStreamFromCaptureDirectory(const QString &sessionDirectory,
                                        const QString &relativeRoot,
                                        CaptureWriterStreamSummary &streamOut)
{
    const QString captureDir =
        QDir(sessionDirectory).filePath(relativeRoot + QStringLiteral("/capture"));
    const QDir dir(captureDir);
    if (!dir.exists())
        return false;

    CaptureWriterStreamSummary stream;
    stream.relativeRoot = relativeRoot;

    const QStringList hdrFiles = dir.entryList({QStringLiteral("*.hdr")}, QDir::Files, QDir::Name);
    for (const QString &hdrName : hdrFiles)
    {
        const QString relativeHdrPath =
            manifestRelativePath(sessionDirectory, dir.filePath(hdrName));
        if (hdrName.startsWith(QStringLiteral("DARKREF_"), Qt::CaseInsensitive))
            applyHdrPathsToStream(stream, sessionDirectory, relativeHdrPath, QStringLiteral("darkref"));
        else if (hdrName.startsWith(QStringLiteral("WHITEREF_"), Qt::CaseInsensitive))
            applyHdrPathsToStream(stream, sessionDirectory, relativeHdrPath, QStringLiteral("whiteref"));
        else if (stream.hdrPath.isEmpty())
            applyHdrPathsToStream(stream, sessionDirectory, relativeHdrPath, QStringLiteral("capture"));
    }

    if (stream.hdrPath.isEmpty())
        return false;

    streamOut = std::move(stream);
    return true;
}

bool includeRelativeRoot(const QString &relativeRoot, const QStringList &relativeRootsFilter)
{
    if (relativeRootsFilter.isEmpty())
        return true;

    for (const QString &filterRoot : relativeRootsFilter)
    {
        if (relativeRoot.compare(filterRoot, Qt::CaseInsensitive) == 0)
            return true;
    }

    return false;
}

bool loadStreamsFromManifest(const QString &sessionDirectory,
                             const QStringList &relativeRootsFilter,
                             QMap<QString, CaptureWriterStreamSummary> &streamsOut,
                             QString *errorMessage)
{
    const QString manifestPath = QDir(sessionDirectory).filePath(QStringLiteral("manifest.xml"));
    if (!QFileInfo::exists(manifestPath))
        return false;

    QFile file(manifestPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open %1").arg(manifestPath);
        return false;
    }

    QXmlStreamReader xml(&file);
    while (!xml.atEnd())
    {
        xml.readNext();
        if (!xml.isStartElement() || xml.name() != QStringLiteral("file"))
            continue;

        const QString extension = xml.attributes().value(QStringLiteral("extension")).toString();
        if (extension.compare(QStringLiteral("hdr"), Qt::CaseInsensitive) != 0)
        {
            xml.skipCurrentElement();
            continue;
        }

        const QString type = xml.attributes().value(QStringLiteral("type")).toString();
        const QString relativePath = xml.readElementText().trimmed();
        if (relativePath.isEmpty())
            continue;

        const QString relativeRoot = streamRootFromManifestRelativePath(relativePath);
        if (relativeRoot.isEmpty() || !includeRelativeRoot(relativeRoot, relativeRootsFilter))
            continue;

        CaptureWriterStreamSummary &stream = streamsOut[relativeRoot];
        stream.relativeRoot = relativeRoot;
        applyHdrPathsToStream(stream, sessionDirectory, relativePath, type);
    }

    if (xml.hasError() && errorMessage != nullptr)
    {
        *errorMessage = QStringLiteral("manifest.xml parse error: %1").arg(xml.errorString());
        return false;
    }

    return !streamsOut.isEmpty();
}

void discoverStreamsFromFilesystem(const QString &sessionDirectory,
                                   const QStringList &relativeRootsFilter,
                                   QMap<QString, CaptureWriterStreamSummary> &streamsOut)
{
    const auto tryDiscover = [&](const QString &relativeRoot) {
        if (!includeRelativeRoot(relativeRoot, relativeRootsFilter))
            return;

        CaptureWriterStreamSummary stream;
        if (!discoverStreamFromCaptureDirectory(sessionDirectory, relativeRoot, stream))
            return;

        streamsOut.insert(relativeRoot, std::move(stream));
    };

    if (!relativeRootsFilter.isEmpty())
    {
        for (const QString &relativeRoot : relativeRootsFilter)
            tryDiscover(relativeRoot);
        return;
    }

    const QStringList modeEntries =
        QDir(sessionDirectory).entryList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    for (const QString &mode : modeEntries)
    {
        for (const QString &camera : {QStringLiteral("fx10e"), QStringLiteral("swir3")})
            tryDiscover(mode + QLatin1Char('/') + camera);
    }
}
} // namespace

CaptureSessionLoadResult loadCaptureSessionSummaryFromDisk(const QString &sessionDirectory,
                                                           const QStringList &relativeRootsFilter)
{
    CaptureSessionLoadResult result;
    const QString session = QDir::fromNativeSeparators(sessionDirectory.trimmed());
    if (session.isEmpty() || !QFileInfo(session).isDir())
    {
        result.errorMessage = QStringLiteral("Session directory not found.");
        return result;
    }

    QMap<QString, CaptureWriterStreamSummary> streams;
    QString manifestError;
    loadStreamsFromManifest(session, relativeRootsFilter, streams, &manifestError);

    for (auto it = streams.begin(); it != streams.end();)
    {
        if (it->hdrPath.isEmpty())
            it = streams.erase(it);
        else
            ++it;
    }

    if (streams.isEmpty())
        discoverStreamsFromFilesystem(session, relativeRootsFilter, streams);

    if (streams.isEmpty())
    {
        result.errorMessage = manifestError.isEmpty()
                                  ? QStringLiteral("No capture streams found under %1.").arg(session)
                                  : manifestError;
        return result;
    }

    result.summary.sessionDirectory = session;
    result.summary.streams = std::move(streams);
    result.summary.active = false;
    result.success = true;
    return result;
}

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

    bool anyProcessed = false;
    for (const StreamProcessReport &report : reports)
    {
        if (report.darkPlotOk || report.whitePlotOk || report.ffcOk || report.rgbOk
            || report.segmentationOk)
        {
            anyProcessed = true;
            break;
        }
    }
    if (anyProcessed)
    {
        const QStringList previewPaths =
            writeSessionPreviewSheets(summary.sessionDirectory, &result.logLines);
        appendPreviewSheetEntries(summary.sessionDirectory, previewPaths);
    }

    if (options.runHfFusion)
    {
        if (!allOk)
        {
            result.logLines.push_back(
                QStringLiteral("Capture fusion: skipped because per-camera post-process had errors."));
        }
        else
        {
            const HfFusionSessionResult fusionResult = runSessionFusion(summary.sessionDirectory);
            result.logLines.append(fusionResult.logLines);
            if (!fusionResult.success)
            {
                result.success = false;
                if (!result.logLines.isEmpty()
                    && result.logLines.front().contains(QStringLiteral("completed successfully")))
                {
                    result.logLines[0] =
                        QStringLiteral("Capture post-process: completed with errors.");
                }
            }
        }
    }

    return result;
}

} // namespace hf::processing
