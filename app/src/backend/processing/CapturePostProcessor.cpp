#include "backend/processing/CapturePostProcessor.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/processing/FlatFieldCorrector.hpp"
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
    QString darkPlotPath;
    QString whitePlotPath;
    QString ffcHdrPath;
    QString ffcRawPath;
    QString rgbPath;
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

bool writeManifest(const QString &preprocessedDir,
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

    const hf::HardwareConfig::PreprocessingConfig &cfg = hf::hardwareConfig().preprocessing;
    QJsonObject configObject;
    configObject.insert(QStringLiteral("illuminantD"), cfg.illuminantD);
    configObject.insert(QStringLiteral("ffcEpsilon"), cfg.ffcEpsilon);
    configObject.insert(QStringLiteral("ffcClampMin"), cfg.ffcClampMin);
    configObject.insert(QStringLiteral("ffcClampMax"), cfg.ffcClampMax);
    configObject.insert(QStringLiteral("truncateNm"), cfg.truncateNm);
    configObject.insert(QStringLiteral("illuminantsJson"), defaultIlluminantsJsonPath());
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
        streamObject.insert(QStringLiteral("darkPlotPath"), report.darkPlotPath);
        streamObject.insert(QStringLiteral("whitePlotPath"), report.whitePlotPath);
        streamObject.insert(QStringLiteral("ffcHdrPath"), report.ffcHdrPath);
        streamObject.insert(QStringLiteral("ffcRawPath"), report.ffcRawPath);
        streamObject.insert(QStringLiteral("rgbPath"), report.rgbPath);
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
                    QStringLiteral("Capture post-process (%1): dark reference plot failed — %2")
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
            logLines.push_back(QStringLiteral("Capture post-process (%1): white row reference failed — %2")
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
                    QStringLiteral("Capture post-process (%1): white reference plot failed — %2")
                        .arg(streamLabel, error));
            }
        }
    }

    const bool canProcessSample = !stream.hdrPath.isEmpty() && stream.frameCount > 0 && darkRowReady
                                  && whiteRowReady;

    if (!canProcessSample)
        return report;

    const QString ffcBaseName = datasetStem + QStringLiteral("_ffc");
    report.ffcHdrPath = QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral(".hdr"));
    report.ffcRawPath = QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral(".raw"));
    report.rgbPath = QDir(preprocessedDir).filePath(datasetStem + QStringLiteral("_rgb.png"));

    const QString sensorLabel = stream.baseName;
    const FlatFieldParams ffcParams = flatFieldParamsFromConfig();

    QString ffcHdrForRgb = report.ffcHdrPath;
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

    if (!writeReflectanceRgbPngFromLines(ffcMetadata,
                                         QFileInfo(ffcHdrForRgb).absolutePath() + QLatin1Char('/')
                                             + QFileInfo(ffcHdrForRgb).completeBaseName()
                                             + QStringLiteral(".raw"),
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
    logLines.push_back(QStringLiteral("Capture post-process (%1): wrote %2")
                           .arg(streamLabel, QFileInfo(report.rgbPath).fileName()));

    if (!options.saveFfcImage)
    {
        QFile::remove(ffcHdrForRgb);
        QFile::remove(QDir(preprocessedDir).filePath(ffcBaseName + QStringLiteral("_tmp.raw")));
    }

    QString manifestError;
    writeManifest(preprocessedDir, summary, {report}, options, &manifestError);

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

    Q_UNUSED(reports);
    return result;
}

} // namespace hf::processing
