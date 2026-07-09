// ROI mean reflectance extraction from FFC ENVI cubes and GSAM2 mask outputs.
#include "backend/camera/processing/Gsam2RoiAnalysis.hpp"

#include "backend/camera/processing/EnviBilReader.hpp"
#include "backend/camera/processing/Gsam2SpectrumPlot.hpp"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>
#include <QTextStream>

#include <cmath>
#include <vector>

namespace hf::processing
{
namespace
{
struct MaskRoi
{
    int roiIndex = 0;
    QString label;
    int pixelCount = 0;
    QString maskPngPath;
    std::vector<std::uint8_t> pixels;
    int width = 0;
    int height = 0;
};

bool loadMaskPng(const QString &path, MaskRoi &roiOut, QString *errorMessage)
{
    const QImage image(path);
    if (image.isNull())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not read mask PNG: %1").arg(path);
        return false;
    }

    const QImage gray = image.convertToFormat(QImage::Format_Grayscale8);
    roiOut.width = gray.width();
    roiOut.height = gray.height();
    roiOut.maskPngPath = path;
    roiOut.pixels.resize(static_cast<std::size_t>(roiOut.width * roiOut.height));

    for (int y = 0; y < roiOut.height; ++y)
    {
        const uchar *scan = gray.constScanLine(y);
        for (int x = 0; x < roiOut.width; ++x)
            roiOut.pixels[static_cast<std::size_t>(y * roiOut.width + x)] = scan[x] >= 128 ? 1 : 0;
    }

    roiOut.pixelCount = 0;
    for (const std::uint8_t value : roiOut.pixels)
        roiOut.pixelCount += value;

    return true;
}

bool readDetectionsFromManifest(const QString &manifestPath,
                                const QString &segmentationDirectory,
                                std::vector<MaskRoi> &roisOut,
                                QString *errorMessage)
{
    QFile file(manifestPath);
    if (!file.open(QIODevice::ReadOnly))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not read segmentation manifest: %1").arg(manifestPath);
        return false;
    }

    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (!doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Invalid segmentation manifest JSON.");
        return false;
    }

    const QJsonArray detections = doc.object().value(QStringLiteral("detections")).toArray();
    for (const QJsonValue &value : detections)
    {
        const QJsonObject detection = value.toObject();
        MaskRoi roi;
        roi.roiIndex = detection.value(QStringLiteral("roi")).toInt(0);
        roi.label = detection.value(QStringLiteral("label")).toString(QStringLiteral("roi"));
        roi.pixelCount = detection.value(QStringLiteral("pixel_count")).toInt(0);

        QString maskPath = detection.value(QStringLiteral("mask_png")).toString();
        if (maskPath.isEmpty())
        {
            maskPath = QDir(segmentationDirectory)
                           .filePath(QStringLiteral("masks/mask_%1.png")
                                         .arg(roi.roiIndex, 3, 10, QLatin1Char('0')));
        }
        else if (!QFileInfo::exists(maskPath))
        {
            const QString fileName = QFileInfo(maskPath).fileName();
            const QString inMasksDir =
                QDir(segmentationDirectory).filePath(QStringLiteral("masks/%1").arg(fileName));
            if (QFileInfo::exists(inMasksDir))
                maskPath = inMasksDir;
            else
                maskPath = QDir(segmentationDirectory).filePath(fileName);
        }

        if (!loadMaskPng(maskPath, roi, errorMessage))
            return false;

        roisOut.push_back(std::move(roi));
    }

    return !roisOut.empty();
}

bool writeRoiCsv(const QString &csvPath,
                 const QString &imageName,
                 const std::vector<MaskRoi> &rois,
                 const std::vector<double> &wavelengthsNm,
                 const std::vector<std::vector<double>> &means,
                 QString *errorMessage)
{
    QSaveFile file(csvPath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write ROI CSV: %1").arg(csvPath);
        return false;
    }

    QTextStream out(&file);
    out << "image,label,roi#,pixel_num";
    for (const double wavelength : wavelengthsNm)
        out << ',' << wavelength;
    out << '\n';

    for (std::size_t roiIndex = 0; roiIndex < rois.size(); ++roiIndex)
    {
        const MaskRoi &roi = rois[roiIndex];
        const std::vector<double> &bandMeans = means[roiIndex];
        const QString escapedLabel = [&roi]() {
            QString text = roi.label;
            text.replace(QLatin1Char('"'), QStringLiteral("\"\""));
            return text;
        }();
        out << imageName << ','
            << '"' << escapedLabel << '"' << ','
            << roi.roiIndex << ','
            << roi.pixelCount;
        for (const double meanValue : bandMeans)
            out << ',' << meanValue;
        out << '\n';
    }

    if (!file.commit())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not save ROI CSV.");
        return false;
    }

    return true;
}
} // namespace

Gsam2RoiAnalysisResult analyzeGsam2SegmentationRois(const QString &ffcHdrPath,
                                                     const QString &segmentationDirectory,
                                                     const QString &imageName,
                                                     const QString &manifestJsonPath,
                                                     const QString &yAxisLabel,
                                                     QString *errorMessage)
{
    Gsam2RoiAnalysisResult result;

    EnviBilMetadata metadata;
    QString localError;
    if (!parseEnviHdr(ffcHdrPath, metadata, &localError))
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    std::vector<MaskRoi> rois;
    if (!readDetectionsFromManifest(manifestJsonPath, segmentationDirectory, rois, &localError))
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (metadata.samples != rois.front().width || metadata.lines != rois.front().height)
    {
        result.errorMessage =
            QStringLiteral("Mask size (%1\u00D7%2) does not match FFC cube (%3\u00D7%4).")
                .arg(rois.front().width)
                .arg(rois.front().height)
                .arg(metadata.samples)
                .arg(metadata.lines);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    const int bands = metadata.bands;
    const int samples = metadata.samples;
    const std::vector<double> wavelengthsNm = metadata.wavelengthsNm;

    std::vector<std::vector<double>> means(rois.size(), std::vector<double>(static_cast<std::size_t>(bands), 0.0));
    std::vector<std::vector<double>> sumSq(rois.size(), std::vector<double>(static_cast<std::size_t>(bands), 0.0));
    std::vector<std::vector<int>> counts(rois.size(), std::vector<int>(static_cast<std::size_t>(bands), 0));

    const QString rawPath = metadata.rawPath.isEmpty()
                                ? QFileInfo(ffcHdrPath).absolutePath() + QLatin1Char('/')
                                      + QFileInfo(ffcHdrPath).completeBaseName() + QStringLiteral(".raw")
                                : metadata.rawPath;

    int lineIndex = 0;
    const bool readOk = readEnviFloatBilLines(
        metadata,
        rawPath,
        [&](const float *linePixels) {
            if (lineIndex < 0 || lineIndex >= rois.front().height)
            {
                ++lineIndex;
                return true;
            }

            for (int sample = 0; sample < samples; ++sample)
            {
                for (std::size_t roiIndex = 0; roiIndex < rois.size(); ++roiIndex)
                {
                    const MaskRoi &roi = rois[roiIndex];
                    const std::size_t maskIndex =
                        static_cast<std::size_t>(lineIndex * roi.width + sample);
                    if (maskIndex >= roi.pixels.size() || roi.pixels[maskIndex] == 0)
                        continue;

                    for (int band = 0; band < bands; ++band)
                    {
                        const float value =
                            linePixels[bilLinePixelIndex(sample, band, samples)];
                        const double dValue = static_cast<double>(value);
                        means[roiIndex][static_cast<std::size_t>(band)] += dValue;
                        sumSq[roiIndex][static_cast<std::size_t>(band)] += dValue * dValue;
                        counts[roiIndex][static_cast<std::size_t>(band)] += 1;
                    }
                }
            }

            ++lineIndex;
            return true;
        },
        &localError);

    if (!readOk)
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    std::vector<RoiSpectrumSeries> plotSeries;
    plotSeries.reserve(rois.size());

    for (std::size_t roiIndex = 0; roiIndex < rois.size(); ++roiIndex)
    {
        RoiSpectrumSeries series;
        series.label = QStringLiteral("ROI %1: %2").arg(rois[roiIndex].roiIndex).arg(rois[roiIndex].label);
        series.mean.resize(static_cast<std::size_t>(bands));
        series.std.resize(static_cast<std::size_t>(bands));

        for (int band = 0; band < bands; ++band)
        {
            const int count = counts[roiIndex][static_cast<std::size_t>(band)];
            if (count <= 0)
            {
                series.mean[static_cast<std::size_t>(band)] = 0.0;
                series.std[static_cast<std::size_t>(band)] = 0.0;
                continue;
            }

            const double mean = means[roiIndex][static_cast<std::size_t>(band)] / static_cast<double>(count);
            const double meanSq =
                sumSq[roiIndex][static_cast<std::size_t>(band)] / static_cast<double>(count);
            const double variance = std::max(0.0, meanSq - mean * mean);
            series.mean[static_cast<std::size_t>(band)] = mean;
            series.std[static_cast<std::size_t>(band)] = std::sqrt(variance);
            means[roiIndex][static_cast<std::size_t>(band)] = mean;
        }

        plotSeries.push_back(std::move(series));
    }

    const QString csvPath = QDir(segmentationDirectory).filePath(QStringLiteral("roi_spectra.csv"));
    if (!writeRoiCsv(csvPath, imageName, rois, wavelengthsNm, means, &localError))
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    const QString plotPath =
        QDir(segmentationDirectory).filePath(QStringLiteral("roi_spectra_plot.png"));
    const QString plotTitle =
        QStringLiteral("ROI %1 spectra").arg(yAxisLabel.toLower());
    if (!saveRoiSpectrumMeanStdPlotPng(wavelengthsNm,
                                       plotSeries,
                                       plotTitle,
                                       yAxisLabel,
                                       plotPath,
                                       &localError))
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    result.success = true;
    result.csvPath = csvPath;
    result.spectrumPlotPath = plotPath;
    return result;
}

} // namespace hf::processing
