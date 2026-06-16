// HTTP client for the GSAM2 WSL sidecar server.
#pragma once

#include <QString>

namespace hf::processing
{
struct Gsam2SegmentationRequest
{
    QString inputRgbPath;
    QString outputDirectory;
    QString imageName;
    QString prompt;
    int maxDetections = 5;
    double boxThreshold = 0.30;
    QString serverUrl = QStringLiteral("http://127.0.0.1:8765");
};

struct Gsam2SegmentationResponse
{
    bool ok = false;
    QString errorMessage;
    int detectionCount = 0;
    QString overlayPngPath;
    QString stackMaskPngPath;
    QString manifestJsonPath;
};

bool gsam2ServerHealthCheck(const QString &serverUrl, bool *modelLoaded, QString *errorMessage = nullptr);

bool gsam2ServerShutdown(const QString &serverUrl, QString *errorMessage = nullptr);

Gsam2SegmentationResponse requestGsam2Segmentation(const Gsam2SegmentationRequest &request,
                                                   QString *errorMessage = nullptr);

} // namespace hf::processing
