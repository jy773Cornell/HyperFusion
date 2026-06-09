#pragma once

#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/CameraTypes.hpp"

#include <QMap>
#include <QString>

#include <vector>

struct CaptureWriterStreamConfig
{
    CameraBackendId source = CameraBackendId::Camera1;
    QString streamName;
    CameraSettings settings;
    std::vector<SpectralBand> spectralBands;
    QString calibrationPackPath;
};

struct CaptureWriterSessionConfig
{
    QString saveFolder;
    QString datasetName;
    QString operatorName;
    QString description;
    std::vector<CaptureWriterStreamConfig> streams;
};

struct CaptureWriterStreamSummary
{
    QString baseName;
    QString rawPath;
    QString hdrPath;
    QString logPath;
    int width = 0;
    int bands = 0;
    std::uint64_t frameCount = 0;
    std::uint64_t bytesWritten = 0;
};

struct CaptureWriterSessionSummary
{
    QString sessionDirectory;
    QMap<CameraBackendId, CaptureWriterStreamSummary> streams;
    bool active = false;
};
