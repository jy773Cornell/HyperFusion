#pragma once

#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/camera/CameraTypes.hpp"

#include <QMap>
#include <QString>

#include <vector>

enum class CaptureIlluminationMode
{
    Reflectance,
    Transmittance,
};

struct CaptureWriterStreamConfig
{
    CameraBackendId source = CameraBackendId::Camera1;
    QString streamName;
    CaptureIlluminationMode illuminationMode = CaptureIlluminationMode::Reflectance;
    /// Relative path under the dataset root, e.g. "reflectance/fx10e" or "recording/fx10e".
    QString relativeRoot;
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
    QString relativeRoot;
    QString baseName;
    QString rawPath;
    QString hdrPath;
    QString logPath;
    int width = 0;
    int bands = 0;
    std::uint64_t frameCount = 0;
    std::uint64_t bytesWritten = 0;
    QString blackReferenceRawPath;
    QString blackReferenceHdrPath;
    QString whiteReferenceRawPath;
    QString whiteReferenceHdrPath;
    std::uint64_t blackReferenceFrameCount = 0;
    std::uint64_t whiteReferenceFrameCount = 0;
};

struct CaptureWriterSessionSummary
{
    QString sessionDirectory;
    QMap<QString, CaptureWriterStreamSummary> streams;
    bool active = false;
};
