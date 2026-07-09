// ENVI BIL float32 writer for preprocessed hyperspectral output.
#pragma once

#include "backend/camera/processing/EnviBilReader.hpp"

#include <QDateTime>
#include <QString>

#include <cstdint>
#include <functional>
#include <vector>

namespace hf::processing
{
struct EnviFloatWriter
{
    QString hdrPath;
    QString rawPath;
    EnviBilMetadata metadata;
    QString sensorTypeLabel;
    QString description;
    int linesWritten = 0;
};

bool beginEnviFloatWriter(EnviFloatWriter &writer,
                          const QString &hdrPath,
                          const EnviBilMetadata &templateMetadata,
                          const QString &sensorTypeLabel,
                          const QString &description,
                          QString *errorMessage = nullptr);

bool appendEnviFloatLine(EnviFloatWriter &writer,
                         const float *linePixels,
                         QString *errorMessage = nullptr);

bool finalizeEnviFloatWriter(EnviFloatWriter &writer, QString *errorMessage = nullptr);

} // namespace hf::processing
