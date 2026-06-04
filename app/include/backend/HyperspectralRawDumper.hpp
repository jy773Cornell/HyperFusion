// Append-only uint16 BIL hyperspectral frame writer (.raw + sidecar metadata).
#pragma once

#include "backend/CameraTypes.hpp"

#include <QMap>
#include <QString>

#include <map>
#include <memory>

class QFile;

struct RawDumpStreamInfo
{
    QString filePath;
    int width = 0;
    int bands = 0;
    std::uint64_t frameCount = 0;
    std::uint64_t bytesWritten = 0;
};

struct RawDumpSessionSummary
{
    QString sessionDirectory;
    QMap<CameraBackendId, RawDumpStreamInfo> streams;
    bool active = false;
};

/// Writes Lumo BIL frames (band-major uint16: bands × width) as little-endian .raw streams.
class HyperspectralRawDumper
{
public:
    HyperspectralRawDumper() = default;
    ~HyperspectralRawDumper();

    static QString streamLabel(CameraBackendId source);

    bool beginSession(const QString &parentDirectory,
                      const QString &datasetName,
                      QString *errorMessage = nullptr);
    bool appendFrame(const FramePacket &frame, QString *errorMessage = nullptr);
    RawDumpSessionSummary endSession();

    bool isActive() const { return active_; }
    const QString &sessionDirectory() const { return sessionDirectory_; }

private:
    bool ensureStream(const FramePacket &frame, QString *errorMessage);
    bool writeFramePayload(QFile &file, const FramePacket &frame, QString *errorMessage);
    void writeSessionMetadata() const;

    bool active_ = false;
    QString sessionDirectory_;
    QString datasetName_;
    std::map<CameraBackendId, std::unique_ptr<QFile>> files_;
    QMap<CameraBackendId, RawDumpStreamInfo> streamInfo_;
};
