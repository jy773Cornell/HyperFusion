#pragma once

#include "backend/CaptureWriterTypes.hpp"

#include <QDateTime>
#include <QFile>
#include <QString>

#include <map>
#include <memory>

class QFile;

/// Writes Lumo Scanner-style dataset folders (capture/, metadata/, manifest.xml, ENVI .raw/.hdr).
class LumoDatasetWriter
{
public:
    LumoDatasetWriter() = default;
    ~LumoDatasetWriter();

    bool begin(const CaptureWriterSessionConfig &config, QString *errorMessage = nullptr);
    bool appendFrame(const FramePacket &frame, QString *errorMessage = nullptr);
    CaptureWriterSessionSummary end();

    bool isActive() const { return active_; }
    const QString &sessionDirectory() const { return sessionDirectory_; }

private:
    struct StreamState
    {
        CaptureWriterStreamConfig config;
        QString baseName;
        std::unique_ptr<QFile> rawFile;
        CaptureWriterStreamSummary summary;
        QDateTime firstFrameUtc;
        QDateTime lastFrameUtc;
    };

    bool ensureStream(const FramePacket &frame, QString *errorMessage);
    bool writeFramePayload(QFile &file, const FramePacket &frame, QString *errorMessage);
    void writePropertiesXml() const;
    void writeMetadataXml() const;
    void writeManifestXml() const;
    void writeStreamHdr(const StreamState &stream) const;
    void writeStreamLog(const StreamState &stream) const;
    bool copyMetadataStylesheet(QString *errorMessage);

    bool active_ = false;
    QString sessionDirectory_;
    QString datasetName_;
    QString operatorName_;
    QString description_;
    QDateTime sessionStartedUtc_;
    std::map<CameraBackendId, StreamState> streams_;
};
