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
    struct ReferenceCaptureState
    {
        std::unique_ptr<QFile> rawFile;
        int width = 0;
        int bands = 0;
        std::uint64_t frameCount = 0;
        QString rawPath;
        QString hdrPath;
        QDateTime firstFrameUtc;
        QDateTime lastFrameUtc;
    };

    struct StreamState
    {
        CaptureWriterStreamConfig config;
        QString streamRoot;
        QString baseName;
        std::unique_ptr<QFile> rawFile;
        ReferenceCaptureState blackReference;
        ReferenceCaptureState whiteReference;
        CaptureWriterStreamSummary summary;
        QDateTime firstFrameUtc;
        QDateTime lastFrameUtc;
    };

    bool ensureStream(const FramePacket &frame, StreamState *&stateOut, QString *errorMessage);
    bool appendReferenceFrame(ReferenceCaptureState &reference,
                              const QString &fileBaseName,
                              StreamState &stream,
                              const FramePacket &frame,
                              QString *errorMessage);
    bool writeFramePayload(QFile &file, const FramePacket &frame, QString *errorMessage);
    void writePropertiesXml() const;
    void writeMetadataXml(const StreamState &stream) const;
    void writeManifestXml() const;
    void writeStreamHdr(const StreamState &stream) const;
    void writeReferenceHdr(const StreamState &stream, const ReferenceCaptureState &reference) const;
    void writeStreamLog(const StreamState &stream) const;
    bool copyMetadataStylesheet(const QString &streamRoot, QString *errorMessage);

    bool active_ = false;
    QString sessionDirectory_;
    QString datasetName_;
    QString operatorName_;
    QString description_;
    QDateTime sessionStartedUtc_;
    std::map<QString, StreamState> streams_;
};
