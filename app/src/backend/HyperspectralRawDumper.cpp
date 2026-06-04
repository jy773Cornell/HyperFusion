#include "backend/HyperspectralRawDumper.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QSaveFile>

#include <cstring>

namespace
{
constexpr char kPixelFormat[] = "uint16_le";
constexpr char kInterleave[] = "bil";
} // namespace

HyperspectralRawDumper::~HyperspectralRawDumper()
{
    endSession();
}

QString HyperspectralRawDumper::streamLabel(const CameraBackendId source)
{
    switch (source)
    {
    case CameraBackendId::Camera1:
        return QStringLiteral("camera1");
    case CameraBackendId::Camera2:
        return QStringLiteral("camera2");
    default:
        return QStringLiteral("camera");
    }
}

bool HyperspectralRawDumper::beginSession(const QString &parentDirectory,
                                         const QString &datasetName,
                                         QString *errorMessage)
{
    endSession();

    if (parentDirectory.trimmed().isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Save folder is empty.");
        return false;
    }

    if (datasetName.trimmed().isEmpty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Dataset name is empty.");
        return false;
    }

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    sessionDirectory_ =
        QDir(parentDirectory).filePath(datasetName.trimmed() + QStringLiteral("_") + timestamp);
    datasetName_ = datasetName.trimmed();

    QDir dir;
    if (!dir.mkpath(sessionDirectory_))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not create output directory: %1").arg(sessionDirectory_);
        sessionDirectory_.clear();
        datasetName_.clear();
        return false;
    }

    active_ = true;
    return true;
}

bool HyperspectralRawDumper::ensureStream(const FramePacket &frame, QString *errorMessage)
{
    if (!active_)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Raw dump session is not active.");
        return false;
    }

    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Frame geometry is empty.");
        return false;
    }

    const std::size_t required =
        static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    if (frame.pixels.size() < required)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Frame buffer is smaller than %1 × %2.")
                                 .arg(frame.width)
                                 .arg(frame.height);
        return false;
    }

    auto streamIt = streamInfo_.constFind(frame.source);
    if (streamIt != streamInfo_.cend())
    {
        if (streamIt->width != frame.width || streamIt->bands != frame.height)
        {
            if (errorMessage != nullptr)
            {
                *errorMessage =
                    QStringLiteral("%1 frame size changed (%2×%3 → %4×%5); open a new session.")
                        .arg(streamLabel(frame.source))
                        .arg(streamIt->width)
                        .arg(streamIt->bands)
                        .arg(frame.width)
                        .arg(frame.height);
            }
            return false;
        }
        return true;
    }

    const QString fileName = datasetName_ + QStringLiteral("_") + streamLabel(frame.source)
                             + QStringLiteral(".raw");
    const QString filePath = QDir(sessionDirectory_).filePath(fileName);

    auto file = std::make_unique<QFile>(filePath);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not open %1 for writing.").arg(filePath);
        return false;
    }

    RawDumpStreamInfo info;
    info.filePath = filePath;
    info.width = frame.width;
    info.bands = frame.height;
    streamInfo_.insert(frame.source, info);
    files_.emplace(frame.source, std::move(file));
    return true;
}

bool HyperspectralRawDumper::writeFramePayload(QFile &file, const FramePacket &frame, QString *errorMessage)
{
    const std::size_t sampleCount =
        static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height);
    const std::size_t byteCount = sampleCount * sizeof(std::uint16_t);

    if (file.write(reinterpret_cast<const char *>(frame.pixels.data()),
                   static_cast<qint64>(byteCount))
        != static_cast<qint64>(byteCount))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Disk write failed for %1.").arg(file.fileName());
        return false;
    }

    return true;
}

bool HyperspectralRawDumper::appendFrame(const FramePacket &frame, QString *errorMessage)
{
    if (!ensureStream(frame, errorMessage))
        return false;

    auto fileIt = files_.find(frame.source);
    if (fileIt == files_.end() || fileIt->second == nullptr)
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("No output file for %1.").arg(streamLabel(frame.source));
        return false;
    }

    if (!writeFramePayload(*fileIt->second, frame, errorMessage))
        return false;

    RawDumpStreamInfo &info = streamInfo_[frame.source];
    ++info.frameCount;
    info.bytesWritten += static_cast<std::uint64_t>(frame.width) * static_cast<std::uint64_t>(frame.height)
                         * sizeof(std::uint16_t);
    return true;
}

void HyperspectralRawDumper::writeSessionMetadata() const
{
    if (sessionDirectory_.isEmpty())
        return;

    QJsonObject root;
    root.insert(QStringLiteral("dataset"), datasetName_);
    root.insert(QStringLiteral("createdUtc"),
                QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
    root.insert(QStringLiteral("pixelFormat"), QString::fromLatin1(kPixelFormat));
    root.insert(QStringLiteral("interleave"), QString::fromLatin1(kInterleave));
    root.insert(QStringLiteral("notes"),
                QStringLiteral("Each .raw file is a sequence of BIL frames: bands × width uint16 LE, "
                               "concatenated in arrival order."));

    QJsonArray streams;
    for (auto it = streamInfo_.cbegin(); it != streamInfo_.cend(); ++it)
    {
        QJsonObject stream;
        stream.insert(QStringLiteral("camera"), streamLabel(it.key()));
        stream.insert(QStringLiteral("file"), QFileInfo(it->filePath).fileName());
        stream.insert(QStringLiteral("width"), it->width);
        stream.insert(QStringLiteral("bands"), it->bands);
        stream.insert(QStringLiteral("frameCount"), static_cast<qint64>(it->frameCount));
        stream.insert(QStringLiteral("bytesWritten"), static_cast<qint64>(it->bytesWritten));
        streams.append(stream);
    }
    root.insert(QStringLiteral("streams"), streams);

    const QString metaPath = QDir(sessionDirectory_).filePath(QStringLiteral("session.json"));
    QSaveFile metaFile(metaPath);
    if (!metaFile.open(QIODevice::WriteOnly | QIODevice::Truncate))
        return;

    metaFile.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    metaFile.commit();
}

RawDumpSessionSummary HyperspectralRawDumper::endSession()
{
    RawDumpSessionSummary summary;
    summary.sessionDirectory = sessionDirectory_;
    summary.streams = streamInfo_;
    summary.active = active_;

    for (auto &entry : files_)
    {
        if (entry.second != nullptr)
            entry.second->close();
    }
    files_.clear();

    if (active_)
        writeSessionMetadata();

    active_ = false;
    streamInfo_.clear();
    sessionDirectory_.clear();
    datasetName_.clear();
    return summary;
}
