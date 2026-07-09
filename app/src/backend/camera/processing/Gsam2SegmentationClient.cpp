// HTTP client for the GSAM2 WSL sidecar server (via wsl curl; WSL2 localhost is not reachable from Windows).
#include "backend/camera/processing/Gsam2SegmentationClient.hpp"

#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/processing/GsamWslPathUtil.hpp"

#include <QDir>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QTemporaryFile>
#include <QUrl>

namespace hf::processing
{
namespace
{
QString wslDistroArgument()
{
    const QString distro = hf::hardwareConfig().segmentation.wslDistro.trimmed();
    return distro;
}

int serverPortFromUrl(const QString &serverUrl)
{
    const QUrl url(serverUrl);
    if (url.port() > 0)
        return url.port();
    return hf::hardwareConfig().segmentation.serverPort;
}

QString wslLocalEndpoint(const QString &serverUrl, const QString &path)
{
    return QStringLiteral("http://127.0.0.1:%1%2")
        .arg(serverPortFromUrl(serverUrl))
        .arg(path.startsWith(QLatin1Char('/')) ? path : QStringLiteral("/") + path);
}

QByteArray runWslCurl(const QStringList &curlArgs,
                      const int timeoutMs,
                      QString *errorMessage)
{
    QStringList wslArgs;
    const QString distro = wslDistroArgument();
    if (!distro.isEmpty())
    {
        wslArgs << QStringLiteral("-d") << distro;
    }
    wslArgs << QStringLiteral("--") << QStringLiteral("curl");
    wslArgs << curlArgs;

    QProcess process;
    process.setProgram(QStringLiteral("wsl.exe"));
    process.setArguments(wslArgs);
    process.start();

    if (!process.waitForStarted(10000))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Failed to start wsl.exe: %1").arg(process.errorString());
        return {};
    }

    if (!process.waitForFinished(timeoutMs + 5000))
    {
        process.kill();
        process.waitForFinished(2000);
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("GSAM2 WSL request timed out.");
        return {};
    }

    const QByteArray stdoutPayload = process.readAllStandardOutput();
    const QByteArray stderrPayload = process.readAllStandardError();

    if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
    {
        if (errorMessage != nullptr)
        {
            const QString detail = QString::fromUtf8(stderrPayload.trimmed());
            if (!detail.isEmpty())
                *errorMessage = detail;
            else
                *errorMessage = QStringLiteral("GSAM2 WSL curl failed (exit %1).")
                                    .arg(process.exitCode());
        }
        return {};
    }

    return stdoutPayload;
}

QJsonObject getJson(const QString &serverUrl,
                    const QString &path,
                    const int timeoutMs,
                    QString *errorMessage)
{
    const QStringList curlArgs = {
        QStringLiteral("-sfS"),
        QStringLiteral("-m"),
        QString::number(qMax(1, timeoutMs / 1000)),
        wslLocalEndpoint(serverUrl, path),
    };

    const QByteArray payload = runWslCurl(curlArgs, timeoutMs, errorMessage);
    if (payload.isEmpty() && (errorMessage == nullptr || !errorMessage->isEmpty()))
        return {};

    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("GSAM2 server returned invalid JSON.");
        return {};
    }

    return doc.object();
}

QJsonObject postJson(const QString &serverUrl,
                     const QString &path,
                     const QJsonObject &body,
                     const int timeoutMs,
                     QString *errorMessage)
{
    QTemporaryFile tempFile;
    tempFile.setAutoRemove(true);
    if (!tempFile.open())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not create temp file for GSAM2 request.");
        return {};
    }

    const QByteArray jsonBody = QJsonDocument(body).toJson(QJsonDocument::Compact);
    if (tempFile.write(jsonBody) != jsonBody.size())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("Could not write GSAM2 request body.");
        return {};
    }
    tempFile.close();

    const QString wslBodyPath = windowsPathToWsl(tempFile.fileName());
    const QStringList curlArgs = {
        QStringLiteral("-sfS"),
        QStringLiteral("-m"),
        QString::number(qMax(1, timeoutMs / 1000)),
        QStringLiteral("-X"),
        QStringLiteral("POST"),
        QStringLiteral("-H"),
        QStringLiteral("Content-Type: application/json"),
        QStringLiteral("-d"),
        QStringLiteral("@%1").arg(wslBodyPath),
        wslLocalEndpoint(serverUrl, path),
    };

    const QByteArray payload = runWslCurl(curlArgs, timeoutMs, errorMessage);
    if (payload.isEmpty() && (errorMessage == nullptr || !errorMessage->isEmpty()))
        return {};

    const QJsonDocument doc = QJsonDocument::fromJson(payload);
    if (!doc.isObject())
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("GSAM2 server returned invalid JSON.");
        return {};
    }

    return doc.object();
}
} // namespace

bool gsam2ServerHealthCheck(const QString &serverUrl, bool *modelLoaded, QString *errorMessage)
{
    const QJsonObject response = getJson(serverUrl, QStringLiteral("/health"), 3000, errorMessage);
    if (response.isEmpty())
        return false;

    if (response.value(QStringLiteral("status")).toString() != QStringLiteral("ok"))
    {
        if (errorMessage != nullptr)
            *errorMessage = QStringLiteral("GSAM2 server health check failed.");
        return false;
    }

    if (modelLoaded != nullptr)
        *modelLoaded = response.value(QStringLiteral("model_loaded")).toBool(false);

    return true;
}

bool gsam2ServerShutdown(const QString &serverUrl, QString *errorMessage)
{
    const QJsonObject response =
        postJson(serverUrl, QStringLiteral("/shutdown"), QJsonObject(), 5000, errorMessage);
    return !response.isEmpty() && response.value(QStringLiteral("ok")).toBool(false);
}

Gsam2SegmentationResponse requestGsam2Segmentation(const Gsam2SegmentationRequest &request,
                                                   QString *errorMessage)
{
    Gsam2SegmentationResponse result;

    QJsonObject body;
    body.insert(QStringLiteral("input_rgb"), windowsPathToWsl(request.inputRgbPath));
    body.insert(QStringLiteral("out_dir"), windowsPathToWsl(request.outputDirectory));
    body.insert(QStringLiteral("image_name"), request.imageName);
    body.insert(QStringLiteral("prompt"), request.prompt);
    body.insert(QStringLiteral("max_dets"), request.maxDetections);
    body.insert(QStringLiteral("box_threshold"), request.boxThreshold);

    QString localError;
    const QJsonObject response =
        postJson(request.serverUrl, QStringLiteral("/segment"), body, 600000, &localError);
    if (response.isEmpty())
    {
        result.errorMessage = localError;
        if (errorMessage != nullptr)
            *errorMessage = localError;
        return result;
    }

    if (!response.value(QStringLiteral("ok")).toBool(false))
    {
        result.errorMessage = response.value(QStringLiteral("error")).toString(localError);
        if (errorMessage != nullptr)
            *errorMessage = result.errorMessage;
        return result;
    }

    result.ok = true;
    result.detectionCount = response.value(QStringLiteral("detection_count")).toInt(0);
    result.overlayPngPath = response.value(QStringLiteral("overlay_png")).toString();
    result.stackMaskPngPath = response.value(QStringLiteral("stack_mask_png")).toString();
    result.manifestJsonPath =
        QDir(request.outputDirectory).filePath(QStringLiteral("segmentation_results.json"));
    return result;
}

} // namespace hf::processing
