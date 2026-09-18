// One-shot subprocess runner for FPP MVS decode→fusion (backend/offline).
#include "backend/camera/processing/FppMvsRunner.hpp"

#include "backend/HyperFusionConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>

namespace hf::processing
{
namespace
{
QString resolveRepoFppDirectory()
{
    if (QCoreApplication::instance() != nullptr)
    {
        QDir dir(QCoreApplication::applicationDirPath());
        for (int depth = 0; depth < 8; ++depth)
        {
            const QString sidecars = dir.filePath(QStringLiteral("sidecars/fpp"));
            if (QFileInfo::exists(QDir(sidecars).filePath(QStringLiteral("fpp_mvs_cli.py"))))
                return QFileInfo(sidecars).absoluteFilePath();
            if (!dir.cdUp())
                break;
        }
    }

#ifdef HF_APP_SOURCE_DIR
    {
        const QString fromSource =
            QDir(QString::fromUtf8(HF_APP_SOURCE_DIR)).filePath(QStringLiteral("sidecars/fpp"));
        if (QFileInfo::exists(QDir(fromSource).filePath(QStringLiteral("fpp_mvs_cli.py"))))
            return QFileInfo(fromSource).absoluteFilePath();
    }
#endif

    return {};
}

bool parseCliJson(const QByteArray &stdoutPayload, FppMvsRunResult *result)
{
    if (result == nullptr)
        return false;

    const QList<QByteArray> lines = stdoutPayload.split('\n');
    for (auto it = lines.crbegin(); it != lines.crend(); ++it)
    {
        const QByteArray trimmed = it->trimmed();
        if (trimmed.isEmpty() || trimmed.at(0) != '{')
            continue;

        QJsonParseError parseError;
        const QJsonDocument document = QJsonDocument::fromJson(trimmed, &parseError);
        if (parseError.error != QJsonParseError::NoError || !document.isObject())
            continue;

        const QJsonObject object = document.object();
        if (!object.contains(QStringLiteral("ok")))
            continue;

        result->success = object.value(QStringLiteral("ok")).toBool(false);
        result->skipped = object.value(QStringLiteral("skipped")).toBool(false);
        result->reason = object.value(QStringLiteral("reason")).toString();
        result->datafolder = object.value(QStringLiteral("datafolder")).toString();
        result->processedDir = object.value(QStringLiteral("processed")).toString();
        result->metadataJsonPath = object.value(QStringLiteral("metadata")).toString();
        result->primaryCloudPath = object.value(QStringLiteral("primary_cloud")).toString();
        result->elapsedS = object.value(QStringLiteral("elapsed_s")).toDouble();
        if (!result->success && object.contains(QStringLiteral("error")))
            result->errorMessage = object.value(QStringLiteral("error")).toString();
        return true;
    }
    return false;
}
} // namespace

QString resolveFppSidecarDirectory()
{
    return resolveRepoFppDirectory();
}

QString resolveFppPythonExecutable()
{
    const QString root = resolveFppSidecarDirectory();
    if (root.isEmpty())
        return {};

    const QStringList candidates = {
        QDir(root).filePath(QStringLiteral(".venv/Scripts/python.exe")),
        QDir(root).filePath(QStringLiteral("depth_fusion/.venv/Scripts/python.exe")),
    };
    for (const QString &c : candidates)
    {
        if (QFileInfo::exists(c))
            return QFileInfo(c).absoluteFilePath();
    }
    return {};
}

bool multiviewHasFppBurst(const QString &multiviewOrDatafolder)
{
    QString dirPath = multiviewOrDatafolder.trimmed();
    if (dirPath.isEmpty())
        return false;

    QDir dir(dirPath);
    if (dir.exists(QStringLiteral("multiview")))
        dir = QDir(dir.filePath(QStringLiteral("multiview")));
    if (!dir.exists())
        return false;

    const QStringList jsons =
        dir.entryList({QStringLiteral("*.json")}, QDir::Files, QDir::Name);
    int checked = 0;
    for (const QString &name : jsons)
    {
        if (name.compare(QStringLiteral("transforms.json"), Qt::CaseInsensitive) == 0)
            continue;
        QFile file(dir.filePath(name));
        if (!file.open(QIODevice::ReadOnly))
            continue;
        const QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
        file.close();
        if (!doc.isObject())
            continue;
        const QJsonObject obj = doc.object();
        if (obj.contains(QStringLiteral("fpp_pattern"))
            || obj.contains(QStringLiteral("fpp_step_index")))
            return true;
        if (++checked >= 40)
            break;
    }
    return false;
}

FppMvsRunResult runFppMvsPipeline(const FppMvsRunRequest &request)
{
    FppMvsRunResult result;
    const QString sidecar = resolveFppSidecarDirectory();
    const QString python = resolveFppPythonExecutable();
    const QString script = QDir(sidecar).filePath(QStringLiteral("fpp_mvs_cli.py"));

    if (sidecar.isEmpty() || !QFileInfo::exists(script))
    {
        result.errorMessage = QStringLiteral("fpp_mvs_cli.py not found (app/sidecars/fpp missing).");
        return result;
    }
    if (python.isEmpty())
    {
        result.errorMessage =
            QStringLiteral("FPP Python venv not found. Run app/sidecars/fpp/setup_venv.ps1 "
                           "(or depth_fusion/setup_venv.ps1).");
        return result;
    }
    if (request.inputPath.trimmed().isEmpty())
    {
        result.errorMessage = QStringLiteral("FPP MVS input path is empty.");
        return result;
    }

    QStringList args;
    args << script << QStringLiteral("--input") << request.inputPath;
    if (!request.handEyeYaml.trimmed().isEmpty())
        args << QStringLiteral("--hand-eye") << request.handEyeYaml;
    if (!request.stereoYaml.trimmed().isEmpty())
        args << QStringLiteral("--stereo") << request.stereoYaml;
    if (!request.mode.trimmed().isEmpty())
        args << QStringLiteral("--mode") << request.mode;

    QProcess process;
    process.setProgram(python);
    process.setArguments(args);
    process.setWorkingDirectory(sidecar);
    process.setProcessChannelMode(QProcess::MergedChannels);

    result.logLines.push_back(
        QStringLiteral("FPP MVS: %1 %2").arg(python, args.join(QLatin1Char(' '))));

    process.start();
    if (!process.waitForStarted(15000))
    {
        result.errorMessage = QStringLiteral("Failed to start FPP MVS Python process.");
        return result;
    }

    const int timeout = request.timeoutMs > 0 ? request.timeoutMs : 3600000;
    if (!process.waitForFinished(timeout))
    {
        process.kill();
        process.waitForFinished(5000);
        result.errorMessage = QStringLiteral("FPP MVS timed out after %1 ms.").arg(timeout);
        result.logLines.append(QString::fromUtf8(process.readAllStandardOutput()));
        return result;
    }

    const QByteArray output = process.readAllStandardOutput();
    const QString text = QString::fromUtf8(output);
    for (const QString &line : text.split(QLatin1Char('\n')))
    {
        const QString trimmed = line.trimmed();
        if (!trimmed.isEmpty())
            result.logLines.push_back(trimmed);
    }

    if (!parseCliJson(output, &result))
    {
        result.success = false;
        if (result.errorMessage.isEmpty())
        {
            result.errorMessage =
                QStringLiteral("FPP MVS finished but no JSON summary was parsed (exit %1).")
                    .arg(process.exitCode());
        }
        return result;
    }

    if (process.exitCode() != 0 && !result.skipped)
        result.success = false;

    if (!result.success && result.errorMessage.isEmpty() && !result.skipped)
        result.errorMessage = QStringLiteral("FPP MVS failed (exit %1).").arg(process.exitCode());

    return result;
}

} // namespace hf::processing
