// DLPC3478 adapter (adapters/dlp). Real USB via 32-bit hf_dlpc_bridge when HF_HAVE_DLPC_API.
#include "adapters/dlp/Dlpc3478Projector.hpp"

#include <string>

#if defined(HF_HAVE_DLPC_API)
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QProcess>
#include <QStringList>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif
#endif

namespace hf::dlp
{
namespace
{
constexpr const char *kStubDeviceId = "stub";
constexpr const char *kStubModel = "DLP3010EVM-LC (stub)";

#if defined(HF_HAVE_DLPC_API)
constexpr int kBridgeTimeoutMs = 15000;
constexpr int kConnectTimeoutMs = 60000;

QString bridgeExecutablePath()
{
    const QString besideApp = QDir(QCoreApplication::applicationDirPath())
                                  .filePath(QStringLiteral("hf_dlpc_bridge.exe"));
    if (QFileInfo::exists(besideApp))
        return besideApp;
    return besideApp;
}
#endif
} // namespace

Dlpc3478Projector::Dlpc3478Projector() = default;

Dlpc3478Projector::~Dlpc3478Projector()
{
    std::lock_guard<std::mutex> lock(mutex_);
#if defined(HF_HAVE_DLPC_API)
    if (bridge_ && bridge_->state() != QProcess::NotRunning)
    {
        DlpError ignored;
        (void)commandOkLocked(QStringLiteral("DISCONNECT"), ignored, kBridgeTimeoutMs);
    }
#endif
    teardownLocked();
}

bool Dlpc3478Projector::sdkAvailable()
{
    return true;
}

bool Dlpc3478Projector::hardwareBackendAvailable()
{
#if defined(HF_HAVE_DLPC_API)
    return true;
#else
    return false;
#endif
}

std::vector<DlpDeviceInfo> Dlpc3478Projector::listDevices(DlpError *error)
{
    if (error != nullptr)
        *error = {};

#if defined(HF_HAVE_DLPC_API)
    std::lock_guard<std::mutex> lock(mutex_);
    DlpError localError;
    if (!ensureBridgeLocked(localError))
    {
        if (error != nullptr)
            *error = localError;
        return {};
    }
    QStringList extra;
    if (!transactLocked(QStringLiteral("ENUM"), &extra, localError, kBridgeTimeoutMs))
    {
        if (error != nullptr)
            *error = localError;
        return {};
    }
    std::vector<DlpDeviceInfo> devices;
    for (const QString &line : extra)
    {
        if (!line.startsWith(QStringLiteral("DEV ")))
            continue;
        const QString payload = line.mid(4);
        const int tab = payload.indexOf(QLatin1Char('\t'));
        DlpDeviceInfo info;
        if (tab < 0)
            info.id = payload.trimmed();
        else
        {
            info.id = payload.left(tab).trimmed();
            info.model = payload.mid(tab + 1).trimmed();
        }
        if (!info.id.isEmpty())
            devices.push_back(info);
    }
    return devices;
#else
    DlpDeviceInfo info;
    info.id = QString::fromUtf8(kStubDeviceId);
    info.model = QString::fromUtf8(kStubModel);
    return {info};
#endif
}

std::vector<DlpDeviceInfo> Dlpc3478Projector::enumerateDevices(DlpError *error)
{
    Dlpc3478Projector enumerator;
    return enumerator.listDevices(error);
}

DlpProjectorState Dlpc3478Projector::state() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

std::string Dlpc3478Projector::connectedId() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return connectedId_;
}

bool Dlpc3478Projector::hardwareOutputEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ledsEnabled_;
}

bool Dlpc3478Projector::requireConnected(DlpError &error) const
{
    if (state_ != DlpProjectorState::Connected && state_ != DlpProjectorState::Armed
        && state_ != DlpProjectorState::Projecting)
    {
        error.code = DlpErrorCode::InvalidState;
        error.message = "Requires a connected projector.";
        return false;
    }
    return true;
}

void Dlpc3478Projector::teardownLocked()
{
#if defined(HF_HAVE_DLPC_API)
    if (bridge_ && bridge_->state() != QProcess::NotRunning)
    {
        bridge_->write("DISCONNECT\n");
        bridge_->waitForBytesWritten(1000);
        bridge_->write("QUIT\n");
        bridge_->waitForBytesWritten(1000);
        if (!bridge_->waitForFinished(2000))
            bridge_->kill();
    }
    bridge_.reset();
#endif
    ledsEnabled_ = false;
    connectedId_.clear();
    state_ = DlpProjectorState::Disconnected;
}

#if defined(HF_HAVE_DLPC_API)
bool Dlpc3478Projector::ensureBridgeLocked(DlpError &error)
{
    if (bridge_ && bridge_->state() != QProcess::NotRunning)
        return true;

    const QString exe = bridgeExecutablePath();
    if (!QFileInfo::exists(exe))
    {
        error.code = DlpErrorCode::NotAvailable;
        error.message = "hf_dlpc_bridge.exe not found next to app.exe.";
        return false;
    }

    bridge_ = std::make_unique<QProcess>();
#ifdef _WIN32
    bridge_->setCreateProcessArgumentsModifier([](QProcess::CreateProcessArguments *args) {
        args->flags |= CREATE_NO_WINDOW;
    });
#endif
    bridge_->setProgram(exe);
    bridge_->setWorkingDirectory(QFileInfo(exe).absolutePath());
    bridge_->start();
    if (!bridge_->waitForStarted(kBridgeTimeoutMs))
    {
        error.code = DlpErrorCode::SdkError;
        error.message = "Failed to start hf_dlpc_bridge.exe (32-bit Cypress helper).";
        bridge_.reset();
        return false;
    }
    return true;
}

void Dlpc3478Projector::restartBridgeLocked()
{
    if (!bridge_)
        return;
    if (bridge_->state() != QProcess::NotRunning)
    {
        bridge_->kill();
        bridge_->waitForFinished(2000);
    }
    bridge_.reset();
}

bool Dlpc3478Projector::transactLocked(const QString &request, QStringList *extraLines, DlpError &error,
                                      int timeoutMs)
{
    if (!ensureBridgeLocked(error))
        return false;

    if (extraLines != nullptr)
        extraLines->clear();

    if (bridge_->bytesAvailable() > 0)
        (void)bridge_->readAll();

    bridge_->write(request.toUtf8());
    bridge_->write("\n");
    if (!bridge_->waitForBytesWritten(timeoutMs))
    {
        error.code = DlpErrorCode::SdkError;
        error.message = "DLP helper write failed.";
        restartBridgeLocked();
        return false;
    }

    if (!bridge_->waitForReadyRead(timeoutMs))
    {
        error.code = DlpErrorCode::SdkError;
        error.message = "DLP helper did not respond.";
        restartBridgeLocked();
        return false;
    }

    auto readLine = [this, timeoutMs]() -> QString {
        while (!bridge_->canReadLine())
        {
            if (!bridge_->waitForReadyRead(timeoutMs))
                return {};
        }
        return QString::fromUtf8(bridge_->readLine()).trimmed();
    };

    const QString first = readLine();
    if (first.isEmpty())
    {
        error.code = DlpErrorCode::SdkError;
        error.message = "DLP helper did not respond.";
        restartBridgeLocked();
        return false;
    }
    if (first.startsWith(QStringLiteral("ERR")))
    {
        error.code = DlpErrorCode::SdkError;
        error.message = first.mid(3).trimmed().toStdString();
        if (error.message.empty())
            error.message = "DLP helper error.";
        return false;
    }
    if (!first.startsWith(QStringLiteral("OK")))
    {
        error.code = DlpErrorCode::SdkError;
        error.message = "Unexpected DLP helper response: " + first.toStdString();
        restartBridgeLocked();
        return false;
    }

    const QString rest = first.mid(2).trimmed();
    if (extraLines != nullptr)
    {
        bool okCount = false;
        const int count = rest.section(QLatin1Char(' '), 0, 0).toInt(&okCount);
        if (okCount && count > 0)
        {
            for (int i = 0; i < count; ++i)
            {
                const QString line = readLine();
                if (line.isEmpty())
                    break;
                extraLines->push_back(line);
            }
        }
    }
    return true;
}

bool Dlpc3478Projector::commandOkLocked(const QString &request, DlpError &error, int timeoutMs)
{
    return transactLocked(request, nullptr, error, timeoutMs);
}
#endif

bool Dlpc3478Projector::connect(const QString &deviceId, DlpError &error)
{
    error = {};
    std::lock_guard<std::mutex> lock(mutex_);

#if defined(HF_HAVE_DLPC_API)
    QString cmd = QStringLiteral("CONNECT");
    if (!deviceId.trimmed().isEmpty())
        cmd += QLatin1Char(' ') + deviceId.trimmed();
    if (!commandOkLocked(cmd, error, kConnectTimeoutMs))
        return false;
    connectedId_ = deviceId.trimmed().isEmpty() ? "cypress-i2c" : deviceId.trimmed().toStdString();
    state_ = DlpProjectorState::Connected;
    return true;
#else
    connectedId_ = deviceId.isEmpty() ? kStubDeviceId : deviceId.toStdString();
    state_ = DlpProjectorState::Connected;
    return true;
#endif
}

bool Dlpc3478Projector::arm(const DlpProjectorSettings &settings, DlpError &error)
{
    error = {};
    std::lock_guard<std::mutex> lock(mutex_);
    if (!requireConnected(error))
        return false;

#if defined(HF_HAVE_DLPC_API)
    const QString cmd = QStringLiteral("ARM %1 %2 %3")
                            .arg(settings.ledRedMa)
                            .arg(settings.ledGreenMa)
                            .arg(settings.ledBlueMa);
    if (!commandOkLocked(cmd, error, kBridgeTimeoutMs))
        return false;
    ledsEnabled_ = true;
    state_ = DlpProjectorState::Armed;
    return true;
#else
    (void)settings;
    state_ = DlpProjectorState::Armed;
    return true;
#endif
}

bool Dlpc3478Projector::blank(DlpError &error)
{
    error = {};
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == DlpProjectorState::Disconnected)
        return true;

#if defined(HF_HAVE_DLPC_API)
    if (!commandOkLocked(QStringLiteral("BLANK"), error, kBridgeTimeoutMs))
        return false;
#endif
    ledsEnabled_ = false;
    state_ = DlpProjectorState::Armed;
    return true;
}

bool Dlpc3478Projector::showTestPattern(const QString &patternName, DlpError &error)
{
    error = {};
    std::lock_guard<std::mutex> lock(mutex_);
    if (!requireConnected(error))
        return false;

#if defined(HF_HAVE_DLPC_API)
    if (!commandOkLocked(QStringLiteral("PATTERN ") + patternName.trimmed(), error, kBridgeTimeoutMs))
        return false;
    ledsEnabled_ = true;
    state_ = DlpProjectorState::Projecting;
    return true;
#else
    (void)patternName;
    state_ = DlpProjectorState::Projecting;
    return true;
#endif
}

bool Dlpc3478Projector::showExternalVideo(DlpError &error)
{
    error = {};
    std::lock_guard<std::mutex> lock(mutex_);
    if (!requireConnected(error))
        return false;

#if defined(HF_HAVE_DLPC_API)
    if (!commandOkLocked(QStringLiteral("VIDEO"), error, kBridgeTimeoutMs))
        return false;
    ledsEnabled_ = true;
    state_ = DlpProjectorState::Projecting;
    return true;
#else
    state_ = DlpProjectorState::Projecting;
    return true;
#endif
}

bool Dlpc3478Projector::applyLedCurrents(const DlpProjectorSettings &settings, DlpError &error)
{
    error = {};
    std::lock_guard<std::mutex> lock(mutex_);
    if (state_ == DlpProjectorState::Disconnected || state_ == DlpProjectorState::Fault)
    {
        error.code = DlpErrorCode::InvalidState;
        error.message = "LED currents require a connected projector.";
        return false;
    }

#if defined(HF_HAVE_DLPC_API)
    const QString cmd = QStringLiteral("LED %1 %2 %3")
                            .arg(settings.ledRedMa)
                            .arg(settings.ledGreenMa)
                            .arg(settings.ledBlueMa);
    return commandOkLocked(cmd, error, kBridgeTimeoutMs);
#else
    (void)settings;
    return true;
#endif
}

void Dlpc3478Projector::disconnect()
{
    std::lock_guard<std::mutex> lock(mutex_);
#if defined(HF_HAVE_DLPC_API)
    if (bridge_ && bridge_->state() != QProcess::NotRunning)
    {
        DlpError ignored;
        (void)commandOkLocked(QStringLiteral("DISCONNECT"), ignored, kBridgeTimeoutMs);
    }
#endif
    ledsEnabled_ = false;
    connectedId_.clear();
    state_ = DlpProjectorState::Disconnected;
}
} // namespace hf::dlp
