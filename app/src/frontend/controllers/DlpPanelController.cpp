// DLP settings tab orchestration: Connect (auto-arm), Blank, test pattern (frontend).
#include "frontend/controllers/DlpPanelController.hpp"

#include "backend/fpp/DlpProjectorWorker.hpp"
#include "adapters/dlp/Dlpc3478Projector.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "frontend/logging/AppLog.hpp"
#include "frontend/widgets/DlpHdmiPatternWindow.hpp"
#include "frontend/widgets/DlpProjectorSettingsWidget.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "backend/fpp/DlpHdmiPatterns.hpp"

#include <QMetaObject>
#include <QPushButton>
#include <QThread>

namespace hf::dlp
{
namespace
{
QString stateLabel(const DlpProjectorState state)
{
    switch (state)
    {
    case DlpProjectorState::Disconnected:
        return QStringLiteral("Disconnected");
    case DlpProjectorState::Connected:
        return QStringLiteral("Connected");
    case DlpProjectorState::Armed:
        return QStringLiteral("Connected");
    case DlpProjectorState::Projecting:
        return QStringLiteral("Projecting");
    case DlpProjectorState::Fault:
        return QStringLiteral("Fault");
    }
    return QStringLiteral("Unknown");
}
} // namespace

DlpPanelController::DlpPanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
{
    ledApplyTimer_ = new QTimer(this);
    ledApplyTimer_->setSingleShot(true);
    ledApplyTimer_->setInterval(400);
    connect(ledApplyTimer_, &QTimer::timeout, this, &DlpPanelController::applyLedCurrentsFromUi);
}

DlpPanelController::~DlpPanelController()
{
    shutdownSync();
}

void DlpPanelController::initializeWorker()
{
    if (worker_ != nullptr)
        return;

    worker_ = std::make_unique<DlpProjectorWorker>();
    worker_->setStateCallback([this](const DlpProjectorState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { onStateChanged(state); },
            Qt::QueuedConnection);
    });
    worker_->setErrorCallback([this](const DlpError &error) {
        const QString message = QString::fromStdString(error.message);
        QMetaObject::invokeMethod(
            this,
            [this, message]() {
                DlpError e;
                e.message = message.toStdString();
                onError(e);
            },
            Qt::QueuedConnection);
    });
    worker_->setDevicesCallback([this](const std::vector<DlpDeviceInfo> &devices) {
        QMetaObject::invokeMethod(
            this,
            [this, devices]() { onDevices(devices); },
            Qt::QueuedConnection);
    });
    worker_->setLogCallback([this](const std::string &line) {
        const QString message = QString::fromStdString(line);
        QMetaObject::invokeMethod(
            this,
            [this, message]() { onWorkerLog(message); },
            Qt::QueuedConnection);
    });
    worker_->setHdmiShowCallback([this](int stepIndex, DlpError &error) {
        if (QThread::currentThread() == thread())
            return showHdmiPngOnGui(stepIndex, error);
        bool ok = false;
        DlpError local;
        QMetaObject::invokeMethod(
            this,
            [this, stepIndex, &ok, &local]() { ok = showHdmiPngOnGui(stepIndex, local); },
            Qt::BlockingQueuedConnection);
        error = local;
        return ok;
    });
    worker_->start();
}

void DlpPanelController::shutdownSync()
{
    if (hdmiWindow_ != nullptr)
        hdmiWindow_->hidePattern();
    if (worker_ == nullptr)
        return;

    const DlpProjectorState state = worker_->currentState();
    if (host_ != nullptr
        && (state == DlpProjectorState::Connected || state == DlpProjectorState::Armed
            || state == DlpProjectorState::Projecting || state == DlpProjectorState::Fault))
    {
        host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("DLP: blanking before close\u2026"));
        if (host_->dlpProjectorSettings_ != nullptr)
            host_->dlpProjectorSettings_->setConnectionStatus(QStringLiteral("Blanking\u2026"));
    }

    worker_->shutdownSync();
    worker_.reset();

    if (host_ != nullptr && host_->dlpProjectorSettings_ != nullptr)
    {
        host_->dlpProjectorSettings_->applyState(DlpProjectorState::Disconnected);
        host_->dlpProjectorSettings_->setConnectionStatus(QStringLiteral("Disconnected"));
    }
}

void DlpPanelController::wireSettingsTabConnections()
{
    if (host_ == nullptr || host_->dlpProjectorSettings_ == nullptr)
        return;

    ui::DlpProjectorSettingsWidget *ui = host_->dlpProjectorSettings_;
    ui->setLedMaxMilliamp(hf::hardwareConfig().dlp.ledMaxMa);

    connect(ui->refreshButton(), &QPushButton::clicked, this, &DlpPanelController::onRefreshClicked);
    connect(ui->connectButton(), &QPushButton::clicked, this, &DlpPanelController::onConnectClicked);
    connect(ui->disconnectButton(), &QPushButton::clicked, this,
            &DlpPanelController::onDisconnectClicked);
    connect(ui->blankButton(), &QPushButton::clicked, this, &DlpPanelController::onBlankClicked);
    connect(ui->testPatternButton(), &QPushButton::clicked, this,
            &DlpPanelController::onTestPatternClicked);
    connect(ui, &ui::DlpProjectorSettingsWidget::settingsEdited, this,
            &DlpPanelController::onSettingsEdited);

    ui->setConnectionStatus(Dlpc3478Projector::hardwareBackendAvailable()
                                ? QStringLiteral("Disconnected")
                                : QStringLiteral("Disconnected (stub — DLPC-API not linked)"));
    onRefreshClicked();
}

bool DlpPanelController::isConnected() const
{
    if (worker_ == nullptr)
        return false;
    const DlpProjectorState state = worker_->currentState();
    return state == DlpProjectorState::Connected || state == DlpProjectorState::Armed
           || state == DlpProjectorState::Projecting;
}

bool DlpPanelController::showHdmiPngOnGui(int stepIndex, DlpError &error)
{
    if (stepIndex < 0 || stepIndex >= kFppScanningStepCount)
    {
        error = {DlpErrorCode::InvalidState, "HDMI FPP step index out of range.", false};
        return false;
    }
    const char *file = kFppScanningSteps[stepIndex].hdmiFile;
    const QString path = hdmiPspPatternFile(file);
    if (path.isEmpty())
    {
        error = {DlpErrorCode::NotAvailable,
                 "HDMI PSP PNGs not found (calibration/multiview/dlp_cal/patterns/psp).",
                 false};
        return false;
    }
    if (hdmiWindow_ == nullptr)
        hdmiWindow_ = std::make_unique<ui::DlpHdmiPatternWindow>();
    QString loadError;
    if (!hdmiWindow_->showPng(path, &loadError))
    {
        error = {DlpErrorCode::InternalError, loadError.toStdString(), false};
        return false;
    }
    return true;
}

bool DlpPanelController::showFppScanStepSync(int stepIndex, QString *errorOut)
{
    if (worker_ == nullptr)
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("DLP worker is not running.");
        return false;
    }
    DlpError error;
    if (!worker_->showFppStepSync(stepIndex, error))
    {
        if (errorOut != nullptr)
            *errorOut = QString::fromStdString(error.message);
        return false;
    }
    return true;
}

bool DlpPanelController::blankSync(QString *errorOut)
{
    if (worker_ == nullptr)
    {
        if (errorOut != nullptr)
            *errorOut = QStringLiteral("DLP worker is not running.");
        return false;
    }
    DlpError error;
    if (!worker_->blankSync(error))
    {
        if (errorOut != nullptr)
            *errorOut = QString::fromStdString(error.message);
        return false;
    }
    if (hdmiWindow_ != nullptr)
    {
        if (QThread::currentThread() == thread())
            hdmiWindow_->hidePattern();
        else
        {
            QMetaObject::invokeMethod(
                this,
                [this]() {
                    if (hdmiWindow_ != nullptr)
                        hdmiWindow_->hidePattern();
                },
                Qt::BlockingQueuedConnection);
        }
    }
    return true;
}

DlpProjectorSettings DlpPanelController::settingsFromUi() const
{
    if (host_ == nullptr || host_->dlpProjectorSettings_ == nullptr)
        return {};
    return host_->dlpProjectorSettings_->currentSettings();
}

DlpProjectorSettings DlpPanelController::currentSettings() const
{
    return settingsFromUi();
}

void DlpPanelController::onRefreshClicked()
{
    if (worker_ == nullptr)
        return;
    if (host_->dlpProjectorSettings_ != nullptr)
        host_->dlpProjectorSettings_->setConnectionStatus(QStringLiteral("Scanning for projector\u2026"));
    worker_->requestEnumerate();
}

void DlpPanelController::onConnectClicked()
{
    if (worker_ == nullptr)
        return;
    if (host_ != nullptr && host_->isCaptureSessionActive())
    {
        onWorkerLog(QStringLiteral("DLP: Connect blocked — HSI capture is active."));
        return;
    }
    if (host_->dlpProjectorSettings_ != nullptr)
        host_->dlpProjectorSettings_->setConnectionStatus(QStringLiteral("Connecting\u2026"));
    worker_->requestConnect(settingsFromUi());
}

void DlpPanelController::onDisconnectClicked()
{
    if (worker_ == nullptr)
        return;
    if (host_->dlpProjectorSettings_ != nullptr)
        host_->dlpProjectorSettings_->setConnectionStatus(QStringLiteral("Disconnecting\u2026"));
    worker_->requestDisconnect();
}

void DlpPanelController::onBlankClicked()
{
    if (worker_ == nullptr)
        return;
    if (host_->dlpProjectorSettings_ != nullptr)
        host_->dlpProjectorSettings_->setConnectionStatus(QStringLiteral("Blanking\u2026"));
    worker_->requestBlank();
}

void DlpPanelController::onTestPatternClicked()
{
    if (worker_ == nullptr)
        return;
    worker_->requestShowTestPattern(settingsFromUi());
}

void DlpPanelController::onSettingsEdited()
{
    if (!isConnected())
        return;
    ledApplyTimer_->start();
}

void DlpPanelController::applyLedCurrentsFromUi()
{
    if (worker_ == nullptr || !isConnected())
        return;
    worker_->requestApplyLedCurrents(settingsFromUi());
}

void DlpPanelController::onStateChanged(const DlpProjectorState state)
{
    if (host_ == nullptr || host_->dlpProjectorSettings_ == nullptr)
        return;
    host_->dlpProjectorSettings_->applyState(state);
    host_->dlpProjectorSettings_->setConnectionStatus(stateLabel(state));
    host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("DLP: %1").arg(stateLabel(state)));
}

void DlpPanelController::onError(const DlpError &error)
{
    const QString message = QString::fromStdString(error.message);
    if (host_ != nullptr)
        host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("DLP error: %1").arg(message));
    if (host_ != nullptr && host_->dlpProjectorSettings_ != nullptr && !message.isEmpty())
        host_->dlpProjectorSettings_->setConnectionStatus(message);
}

void DlpPanelController::onDevices(const std::vector<DlpDeviceInfo> &devices)
{
    if (host_ == nullptr || host_->dlpProjectorSettings_ == nullptr)
        return;
    const QString preferred = host_->dlpProjectorSettings_->currentSettings().deviceId;
    host_->dlpProjectorSettings_->setDevices(devices, preferred);
    if (worker_ != nullptr && worker_->currentState() == DlpProjectorState::Disconnected)
    {
        host_->dlpProjectorSettings_->setConnectionStatus(
            devices.empty()
                ? QStringLiteral("No projector found")
                : (Dlpc3478Projector::hardwareBackendAvailable()
                       ? QStringLiteral("Disconnected")
                       : QStringLiteral("Disconnected (stub — DLPC-API not linked)")));
    }
    host_->appendLog(hf::log::Channel::Ur3e,
                     QStringLiteral("DLP: found %1 device(s).").arg(static_cast<int>(devices.size())));
}

void DlpPanelController::onWorkerLog(const QString &message)
{
    if (host_ != nullptr)
        host_->appendLog(hf::log::Channel::Ur3e, message);
}
} // namespace hf::dlp
