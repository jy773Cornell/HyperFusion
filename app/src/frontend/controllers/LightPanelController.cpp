// Light / lighthouse tab orchestration implementation.
#include "frontend/controllers/LightPanelController.hpp"

#include "adapters/mcc/Mcc1208LighthouseController.hpp"
#include "backend/light/LighthouseWorker.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "frontend/settings/AppSettingsStore.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/widgets/IntensityBarWidget.hpp"
#include "frontend/widgets/MainWindow.hpp"

#include <QCheckBox>
#include <QElapsedTimer>
#include <QGroupBox>
#include <QLabel>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QTimer>

#include <algorithm>
#include <memory>

namespace {

QString formatLighthouseUptime(const qint64 elapsedMs)
{
    const qint64 totalSeconds = std::max<qint64>(0, elapsedMs / 1000);
    const qint64 hours = totalSeconds / 3600;
    const qint64 minutes = (totalSeconds % 3600) / 60;
    const qint64 seconds = totalSeconds % 60;

    if (hours > 0)
    {
        return QStringLiteral("%1:%2:%3")
            .arg(hours)
            .arg(minutes, 2, 10, QChar('0'))
            .arg(seconds, 2, 10, QChar('0'));
    }

    return QStringLiteral("%1:%2")
        .arg(minutes)
        .arg(seconds, 2, 10, QChar('0'));
}

} // namespace

namespace hf::light {

LightPanelController::LightPanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
{
}

LighthouseWorker *LightPanelController::worker() const { return lighthouseWorker_.get(); }

bool LightPanelController::isSessionActive() const
{
    return lighthouseWorker_ != nullptr
           && lighthouseWorker_->currentState() == LighthouseState::Connected;
}

void LightPanelController::shutdownSync()
{
    if (lighthouseWorker_ != nullptr)
        lighthouseWorker_->shutdownSync();
}

void LightPanelController::initializeWorker()
{
    auto controller = std::make_shared<Mcc1208LighthouseController>();
    controller->setLogCallback([this](const std::string &message) {
        const QString line = QString::fromStdString(message);
        QMetaObject::invokeMethod(
            host_,
            [this, line]() { host_->appendLog(line); },
            Qt::QueuedConnection);
    });

    lighthouseWorker_ = std::make_unique<LighthouseWorker>(controller);
    lighthouseWorker_->setStateCallback([this](const LighthouseState state) {
        QMetaObject::invokeMethod(
            host_,
            [this, state]() { onStateChanged(state); },
            Qt::QueuedConnection);
    });
    lighthouseWorker_->setDeviceInfoCallback([this](const LighthouseDeviceInfo &info) {
        QMetaObject::invokeMethod(
            host_,
            [this, info]() { onDeviceInfoChanged(info); },
            Qt::QueuedConnection);
    });
    lighthouseWorker_->setSettingsCallback([this](const LighthouseSettings &settings) {
        QMetaObject::invokeMethod(
            host_,
            [this, settings]() { onSettingsChanged(settings); },
            Qt::QueuedConnection);
    });
    lighthouseWorker_->setErrorCallback([this](const LighthouseError &error) {
        QMetaObject::invokeMethod(
            host_,
            [this, error]() { onError(error); },
            Qt::QueuedConnection);
    });
    lighthouseWorker_->setPowerStatusCallback([this](const LighthouseControllerPowerStatus &status) {
        QMetaObject::invokeMethod(
            host_,
            [this, status]() { onPowerStatusChanged(status); },
            Qt::QueuedConnection);
    });

    powerPollTimer_ = new QTimer(this);
    powerPollTimer_->setInterval(1000);
    connect(powerPollTimer_, &QTimer::timeout, host_, [this]() {
        updateLampUptimeDisplay();
        if (lighthouseWorker_ != nullptr && isSessionActive())
            lighthouseWorker_->requestPollControllerPowerStatus();
    });

    lighthouseWorker_->start();
    syncUiFromBackend();
}

void LightPanelController::syncUiFromBackend()
{
    if (lighthouseWorker_ == nullptr)
        return;

    updateConnectionDisplay();
    updateControlsEnabled();
    onDeviceInfoChanged(lighthouseWorker_->currentDeviceInfo());
    applySettingsToUi(lighthouseWorker_->currentSettings());
    updatePowerDisplay(lighthouseWorker_->currentControllerPowerStatus());

    if (isSessionActive())
    {
        if (powerPollTimer_ != nullptr && !powerPollTimer_->isActive())
            powerPollTimer_->start();
    }
    else
    {
        resetLampUptimes();
    }
    updateLampUptimeDisplay();
}

void LightPanelController::onStateChanged(const LighthouseState state)
{
    updateConnectionDisplay();
    updateControlsEnabled();

    switch (state)
    {
    case LighthouseState::Connected:
        host_->appendLog(QStringLiteral("Light: USB-1208FS-Plus connected"));
        if (powerPollTimer_ != nullptr)
        {
            powerPollTimer_->start();
            if (lighthouseWorker_ != nullptr)
                lighthouseWorker_->requestPollControllerPowerStatus();
        }
        break;
    case LighthouseState::Disconnected:
        if (powerPollTimer_ != nullptr)
            powerPollTimer_->stop();
        resetLampUptimes();
        updatePowerDisplay(LighthouseControllerPowerStatus{});
        host_->appendLog(QStringLiteral("Light: disconnected"));
        break;
    case LighthouseState::Fault:
        if (powerPollTimer_ != nullptr)
            powerPollTimer_->stop();
        resetLampUptimes();
        updatePowerDisplay(LighthouseControllerPowerStatus{});
        host_->appendLog(QStringLiteral("Light: fault"));
        break;
    default:
        break;
    }
}

void LightPanelController::onDeviceInfoChanged(const LighthouseDeviceInfo &info)
{
    if (host_->lightDaqInfoDisplay_ == nullptr)
        return;

    if (info.details.empty())
        host_->lightDaqInfoDisplay_->setPlainText(QString::fromUtf8(lighthouseWiringDetailsText()));
    else
        host_->lightDaqInfoDisplay_->setPlainText(QString::fromStdString(info.details));
}

void LightPanelController::onSettingsChanged(const LighthouseSettings &settings)
{
    applySettingsToUi(settings);
    host_->settingsPanel()->schedulePersistedUiSettingsSave();
}

void LightPanelController::onPowerStatusChanged(const LighthouseControllerPowerStatus &status)
{
    updatePowerDisplay(status);
}

void LightPanelController::onError(const LighthouseError &error)
{
    if (error.message.empty())
        return;

    host_->appendLog(QStringLiteral("Light error: %1").arg(QString::fromStdString(error.message)));
}

void LightPanelController::applySettingsToUi(const LighthouseSettings &settings)
{
    for (int rowIndex = 0; rowIndex < kLighthouseLampCount; ++rowIndex)
    {
        const int percent = rowIndex < 2 ? settings.reflectancePercent : settings.transmittancePercent;
        setRowIntensity(rowIndex, percent);

        auto &rowUi = host_->lighthouseRows_[static_cast<std::size_t>(rowIndex)];
        if (rowUi.onOffSwitch != nullptr)
        {
            const QSignalBlocker blocker(rowUi.onOffSwitch);
            rowUi.onOffSwitch->setChecked(settings.lampOn[static_cast<std::size_t>(rowIndex)]);
        }
    }

    updateLampUptimeDisplay();
}
void LightPanelController::updateConnectionDisplay()
{
    const LighthouseState state = lighthouseWorker_ != nullptr ? lighthouseWorker_->currentState()
                                                               : LighthouseState::Disconnected;

    QString statusText;
    QString indicatorStyle;

    switch (state)
    {
    case LighthouseState::Connected:
        statusText = QStringLiteral("Connected");
        indicatorStyle = QStringLiteral("background-color: #27ae60; border-radius: 7px;");
        break;
    case LighthouseState::Detected:
        statusText = QStringLiteral("Detected (not connected)");
        indicatorStyle = QStringLiteral("background-color: #f39c12; border-radius: 7px;");
        break;
    case LighthouseState::Scanning:
        statusText = QStringLiteral("Scanning\u2026");
        indicatorStyle = QStringLiteral("background-color: #f39c12; border-radius: 7px;");
        break;
    case LighthouseState::Connecting:
        statusText = QStringLiteral("Connecting\u2026");
        indicatorStyle = QStringLiteral("background-color: #f39c12; border-radius: 7px;");
        break;
    case LighthouseState::Fault:
        statusText = QStringLiteral("Fault");
        indicatorStyle = QStringLiteral("background-color: #c0392b; border-radius: 7px;");
        break;
    case LighthouseState::Disconnected:
    default:
        statusText = QStringLiteral("Disconnected");
        indicatorStyle = QStringLiteral("background-color: #95a5a6; border-radius: 7px;");
        break;
    }

    if (host_->lightDaqStatusLabel_ != nullptr)
        host_->lightDaqStatusLabel_->setText(statusText);
    if (host_->lightDaqStatusIndicator_ != nullptr)
        host_->lightDaqStatusIndicator_->setStyleSheet(indicatorStyle);

    const bool connected = state == LighthouseState::Connected;
    const bool detected = state == LighthouseState::Detected;
    const bool busy = state == LighthouseState::Scanning || state == LighthouseState::Connecting;
    const bool captureActive = host_->isCaptureSessionActive();

    if (host_->lightRefreshBtn_ != nullptr)
        host_->lightRefreshBtn_->setEnabled(!connected && !busy && !captureActive);
    if (host_->lightConnectBtn_ != nullptr)
        host_->lightConnectBtn_->setEnabled(detected && !busy && !captureActive);
    if (host_->lightDisconnectBtn_ != nullptr)
        host_->lightDisconnectBtn_->setEnabled((connected || busy) && !captureActive);
}

namespace
{
QString formatLighthouseUptime(const qint64 elapsedMs)
{
    const qint64 totalSeconds = std::max<qint64>(0, elapsedMs / 1000);
    const qint64 hours = totalSeconds / 3600;
    const qint64 minutes = (totalSeconds % 3600) / 60;
    const qint64 seconds = totalSeconds % 60;

    if (hours > 0)
    {
        return QStringLiteral("%1:%2:%3")
            .arg(hours)
            .arg(minutes, 2, 10, QChar('0'))
            .arg(seconds, 2, 10, QChar('0'));
    }

    return QStringLiteral("%1:%2")
        .arg(minutes)
        .arg(seconds, 2, 10, QChar('0'));
}
} // namespace

void LightPanelController::resetLampUptimes()
{
    for (QElapsedTimer &timer : lampUptimeElapsed_)
        timer.invalidate();
}

void LightPanelController::updateLampUptimeDisplay()
{
    const bool sessionActive = isSessionActive();
    const LighthouseControllerPowerStatus status =
        lighthouseWorker_ != nullptr ? lighthouseWorker_->currentControllerPowerStatus()
                                     : LighthouseControllerPowerStatus{};

    for (int rowIndex = 0; rowIndex < kLighthouseLampCount; ++rowIndex)
    {
        auto &rowUi = host_->lighthouseRows_[static_cast<std::size_t>(rowIndex)];
        if (rowUi.powerStatusLabel == nullptr)
            continue;

        const bool alive =
            sessionActive && status.valid && status.controllerAlive[static_cast<std::size_t>(rowIndex)];
        const bool lampOn =
            rowUi.onOffSwitch != nullptr && rowUi.onOffSwitch->isChecked();
        const bool running = alive && lampOn;

        QElapsedTimer &timer = lampUptimeElapsed_[static_cast<std::size_t>(rowIndex)];
        if (running)
        {
            if (!timer.isValid())
                timer.start();
            rowUi.powerStatusLabel->setText(formatLighthouseUptime(timer.elapsed()));
        }
        else
        {
            timer.invalidate();
            if (!sessionActive || !status.valid)
                rowUi.powerStatusLabel->setText(QStringLiteral("\u2014"));
            else
                rowUi.powerStatusLabel->setText(QStringLiteral("Off"));
        }
    }
}

void LightPanelController::updateControlsEnabled()
{
    const bool captureActive = host_->isCaptureSessionActive();
    const bool sessionControlsEnabled = isSessionActive() && !captureActive;

    if (host_->lightConnectionBox_ != nullptr)
        host_->lightConnectionBox_->setEnabled(!captureActive);
    if (host_->lightLightingBox_ != nullptr)
        host_->lightLightingBox_->setEnabled(!captureActive);

    for (auto &row : host_->lighthouseRows_)
    {
        if (row.nameLabel != nullptr)
            row.nameLabel->setEnabled(true);
        if (row.powerIndicator != nullptr)
            row.powerIndicator->setEnabled(true);
        if (row.powerStatusLabel != nullptr)
            row.powerStatusLabel->setEnabled(true);
        if (row.bar != nullptr)
            row.bar->setEnabled(sessionControlsEnabled);
        if (row.onOffSwitch != nullptr)
            row.onOffSwitch->setEnabled(sessionControlsEnabled);
    }
}

void LightPanelController::updatePowerDisplay(const LighthouseControllerPowerStatus &status)
{
    for (int rowIndex = 0; rowIndex < kLighthouseLampCount; ++rowIndex)
    {
        auto &rowUi = host_->lighthouseRows_[static_cast<std::size_t>(rowIndex)];
        if (rowUi.powerIndicator == nullptr || rowUi.powerStatusLabel == nullptr)
            continue;

        if (!status.valid)
        {
            rowUi.powerIndicator->setStyleSheet(
                QStringLiteral("background-color: #95a5a6; border-radius: 7px;"));
            rowUi.powerIndicator->setToolTip(
                tr("DC950 controller power monitor (AI CH%1) \u2014 not connected").arg(rowIndex));
            continue;
        }

        const float volts = status.monitorVolts[static_cast<std::size_t>(rowIndex)];
        const bool alive = status.controllerAlive[static_cast<std::size_t>(rowIndex)];
        const int monitorChannel = lighthousePowerMonitorChannelForLamp(
            static_cast<LighthouseLamp>(rowIndex), status.analogChannelCount);
        rowUi.powerIndicator->setStyleSheet(
            alive ? QStringLiteral("background-color: #27ae60; border-radius: 7px;")
                  : QStringLiteral("background-color: #c0392b; border-radius: 7px;"));
        if (monitorChannel < 0)
        {
            rowUi.powerIndicator->setToolTip(
                tr("DC950 power monitor is not independent for this lamp in differential analog mode"));
        }
        else
        {
            rowUi.powerIndicator->setToolTip(
                tr("DC950 controller power monitor (AI CH%1): %2 V \u2014 %3")
                    .arg(monitorChannel)
                    .arg(static_cast<double>(volts), 0, 'f', 2)
                    .arg(alive ? QStringLiteral("Alive") : QStringLiteral("Off")));
        }
    }

    updateLampUptimeDisplay();
}

int LightPanelController::partnerIndex(const int rowIndex)
{
    if (rowIndex < 0 || rowIndex >= 4)
        return -1;
    return (rowIndex < 2) ? (1 - rowIndex) : (5 - rowIndex);
}

void LightPanelController::setRowIntensity(const int rowIndex, const int percent)
{
    if (rowIndex < 0 || rowIndex >= 4)
        return;

    const int clamped = std::clamp(percent, 0, kLighthouseIntensityPercentMax);
    const int partner = partnerIndex(rowIndex);

    for (const int idx : {rowIndex, partner})
    {
        if (idx < 0)
            continue;

        auto &rowUi = host_->lighthouseRows_[static_cast<std::size_t>(idx)];
        if (rowUi.bar != nullptr)
            rowUi.bar->setPercent(clamped);
    }
}

void LightPanelController::applyPersistedUiValues()
{
    const PersistedLighthouseSettings saved = AppSettingsStore::loadLighthouseSettings();
    setRowIntensity(0, saved.reflectancePercent);
    setRowIntensity(2, saved.transmittancePercent);
}

LighthouseSettings LightPanelController::buildConnectDefaults() const
{
    const hf::HardwareConfig &config = hf::hardwareConfig();
    LighthouseSettings settings;
    settings.reflectancePercent = config.lighthouseReflectancePercent;
    settings.transmittancePercent = config.lighthouseTransmittancePercent;
    settings.lampOn.fill(true);
    return settings;
}

void LightPanelController::savePersistedSettings() const
{
    PersistedLighthouseSettings saved;
    if (host_->lighthouseRows_[0].bar != nullptr)
        saved.reflectancePercent = host_->lighthouseRows_[0].bar->percent();
    if (host_->lighthouseRows_[2].bar != nullptr)
        saved.transmittancePercent = host_->lighthouseRows_[2].bar->percent();
    AppSettingsStore::saveLighthouseSettings(saved);
}




} // namespace hf::light
