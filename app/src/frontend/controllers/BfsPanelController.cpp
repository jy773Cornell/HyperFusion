// BFS settings tab orchestration: Spinnaker worker, connect, RGB preview.
#include "frontend/controllers/BfsPanelController.hpp"

#include "backend/3dscanning/BfsCameraWorker.hpp"
#include "backend/3dscanning/BfsSpinnakerCamera.hpp"
#include "backend/3dscanning/BfsTiffIo.hpp"
#include "frontend/logging/AppLog.hpp"
#include "frontend/widgets/BfsCameraSettingsWidget.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "frontend/widgets/StreamPaneHelpers.hpp"

#include <QDateTime>
#include <QFileDialog>
#include <QFont>
#include <QImage>
#include <QLabel>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPixmap>
#include <QPushButton>
#include <QSizePolicy>
#include <QTimer>

#include <string>

namespace hf::bfs
{
namespace
{
QString stateLabel(const BfsCameraState state)
{
    switch (state)
    {
    case BfsCameraState::Disconnected:
        return QStringLiteral("Disconnected");
    case BfsCameraState::Connected:
        return QStringLiteral("Connected");
    case BfsCameraState::Streaming:
        return QStringLiteral("Streaming");
    case BfsCameraState::Fault:
        return QStringLiteral("Fault");
    }
    return QStringLiteral("Unknown");
}
} // namespace

BfsPanelController::BfsPanelController(MainWindow *host, QObject *parent)
    : QObject(parent)
    , host_(host)
{
    settingsApplyTimer_ = new QTimer(this);
    settingsApplyTimer_->setSingleShot(true);
    settingsApplyTimer_->setInterval(400);
    connect(settingsApplyTimer_, &QTimer::timeout, this, &BfsPanelController::applySettingsFromUi);
}

BfsPanelController::~BfsPanelController()
{
    shutdownSync();
}

void BfsPanelController::initializeWorker()
{
    if (worker_ != nullptr)
        return;

    worker_ = std::make_unique<BfsCameraWorker>();
    worker_->setStateCallback([this](const BfsCameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { onStateChanged(state); },
            Qt::QueuedConnection);
    });
    worker_->setErrorCallback([this](const BfsError &error) {
        const QString message = QString::fromStdString(error.message);
        QMetaObject::invokeMethod(
            this,
            [this, message]() {
                BfsError e;
                e.message = message.toStdString();
                onError(e);
            },
            Qt::QueuedConnection);
    });
    worker_->setDevicesCallback([this](const std::vector<BfsDeviceInfo> &devices) {
        QMetaObject::invokeMethod(
            this,
            [this, devices]() { onDevices(devices); },
            Qt::QueuedConnection);
    });
    worker_->setFrameCallback([this](const BfsRgbFrame &frame) {
        queueFrame(frame);
    });
    worker_->start();
}

void BfsPanelController::shutdownSync()
{
    if (worker_ == nullptr)
        return;

    const BfsCameraState state = worker_->currentState();
    if (host_ != nullptr
        && (state == BfsCameraState::Connected || state == BfsCameraState::Streaming
            || state == BfsCameraState::Fault))
    {
        host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("BFS: disconnecting before close\u2026"));
        if (host_->bfsCameraSettings_ != nullptr)
            host_->bfsCameraSettings_->setConnectionStatus(QStringLiteral("Disconnecting\u2026"));
    }

    worker_->shutdownSync();
    worker_.reset();
    {
        std::lock_guard<std::mutex> lock(pendingFrameMutex_);
        pendingFrame_.reset();
        frameFlushQueued_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(lastFrameMutex_);
        lastFrame_.reset();
    }
    updatePreviewDisconnected();

    if (host_ != nullptr && host_->bfsCameraSettings_ != nullptr)
    {
        host_->bfsCameraSettings_->setConnectedUi(false);
        host_->bfsCameraSettings_->setConnectionStatus(QStringLiteral("Disconnected"));
        host_->bfsCameraSettings_->setCaptureEnabled(false);
    }
}

void BfsPanelController::wireSettingsTabConnections()
{
    if (host_ == nullptr || host_->bfsCameraSettings_ == nullptr)
        return;

    ui::BfsCameraSettingsWidget *ui = host_->bfsCameraSettings_;
    connect(ui->refreshButton(), &QPushButton::clicked, this, &BfsPanelController::onRefreshClicked);
    connect(ui->connectButton(), &QPushButton::clicked, this, &BfsPanelController::onConnectClicked);
    connect(ui->disconnectButton(), &QPushButton::clicked, this,
            &BfsPanelController::onDisconnectClicked);
    connect(ui->captureButton(), &QPushButton::clicked, this, &BfsPanelController::onCaptureClicked);
    connect(ui, &ui::BfsCameraSettingsWidget::settingsEdited, this,
            &BfsPanelController::onSettingsEdited);

    if (!BfsSpinnakerCamera::sdkAvailable())
    {
        ui->setConnectionStatus(QStringLiteral("Spinnaker SDK not found — BFS disabled."));
        ui->setConnectedUi(false);
        ui->connectButton()->setEnabled(false);
        ui->refreshButton()->setEnabled(false);
        return;
    }

    ui->setConnectionStatus(QStringLiteral("Disconnected"));
    onRefreshClicked();
}

BfsCameraSettings BfsPanelController::settingsFromUi() const
{
    if (host_ == nullptr || host_->bfsCameraSettings_ == nullptr)
        return {};
    return host_->bfsCameraSettings_->currentSettings();
}

bool BfsPanelController::isCameraConnected() const
{
    if (worker_ == nullptr)
        return false;
    const BfsCameraState state = worker_->currentState();
    return state == BfsCameraState::Connected || state == BfsCameraState::Streaming;
}

void BfsPanelController::onRefreshClicked()
{
    if (worker_ == nullptr)
        return;
    if (host_->bfsCameraSettings_ != nullptr)
        host_->bfsCameraSettings_->setConnectionStatus(QStringLiteral("Scanning for cameras\u2026"));
    worker_->requestEnumerate();
}

void BfsPanelController::onConnectClicked()
{
    if (worker_ == nullptr)
        return;
    const BfsCameraSettings settings = settingsFromUi();
    if (host_->bfsCameraSettings_ != nullptr)
    {
        host_->bfsCameraSettings_->setConnectionStatus(QStringLiteral("Connecting\u2026"));
        host_->bfsCameraSettings_->connectButton()->setEnabled(false);
        host_->bfsCameraSettings_->refreshButton()->setEnabled(false);
        host_->bfsCameraSettings_->disconnectButton()->setEnabled(true);
    }
    host_->appendLog(hf::log::Channel::Ur3e,
                     QStringLiteral("BFS: connecting %1").arg(settings.cameraId));
    worker_->requestConnect(settings);
}

void BfsPanelController::onDisconnectClicked()
{
    if (worker_ == nullptr)
        return;
    if (host_->bfsCameraSettings_ != nullptr)
        host_->bfsCameraSettings_->setConnectionStatus(QStringLiteral("Disconnecting\u2026"));
    worker_->requestDisconnect();
}

bool BfsPanelController::tryCopyLastFrame(BfsRgbFrame &out) const
{
    std::lock_guard<std::mutex> lock(lastFrameMutex_);
    if (!lastFrame_.has_value() || lastFrame_->width <= 0 || lastFrame_->height <= 0
        || lastFrame_->rgb.size()
               < static_cast<std::size_t>(lastFrame_->width)
                     * static_cast<std::size_t>(lastFrame_->height) * 3u)
    {
        return false;
    }
    out = *lastFrame_;
    return true;
}

void BfsPanelController::onCaptureClicked()
{
    if (host_ == nullptr)
        return;

    BfsRgbFrame frame;
    if (!tryCopyLastFrame(frame))
    {
        QMessageBox::warning(host_, QStringLiteral("BFS Capture"),
                             QStringLiteral("No streamed frame available to save."));
        return;
    }

    const QString defaultName =
        QStringLiteral("bfs_%1.tif")
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss")));
    QString path = QFileDialog::getSaveFileName(
        host_,
        QStringLiteral("Save BFS capture"),
        defaultName,
        QStringLiteral("TIFF Image (*.tif *.tiff)"));
    if (path.isEmpty())
        return;

    if (!path.endsWith(QStringLiteral(".tif"), Qt::CaseInsensitive)
        && !path.endsWith(QStringLiteral(".tiff"), Qt::CaseInsensitive))
        path += QStringLiteral(".tif");

    const std::string error = saveRgb8AsTiff(path,
                                             frame.width,
                                             frame.height,
                                             frame.rgb.data(),
                                             frame.rgb.size());
    if (!error.empty())
    {
        const QString detail = QString::fromStdString(error);
        QMessageBox::critical(host_, QStringLiteral("BFS Capture"),
                              QStringLiteral("Failed to save TIFF:\n%1\n%2").arg(path, detail));
        host_->appendLog(hf::log::Channel::Ur3e,
                         QStringLiteral("BFS capture failed: %1 (%2)").arg(path, detail));
        return;
    }

    host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("BFS capture saved: %1").arg(path));
}

void BfsPanelController::onSettingsEdited()
{
    if (worker_ == nullptr || settingsApplyTimer_ == nullptr)
        return;
    if (worker_->currentState() != BfsCameraState::Streaming
        && worker_->currentState() != BfsCameraState::Connected)
        return;
    // Debounce: UI edits restart acquisition; rapid applies prevent any frames.
    settingsApplyTimer_->start();
}

void BfsPanelController::applySettingsFromUi()
{
    if (worker_ == nullptr)
        return;
    if (worker_->currentState() != BfsCameraState::Streaming
        && worker_->currentState() != BfsCameraState::Connected)
        return;
    worker_->requestApplySettings(settingsFromUi());
}

void BfsPanelController::onStateChanged(const BfsCameraState state)
{
    if (host_->bfsCameraSettings_ == nullptr)
        return;

    const bool connected =
        state == BfsCameraState::Connected || state == BfsCameraState::Streaming;
    host_->bfsCameraSettings_->setConnectedUi(connected);
    host_->bfsCameraSettings_->setConnectionStatus(stateLabel(state));

    if (!connected)
    {
        {
            std::lock_guard<std::mutex> lock(lastFrameMutex_);
            lastFrame_.reset();
        }
        streamFps_.reset();
        updatePreviewDisconnected();
        if (host_->bfsCameraSettings_ != nullptr)
            host_->bfsCameraSettings_->setCaptureEnabled(false);
    }

    if (host_->capturePanel() != nullptr)
        host_->capturePanel()->updateCamerasList();

    host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("BFS: %1").arg(stateLabel(state)));
}

void BfsPanelController::onError(const BfsError &error)
{
    const QString message = QString::fromStdString(error.message);
    if (host_->bfsCameraSettings_ != nullptr)
        host_->bfsCameraSettings_->setConnectionStatus(message);
    host_->appendLog(hf::log::Channel::Ur3e, QStringLiteral("BFS error: %1").arg(message));
}

void BfsPanelController::onDevices(const std::vector<BfsDeviceInfo> &devices)
{
    if (host_->bfsCameraSettings_ == nullptr)
        return;

    applyingDevices_ = true;
    const QString preferred = host_->bfsCameraSettings_->currentSettings().cameraId;
    host_->bfsCameraSettings_->setDevices(devices, preferred);
    applyingDevices_ = false;

    if (devices.empty())
    {
        host_->bfsCameraSettings_->setConnectionStatus(
            QStringLiteral("No Spinnaker cameras found."));
    }
    else if (worker_ != nullptr && worker_->currentState() == BfsCameraState::Disconnected)
    {
        host_->bfsCameraSettings_->setConnectionStatus(
            QStringLiteral("Found %1 camera(s). Disconnected.").arg(devices.size()));
    }

    host_->appendLog(hf::log::Channel::Ur3e,
                     QStringLiteral("BFS: enumerated %1 camera(s)").arg(devices.size()));
}

void BfsPanelController::queueFrame(BfsRgbFrame frame)
{
    streamFps_.noteFrame();
    bool scheduleFlush = false;
    {
        std::lock_guard<std::mutex> lock(pendingFrameMutex_);
        pendingFrame_ = std::move(frame);
        if (!frameFlushQueued_)
        {
            frameFlushQueued_ = true;
            scheduleFlush = true;
        }
    }
    if (scheduleFlush)
    {
        QMetaObject::invokeMethod(
            this,
            [this]() { flushPendingFrame(); },
            Qt::QueuedConnection);
    }
}

void BfsPanelController::flushPendingFrame()
{
    BfsRgbFrame frame;
    {
        std::lock_guard<std::mutex> lock(pendingFrameMutex_);
        if (!pendingFrame_.has_value())
        {
            frameFlushQueued_ = false;
            return;
        }
        frame = std::move(*pendingFrame_);
        pendingFrame_.reset();
        frameFlushQueued_ = false;
    }
    showFrameOnPreview(frame);
    {
        std::lock_guard<std::mutex> lock(lastFrameMutex_);
        lastFrame_ = std::move(frame);
    }
    if (host_->bfsCameraSettings_ != nullptr)
        host_->bfsCameraSettings_->setCaptureEnabled(true);
}

void BfsPanelController::showFrameOnPreview(const BfsRgbFrame &frame)
{
    if (host_ == nullptr || frame.width <= 0 || frame.height <= 0
        || frame.rgb.size()
               < static_cast<std::size_t>(frame.width) * static_cast<std::size_t>(frame.height) * 3u)
        return;

    QImage image(frame.rgb.data(),
                 frame.width,
                 frame.height,
                 frame.width * 3,
                 QImage::Format_RGB888);
    const QImage owned = image.copy();
    const double fps = streamFps_.fps();

    const auto paintLabel = [&owned, fps](QLabel *label) {
        if (label == nullptr)
            return;
        label->setText(QString());
        label->setWordWrap(false);
        label->setAlignment(Qt::AlignCenter);
        label->setMinimumSize(1, 1);
        label->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

        const QSize target = label->contentsRect().size();
        if (target.width() <= 1 || target.height() <= 1)
        {
            label->setPixmap(QPixmap::fromImage(owned));
            return;
        }

        QPixmap canvas(target);
        canvas.fill(QColor(0x11, 0x11, 0x11));

        const QPixmap framePm = QPixmap::fromImage(
            owned.scaled(target, Qt::KeepAspectRatio, Qt::FastTransformation));
        const QPoint origin((target.width() - framePm.width()) / 2,
                            (target.height() - framePm.height()) / 2);

        QPainter painter(&canvas);
        painter.drawPixmap(origin, framePm);

        if (fps > 0.0)
        {
            QFont font = painter.font();
            font.setBold(true);
            font.setPointSize(10);
            painter.setFont(font);
            painter.setPen(QColor(0xcc, 0xcc, 0xcc));
            painter.drawText(QRect(8, 8, target.width() - 16, 24),
                             Qt::AlignTop | Qt::AlignLeft,
                             QStringLiteral("%1 fps").arg(fps, 0, 'f', 1));
        }

        label->setPixmap(canvas);
    };

    paintLabel(host_->ur3eRgbPreviewLabel_);
    if (host_->capturePanel() != nullptr && host_->capturePanel()->isBfsCaptureSelected())
        paintLabel(host_->captureBfsPreviewLabel_);
}

void BfsPanelController::updatePreviewDisconnected()
{
    const auto clearLabel = [](QLabel *label) {
        if (label == nullptr)
            return;
        label->setPixmap(QPixmap());
        ui::setPreviewDisconnectedText(label, QStringLiteral("stream"), QStringLiteral("BFS"));
    };
    clearLabel(host_->ur3eRgbPreviewLabel_);
    clearLabel(host_->captureBfsPreviewLabel_);
}
} // namespace hf::bfs
