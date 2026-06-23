// Camera session logic for the Qt UI: coordinator wiring, connect/stream policy, profile tabs.
// Keeps MainWindow focused on layout; posts state, frames, and errors back via Qt signals.
#include "frontend/controllers/CameraAppController.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/CalibrationPackPaths.hpp"
#include "adapters/lumo/Swir3NiCamera.hpp"
#include "frontend/widgets/LumoCameraUi.hpp"
#include "backend/CameraCoordinator.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "frontend/processing/DetectorFrameConverter.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFileInfo>
#include <QImage>

#include <functional>
#include <vector>

namespace
{
bool lumoProfileMatchesFx10eSlot(const QString &name)
{
    if (name.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive))
        return false;
    return name.contains(QStringLiteral("FX10"), Qt::CaseInsensitive)
           || name.contains(QStringLiteral("Pleora"), Qt::CaseInsensitive);
}

bool lumoProfileMatchesSwir3Slot(const QString &name)
{
    return name.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive)
           || name.contains(QStringLiteral("NI"), Qt::CaseInsensitive);
}
} // namespace

CameraAppController::CameraAppController(QObject *parent) : QObject(parent) {}

CameraAppController::~CameraAppController()
{
    shutdown();
}

void CameraAppController::bindCameras(LumoCameraUi &camera1, LumoCameraUi &camera2)
{
    camera1_ = &camera1;
    camera2_ = &camera2;

    if (camera1_.camera == nullptr)
        camera1_.camera = std::make_shared<LumoCamera>(CameraBackendId::Camera1,
                                                         "FX10e",
                                                         LumoSensorKind::Fx10ePleora);
    if (camera2_.camera == nullptr)
        camera2_.camera =
            std::make_shared<Swir3NiCamera>(CameraBackendId::Camera2, "SWIR3");

    camera1_.cameraIndex = 0;
    camera2_.cameraIndex = 1;

    coordinator_ = std::make_unique<CameraCoordinator>(
        std::vector<std::shared_ptr<ICameraController>>{camera1_.camera, camera2_.camera});

    coordinator_->setLogCallback([this](const std::string &message) {
        const QString line = QString::fromStdString(message);
        QMetaObject::invokeMethod(
            this,
            [this, line]() { emit logMessage(line); },
            Qt::QueuedConnection);
    });

    coordinator_->setGuiTaskRunner([](std::function<void()> task) {
        QMetaObject::invokeMethod(
            qApp,
            [t = std::move(task)]() {
                t();
                for (int i = 0; i < 8; ++i)
                    QCoreApplication::processEvents(QEventLoop::AllEvents);
            },
            Qt::BlockingQueuedConnection);
    });

    coordinator_->setCameraStateCallback(0, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() {
                if (camera1_ != nullptr)
                    onCameraStateChanged(*camera1_, state);
            },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraStateCallback(1, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() {
                if (camera2_ != nullptr)
                    onCameraStateChanged(*camera2_, state);
            },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraErrorCallback(0, [this](const CameraError &error) {
        QMetaObject::invokeMethod(
            this,
            [this, error]() {
                if (camera1_ != nullptr)
                    onCameraError(*camera1_, error);
            },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraErrorCallback(1, [this](const CameraError &error) {
        QMetaObject::invokeMethod(
            this,
            [this, error]() {
                if (camera2_ != nullptr)
                    onCameraError(*camera2_, error);
            },
            Qt::QueuedConnection);
    });

    coordinator_->setFrameCallback([this](const FramePacket &frame) {
        QMetaObject::invokeMethod(
            this,
            [this, frame]() { onFrame(frame); },
            Qt::QueuedConnection);
    });
}

void CameraAppController::start()
{
    if (!coordinator_)
        return;

    coordinator_->start();
    emit logMessage(QStringLiteral("HyperFusion camera controller started."));
}

void CameraAppController::shutdown()
{
    if (!coordinator_)
        return;

    coordinator_->shutdownSync();
    coordinator_.reset();
}

QString CameraAppController::defaultFx10eCalibrationPackPath()
{
    return lumo::resolveBundledCalibrationPackPath(LumoSensorKind::Fx10ePleora);
}

CameraSettings CameraAppController::settingsFromUi(const LumoCameraUi &ui)
{
    CameraSettings settings;
    settings.frameRateHz = ui.frameRateSpin->value();
    settings.exposureMs = ui.exposureSpin->value();
    if (ui.spectralBinningCombo != nullptr)
        settings.spectralBinning = ui.spectralBinningCombo->currentText().toInt();
    if (ui.spatialBinningCombo != nullptr)
        settings.spatialBinning = ui.spatialBinningCombo->currentText().toInt();
    settings.externalTrigger = false;
    settings.acquisitionTimeoutMs =
        ui.camera != nullptr && ui.camera->sensorKind() == LumoSensorKind::Swir3Ni ? 30000U : 5000U;
    if (ui.camera != nullptr && ui.camera->sensorKind() == LumoSensorKind::Swir3Ni)
    {
        settings.niGrabberChannel = "img0";
        settings.niImaqCameraFile = "Specim_SWIR3.icd";
        settings.niCameraSerialPort.clear();
    }
    settings.deviceIndex = ui.deviceCombo->currentData().toInt();
    if (ui.deviceCombo != nullptr)
        settings.profileName = ui.deviceCombo->currentText().toStdString();
    if (ui.calibrationPackEdit != nullptr)
    {
        QString stored = ui.calibrationPackEdit->property(QStringLiteral("hf_calibrationPackPath")).toString();
        if (stored.isEmpty())
            stored = ui.calibrationPackEdit->text();
        settings.lumoCalibrationPackPath =
            lumo::resolveCalibrationPackPath(stored, ui.sensorKind).toStdString();
    }
    return settings;
}

void CameraAppController::refreshDeviceProfiles()
{
    if (camera1_ == nullptr || camera1_->camera == nullptr)
        return;

    const CameraSettings prep;

    std::vector<LumoDeviceEntry> devices;
    CameraError error;
    if (!LumoCamera::enumerateDevices(prep, devices, error))
    {
        emit logMessage(QStringLiteral("Lumo: profile refresh failed \u2014 %1")
                            .arg(QString::fromStdString(error.message)));
        return;
    }

    auto populateCombo = [&devices](QComboBox *combo,
                                    const std::function<bool(const LumoDeviceEntry &)> &include) {
        if (combo == nullptr)
            return;

        combo->clear();
        for (const LumoDeviceEntry &device : devices)
        {
            if (!include(device))
                continue;

            const QString label = QString::fromStdString(device.name);
            combo->addItem(label, device.index);
        }
    };

    auto selectProfileHint = [](QComboBox *combo, const QString &hint) -> bool {
        if (combo == nullptr)
            return false;
        for (int i = 0; i < combo->count(); ++i)
        {
            if (combo->itemText(i).contains(hint, Qt::CaseInsensitive))
            {
                combo->setCurrentIndex(i);
                return true;
            }
        }
        return false;
    };

    populateCombo(camera1_->deviceCombo, [](const LumoDeviceEntry &device) {
        return lumoProfileMatchesFx10eSlot(QString::fromStdString(device.name));
    });
    populateCombo(camera2_->deviceCombo, [](const LumoDeviceEntry &device) {
        return lumoProfileMatchesSwir3Slot(QString::fromStdString(device.name));
    });

    if (!selectProfileHint(camera1_->deviceCombo, QStringLiteral("FX10e with Pleora")))
        selectProfileHint(camera1_->deviceCombo, QStringLiteral("FX10"));
    selectProfileHint(camera2_->deviceCombo, QStringLiteral("SWIR"));

    emit logMessage(QStringLiteral("Lumo: found %1 SSP profile(s) (from SDK install).").arg(devices.size()));
    for (const LumoDeviceEntry &device : devices)
        emit logMessage(QStringLiteral("  [%1] %2").arg(device.index).arg(QString::fromStdString(device.name)));
}

void CameraAppController::connectOrDisconnect(LumoCameraUi &ui, const QString &panelTitle)
{
    if (!coordinator_ || !ui.camera)
        return;

    auto &session = sessionFor(ui.cameraIndex);

    const bool disconnectRequested =
        ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;

    if (disconnectRequested)
    {
        session.connectAttemptActive = false;
        coordinator_->disconnectOnGuiThread(ui.cameraIndex);
        emit logMessage(QStringLiteral("%1: disconnect requested.").arg(panelTitle));
        return;
    }

    if (ui.deviceCombo == nullptr || ui.deviceCombo->count() == 0)
    {
        emit logMessage(QStringLiteral("%1: refresh SSP profiles before connecting.").arg(panelTitle));
        return;
    }

    const CameraSettings connectionSettings = settingsFromUi(ui);
    ui.camera->prepareConnection(connectionSettings);
    session.connectAttemptActive = true;
    emit logMessage(QStringLiteral("%1: connect camera \u2014 profile %2 (eBUS picker may appear; not ready until Initialized).")
                        .arg(panelTitle, ui.deviceCombo->currentText()));

    coordinator_->connectAndInitializeOnGuiThread(ui.cameraIndex);
}

void CameraAppController::applySettings(LumoCameraUi &ui, const QString &panelTitle)
{
    if (!coordinator_)
        return;

    const CameraSettings settings = settingsFromUi(ui);
    coordinator_->applySettings(ui.cameraIndex, settings);
    emit logMessage(QStringLiteral("%1: apply settings (exposure=%2 ms, fps=%3)")
                        .arg(panelTitle)
                        .arg(settings.exposureMs, 0, 'f', 3)
                        .arg(settings.frameRateHz, 0, 'f', 1));
}

LumoCameraUi *CameraAppController::cameraUi(const std::size_t cameraIndex)
{
    if (camera1_ != nullptr && camera1_->cameraIndex == cameraIndex)
        return camera1_;
    if (camera2_ != nullptr && camera2_->cameraIndex == cameraIndex)
        return camera2_;
    return nullptr;
}

CameraAppController::CameraSession &CameraAppController::sessionFor(const std::size_t cameraIndex)
{
    if (cameraIndex == 0)
        return session1_;
    return session2_;
}

void CameraAppController::onCameraError(LumoCameraUi &ui, const CameraError &error)
{
    auto &session = sessionFor(ui.cameraIndex);
    if (!session.connectAttemptActive)
        return;

    session.connectAttemptActive = false;
    session.autoStreamStarted = false;

    const QString title = QStringLiteral("Camera %1 connection failed").arg(ui.cameraIndex + 1);
    const QString message = QString::fromStdString(error.message);
    emit logMessage(QStringLiteral("%1: %2").arg(title, message));
    emit connectionFailed(ui.cameraIndex, title, message);
}

void CameraAppController::onCameraStateChanged(LumoCameraUi &ui, const CameraState state)
{
    ui.state = state;
    emit cameraStateUpdated(ui, state);

    auto &session = sessionFor(ui.cameraIndex);

    if (state == CameraState::Initialized && coordinator_ != nullptr && !session.autoStreamStarted)
    {
        session.autoStreamStarted = true;
        const CameraSettings settings = settingsFromUi(ui);
        coordinator_->beginStreaming(ui.cameraIndex, settings);
        emit logMessage(QStringLiteral("Camera %1: streaming started automatically.")
                            .arg(ui.cameraIndex + 1));
    }

    if (state == CameraState::Streaming)
        session.connectAttemptActive = false;

    if (state == CameraState::Disconnected)
    {
        session.autoStreamStarted = false;
        session.connectAttemptActive = false;
    }
}

void CameraAppController::onFrame(const FramePacket &frame)
{
    std::size_t cameraIndex = 0;
    if (frame.source == CameraBackendId::Camera2)
        cameraIndex = 1;
    else if (frame.source != CameraBackendId::Camera1)
        return;

    const QImage image = ui::framePacketToQImage(frame);
    if (image.isNull())
        return;

    emit detectorFrameReady(cameraIndex, image);
}
