// Qt main window: settings tabs, stream previews, camera controls, and application log.
// Hardware access goes through CameraCoordinator; this file is UI layout and wiring only.
#include "frontend/widgets/MainWindow.hpp"

#include "frontend/logging/AppLog.hpp"
#include "frontend/logging/AppLogSession.hpp"
#include "frontend/controllers/CameraPanelController.hpp"
#include "frontend/controllers/LightPanelController.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/controllers/StagePanelController.hpp"
#include "frontend/controllers/Ur3ePanelController.hpp"
#include "frontend/controllers/BfsPanelController.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "adapters/lumo/Swir3NiCamera.hpp"
#include "frontend/processing/Overexposure.hpp"
#include "adapters/zaber/ZaberStageController.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/stage/StageWorker.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/camera/DualCameraScanOrchestrator.hpp"
#include "backend/light/LighthouseWorker.hpp"
#include "backend/camera/CameraCoordinator.hpp"
#include "adapters/mcc/Mcc1208LighthouseController.hpp"
#include "frontend/widgets/DetectorCrosshairWidget.hpp"
#include "frontend/widgets/ProfilePlotWidget.hpp"
#include "frontend/processing/ProfileProcessor.hpp"
#include "frontend/processing/DetectorFrameConverter.hpp"
#include "frontend/widgets/IntensityBarWidget.hpp"
#include "frontend/widgets/StageAxisWidget.hpp"
#include "frontend/widgets/OperationWaitDialog.hpp"
#include "frontend/widgets/CameraStreamTabBuilder.hpp"
#include "frontend/widgets/StreamPaneHelpers.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/widgets/WaterfallDisplayWidget.hpp"
#include "frontend/processing/WavelengthLookup.hpp"

#include "frontend/settings/AppSettingsStore.hpp"
#include "backend/camera/CaptureWriterWorker.hpp"
#include "backend/camera/processing/CapturePostProcessorWorker.hpp"
#include "backend/camera/processing/Gsam2ServerManager.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <thread>

#include <QComboBox>
#include <QMetaObject>
#include <QSignalBlocker>
#include <QCheckBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QApplication>
#include <QCloseEvent>
#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QImage>
#include <QAbstractButton>
#include <QMessageBox>
#include <QPainter>
#include <QPixmap>
#include <QPolygon>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QStyle>
#include <QTimer>
#include <QToolButton>
#include <QWidget>

#include "frontend/utils/SerialPortEnumerator.hpp"

#include <algorithm>
#include <vector>

namespace
{
constexpr int kDefaultRedBandIndex = 193;
constexpr int kDefaultGreenBandIndex = 112;
constexpr int kDefaultBlueBandIndex = 25;
constexpr int kFullCalpackBandCount = 448;

int scaledDefaultBandIndex(const int fullCalpackIndex, const int tableBandCount)
{
    if (tableBandCount <= 0)
        return 0;
    if (tableBandCount >= kFullCalpackBandCount)
        return std::clamp(fullCalpackIndex, 0, tableBandCount - 1);

    const int scaled =
        (fullCalpackIndex * (tableBandCount - 1)) / (kFullCalpackBandCount - 1);
    return std::clamp(scaled, 0, tableBandCount - 1);
}

QString defaultCameraTabName(const std::size_t cameraIndex)
{
    return QStringLiteral("Camera %1").arg(cameraIndex + 1);
}

QString ordinalSuffix(const int oneBased)
{
    const int mod100 = oneBased % 100;
    if (mod100 >= 11 && mod100 <= 13)
        return QStringLiteral("th");

    switch (oneBased % 10)
    {
    case 1:
        return QStringLiteral("st");
    case 2:
        return QStringLiteral("nd");
    case 3:
        return QStringLiteral("rd");
    default:
        return QStringLiteral("th");
    }
}

QString formatOrdinalPixel(const int zeroBasedSpatialIndex)
{
    const int pixelNumber = zeroBasedSpatialIndex + 1;
    return QStringLiteral("%1%2 pixel").arg(pixelNumber).arg(ordinalSuffix(pixelNumber));
}

} // namespace

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle("HyperFusion");
    resize(1550, 920);
    setMinimumSize(1180, 760);

    (void)hf::loadHardwareConfig();
    use3dScanning_ = hf::hardwareConfig().ur3e.use3dScanning;

    stagePanel_ = std::make_unique<hf::stage::StagePanelController>(this);
    if (use3dScanning_)
    {
        ur3ePanel_ = std::make_unique<hf::ur3e::Ur3ePanelController>(this);
        bfsPanel_ = std::make_unique<hf::bfs::BfsPanelController>(this);
    }
    lightPanel_ = std::make_unique<hf::light::LightPanelController>(this);
    cameraPanel_ = std::make_unique<hf::camera::CameraPanelController>(this);
    settingsPanel_ = std::make_unique<hf::settings::UiSettingsController>(this);
    capturePanel_ = std::make_unique<hf::capture::CapturePanelController>(this);

    auto *central = new QWidget(this);
    auto *rootLayout = new QVBoxLayout(central);

    auto *splitter = new QSplitter(Qt::Horizontal, central);
    splitter->setChildrenCollapsible(false);
    splitter->addWidget(createSettingsPanel());
    splitter->addWidget(createStreamTabsPanel());
    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);
    splitter->setSizes({420, 1130});

    auto *logBox = new QGroupBox(QStringLiteral("Log"), central);
    auto *logLayout = new QVBoxLayout(logBox);
    logTabs_ = new QTabWidget(logBox);
    logTabs_->setDocumentMode(true);
    logTabs_->setMinimumHeight(140);
    logTabs_->setMaximumHeight(220);

    for (int i = 0; i < hf::log::channelCount(); ++i)
    {
        const auto channel = static_cast<hf::log::Channel>(i);
        auto *output = new QPlainTextEdit(logTabs_);
        output->setReadOnly(true);
        output->setMaximumBlockCount(5000);
        output->setPlaceholderText(
            QStringLiteral("%1 log output…").arg(hf::log::channelLabel(channel)));
        logOutputs_[static_cast<std::size_t>(i)] = output;
        logTabs_->addTab(output, hf::log::channelTabTitle(channel));
    }

    logLayout->addWidget(logTabs_);

    if (QCoreApplication::instance() != nullptr)
        sessionLog_.begin(QCoreApplication::applicationDirPath());

    rootLayout->addWidget(splitter, 1);
    rootLayout->addWidget(logBox, 0);
    setCentralWidget(central);

    cameraPanel_->initializeCameras();

    settingsPanel_->loadHardwareConfig();
    settingsPanel_->loadPersistedUiSettings();
    settingsPanel_->applyHardwareConfigToUi();
    capturePanel_->applyDualCameraScanSync(false);
    stagePanel_->refreshComPortList();
    settingsPanel_->connectAutosave();

    QTimer::singleShot(0, this, [this]() {
        cameraPanel_->refreshDeviceLists();
        settingsPanel_->applyPersistedCameraProfilesAndBands();
    });

    stagePanel_->initializeWorker();
    stagePanel_->wireSettingsTabConnections();
    if (use3dScanning_)
    {
        ur3ePanel_->applyHardwareConfigToUi();
        ur3ePanel_->wireSettingsTabConnections();
        bfsPanel_->initializeWorker();
        bfsPanel_->wireSettingsTabConnections();
    }
    lightPanel_->initializeWorker();

    capturePanel_->initializeWorkers();
    capturePanel_->wireSettingsTabConnections();
    capturePanel_->updateGsamServerUi();
    capturePanel_->updateRecorderControls();

    QTimer::singleShot(250, this, [this]() {
        if (sessionLog_.isOpen())
        {
            appendLog(hf::log::Channel::App,
                      QStringLiteral("Session log file: %1").arg(sessionLog_.filePath()));
        }
        if (use3dScanning_ && ur3ePanel_ != nullptr)
            ur3ePanel_->startSidecarOnLaunch();
    });

    // Defer GSAM2 so WSL is not hammered at the same moment as UR3e startup.
    QTimer::singleShot(3000, this, [this]() {
        if (capturePanel_ != nullptr)
            capturePanel_->tryAutoStartGsamServer();
    });
}

MainWindow::~MainWindow()
{
    if (!gracefulShutdownDone_)
        (void)performGracefulShutdown();
}

hf::capture::CapturePanelController *MainWindow::capturePanel() const
{
    return capturePanel_.get();
}

bool MainWindow::isCaptureSessionActive() const
{
    return capturePanel_ != nullptr && capturePanel_->isSessionActive();
}

void MainWindow::onSettingsTabChanged(const int index)
{
    if (index == kSettingsTabStage)
        stagePanel_->refreshComPortList();
    else if (index == captureSettingsTabIndex_)
    {
        capturePanel_->updateCamerasList();
        if (stagePanel_->worker() != nullptr)
            capturePanel_->updatePositionControls(stagePanel_->worker()->currentState());
    }
    else if (index == kSettingsTabLight)
        lightPanel_->syncUiFromBackend();
    else if (ur3eSettingsTabIndex_ >= 0 && index == ur3eSettingsTabIndex_ && ur3ePanel_ != nullptr)
        ur3ePanel_->refreshUi();

    if (streamTabs_ != nullptr && streamTabs_->currentIndex() == captureStreamTabIndex_
        && cameraPanel_ != nullptr)
    {
        cameraPanel_->refreshWaterfallDisplayTargets();
    }
}

hf::stage::StagePanelController *MainWindow::stagePanel() const { return stagePanel_.get(); }
hf::ur3e::Ur3ePanelController *MainWindow::ur3ePanel() const { return ur3ePanel_.get(); }
hf::light::LightPanelController *MainWindow::lightPanel() const { return lightPanel_.get(); }
hf::camera::CameraPanelController *MainWindow::cameraPanel() const { return cameraPanel_.get(); }
hf::settings::UiSettingsController *MainWindow::settingsPanel() const { return settingsPanel_.get(); }

StageWorker *MainWindow::stageWorker() const
{
    return stagePanel_ != nullptr ? stagePanel_->worker() : nullptr;
}

LighthouseWorker *MainWindow::lighthouseWorker() const
{
    return lightPanel_ != nullptr ? lightPanel_->worker() : nullptr;
}

CameraCoordinator *MainWindow::coordinator() const
{
    return cameraPanel_ != nullptr ? cameraPanel_->coordinator() : nullptr;
}


QWidget *MainWindow::createStreamTabsPanel()
{
    auto *panel = new QWidget(this);
    auto *layout = new QVBoxLayout(panel);

    streamTabs_ = new QTabWidget(panel);
    const ui::CameraStreamTabBuilder::StreamTabHooks streamHooks{
        [this](LumoCameraUi &ui, const int spatialIndex, const int bandIndex) {
            cameraPanel_->onProfileLinesChanged(ui, spatialIndex, bandIndex);
        },
        [this](LumoCameraUi &ui) { cameraPanel_->syncProfileRgbMarkers(ui); },
        [this](LumoCameraUi &ui) { cameraPanel_->updateStreamPaneTitles(ui); },
        [this](const LumoCameraUi &ui) { return cameraPanel_->profileTabNameForUi(ui); },
    };
    streamTabs_->addTab(ui::CameraStreamTabBuilder::buildCameraStreamTab(
                            this, cameraPanel_->profileTabNameForUi(camera1Ui_), camera1Ui_, streamHooks),
                        cameraPanel_->profileTabNameForUi(camera1Ui_));
    streamTabs_->addTab(ui::CameraStreamTabBuilder::buildCameraStreamTab(
                            this, cameraPanel_->profileTabNameForUi(camera2Ui_), camera2Ui_, streamHooks),
                        cameraPanel_->profileTabNameForUi(camera2Ui_));
    if (use3dScanning_)
    {
        ur3eStreamTabIndex_ = streamTabs_->count();
        streamTabs_->addTab(createUr3eStreamTab(), QStringLiteral("3D Scanning"));
    }
    else
    {
        ur3eStreamTabIndex_ = -1;
    }
    captureStreamTabIndex_ = streamTabs_->count();
    streamTabs_->addTab(capturePanel_->createStreamTab(), QStringLiteral("Capture"));

    layout->addWidget(streamTabs_, 1);
    return panel;
}

bool MainWindow::isCameraSessionActive(const CameraState state)
{
    return hf::camera::CameraPanelController::isSessionActive(state);
}

bool MainWindow::anyCameraSessionActive() const
{
    return cameraPanel_ != nullptr && cameraPanel_->anySessionActive();
}

bool MainWindow::isStageSessionActive() const
{
    return stagePanel_ != nullptr && stagePanel_->isSessionActive();
}

bool MainWindow::isLighthouseSessionActive() const
{
    return lightPanel_ != nullptr && lightPanel_->isSessionActive();
}

void MainWindow::waitWithBusyDialog(OperationWaitDialog &dialog, const std::function<void()> &work)
{
    std::atomic<bool> done{false};
    std::thread worker([&done, &work]() {
        work();
        done.store(true, std::memory_order_release);
    });

    while (!done.load(std::memory_order_acquire))
    {
        dialog.raise();
        QApplication::processEvents(QEventLoop::AllEvents);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (worker.joinable())
        worker.join();
}

bool MainWindow::performGracefulShutdown()
{
    if (gracefulShutdownDone_)
        return true;

    cameraPanel_->dismissAllCameraOperationWaits();
    if (capturePanel_ != nullptr)
        capturePanel_->dismissDualCameraSyncWaitDialog();

    performingGracefulShutdown_ = true;

    OperationWaitDialog waitDialog(this);
    waitDialog.show();
    QApplication::processEvents();

    if (capturePanel_ != nullptr)
    {
        capturePanel_->stopRecorder();
        capturePanel_->shutdownWorkers();
    }

    if (cameraPanel_ != nullptr && anyCameraSessionActive())
    {
        waitDialog.setStatusText(tr("Disconnecting cameras\u2026"));
        QApplication::processEvents();
        waitWithBusyDialog(waitDialog, [this]() {
            if (cameraPanel_ != nullptr)
                cameraPanel_->shutdownCoordinatorSync();
        });
    }

    // Release Spinnaker before stage/light/UR3e teardown so GigE camera is free on exit.
    if (use3dScanning_ && bfsPanel_ != nullptr)
    {
        waitDialog.setStatusText(tr("Disconnecting BFS camera\u2026"));
        QApplication::processEvents();
        waitWithBusyDialog(waitDialog, [this]() {
            if (bfsPanel_ != nullptr)
                bfsPanel_->shutdownSync();
        });
    }

    if (stagePanel_ != nullptr)
    {
        if (isStageSessionActive())
        {
            waitDialog.setStatusText(tr("Homing stage and disconnecting\u2026"));
            QApplication::processEvents();
        }
        waitWithBusyDialog(waitDialog, [this]() {
            if (stagePanel_ != nullptr)
                stagePanel_->shutdownSync(isStageSessionActive());
        });
    }

    if (lightPanel_ != nullptr)
    {
        if (isLighthouseSessionActive())
        {
            waitDialog.setStatusText(tr("Shutting down lighthouse outputs\u2026"));
            QApplication::processEvents();
        }
        waitWithBusyDialog(waitDialog, [this]() {
            if (lightPanel_ != nullptr)
                lightPanel_->shutdownSync();
        });
    }

    if (use3dScanning_ && ur3ePanel_ != nullptr)
    {
        if (ur3ePanel_->isRobotConnected())
        {
            waitDialog.setStatusText(tr("Moving UR3e to scan home\u2026"));
            QApplication::processEvents();
        }
        bool shutdownCompleted = true;
        waitWithBusyDialog(waitDialog, [this, &shutdownCompleted]() {
            if (ur3ePanel_ != nullptr)
                shutdownCompleted = ur3ePanel_->shutdownSync();
        });
        if (!shutdownCompleted)
        {
            performingGracefulShutdown_ = false;
            return false;
        }
    }

    if (cameraPanel_ != nullptr)
        cameraPanel_->stopStreamPipeline();

    

    settingsPanel_->savePersistedUiSettings();
    sessionLog_.close();
    gracefulShutdownDone_ = true;
    return true;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    if (isCaptureSessionActive())
    {
        event->ignore();
        QMessageBox::warning(this,
                             tr("Capture in progress"),
                             tr("Stop preview or recording before closing the application."));
        return;
    }

    event->ignore();
    if (!performGracefulShutdown())
        return;
    event->accept();
    QMainWindow::closeEvent(event);
}
void MainWindow::appendLog(const QString &message)
{
    appendLog(hf::log::classifyMessage(message), message);
}

void MainWindow::appendLog(const hf::log::Channel channel, const QString &message)
{
    const std::size_t index = static_cast<std::size_t>(channel);
    if (index >= logOutputs_.size() || logOutputs_[index] == nullptr)
        return;

    const QString ts = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss"));
    logOutputs_[index]->appendPlainText(QStringLiteral("[%1] %2").arg(ts, message));
    sessionLog_.write(channel, message);
}
