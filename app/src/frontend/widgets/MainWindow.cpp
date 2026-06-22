// Qt main window: settings tabs, stream previews, camera controls, and application log.
// Hardware access goes through CameraCoordinator; this file is UI layout and wiring only.
#include "frontend/widgets/MainWindow.hpp"

#include "frontend/controllers/CameraPanelController.hpp"
#include "frontend/controllers/LightPanelController.hpp"
#include "frontend/controllers/UiSettingsController.hpp"
#include "frontend/controllers/StagePanelController.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/LumoDeviceTypes.hpp"
#include "adapters/lumo/Swir3NiCamera.hpp"
#include "frontend/processing/Overexposure.hpp"
#include "adapters/zaber/ZaberStageController.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/StageWorker.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/DualCameraScanOrchestrator.hpp"
#include "backend/LighthouseWorker.hpp"
#include "backend/CameraCoordinator.hpp"
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
#include "backend/CaptureWriterWorker.hpp"
#include "backend/processing/CapturePostProcessorWorker.hpp"
#include "backend/processing/Gsam2ServerManager.hpp"

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
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
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
constexpr auto kFx10eCalibrationFileName = "3210441_20211027_calpack.scp";
constexpr auto kSwir3CalibrationFileName = "462111_OLES15_20250109_calpack.scp";
constexpr auto kCalibrationPackPathProperty = "hf_calibrationPackPath";
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

    stagePanel_ = std::make_unique<hf::stage::StagePanelController>(this);
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

    auto *logBox = new QGroupBox("Log", central);
    auto *logLayout = new QVBoxLayout(logBox);
    logOutput_ = new QPlainTextEdit(logBox);
    logOutput_->setReadOnly(true);
    logOutput_->setMaximumBlockCount(5000);
    logOutput_->setPlaceholderText("Application log output...");
    logOutput_->setMinimumHeight(140);
    logOutput_->setMaximumHeight(220);
    logLayout->addWidget(logOutput_);

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
    lightPanel_->initializeWorker();

    capturePanel_->initializeWorkers();
}

MainWindow::~MainWindow()
{
    if (!gracefulShutdownDone_)
        performGracefulShutdown();
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
    else if (index == kSettingsTabCapture)
    {
        capturePanel_->updateCamerasList();
        if (capturePanel_ != nullptr)
            capturePanel_->updateCaptureCameraPositionRows();
        if (stagePanel_->worker() != nullptr)
            capturePanel_->updatePositionControls(stagePanel_->worker()->currentState());
    }
    else if (index == kSettingsTabLight)
        lightPanel_->syncUiFromBackend();
}

hf::stage::StagePanelController *MainWindow::stagePanel() const { return stagePanel_.get(); }
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
    streamTabs_->addTab(createRgbUr3eStreamTab(), QStringLiteral("UR3e"));
    streamTabs_->addTab(capturePanel_->createStreamTab(), QStringLiteral("Capture"));

    layout->addWidget(streamTabs_, 1);
    return panel;
}


QWidget *MainWindow::createRgbUr3eStreamTab()
{
    auto *tab = new QWidget(this);
    auto *layout = new QVBoxLayout(tab);

    auto *gridHost = new QWidget(tab);
    auto *grid = new QGridLayout(gridHost);
    grid->setContentsMargins(8, 8, 8, 8);
    grid->setSpacing(10);

    QLabel *rgbLabel = nullptr;
    QLabel *poseLabel = nullptr;

    auto *rgbPane = ui::createPreviewPane(tab, QStringLiteral("RGB"), rgbLabel);
    auto *posePane = ui::createPreviewPane(tab, QStringLiteral("Robot / pose"), poseLabel);

    rgbLabel->setText("UR3e RGB preview (multi-angle capture) \u2014 disconnected");
    poseLabel->setText("UR3e pose / path preview (TODO) \u2014 disconnected");

    grid->addWidget(rgbPane, 0, 0);
    grid->addWidget(posePane, 0, 1);
    grid->setRowStretch(0, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    layout->addWidget(gridHost, 1);
    return tab;
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

void MainWindow::performGracefulShutdown()
{
    if (gracefulShutdownDone_)
        return;

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

    if (cameraPanel_ != nullptr)
        cameraPanel_->stopStreamPipeline();

    

    settingsPanel_->savePersistedUiSettings();
    gracefulShutdownDone_ = true;
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
    performGracefulShutdown();
    event->accept();
    QMainWindow::closeEvent(event);
}
void MainWindow::appendLog(const QString &message)
{
    if (logOutput_ == nullptr)
        return;

    const QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    logOutput_->appendPlainText(QString("[%1] %2").arg(ts, message));
}
