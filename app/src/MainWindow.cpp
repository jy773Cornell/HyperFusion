// Qt main window: settings tabs, stream previews, camera controls, and application log.
// Hardware access goes through CameraCoordinator; this file is UI layout and wiring only.
#include "MainWindow.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/Swir3NiCamera.hpp"
#include "adapters/zaber/ZaberStageController.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "core/StageWorker.hpp"
#include "orchestrator/CameraCoordinator.hpp"
#include "ui/DetectorCrosshairWidget.hpp"
#include "ui/ProfilePlotWidget.hpp"
#include "ui/ProfileProcessor.hpp"

#include <algorithm>
#include <cmath>

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
#include <QSlider>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QImage>
#include <QMessageBox>
#include <QPixmap>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QTimer>
#include <QWidget>

#include "ui/SerialPortEnumerator.hpp"

#include <algorithm>
#include <vector>

namespace
{
QImage framePacketToQImage(const FramePacket &frame)
{
    if (frame.width <= 0 || frame.height <= 0 || frame.pixels.empty())
        return {};

    const int width = frame.width;
    const int height = frame.height;
    const std::size_t pixelCount = static_cast<std::size_t>(width * height);
    if (frame.pixels.size() < pixelCount)
        return {};

    std::uint16_t minValue = frame.pixels[0];
    std::uint16_t maxValue = frame.pixels[0];
    for (std::size_t i = 1; i < pixelCount; ++i)
    {
        minValue = std::min(minValue, frame.pixels[i]);
        maxValue = std::max(maxValue, frame.pixels[i]);
    }

    QImage image(width, height, QImage::Format_Grayscale8);
    if (maxValue == minValue)
    {
        image.fill(0);
        return image;
    }

    const double scale = 255.0 / static_cast<double>(maxValue - minValue);
    for (int y = 0; y < height; ++y)
    {
        auto *scanLine = image.scanLine(y);
        for (int x = 0; x < width; ++x)
        {
            const std::uint16_t value = frame.pixels[static_cast<std::size_t>(y * width + x)];
            scanLine[x] = static_cast<unsigned char>((value - minValue) * scale);
        }
    }

    return image;
}

constexpr auto kFx10eCalibrationFileName = "3210441_20211027_calpack.scp";
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

void setPreviewDisconnectedText(QLabel *label, const QString &paneTitle, const QString &cameraName)
{
    if (label == nullptr)
        return;

    label->setText(cameraName + QStringLiteral(" ") + paneTitle + QStringLiteral(" (disconnected)"));
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

std::vector<double> buildWavelengthNmLookup(const std::vector<SpectralBand> &bands, const int bandCount);

QString formatStageTopology(const StageTopology &topology)
{
    if (topology.devices.empty())
        return QStringLiteral("Not connected.");

    QString text;
    if (!topology.portName.empty())
    {
        text += QStringLiteral("Port: %1 @ %2\n\n")
                    .arg(QString::fromStdString(topology.portName))
                    .arg(topology.baudRate);
    }

    for (const StageDeviceInfo &device : topology.devices)
    {
        text += QStringLiteral("Device %1: %2\n")
                    .arg(device.deviceAddress)
                    .arg(QString::fromStdString(device.name));
        text += QStringLiteral("  Serial: %1\n").arg(device.serialNumber);
        text += QStringLiteral("  Firmware: %1\n")
                    .arg(QString::fromStdString(device.firmwareVersion));
        text += QStringLiteral("  Axes: %1\n").arg(device.axisCount);

        for (const StageAxisInfo &axis : device.axes)
        {
            text += QStringLiteral("    Axis %1: %2")
                        .arg(axis.axisNumber)
                        .arg(QString::fromStdString(axis.peripheralName));
            if (axis.peripheralSerialNumber != 0)
                text += QStringLiteral(" (SN %1)").arg(axis.peripheralSerialNumber);
            if (axis.axisNumber == 1)
                text += QStringLiteral(" — right");
            else if (axis.axisNumber == 2)
                text += QStringLiteral(" — left");
            text += QLatin1Char('\n');
        }

        text += QLatin1Char('\n');
    }

    if (!topology.stageType.empty() || topology.travelLengthMm > 0.0 || topology.lockstepEnabled)
    {
        text += QStringLiteral("Configuration\n");
        if (!topology.stageType.empty())
            text += QStringLiteral("  Stage: %1\n").arg(QString::fromStdString(topology.stageType));
        if (topology.travelLengthMm > 0.0)
            text += QStringLiteral("  Travel limit: %1 mm\n").arg(topology.travelLengthMm, 0, 'f', 0);
        if (topology.lockstepEnabled)
        {
            text += QStringLiteral("  Lockstep group %1: axis %2 (primary), axis %3\n")
                        .arg(topology.lockstepGroupId)
                        .arg(topology.lockstepPrimaryAxis)
                        .arg(topology.lockstepSecondaryAxis);
        }
    }

    return text.trimmed();
}
} // namespace

QString MainWindow::shortProfileTabName(const QString &profileName)
{
    QString name = profileName.trimmed();
    if (name.isEmpty())
        return {};

    const int withIndex = name.indexOf(QStringLiteral(" with "), Qt::CaseInsensitive);
    if (withIndex > 0)
        name = name.left(withIndex).trimmed();

    return name;
}

QString MainWindow::profileTabNameForUi(const LumoCameraUi &ui) const
{
    if (ui.deviceCombo != nullptr && ui.deviceCombo->count() > 0)
    {
        const QString shortName = shortProfileTabName(ui.deviceCombo->currentText());
        if (!shortName.isEmpty())
            return shortName;
    }

    return defaultCameraTabName(ui.cameraIndex);
}

void MainWindow::updateCameraTabLabel(const LumoCameraUi &ui)
{
    const QString tabName = profileTabNameForUi(ui);
    const int tabIndex = static_cast<int>(ui.cameraIndex);

    if (cameraSettingsTabs_ != nullptr && tabIndex >= 0 && tabIndex < cameraSettingsTabs_->count())
        cameraSettingsTabs_->setTabText(tabIndex, tabName);

    if (streamTabs_ != nullptr && tabIndex >= 0 && tabIndex < streamTabs_->count())
        streamTabs_->setTabText(tabIndex, tabName);
}

void MainWindow::onCameraSettingsTabChanged(const int index)
{
    if (index < 0 || index > 1)
        return;

    QSignalBlocker settingsBlocker(settingsTabs_);
    QSignalBlocker streamBlocker(streamTabs_);

    if (settingsTabs_ != nullptr && settingsTabs_->currentIndex() != kSettingsTabCamera)
        settingsTabs_->setCurrentIndex(kSettingsTabCamera);

    if (streamTabs_ != nullptr && streamTabs_->currentIndex() != index)
        streamTabs_->setCurrentIndex(index);
}

void MainWindow::onSettingsTabChanged(const int index)
{
    if (index == kSettingsTabStage)
        refreshStageComPortList();
}

void MainWindow::selectBandComboIndex(QComboBox *combo, const int bandIndex)
{
    if (combo == nullptr)
        return;

    for (int i = 0; i < combo->count(); ++i)
    {
        if (combo->itemData(i).toInt() == bandIndex)
        {
            combo->setCurrentIndex(i);
            return;
        }
    }
}

void MainWindow::refreshBandCombos(LumoCameraUi &ui)
{
    if (ui.redBandCombo == nullptr || ui.greenBandCombo == nullptr || ui.blueBandCombo == nullptr)
        return;

    QString calpackPath = calibrationPackPath(ui);
    if (calpackPath.isEmpty())
        calpackPath = defaultFx10eCalibrationPackPath();

    if (calpackPath.isEmpty() || !QFileInfo::exists(calpackPath))
    {
        for (QComboBox *combo : {ui.redBandCombo, ui.greenBandCombo, ui.blueBandCombo})
        {
            combo->clear();
            combo->addItem(QStringLiteral("(Set calibration pack — Browse…)"));
            combo->setEnabled(true);
        }
        appendLog(QStringLiteral("%1: calibration pack not set or not found — use Browse to load "
                                "an .scp file for RGB band selection.")
                      .arg(profileTabNameForUi(ui)));
        return;
    }

    int spectralBinning = 1;
    if (ui.spectralBinningCombo != nullptr && ui.spectralBinningCombo->currentIndex() >= 0)
        spectralBinning = ui.spectralBinningCombo->currentText().toInt();

    std::vector<SpectralBand> bands;
    std::string error;
    if (!CalpackBandCatalog::loadFromCalpack(calpackPath, spectralBinning, bands, error))
    {
        for (QComboBox *combo : {ui.redBandCombo, ui.greenBandCombo, ui.blueBandCombo})
        {
            combo->clear();
            combo->addItem(QStringLiteral("(Failed to read calibration pack)"));
        }
        appendLog(QStringLiteral("%1: band list — %2")
                      .arg(profileTabNameForUi(ui), QString::fromStdString(error)));
        return;
    }

    const auto populateCombo = [&bands](QComboBox *combo) {
        combo->clear();
        for (const SpectralBand &band : bands)
            combo->addItem(CalpackBandCatalog::formatBandLabel(band), band.index);
    };

    const int prevRed =
        ui.redBandCombo->currentIndex() >= 0 ? ui.redBandCombo->currentData().toInt() : -1;
    const int prevGreen =
        ui.greenBandCombo->currentIndex() >= 0 ? ui.greenBandCombo->currentData().toInt() : -1;
    const int prevBlue =
        ui.blueBandCombo->currentIndex() >= 0 ? ui.blueBandCombo->currentData().toInt() : -1;

    populateCombo(ui.redBandCombo);
    populateCombo(ui.greenBandCombo);
    populateCombo(ui.blueBandCombo);
    ui.spectralBands = bands;

    if (prevRed >= 0)
        selectBandComboIndex(ui.redBandCombo, prevRed);
    else
        selectBandComboIndex(ui.redBandCombo,
                             scaledDefaultBandIndex(kDefaultRedBandIndex,
                                                    static_cast<int>(bands.size())));
    if (prevGreen >= 0)
        selectBandComboIndex(ui.greenBandCombo, prevGreen);
    else
        selectBandComboIndex(ui.greenBandCombo,
                             scaledDefaultBandIndex(kDefaultGreenBandIndex,
                                                    static_cast<int>(bands.size())));
    if (prevBlue >= 0)
        selectBandComboIndex(ui.blueBandCombo, prevBlue);
    else
        selectBandComboIndex(ui.blueBandCombo,
                             scaledDefaultBandIndex(kDefaultBlueBandIndex,
                                                    static_cast<int>(bands.size())));

    syncWaterfallBands(ui);
    syncProfileRgbMarkers(ui);

    appendLog(QStringLiteral("%1: loaded %2 spectral bands (binning %3) from %4")
                  .arg(profileTabNameForUi(ui))
                  .arg(bands.size())
                  .arg(spectralBinning)
                  .arg(QFileInfo(calpackPath).fileName()));
}

void MainWindow::onStreamTabChanged(const int index)
{
    QSignalBlocker settingsBlocker(settingsTabs_);
    QSignalBlocker cameraSettingsBlocker(cameraSettingsTabs_);

    if (index == 0 || index == 1)
    {
        if (settingsTabs_ != nullptr && settingsTabs_->currentIndex() != kSettingsTabCamera)
            settingsTabs_->setCurrentIndex(kSettingsTabCamera);

        if (cameraSettingsTabs_ != nullptr && cameraSettingsTabs_->currentIndex() != index)
            cameraSettingsTabs_->setCurrentIndex(index);
        return;
    }

    if (index == 2 && settingsTabs_ != nullptr
        && settingsTabs_->currentIndex() != kSettingsTabUr3e)
        settingsTabs_->setCurrentIndex(kSettingsTabUr3e);
}

QString MainWindow::calibrationPackPath(const LumoCameraUi &ui)
{
    if (ui.calibrationPackEdit == nullptr)
        return {};

    const QVariant stored = ui.calibrationPackEdit->property(kCalibrationPackPathProperty);
    if (stored.isValid())
    {
        const QString path = stored.toString();
        if (!path.isEmpty())
            return path;
    }

    return ui.calibrationPackEdit->text().trimmed();
}

void MainWindow::setCalibrationPackDisplay(QLineEdit *edit, const QString &fullPath)
{
    if (edit == nullptr)
        return;

    const QString cleaned = QDir::cleanPath(fullPath);
    edit->setProperty(kCalibrationPackPathProperty, cleaned);
    edit->setText(QFileInfo(cleaned).fileName());
    edit->setToolTip(cleaned);
}

QString MainWindow::defaultFx10eCalibrationPackPath()
{
    const QString fileName = QString::fromLatin1(kFx10eCalibrationFileName);
    const QString appDir = QCoreApplication::applicationDirPath();

    QStringList candidates = {
        QDir(appDir).filePath(QStringLiteral("calibration/") + fileName),
        QDir(appDir).filePath(QStringLiteral("../calibration/") + fileName),
        QDir(appDir).filePath(QStringLiteral("../../calibration/") + fileName),
        QDir(appDir).filePath(QStringLiteral("../../app/calibration/") + fileName),
        QDir(appDir).filePath(QStringLiteral("../../../app/calibration/") + fileName),
    };

#ifdef HF_APP_SOURCE_DIR
    candidates.prepend(
        QDir(QString::fromUtf8(HF_APP_SOURCE_DIR)).filePath(QStringLiteral("calibration/") + fileName));
#endif

    for (const QString &candidate : candidates)
    {
        if (QFileInfo::exists(candidate))
            return QDir::cleanPath(candidate);
    }

    return {};
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    setWindowTitle("HyperFusion");
    resize(1550, 920);
    setMinimumSize(1180, 760);

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

    camera1Ui_.sensorKind = LumoSensorKind::Fx10ePleora;
    camera1Ui_.camera = std::make_shared<LumoCamera>(CameraBackendId::Camera1,
                                                      QStringLiteral("FX10e").toStdString(),
                                                      LumoSensorKind::Fx10ePleora);
    camera1Ui_.cameraIndex = 0;

    camera2Ui_.sensorKind = LumoSensorKind::Swir3Ni;
    camera2Ui_.camera =
        std::make_shared<Swir3NiCamera>(CameraBackendId::Camera2, QStringLiteral("SWIR3").toStdString());
    camera2Ui_.cameraIndex = 1;

    coordinator_ = std::make_unique<CameraCoordinator>(
        std::vector<std::shared_ptr<ICameraController>>{camera1Ui_.camera, camera2Ui_.camera});

    coordinator_->setLogCallback([this](const std::string &message) {
        const QString line = QString::fromStdString(message);
        QMetaObject::invokeMethod(
            this,
            [this, line]() { appendLog(line); },
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

    coordinator_->setGuiAsyncTaskRunner([](std::function<void()> task) {
        QMetaObject::invokeMethod(qApp, std::move(task), Qt::QueuedConnection);
    });

    coordinator_->setCameraStateCallback(0, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { onCameraStateChanged(camera1Ui_, state); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraStateCallback(1, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { onCameraStateChanged(camera2Ui_, state); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraErrorCallback(0, [this](const CameraError &error) {
        QMetaObject::invokeMethod(
            this,
            [this, error]() { onCameraError(camera1Ui_, error); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraErrorCallback(1, [this](const CameraError &error) {
        QMetaObject::invokeMethod(
            this,
            [this, error]() { onCameraError(camera2Ui_, error); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraShutterStateCallback(0, [this](const bool isOpen) {
        QMetaObject::invokeMethod(
            this,
            [this, isOpen]() { onShutterStateChanged(camera1Ui_, isOpen); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraShutterStateCallback(1, [this](const bool isOpen) {
        QMetaObject::invokeMethod(
            this,
            [this, isOpen]() { onShutterStateChanged(camera2Ui_, isOpen); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraSettingsAppliedCallback(0, [this](const CameraSettingsApplyReport &report) {
        QMetaObject::invokeMethod(
            this,
            [this, report]() { onSettingsApplied(camera1Ui_, report); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraSettingsAppliedCallback(1, [this](const CameraSettingsApplyReport &report) {
        QMetaObject::invokeMethod(
            this,
            [this, report]() { onSettingsApplied(camera2Ui_, report); },
            Qt::QueuedConnection);
    });

    setupWaterfallProcessors();
    setupProfileProcessors();

    coordinator_->setFrameCallback([this](const FramePacket &frame) {
        QMetaObject::invokeMethod(
            this,
            [this, frame]() { onStreamFrame(frame); },
            Qt::QueuedConnection);
    });

    coordinator_->start();
    appendLog("HyperFusion UI initialized; camera coordinator started.");

    QTimer::singleShot(0, this, [this]() {
        refreshLumoDeviceLists();
        refreshBandCombos(camera1Ui_);
        refreshBandCombos(camera2Ui_);
    });

    setupStageWorker();
}

MainWindow::~MainWindow()
{
    if (waterfallProcessor1_)
        waterfallProcessor1_->stop();
    if (waterfallProcessor2_)
        waterfallProcessor2_->stop();
    if (profileProcessor1_)
        profileProcessor1_->stop();
    if (profileProcessor2_)
        profileProcessor2_->stop();

    if (stageWorker_)
        stageWorker_->stop();

    if (!coordinator_)
        return;

    coordinator_->shutdownSync();
}

QWidget *MainWindow::createStreamTabsPanel()
{
    auto *panel = new QWidget(this);
    auto *layout = new QVBoxLayout(panel);

    streamTabs_ = new QTabWidget(panel);
    streamTabs_->addTab(createStreamTabPage(profileTabNameForUi(camera1Ui_), camera1Ui_),
                        profileTabNameForUi(camera1Ui_));
    streamTabs_->addTab(createStreamTabPage(profileTabNameForUi(camera2Ui_), camera2Ui_),
                        profileTabNameForUi(camera2Ui_));
    streamTabs_->addTab(createRgbUr3eStreamTab(), QStringLiteral("UR3e"));

    connect(streamTabs_, &QTabWidget::currentChanged, this, &MainWindow::onStreamTabChanged);

    layout->addWidget(streamTabs_, 1);
    return panel;
}

QWidget *MainWindow::createStreamTabPage(const QString &cameraName, LumoCameraUi &cameraUi)
{
    auto *tab = new QWidget(this);
    auto *grid = new QGridLayout(tab);
    grid->setContentsMargins(8, 8, 8, 8);
    grid->setSpacing(10);

    QLabel *waterfallLabel = nullptr;

    cameraUi.detectorView = new ui::DetectorCrosshairWidget(tab);
    cameraUi.wavelengthView =
        new ui::ProfilePlotWidget(ui::ProfilePlotWidget::Mode::Wavelength, tab);
    cameraUi.pixelStreamView = new ui::ProfilePlotWidget(ui::ProfilePlotWidget::Mode::Spatial, tab);

    auto *detectorPane = createStreamPane(QStringLiteral("Detector"), cameraUi.detectorView);
    auto *waterfallPane = createPreviewPane(QStringLiteral("Waterfall"), waterfallLabel);
    auto *wavelengthPane = createStreamPane(QStringLiteral("Wavelength"), cameraUi.wavelengthView);
    auto *pixelStreamPane = createStreamPane(QStringLiteral("Pixel"), cameraUi.pixelStreamView);

    cameraUi.detectorPane = detectorPane;
    cameraUi.waterfallPane = waterfallPane;
    cameraUi.wavelengthPane = wavelengthPane;
    cameraUi.pixelStreamPane = pixelStreamPane;
    cameraUi.waterfallView = waterfallLabel;

    connect(cameraUi.detectorView,
            &ui::DetectorCrosshairWidget::linesChanged,
            this,
            [this, &cameraUi](const int spatialIndex, const int bandIndex) {
                onProfileLinesChanged(cameraUi, spatialIndex, bandIndex);
            });
    if (cameraUi.waterfallView != nullptr)
    {
        cameraUi.waterfallView->setScaledContents(true);
        cameraUi.waterfallView->setAlignment(Qt::AlignCenter);
    }
    updateStreamPaneTitles(cameraUi);

    const QString detectorMsg = cameraName + QStringLiteral(" detector (disconnected)");
    const QString wavelengthMsg = cameraName + QStringLiteral(" wavelength (disconnected)");
    const QString pixelMsg = cameraName + QStringLiteral(" pixel (disconnected)");
    cameraUi.detectorView->clearDisplay(detectorMsg);
    cameraUi.wavelengthView->clearDisplay(wavelengthMsg);
    cameraUi.pixelStreamView->clearDisplay(pixelMsg);
    setPreviewDisconnectedText(cameraUi.waterfallView, QStringLiteral("waterfall"), cameraName);
    syncProfileRgbMarkers(cameraUi);

    grid->addWidget(detectorPane, 0, 0);
    grid->addWidget(waterfallPane, 0, 1);
    grid->addWidget(wavelengthPane, 1, 0);
    grid->addWidget(pixelStreamPane, 1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);

    return tab;
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

    auto *rgbPane = createPreviewPane("RGB", rgbLabel);
    auto *posePane = createPreviewPane("Robot / pose", poseLabel);

    rgbLabel->setText("UR3e RGB preview (multi-angle capture) — disconnected");
    poseLabel->setText("UR3e pose / path preview (TODO) — disconnected");

    grid->addWidget(rgbPane, 0, 0);
    grid->addWidget(posePane, 0, 1);
    grid->setRowStretch(0, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    layout->addWidget(gridHost, 1);
    return tab;
}

QGroupBox *MainWindow::createStreamPane(const QString &title, QWidget *contentWidget)
{
    auto *box = new QGroupBox(title, this);
    auto *layout = new QVBoxLayout(box);

    auto *placeholder = new QFrame(box);
    placeholder->setFrameShape(QFrame::StyledPanel);
    placeholder->setMinimumSize(320, 200);
    placeholder->setStyleSheet(QStringLiteral("background-color: #111111;"));

    auto *placeholderLayout = new QVBoxLayout(placeholder);
    placeholderLayout->setContentsMargins(6, 6, 6, 6);
    placeholderLayout->addWidget(contentWidget, 1);

    layout->addWidget(placeholder, 1);
    return box;
}

QGroupBox *MainWindow::createPreviewPane(const QString &title, QLabel *&labelOut)
{
    auto *box = new QGroupBox(title, this);
    auto *layout = new QVBoxLayout(box);

    auto *placeholder = new QFrame(box);
    placeholder->setFrameShape(QFrame::StyledPanel);
    placeholder->setMinimumSize(320, 200);
    placeholder->setStyleSheet("background-color: #111111;");

    labelOut = new QLabel("No stream", placeholder);
    labelOut->setAlignment(Qt::AlignCenter);
    labelOut->setStyleSheet("color: #7ec8ff; background-color: transparent;");
    labelOut->setWordWrap(true);

    auto *placeholderLayout = new QVBoxLayout(placeholder);
    placeholderLayout->setContentsMargins(6, 6, 6, 6);
    placeholderLayout->addWidget(labelOut, 1);

    layout->addWidget(placeholder, 1);
    return box;
}

QWidget *MainWindow::createSettingsPanel()
{
    auto *panel = new QWidget(this);
    panel->setMinimumWidth(380);
    panel->setMaximumWidth(520);
    auto *layout = new QVBoxLayout(panel);

    settingsTabs_ = new QTabWidget(panel);
    settingsTabs_->addTab(createCameraSettingsTab(), QStringLiteral("Camera"));
    settingsTabs_->addTab(createStageSettingsTab(), QStringLiteral("Stage"));
    settingsTabs_->addTab(createLightSettingsTab(), QStringLiteral("Light"));
    settingsTabs_->addTab(createUr3eSettingsTab(), QStringLiteral("UR3e"));
    settingsTabs_->addTab(createCaptureSettingsTab(), QStringLiteral("Capture"));

    connect(settingsTabs_,
            &QTabWidget::currentChanged,
            this,
            &MainWindow::onSettingsTabChanged);

    layout->addWidget(settingsTabs_, 1);
    return panel;
}

QWidget *MainWindow::createCameraSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);
    layout->setContentsMargins(0, 0, 0, 0);

    cameraSettingsTabs_ = new QTabWidget(page);
    cameraSettingsTabs_->addTab(createLumoCameraGroup(cameraSettingsTabs_, camera1Ui_, LumoSensorKind::Fx10ePleora),
                                defaultCameraTabName(camera1Ui_.cameraIndex));
    cameraSettingsTabs_->addTab(createLumoCameraGroup(cameraSettingsTabs_, camera2Ui_, LumoSensorKind::Swir3Ni),
                                defaultCameraTabName(camera2Ui_.cameraIndex));

    connect(cameraSettingsTabs_,
            &QTabWidget::currentChanged,
            this,
            &MainWindow::onCameraSettingsTabChanged);

    layout->addWidget(cameraSettingsTabs_, 1);
    return page;
}

QWidget *MainWindow::createLumoCameraGroup(QWidget *parent,
                                           LumoCameraUi &ui,
                                           const LumoSensorKind sensorKind)
{
    ui.sensorKind = sensorKind;
    const bool fx10eDefaults = sensorKind == LumoSensorKind::Fx10ePleora;

    auto *page = new QWidget(parent);
    auto *form = new QFormLayout(page);
    form->setContentsMargins(8, 8, 8, 8);

    ui.deviceCombo = new QComboBox(page);
    ui.deviceCombo->setMinimumWidth(260);

    ui.calibrationPackEdit = new QLineEdit(page);
    ui.calibrationPackEdit->setMinimumWidth(0);
    ui.calibrationPackEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    ui.calibrationPackBrowseBtn = new QPushButton(QStringLiteral("Browse…"), page);
    ui.calibrationPackBrowseBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    ui.calibrationPackBrowseBtn->setFixedWidth(72);

    if (fx10eDefaults)
        setCalibrationPackDisplay(ui.calibrationPackEdit, defaultFx10eCalibrationPackPath());

    auto *calibrationRow = new QWidget(page);
    auto *calibrationLayout = new QHBoxLayout(calibrationRow);
    calibrationLayout->setContentsMargins(0, 0, 0, 0);
    calibrationLayout->setSpacing(6);
    calibrationLayout->addWidget(ui.calibrationPackEdit, 1);
    calibrationLayout->addWidget(ui.calibrationPackBrowseBtn, 0);

    auto *shutterRow = new QWidget(page);
    auto *shutterLayout = new QHBoxLayout(shutterRow);
    shutterLayout->setContentsMargins(0, 0, 0, 0);
    shutterLayout->setSpacing(8);

    ui.shutterIndicator = new QLabel(shutterRow);
    ui.shutterIndicator->setFixedSize(14, 14);
    ui.shutterStatusLabel = new QLabel(QStringLiteral("—"), shutterRow);
    ui.shutterToggleBtn = new QPushButton(QStringLiteral("Toggle"), shutterRow);
    ui.shutterToggleBtn->setEnabled(false);
    shutterLayout->addWidget(ui.shutterIndicator);
    shutterLayout->addWidget(ui.shutterStatusLabel, 1);
    shutterLayout->addWidget(ui.shutterToggleBtn);

    ui.frameRateSpin = new QDoubleSpinBox(page);
    ui.frameRateSpin->setRange(1.0, 500.0);
    ui.frameRateSpin->setDecimals(2);
    ui.frameRateSpin->setValue(50.0);
    ui.frameRateSpin->setSuffix(QStringLiteral(" Hz"));

    ui.exposureSpin = new QDoubleSpinBox(page);
    ui.exposureSpin->setRange(0.01, 400.0);
    ui.exposureSpin->setDecimals(2);
    ui.exposureSpin->setSingleStep(0.1);
    ui.exposureSpin->setValue(18.0);
    ui.exposureSpin->setSuffix(QStringLiteral(" ms"));

    ui.spectralBinningCombo = new QComboBox(page);
    ui.spatialBinningCombo = new QComboBox(page);
    for (QComboBox *binningCombo : {ui.spectralBinningCombo, ui.spatialBinningCombo})
    {
        binningCombo->addItems({QStringLiteral("1"),
                                QStringLiteral("2"),
                                QStringLiteral("4"),
                                QStringLiteral("8")});
    }
    if (ui.spectralBinningCombo != nullptr)
    {
        connect(ui.spectralBinningCombo,
                &QComboBox::currentIndexChanged,
                this,
                [this, &ui](const int) { refreshBandCombos(ui); });
    }

    ui.triggerCombo = new QComboBox(page);
    ui.triggerCombo->addItems({"Internal", "External"});

    ui.redBandCombo = new QComboBox(page);
    ui.greenBandCombo = new QComboBox(page);
    ui.blueBandCombo = new QComboBox(page);
    for (QComboBox *combo : {ui.redBandCombo, ui.greenBandCombo, ui.blueBandCombo})
        combo->setMinimumWidth(220);

    ui.connectBtn = new QPushButton("Connect camera", page);
    ui.applyBtn = new QPushButton("Apply settings", page);

    ui.applyBtn->setEnabled(false);

    form->addRow("Sensor profile", ui.deviceCombo);
    form->addRow("Calibration pack", calibrationRow);
    form->addRow("Shutter", shutterRow);
    form->addRow("Frame rate (Hz)", ui.frameRateSpin);
    form->addRow("Exposure time (ms)", ui.exposureSpin);
    form->addRow("Spectral binning", ui.spectralBinningCombo);
    form->addRow("Spatial binning", ui.spatialBinningCombo);
    form->addRow("Trigger mode", ui.triggerCombo);
    form->addRow("Red band", ui.redBandCombo);
    form->addRow("Green band", ui.greenBandCombo);
    form->addRow("Blue band", ui.blueBandCombo);
    form->addRow("", ui.connectBtn);
    form->addRow("", ui.applyBtn);

    connect(ui.calibrationPackBrowseBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (ui.calibrationPackEdit == nullptr)
            return;

        const QString currentPath = calibrationPackPath(ui);
        const QString startDir =
            currentPath.isEmpty() ? QFileInfo(defaultFx10eCalibrationPackPath()).absolutePath()
                                : QFileInfo(currentPath).absolutePath();
        const QString path = QFileDialog::getOpenFileName(
            this,
            QStringLiteral("Select calibration pack"),
            startDir,
            QStringLiteral("Specim calibration (*.scp);;All files (*.*)"));
        if (!path.isEmpty())
        {
            setCalibrationPackDisplay(ui.calibrationPackEdit, path);
            refreshBandCombos(ui);
        }
    });

    connect(ui.deviceCombo,
            &QComboBox::currentIndexChanged,
            this,
            [this, &ui](const int) { updateCameraTabLabel(ui); });

    connect(ui.connectBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (!coordinator_ || !ui.camera)
            return;

        const QString cameraLabel = profileTabNameForUi(ui);
        const bool disconnectRequested =
            ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;

        if (disconnectRequested)
        {
            ui.connectAttemptActive = false;
            if (ui.connectBtn != nullptr)
            {
                ui.connectBtn->setEnabled(false);
                ui.connectBtn->setText(QStringLiteral("Disconnecting…"));
            }
            coordinator_->disconnectOnGuiThread(ui.cameraIndex);
            appendLog(QString("%1: disconnect requested.").arg(cameraLabel));
            return;
        }

        if (ui.deviceCombo->count() == 0)
        {
            appendLog(QString("%1: refresh SSP profiles before connecting.").arg(cameraLabel));
            return;
        }

        const CameraSettings connectionSettings = buildCameraSettings(ui);
        ui.camera->prepareConnection(connectionSettings);
        ui.connectAttemptActive = true;
        const QString grabberNote = ui.sensorKind == LumoSensorKind::Swir3Ni
                                        ? QStringLiteral("NI frame grabber — configure in NI MAX if prompted")
                                        : QStringLiteral("Pleora eBUS picker may appear");
        appendLog(QString("%1: connect camera — profile %2 (%3; not ready until Initialized).")
                      .arg(cameraLabel, ui.deviceCombo->currentText(), grabberNote));

        coordinator_->connectAndInitializeOnGuiThread(ui.cameraIndex);
    });

    connect(ui.shutterToggleBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (!coordinator_)
            return;

        if (ui.shutterReportedOpen)
            coordinator_->closeShutter(ui.cameraIndex);
        else
            coordinator_->openShutter(ui.cameraIndex);
    });

    connect(ui.applyBtn, &QPushButton::clicked, this, [this, &ui]() {
        if (!coordinator_)
            return;

        const QString cameraLabel = profileTabNameForUi(ui);
        const CameraSettings settings = buildCameraSettings(ui);
        coordinator_->applySettings(ui.cameraIndex, settings);
        syncWaterfallBands(ui);
        syncProfileRgbMarkers(ui);
        if (ui::WaterfallProcessor *processor = waterfallProcessorFor(ui))
            processor->reset();

        appendLog(QString("%1: apply settings (fps=%2 Hz, exposure=%3 ms, spectral=%4, spatial=%5, "
                          "RGB=%6/%7/%8, trigger=%9)")
                      .arg(cameraLabel)
                      .arg(settings.frameRateHz, 0, 'f', 2)
                      .arg(settings.exposureMs, 0, 'f', 2)
                      .arg(settings.spectralBinning)
                      .arg(settings.spatialBinning)
                      .arg(settings.redBandIndex)
                      .arg(settings.greenBandIndex)
                      .arg(settings.blueBandIndex)
                      .arg(ui.triggerCombo->currentText()));
    });

    const auto onBandSelectionChanged = [this, &ui]() {
        syncWaterfallBands(ui);
        syncProfileRgbMarkers(ui);
        if (ui::WaterfallProcessor *processor = waterfallProcessorFor(ui))
            processor->reset();
        if (ui::ProfileProcessor *profileProcessor = profileProcessorFor(ui))
            profileProcessor->requestRefresh();
    };
    if (ui.redBandCombo != nullptr)
        connect(ui.redBandCombo, &QComboBox::currentIndexChanged, this, onBandSelectionChanged);
    if (ui.greenBandCombo != nullptr)
        connect(ui.greenBandCombo, &QComboBox::currentIndexChanged, this, onBandSelectionChanged);
    if (ui.blueBandCombo != nullptr)
        connect(ui.blueBandCombo, &QComboBox::currentIndexChanged, this, onBandSelectionChanged);

    updateShutterDisplay(ui, false);
    if (!calibrationPackPath(ui).isEmpty())
        refreshBandCombos(ui);

    return page;
}

CameraSettings MainWindow::buildCameraSettings(const LumoCameraUi &ui) const
{
    CameraSettings settings;
    settings.frameRateHz = ui.frameRateSpin->value();
    settings.exposureMs = ui.exposureSpin->value();
    if (ui.spectralBinningCombo != nullptr)
        settings.spectralBinning = ui.spectralBinningCombo->currentText().toInt();
    if (ui.spatialBinningCombo != nullptr)
        settings.spatialBinning = ui.spatialBinningCombo->currentText().toInt();
    settings.externalTrigger = ui.triggerCombo->currentText() == QLatin1String("External");
    settings.acquisitionTimeoutMs = 5000;
    settings.deviceIndex = ui.deviceCombo->currentData().toInt();
    if (ui.deviceCombo != nullptr)
        settings.profileName = ui.deviceCombo->currentText().toStdString();
    settings.lumoCalibrationPackPath = calibrationPackPath(ui).toStdString();
    const auto bandIndexFromCombo = [](const QComboBox *combo) -> int {
        if (combo == nullptr || combo->count() == 0 || combo->currentIndex() < 0)
            return -1;
        const QVariant data = combo->currentData();
        if (!data.isValid())
            return -1;
        return data.toInt();
    };

    const int red = bandIndexFromCombo(ui.redBandCombo);
    const int green = bandIndexFromCombo(ui.greenBandCombo);
    const int blue = bandIndexFromCombo(ui.blueBandCombo);
    if (red >= 0)
        settings.redBandIndex = red;
    if (green >= 0)
        settings.greenBandIndex = green;
    if (blue >= 0)
        settings.blueBandIndex = blue;
    return settings;
}

void MainWindow::refreshStageComPortList()
{
    if (stagePortCombo_ == nullptr)
        return;

    QString previousPort = stagePortCombo_->currentData().toString();
    if (previousPort.isEmpty())
        previousPort = stagePortCombo_->currentText();

    QSignalBlocker blocker(stagePortCombo_);
    stagePortCombo_->clear();

    const QStringList ports = ui::enumerateSerialPortNames();
    const QString preferredPort = QStringLiteral("COM4");
    int selectIndex = -1;

    for (const QString &portName : ports)
    {
        stagePortCombo_->addItem(portName, portName);

        if (portName.compare(previousPort, Qt::CaseInsensitive) == 0)
            selectIndex = stagePortCombo_->count() - 1;
        else if (selectIndex < 0 && portName.compare(preferredPort, Qt::CaseInsensitive) == 0)
            selectIndex = stagePortCombo_->count() - 1;
    }

    if (stagePortCombo_->count() == 0)
    {
        stagePortCombo_->addItem(QStringLiteral("(no serial ports found)"), QString());
        stagePortCombo_->setEnabled(false);
        return;
    }

    stagePortCombo_->setEnabled(true);
    if (selectIndex >= 0)
        stagePortCombo_->setCurrentIndex(selectIndex);
    else
        stagePortCombo_->setCurrentIndex(0);
}

QString MainWindow::selectedStagePortName() const
{
    if (stagePortCombo_ == nullptr)
        return QStringLiteral("COM4");

    const QString portName = stagePortCombo_->currentData().toString();
    return portName.isEmpty() ? stagePortCombo_->currentText() : portName;
}

void MainWindow::setupStageWorker()
{
    auto controller = std::make_shared<ZaberStageController>();
    stageWorker_ = std::make_unique<StageWorker>(controller);
    stageWorker_->setStateCallback([this](const StageState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { onStageStateChanged(state); },
            Qt::QueuedConnection);
    });
    stageWorker_->setTopologyCallback([this](const StageTopology &topology) {
        QMetaObject::invokeMethod(
            this,
            [this, topology]() { onStageTopologyChanged(topology); },
            Qt::QueuedConnection);
    });
    stageWorker_->setErrorCallback([this](const StageError &error) {
        QMetaObject::invokeMethod(
            this,
            [this, error]() { onStageError(error); },
            Qt::QueuedConnection);
    });
    stageWorker_->start();
}

void MainWindow::updateStageConnectionControls(const StageState state)
{
    const bool connected = state == StageState::Connected;
    const bool busy = state == StageState::Connecting;

    if (stagePortCombo_ != nullptr)
        stagePortCombo_->setEnabled(!connected && !busy);
    if (stageBaudCombo_ != nullptr)
        stageBaudCombo_->setEnabled(!connected && !busy);
    if (stageConnectBtn_ != nullptr)
        stageConnectBtn_->setEnabled(!connected && !busy);
    if (stageDisconnectBtn_ != nullptr)
        stageDisconnectBtn_->setEnabled(connected);
}

void MainWindow::updateStageDeviceDisplay(const StageTopology &topology)
{
    if (stageDeviceDisplay_ == nullptr)
        return;

    stageDeviceDisplay_->setPlainText(formatStageTopology(topology));
}

void MainWindow::clearStageDeviceDisplay()
{
    updateStageDeviceDisplay({});
}

void MainWindow::onStageStateChanged(const StageState state)
{
    updateStageConnectionControls(state);

    if (state == StageState::Disconnected || state == StageState::Fault)
        clearStageDeviceDisplay();
}

void MainWindow::onStageTopologyChanged(const StageTopology &topology)
{
    updateStageDeviceDisplay(topology);

    if (topology.devices.empty())
        return;

    appendLog(QString("Stage: detected %1 device(s) on %2")
                  .arg(topology.devices.size())
                  .arg(QString::fromStdString(topology.portName)));

    for (const StageDeviceInfo &device : topology.devices)
    {
        appendLog(QString("Stage: device %1 = %2 (%3 axis(es), FW %4)")
                      .arg(device.deviceAddress)
                      .arg(QString::fromStdString(device.name))
                      .arg(device.axisCount)
                      .arg(QString::fromStdString(device.firmwareVersion)));
    }

    if (topology.lockstepEnabled)
    {
        appendLog(QString("Stage: lockstep group %1 enabled (primary axis %2, secondary axis %3, travel %4 mm)")
                      .arg(topology.lockstepGroupId)
                      .arg(topology.lockstepPrimaryAxis)
                      .arg(topology.lockstepSecondaryAxis)
                      .arg(topology.travelLengthMm, 0, 'f', 0));
    }
}

void MainWindow::onStageError(const StageError &error)
{
    if (error.message.empty())
        return;

    appendLog(QString("Stage error: %1").arg(QString::fromStdString(error.message)));
}

QWidget *MainWindow::createStageSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *connBox = new QGroupBox("Connection", page);
    auto *connForm = new QFormLayout(connBox);
    stagePortCombo_ = new QComboBox(connBox);
    stagePortCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    stagePortCombo_->setMinimumContentsLength(18);

    auto *refreshPortsBtn = new QPushButton("Refresh", connBox);
    refreshPortsBtn->setToolTip("Rescan available COM ports");
    auto *portRow = new QWidget(connBox);
    auto *portRowLayout = new QHBoxLayout(portRow);
    portRowLayout->setContentsMargins(0, 0, 0, 0);
    portRowLayout->addWidget(stagePortCombo_, 1);
    portRowLayout->addWidget(refreshPortsBtn);

    stageBaudCombo_ = new QComboBox(connBox);
    stageBaudCombo_->addItems({"9600", "19200", "38400", "57600", "115200"});
    stageBaudCombo_->setCurrentText("115200");
    stageConnectBtn_ = new QPushButton("Connect", connBox);
    stageDisconnectBtn_ = new QPushButton("Disconnect", connBox);
    stageDisconnectBtn_->setEnabled(false);
    connForm->addRow("Port", portRow);
    connForm->addRow("Baud", stageBaudCombo_);
    connForm->addRow("", stageConnectBtn_);
    connForm->addRow("", stageDisconnectBtn_);

    refreshStageComPortList();

    connect(refreshPortsBtn, &QPushButton::clicked, this, &MainWindow::refreshStageComPortList);
    connect(stageConnectBtn_, &QPushButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;

        const QString portName = selectedStagePortName();
        if (portName.isEmpty() || portName.startsWith('('))
        {
            appendLog("Stage: connect requested but no COM port is selected");
            return;
        }

        StageConnectSettings settings;
        settings.portName = portName.toStdString();
        settings.baudRate = stageBaudCombo_->currentText().toInt();

        appendLog(QString("Stage: connecting to %1 @ %2...")
                      .arg(portName, stageBaudCombo_->currentText()));
        stageWorker_->requestConnect(settings);
    });
    connect(stageDisconnectBtn_, &QPushButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;

        appendLog("Stage: disconnect requested");
        stageWorker_->requestDisconnect();
    });

    auto *deviceBox = new QGroupBox("Device", page);
    deviceBox->setCheckable(true);
    deviceBox->setChecked(false);
    auto *deviceLayout = new QVBoxLayout(deviceBox);
    stageDeviceDisplay_ = new QPlainTextEdit(deviceBox);
    stageDeviceDisplay_->setReadOnly(true);
    stageDeviceDisplay_->setPlaceholderText("Connect to discover the controller and axes.");
    stageDeviceDisplay_->setMinimumHeight(180);
    stageDeviceDisplay_->setMaximumHeight(320);
    deviceLayout->addWidget(stageDeviceDisplay_);

    layout->addWidget(connBox);
    layout->addWidget(deviceBox);
    layout->addStretch();
    return page;
}

QWidget *MainWindow::createLightSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *connBox = new QGroupBox("Lighthouse / lighting", page);
    auto *connForm = new QFormLayout(connBox);
    auto *interfaceCombo = new QComboBox(connBox);
    interfaceCombo->addItems({"Serial", "USB", "Network (TODO)"});
    auto *addressEdit = new QLineEdit(connBox);
    addressEdit->setPlaceholderText("COM port, IP:port, or device id");
    auto *connectBtn = new QPushButton("Connect", connBox);
    connForm->addRow("Interface", interfaceCombo);
    connForm->addRow("Address", addressEdit);
    connForm->addRow("", connectBtn);

    connect(connectBtn, &QPushButton::clicked, this, [this, interfaceCombo, addressEdit]() {
        appendLog(QString("Light: connect requested (%1, %2)")
                      .arg(interfaceCombo->currentText(), addressEdit->text()));
    });

    auto *ctrlBox = new QGroupBox("Output", page);
    auto *ctrlForm = new QFormLayout(ctrlBox);
    auto *enableCheck = new QCheckBox("Enable output", ctrlBox);
    auto *brightness = new QSlider(Qt::Horizontal, ctrlBox);
    brightness->setRange(0, 100);
    brightness->setValue(50);
    auto *brightnessLabel = new QLabel("50%", ctrlBox);
    auto *warmth = new QSlider(Qt::Horizontal, ctrlBox);
    warmth->setRange(0, 100);
    warmth->setValue(50);
    auto *warmthLabel = new QLabel("50%", ctrlBox);
    auto *applyBtn = new QPushButton("Apply", ctrlBox);

    QObject::connect(brightness, &QSlider::valueChanged, this, [brightnessLabel](int v) {
        brightnessLabel->setText(QString("%1%").arg(v));
    });
    QObject::connect(warmth, &QSlider::valueChanged, this, [warmthLabel](int v) {
        warmthLabel->setText(QString("%1%").arg(v));
    });

    ctrlForm->addRow(enableCheck);
    ctrlForm->addRow("Brightness", brightness);
    ctrlForm->addRow("", brightnessLabel);
    ctrlForm->addRow("Warmth", warmth);
    ctrlForm->addRow("", warmthLabel);
    ctrlForm->addRow("", applyBtn);

    connect(applyBtn, &QPushButton::clicked, this, [this, enableCheck, brightness, warmth]() {
        appendLog(QString("Light: apply (enabled=%1, brightness=%2%, warmth=%3%)")
                      .arg(enableCheck->isChecked() ? "yes" : "no")
                      .arg(brightness->value())
                      .arg(warmth->value()));
    });

    layout->addWidget(connBox);
    layout->addWidget(ctrlBox);
    layout->addStretch();
    return page;
}

QWidget *MainWindow::createUr3eSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *connBox = new QGroupBox("UR3e connection", page);
    auto *connForm = new QFormLayout(connBox);
    auto *robotIp = new QLineEdit(connBox);
    robotIp->setPlaceholderText("Robot controller IP / hostname");
    robotIp->setText("192.168.0.10");
    auto *dashPort = new QSpinBox(connBox);
    dashPort->setRange(1, 65535);
    dashPort->setValue(29999);
    auto *rtdePort = new QSpinBox(connBox);
    rtdePort->setRange(1, 65535);
    rtdePort->setValue(30004);
    auto *connectBtn = new QPushButton("Connect", connBox);
    auto *disconnectBtn = new QPushButton("Disconnect", connBox);
    disconnectBtn->setEnabled(false);
    connForm->addRow("Controller", robotIp);
    connForm->addRow("Dashboard port", dashPort);
    connForm->addRow("RTDE port", rtdePort);
    connForm->addRow("", connectBtn);
    connForm->addRow("", disconnectBtn);

    connect(connectBtn, &QPushButton::clicked, this,
            [this, robotIp, dashPort, rtdePort, disconnectBtn]() {
                appendLog(QString("UR3e: connect requested (%1, dashboard=%2, rtde=%3)")
                              .arg(robotIp->text())
                              .arg(dashPort->value())
                              .arg(rtdePort->value()));
                disconnectBtn->setEnabled(true);
            });
    connect(disconnectBtn, &QPushButton::clicked, this, [this, disconnectBtn]() {
        appendLog("UR3e: disconnect requested (not implemented)");
        disconnectBtn->setEnabled(false);
    });

    auto *tcpBox = new QGroupBox("TCP pose (placeholder)", page);
    auto *tcpForm = new QFormLayout(tcpBox);
    auto *tcpX = new QDoubleSpinBox(tcpBox);
    auto *tcpY = new QDoubleSpinBox(tcpBox);
    auto *tcpZ = new QDoubleSpinBox(tcpBox);
    auto *tcpRx = new QDoubleSpinBox(tcpBox);
    auto *tcpRy = new QDoubleSpinBox(tcpBox);
    auto *tcpRz = new QDoubleSpinBox(tcpBox);
    for (auto *s : {tcpX, tcpY, tcpZ, tcpRx, tcpRy, tcpRz}) {
        s->setRange(-10000.0, 10000.0);
        s->setDecimals(3);
    }
    tcpX->setSuffix(" m");
    tcpY->setSuffix(" m");
    tcpZ->setSuffix(" m");
    tcpRx->setSuffix(" rad");
    tcpRy->setSuffix(" rad");
    tcpRz->setSuffix(" rad");
    auto *moveTcpBtn = new QPushButton("MoveL (TODO)", tcpBox);
    auto *stopjBtn = new QPushButton("StopJ / stopL (TODO)", tcpBox);
    tcpForm->addRow("TCP X", tcpX);
    tcpForm->addRow("TCP Y", tcpY);
    tcpForm->addRow("TCP Z", tcpZ);
    tcpForm->addRow("TCP RX", tcpRx);
    tcpForm->addRow("TCP RY", tcpRy);
    tcpForm->addRow("TCP RZ", tcpRz);
    tcpForm->addRow("", moveTcpBtn);
    tcpForm->addRow("", stopjBtn);

    connect(moveTcpBtn, &QPushButton::clicked, this, [this, tcpX, tcpY, tcpZ, tcpRx, tcpRy, tcpRz]() {
        appendLog(QString("UR3e: MoveL requested (TCP pose %1,%2,%3 / %4,%5,%6) — not implemented")
                      .arg(tcpX->value(), 0, 'f', 3)
                      .arg(tcpY->value(), 0, 'f', 3)
                      .arg(tcpZ->value(), 0, 'f', 3)
                      .arg(tcpRx->value(), 0, 'f', 3)
                      .arg(tcpRy->value(), 0, 'f', 3)
                      .arg(tcpRz->value(), 0, 'f', 3));
    });
    connect(stopjBtn, &QPushButton::clicked, this, [this]() {
        appendLog("UR3e: stop motion requested (not implemented)");
    });

    auto *rgbBox = new QGroupBox("Multi-angle RGB capture (UR3e)", page);
    auto *rgbForm = new QFormLayout(rgbBox);
    auto *poseCount = new QSpinBox(rgbBox);
    poseCount->setRange(1, 64);
    poseCount->setValue(6);
    auto *dwellMs = new QSpinBox(rgbBox);
    dwellMs->setRange(0, 600000);
    dwellMs->setValue(250);
    dwellMs->setSuffix(" ms");
    auto *armRgbBtn = new QPushButton("Arm RGB pose set", rgbBox);
    auto *runRgbBtn = new QPushButton("Run pose + capture sequence", rgbBox);
    runRgbBtn->setEnabled(false);
    rgbForm->addRow("Poses", poseCount);
    rgbForm->addRow("Settle time", dwellMs);
    rgbForm->addRow("", armRgbBtn);
    rgbForm->addRow("", runRgbBtn);

    connect(armRgbBtn, &QPushButton::clicked, this, [this, poseCount, dwellMs, runRgbBtn]() {
        appendLog(QString("UR3e: arm RGB pose set (poses=%1, dwell=%2ms) — not implemented")
                      .arg(poseCount->value())
                      .arg(dwellMs->value()));
        runRgbBtn->setEnabled(true);
    });
    connect(runRgbBtn, &QPushButton::clicked, this, [this]() {
        appendLog("UR3e: run RGB capture sequence requested (not implemented)");
    });

    layout->addWidget(connBox);
    layout->addWidget(tcpBox);
    layout->addWidget(rgbBox);
    layout->addStretch();
    return page;
}

QWidget *MainWindow::createCaptureSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *scanBox = new QGroupBox("Scan / capture", page);
    auto *scanForm = new QFormLayout(scanBox);
    auto *datasetName = new QLineEdit(scanBox);
    datasetName->setPlaceholderText("Dataset name");
    auto *saveFolder = new QLineEdit(scanBox);
    saveFolder->setPlaceholderText("Output folder");
    auto *scanSpeed = new QDoubleSpinBox(scanBox);
    scanSpeed->setRange(0.1, 5000.0);
    scanSpeed->setDecimals(1);
    scanSpeed->setValue(25.0);
    scanSpeed->setSuffix(" mm/min");
    auto *linesSpin = new QSpinBox(scanBox);
    linesSpin->setRange(1, 1000000);
    linesSpin->setValue(1000);
    auto *armScanBtn = new QPushButton("Arm scan", scanBox);
    auto *startScanBtn = new QPushButton("Start scan", scanBox);
    auto *stopScanBtn = new QPushButton("Stop scan", scanBox);
    startScanBtn->setEnabled(false);
    stopScanBtn->setEnabled(false);
    scanForm->addRow("Dataset", datasetName);
    scanForm->addRow("Save folder", saveFolder);
    scanForm->addRow("Scan speed", scanSpeed);
    scanForm->addRow("Lines / frames", linesSpin);
    scanForm->addRow("", armScanBtn);
    scanForm->addRow("", startScanBtn);
    scanForm->addRow("", stopScanBtn);

    connect(armScanBtn, &QPushButton::clicked, this,
            [this, datasetName, saveFolder, scanSpeed, linesSpin, startScanBtn, stopScanBtn]() {
                appendLog(QString("Capture: arm scan (dataset=%1, folder=%2, speed=%3 mm/min, lines=%4)")
                              .arg(datasetName->text(), saveFolder->text())
                              .arg(scanSpeed->value(), 0, 'f', 1)
                              .arg(linesSpin->value()));
                // UI-only: allow starting after arm for now.
                startScanBtn->setEnabled(true);
                stopScanBtn->setEnabled(true);
            });
    connect(startScanBtn, &QPushButton::clicked, this, [this]() {
        appendLog("Capture: start scan requested (not implemented)");
    });
    connect(stopScanBtn, &QPushButton::clicked, this, [this, startScanBtn, stopScanBtn]() {
        appendLog("Capture: stop scan requested (not implemented)");
        startScanBtn->setEnabled(false);
        stopScanBtn->setEnabled(false);
    });

    layout->addWidget(scanBox);
    layout->addStretch();
    return page;
}

void MainWindow::appendLog(const QString &message)
{
    if (logOutput_ == nullptr)
        return;

    const QString ts = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    logOutput_->appendPlainText(QString("[%1] %2").arg(ts, message));
}

void MainWindow::refreshLumoDeviceLists()
{
    if (camera1Ui_.camera == nullptr)
        return;

    const CameraSettings prep;

    std::vector<LumoDeviceEntry> devices;
    CameraError error;
    if (!LumoCamera::enumerateDevices(prep, devices, error))
    {
        appendLog(QString("Lumo: profile refresh failed — %1").arg(QString::fromStdString(error.message)));
        return;
    }

    auto populateCombo = [&devices](QComboBox *combo) {
        if (combo == nullptr)
            return;

        combo->clear();
        for (const LumoDeviceEntry &device : devices)
        {
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

    populateCombo(camera1Ui_.deviceCombo);
    populateCombo(camera2Ui_.deviceCombo);

    if (!selectProfileHint(camera1Ui_.deviceCombo, QStringLiteral("FX10e with Pleora")))
        selectProfileHint(camera1Ui_.deviceCombo, QStringLiteral("FX10"));
    if (!selectProfileHint(camera2Ui_.deviceCombo, QStringLiteral("SWIR3 with NI")))
        selectProfileHint(camera2Ui_.deviceCombo, QStringLiteral("SWIR"));

    updateCameraTabLabel(camera1Ui_);
    updateCameraTabLabel(camera2Ui_);

    appendLog(QString("Lumo: found %1 SSP profile(s) (from SDK install).").arg(devices.size()));
    for (const LumoDeviceEntry &device : devices)
        appendLog(QString("  [%1] %2").arg(device.index).arg(QString::fromStdString(device.name)));
}

void MainWindow::onSettingsApplied(LumoCameraUi &ui, const CameraSettingsApplyReport &report)
{
    const CameraTimingApplyResult &timing = report.timing;
    if (!timing.valid)
        return;

    constexpr double kHzTolerance = 0.05;
    constexpr double kMsTolerance = 0.05;
    const bool frameRateDiffers =
        std::abs(timing.requestedFrameRateHz - timing.appliedFrameRateHz) > kHzTolerance;
    const bool exposureDiffers =
        std::abs(timing.requestedExposureMs - timing.appliedExposureMs) > kMsTolerance;

    if (ui.frameRateSpin != nullptr)
    {
        QSignalBlocker blocker(ui.frameRateSpin);
        ui.frameRateSpin->setValue(timing.appliedFrameRateHz);
    }
    if (ui.exposureSpin != nullptr)
    {
        QSignalBlocker blocker(ui.exposureSpin);
        ui.exposureSpin->setValue(timing.appliedExposureMs);
    }

    if (!frameRateDiffers && !exposureDiffers)
        return;

    const QString cameraLabel = QStringLiteral("Camera %1").arg(ui.cameraIndex + 1);
    QString reason;
    if (timing.exposureTimeAutoEnabled)
    {
        reason = QStringLiteral(
            "Camera.ExposureTime.Auto is enabled. When frame rate is applied, the Lumo SDK "
            "sets exposure so readout time + exposure time fits the frame period "
            "(1000 / frame rate ms).");
    }
    else
    {
        reason = QStringLiteral(
            "The Lumo SDK limited timing to the valid range for the current frame rate "
            "(readout + exposure must fit within the frame period).");
    }

    const QString message = QStringLiteral(
                                "%1\n\n"
                                "Requested:  %2 Hz, %3 ms\n"
                                "Applied:    %4 Hz, %5 ms\n"
                                "Readout:    %6 ms (Camera.Image.ReadoutTime)")
                                .arg(reason)
                                .arg(timing.requestedFrameRateHz, 0, 'f', 2)
                                .arg(timing.requestedExposureMs, 0, 'f', 2)
                                .arg(timing.appliedFrameRateHz, 0, 'f', 2)
                                .arg(timing.appliedExposureMs, 0, 'f', 2)
                                .arg(timing.readoutTimeMs, 0, 'f', 2);

    QMessageBox::information(this,
                             QStringLiteral("%1 timing adjusted").arg(cameraLabel),
                             message);
    appendLog(QStringLiteral("%1: timing adjusted — requested %2 Hz / %3 ms, applied %4 Hz / %5 ms")
                  .arg(cameraLabel)
                  .arg(timing.requestedFrameRateHz, 0, 'f', 2)
                  .arg(timing.requestedExposureMs, 0, 'f', 2)
                  .arg(timing.appliedFrameRateHz, 0, 'f', 2)
                  .arg(timing.appliedExposureMs, 0, 'f', 2));
}

void MainWindow::onCameraError(LumoCameraUi &ui, const CameraError &error)
{
    if (!ui.connectAttemptActive)
        return;

    ui.connectAttemptActive = false;
    ui.autoStreamStarted = false;

    const QString title = QString("Camera %1 connection failed").arg(ui.cameraIndex + 1);
    const QString message = QString::fromStdString(error.message);
    appendLog(QString("%1: %2").arg(title, message));
    QMessageBox::warning(this, title, message);
}

void MainWindow::updateShutterDisplay(LumoCameraUi &ui, const bool isOpen)
{
    ui.shutterReportedOpen = isOpen;

    if (ui.shutterIndicator != nullptr)
    {
        ui.shutterIndicator->setStyleSheet(
            isOpen ? QStringLiteral("background-color: #e67e22; border-radius: 2px;")
                   : QStringLiteral("background-color: #666666; border-radius: 2px;"));
    }

    if (ui.shutterStatusLabel != nullptr)
    {
        ui.shutterStatusLabel->setText(isOpen ? QStringLiteral("Opened") : QStringLiteral("Closed"));
    }

    if (ui.shutterToggleBtn != nullptr)
        ui.shutterToggleBtn->setText(isOpen ? QStringLiteral("Close") : QStringLiteral("Open"));
}

void MainWindow::onShutterStateChanged(LumoCameraUi &ui, const bool isOpen)
{
    updateShutterDisplay(ui, isOpen);
}

void MainWindow::onCameraStateChanged(LumoCameraUi &ui, const CameraState state)
{
    updateCameraControls(ui, state);

    if (coordinator_ != nullptr
        && (state == CameraState::Initialized || state == CameraState::Configured
            || state == CameraState::Armed || state == CameraState::Streaming
            || state == CameraState::SafeStopped))
        coordinator_->refreshShutterState(ui.cameraIndex);

    if (state == CameraState::Initialized && coordinator_ != nullptr && !ui.autoStreamStarted)
    {
        ui.autoStreamStarted = true;
        refreshBandCombos(ui);
        syncWaterfallBands(ui);
        syncProfileRgbMarkers(ui);
        if (ui::WaterfallProcessor *processor = waterfallProcessorFor(ui))
            processor->reset();
        if (ui::ProfileProcessor *profileProcessor = profileProcessorFor(ui))
            profileProcessor->reset();

        const CameraSettings settings = buildCameraSettings(ui);
        coordinator_->beginStreaming(ui.cameraIndex, settings);
        appendLog(QString("Camera %1: streaming started automatically.")
                      .arg(ui.cameraIndex + 1));
    }

    if (state == CameraState::Streaming)
        ui.connectAttemptActive = false;

    if (state == CameraState::Disconnected)
    {
        ui.autoStreamStarted = false;
        ui.connectAttemptActive = false;
        updateShutterDisplay(ui, false);
        clearDetectorView(ui);
    }
}

void MainWindow::updateStreamPaneTitles(LumoCameraUi &ui)
{
    const int width = ui.frameWidth;
    const int height = ui.frameHeight;

    if (ui.detectorPane != nullptr)
    {
        if (width > 0 && height > 0)
        {
            ui.detectorPane->setTitle(
                QStringLiteral("Detector (%1 × %2)").arg(width).arg(height));
        }
        else
            ui.detectorPane->setTitle(QStringLiteral("Detector"));
    }

    updateProfilePaneTitles(ui);
}

void MainWindow::updateProfilePaneTitles(LumoCameraUi &ui)
{
    const int width = ui.frameWidth;
    const int bands = ui.frameHeight;

    int spatialIndex = 0;
    int bandIndex = 0;
    if (ui.detectorView != nullptr && width > 0 && bands > 0)
    {
        spatialIndex = ui.detectorView->spatialIndex();
        bandIndex = ui.detectorView->bandIndex();
    }

    if (ui.wavelengthPane != nullptr)
    {
        if (bands > 0 && width > 0)
        {
            ui.wavelengthPane->setTitle(
                QStringLiteral("Wavelength (%1 bands @ %2)")
                    .arg(bands)
                    .arg(formatOrdinalPixel(spatialIndex)));
        }
        else if (bands > 0)
            ui.wavelengthPane->setTitle(QStringLiteral("Wavelength (%1 bands)").arg(bands));
        else
            ui.wavelengthPane->setTitle(QStringLiteral("Wavelength"));
    }

    if (ui.pixelStreamPane != nullptr)
    {
        if (width > 0 && bands > 0)
        {
            const std::vector<double> wlLookup = buildWavelengthNmLookup(ui.spectralBands, bands);
            double wavelengthNm = 0.0;
            if (bandIndex >= 0 && bandIndex < static_cast<int>(wlLookup.size()))
                wavelengthNm = wlLookup[static_cast<std::size_t>(bandIndex)];

            ui.pixelStreamPane->setTitle(
                QStringLiteral("Pixel (%1 positions @ %2 nm)")
                    .arg(width)
                    .arg(wavelengthNm, 0, 'f', 2));
        }
        else if (width > 0)
            ui.pixelStreamPane->setTitle(QStringLiteral("Pixel (%1 positions)").arg(width));
        else
            ui.pixelStreamPane->setTitle(QStringLiteral("Pixel"));
    }
}

void MainWindow::setupWaterfallProcessors()
{
    waterfallProcessor1_ = std::make_unique<ui::WaterfallProcessor>();
    waterfallProcessor2_ = std::make_unique<ui::WaterfallProcessor>();

    const auto wireProcessor = [this](ui::WaterfallProcessor &processor, LumoCameraUi &ui) {
        processor.setImageReadyCallback([this, &ui](QImage image) {
            QMetaObject::invokeMethod(
                this,
                [this, &ui, image = std::move(image)]() { updateWaterfallView(ui, image); },
                Qt::QueuedConnection);
        });
        processor.start();
        syncWaterfallBands(ui);
    };

    wireProcessor(*waterfallProcessor1_, camera1Ui_);
    wireProcessor(*waterfallProcessor2_, camera2Ui_);
}

ui::WaterfallProcessor *MainWindow::waterfallProcessorFor(const LumoCameraUi &ui)
{
    if (ui.cameraIndex == 0)
        return waterfallProcessor1_.get();
    if (ui.cameraIndex == 1)
        return waterfallProcessor2_.get();
    return nullptr;
}

void MainWindow::syncWaterfallBands(LumoCameraUi &ui)
{
    ui::WaterfallProcessor *processor = waterfallProcessorFor(ui);
    if (processor == nullptr)
        return;

    const CameraSettings settings = buildCameraSettings(ui);
    const int spectralBin = std::max(1, settings.spectralBinning);
    const int frameBands = ui.frameHeight > 0 ? ui.frameHeight : 0;

    ui::RgbBandIndices bands;
    bands.red = settings.redBandIndex;
    bands.green = settings.greenBandIndex;
    bands.blue = settings.blueBandIndex;
    if (frameBands > 0)
    {
        bands.red = ui::mapCalpackBandToBilRow(bands.red, frameBands, spectralBin);
        bands.green = ui::mapCalpackBandToBilRow(bands.green, frameBands, spectralBin);
        bands.blue = ui::mapCalpackBandToBilRow(bands.blue, frameBands, spectralBin);
    }
    processor->setBandIndices(bands);
}

void MainWindow::onStreamFrame(const FramePacket &frame)
{
    updateDetectorFrame(frame);

    LumoCameraUi *ui = nullptr;
    if (frame.source == CameraBackendId::Camera1)
        ui = &camera1Ui_;
    else if (frame.source == CameraBackendId::Camera2)
        ui = &camera2Ui_;

    if (ui == nullptr)
        return;

    if (ui::WaterfallProcessor *waterfallProcessor = waterfallProcessorFor(*ui))
        waterfallProcessor->submitFrame(frame);

    if (ui::ProfileProcessor *profileProcessor = profileProcessorFor(*ui))
        profileProcessor->submitFrame(frame);
}

void MainWindow::setupProfileProcessors()
{
    profileProcessor1_ = std::make_unique<ui::ProfileProcessor>();
    profileProcessor2_ = std::make_unique<ui::ProfileProcessor>();

    const auto wireProcessor = [this](ui::ProfileProcessor &processor, LumoCameraUi &ui) {
        processor.setProfilesReadyCallback([this, &ui](ui::ProfileExtraction profiles) {
            QMetaObject::invokeMethod(
                this,
                [this, &ui, profiles = std::move(profiles)]() { updateProfilePlots(ui, profiles); },
                Qt::QueuedConnection);
        });
        processor.start();

        if (ui.detectorView != nullptr)
        {
            ui::ProfileCursor cursor;
            cursor.spatialX = ui.detectorView->spatialIndex();
            cursor.bandY = ui.detectorView->bandIndex();
            processor.setCursor(cursor);
        }
        syncProfileRgbMarkers(ui);
    };

    wireProcessor(*profileProcessor1_, camera1Ui_);
    wireProcessor(*profileProcessor2_, camera2Ui_);
}

ui::ProfileProcessor *MainWindow::profileProcessorFor(const LumoCameraUi &ui)
{
    if (ui.cameraIndex == 0)
        return profileProcessor1_.get();
    if (ui.cameraIndex == 1)
        return profileProcessor2_.get();
    return nullptr;
}

void MainWindow::syncProfileRgbMarkers(LumoCameraUi &ui)
{
    if (ui.wavelengthView == nullptr)
        return;

    const CameraSettings settings = buildCameraSettings(ui);
    const int spectralBin = std::max(1, settings.spectralBinning);
    const int frameBands = ui.frameHeight > 0 ? ui.frameHeight : 0;

    int red = settings.redBandIndex;
    int green = settings.greenBandIndex;
    int blue = settings.blueBandIndex;
    if (frameBands > 0)
    {
        red = ui::mapCalpackBandToBilRow(red, frameBands, spectralBin);
        green = ui::mapCalpackBandToBilRow(green, frameBands, spectralBin);
        blue = ui::mapCalpackBandToBilRow(blue, frameBands, spectralBin);
    }

    ui.wavelengthView->setRgbBandMarkers(red, green, blue);
}

void MainWindow::onProfileLinesChanged(LumoCameraUi &ui,
                                       const int spatialIndex,
                                       const int bandIndex)
{
    Q_UNUSED(spatialIndex);
    Q_UNUSED(bandIndex);

    updateProfilePaneTitles(ui);

    ui::ProfileProcessor *processor = profileProcessorFor(ui);
    if (processor == nullptr)
        return;

    ui::ProfileCursor cursor;
    cursor.spatialX = spatialIndex;
    cursor.bandY = bandIndex;
    processor->setCursor(cursor);
    processor->requestRefresh();
}

namespace
{
std::vector<double> buildWavelengthNmLookup(const std::vector<SpectralBand> &bands, const int bandCount)
{
    if (bandCount <= 0)
        return {};

    std::vector<double> lookup(static_cast<std::size_t>(bandCount), 0.0);
    for (const SpectralBand &band : bands)
    {
        if (band.index >= 0 && band.index < bandCount)
            lookup[static_cast<std::size_t>(band.index)] = band.wavelengthNm;
    }

    int firstKnown = -1;
    int lastKnown = -1;
    for (int i = 0; i < bandCount; ++i)
    {
        if (lookup[static_cast<std::size_t>(i)] > 0.0)
        {
            if (firstKnown < 0)
                firstKnown = i;
            lastKnown = i;
        }
    }

    if (firstKnown < 0)
    {
        for (int i = 0; i < bandCount; ++i)
            lookup[static_cast<std::size_t>(i)] = static_cast<double>(i);
        return lookup;
    }

    const double wlStart = lookup[static_cast<std::size_t>(firstKnown)];
    const double wlEnd = lookup[static_cast<std::size_t>(lastKnown)];
    for (int i = 0; i < bandCount; ++i)
    {
        if (lookup[static_cast<std::size_t>(i)] > 0.0)
            continue;

        const double t = (bandCount > 1)
                             ? static_cast<double>(i) / static_cast<double>(bandCount - 1)
                             : 0.0;
        lookup[static_cast<std::size_t>(i)] = wlStart + (wlEnd - wlStart) * t;
    }

    return lookup;
}
} // namespace

void MainWindow::updateProfilePlots(LumoCameraUi &ui, const ui::ProfileExtraction &profiles)
{
    if (!profiles.valid)
        return;

    if (ui.wavelengthView != nullptr && !profiles.wavelengthDn.empty())
    {
        const int bandMax = std::max(0, profiles.frameBands - 1);
        ui.wavelengthView->setWavelengthAxis(buildWavelengthNmLookup(ui.spectralBands, profiles.frameBands));
        ui.wavelengthView->setProfile(profiles.wavelengthDn, bandMax);
    }

    if (ui.pixelStreamView != nullptr && !profiles.spatialDn.empty())
    {
        ui.pixelStreamView->setProfile(profiles.spatialDn, std::max(0, profiles.frameWidth - 1));
    }
}

void MainWindow::updateWaterfallView(LumoCameraUi &ui, const QImage &image)
{
    if (ui.waterfallView == nullptr || image.isNull())
        return;

    ui.waterfallView->setPixmap(QPixmap::fromImage(image));
}

void MainWindow::updateDetectorFrame(const FramePacket &frame)
{
    LumoCameraUi *ui = nullptr;
    if (frame.source == CameraBackendId::Camera1)
        ui = &camera1Ui_;
    else if (frame.source == CameraBackendId::Camera2)
        ui = &camera2Ui_;

    if (ui == nullptr || ui->detectorView == nullptr)
        return;

    const bool geometryChanged =
        frame.width > 0 && frame.height > 0
        && (ui->frameWidth != frame.width || ui->frameHeight != frame.height);
    if (geometryChanged)
    {
        ui->frameWidth = frame.width;
        ui->frameHeight = frame.height;
        ui->detectorView->setFrameSize(frame.width, frame.height);
        updateStreamPaneTitles(*ui);
        syncWaterfallBands(*ui);
        syncProfileRgbMarkers(*ui);
    }

    const QImage image = framePacketToQImage(frame);
    if (image.isNull())
        return;

    ui->detectorView->setDetectorImage(QPixmap::fromImage(image));
}

void MainWindow::clearDetectorView(LumoCameraUi &ui)
{
    ui.frameWidth = 0;
    ui.frameHeight = 0;
    updateStreamPaneTitles(ui);

    if (ui::WaterfallProcessor *processor = waterfallProcessorFor(ui))
        processor->reset();
    if (ui::ProfileProcessor *profileProcessor = profileProcessorFor(ui))
        profileProcessor->reset();

    const QString cameraName = QStringLiteral("Camera %1").arg(ui.cameraIndex + 1);
    if (ui.detectorView != nullptr)
        ui.detectorView->clearDisplay(cameraName + QStringLiteral(" detector (disconnected)"));
    if (ui.waterfallView != nullptr)
    {
        ui.waterfallView->clear();
        setPreviewDisconnectedText(ui.waterfallView, QStringLiteral("waterfall"), cameraName);
    }
    if (ui.wavelengthView != nullptr)
        ui.wavelengthView->clearDisplay(cameraName + QStringLiteral(" wavelength (disconnected)"));
    if (ui.pixelStreamView != nullptr)
        ui.pixelStreamView->clearDisplay(cameraName + QStringLiteral(" pixel (disconnected)"));
}

void MainWindow::updateCameraControls(LumoCameraUi &ui, const CameraState state)
{
    ui.state = state;

    const bool connected = state != CameraState::Disconnected && state != CameraState::Fault;

    if (ui.connectBtn != nullptr)
    {
        ui.connectBtn->setEnabled(true);
        ui.connectBtn->setText(connected ? QStringLiteral("Disconnect camera")
                                         : QStringLiteral("Connect camera"));
    }

    if (ui.deviceCombo != nullptr)
        ui.deviceCombo->setEnabled(!connected);

    if (ui.calibrationPackEdit != nullptr)
        ui.calibrationPackEdit->setEnabled(!connected);
    if (ui.calibrationPackBrowseBtn != nullptr)
        ui.calibrationPackBrowseBtn->setEnabled(!connected);

    const bool readyForCameraFeatures = state == CameraState::Initialized || state == CameraState::Configured
                                        || state == CameraState::Armed || state == CameraState::Streaming
                                        || state == CameraState::SafeStopped;
    if (ui.shutterToggleBtn != nullptr)
        ui.shutterToggleBtn->setEnabled(readyForCameraFeatures);
    if (ui.spectralBinningCombo != nullptr)
        ui.spectralBinningCombo->setEnabled(readyForCameraFeatures);
    if (ui.spatialBinningCombo != nullptr)
        ui.spatialBinningCombo->setEnabled(readyForCameraFeatures);
    if (ui.exposureSpin != nullptr)
        ui.exposureSpin->setEnabled(readyForCameraFeatures);
    if (ui.frameRateSpin != nullptr)
        ui.frameRateSpin->setEnabled(readyForCameraFeatures);
    if (ui.triggerCombo != nullptr)
        ui.triggerCombo->setEnabled(readyForCameraFeatures);
    if (ui.redBandCombo != nullptr)
        ui.redBandCombo->setEnabled(readyForCameraFeatures);
    if (ui.greenBandCombo != nullptr)
        ui.greenBandCombo->setEnabled(readyForCameraFeatures);
    if (ui.blueBandCombo != nullptr)
        ui.blueBandCombo->setEnabled(readyForCameraFeatures);

    const bool readyForApply = state == CameraState::Initialized || state == CameraState::Configured
                               || state == CameraState::Armed || state == CameraState::SafeStopped
                               || state == CameraState::Streaming;
    if (ui.applyBtn != nullptr)
        ui.applyBtn->setEnabled(readyForApply);
}
