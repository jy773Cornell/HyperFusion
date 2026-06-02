// Main window implementation for stream views, settings tabs, and logging.
#include "MainWindow.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "orchestrator/CameraCoordinator.hpp"

#include <QComboBox>
#include <QMetaObject>
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
#include <QSizePolicy>
#include <QTimer>
#include <QWidget>

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

void setPreviewDisconnectedText(QLabel *label, const QString &paneTitle, const QString &cameraName)
{
    if (label == nullptr)
        return;

    label->setText(cameraName + QStringLiteral(" ") + paneTitle + QStringLiteral(" (disconnected)"));
}
} // namespace

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

    const QStringList candidates = {
        QDir(appDir).filePath(QStringLiteral("calibration/") + fileName),
        QDir(appDir).filePath(QStringLiteral("../calibration/") + fileName),
        QDir(appDir).filePath(QStringLiteral("../../app/calibration/") + fileName),
    };

    for (const QString &candidate : candidates)
    {
        if (QFileInfo::exists(candidate))
            return QDir::cleanPath(candidate);
    }

    return QDir::cleanPath(QDir(appDir).filePath(QStringLiteral("../calibration/") + fileName));
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

    camera1Ui_.camera = std::make_shared<LumoCamera>(CameraBackendId::Camera1, "Camera 1");
    camera1Ui_.cameraIndex = 0;
    camera2Ui_.camera = std::make_shared<LumoCamera>(CameraBackendId::Camera2, "Camera 2");
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

    coordinator_->setFrameCallback([this](const FramePacket &frame) {
        QMetaObject::invokeMethod(
            this,
            [this, frame]() { updateDetectorFrame(frame); },
            Qt::QueuedConnection);
    });

    coordinator_->start();
    appendLog("HyperFusion UI initialized; camera coordinator started.");

    QTimer::singleShot(0, this, [this]() { refreshLumoDeviceLists(); });
}

MainWindow::~MainWindow()
{
    if (!coordinator_)
        return;

    coordinator_->shutdownSync();
}

QWidget *MainWindow::createStreamTabsPanel()
{
    auto *panel = new QWidget(this);
    auto *layout = new QVBoxLayout(panel);

    auto *tabs = new QTabWidget(panel);
    tabs->addTab(createStreamTabPage(QStringLiteral("Camera 1"), camera1Ui_), "Camera 1");
    tabs->addTab(createStreamTabPage(QStringLiteral("Camera 2"), camera2Ui_), "Camera 2");
    tabs->addTab(createRgbUr3eStreamTab(), "UR3e");

    layout->addWidget(tabs, 1);
    return panel;
}

QWidget *MainWindow::createStreamTabPage(const QString &cameraName, LumoCameraUi &cameraUi)
{
    auto *tab = new QWidget(this);
    auto *grid = new QGridLayout(tab);
    grid->setContentsMargins(8, 8, 8, 8);
    grid->setSpacing(10);

    QLabel *detectorLabel = nullptr;
    QLabel *waterfallLabel = nullptr;
    QLabel *wavelengthLabel = nullptr;
    QLabel *pixelStreamLabel = nullptr;

    auto *detectorPane = createPreviewPane(QStringLiteral("Detector"), detectorLabel);
    auto *waterfallPane = createPreviewPane(QStringLiteral("Waterfall"), waterfallLabel);
    auto *wavelengthPane = createPreviewPane(QStringLiteral("Wavelength"), wavelengthLabel);
    auto *pixelStreamPane = createPreviewPane(QStringLiteral("Pixel stream"), pixelStreamLabel);

    cameraUi.detectorView = detectorLabel;
    cameraUi.waterfallView = waterfallLabel;
    cameraUi.wavelengthView = wavelengthLabel;
    cameraUi.pixelStreamView = pixelStreamLabel;

    setPreviewDisconnectedText(cameraUi.detectorView, QStringLiteral("detector"), cameraName);
    setPreviewDisconnectedText(cameraUi.waterfallView, QStringLiteral("waterfall"), cameraName);
    setPreviewDisconnectedText(cameraUi.wavelengthView, QStringLiteral("wavelength"), cameraName);
    setPreviewDisconnectedText(cameraUi.pixelStreamView, QStringLiteral("pixel stream"), cameraName);

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

    auto *tabs = new QTabWidget(panel);
    tabs->addTab(createCameraSettingsTab(), "Camera");
    tabs->addTab(createStageSettingsTab(), "Stage");
    tabs->addTab(createLightSettingsTab(), "Light");
    tabs->addTab(createUr3eSettingsTab(), "UR3e");
    tabs->addTab(createCaptureSettingsTab(), "Capture");

    layout->addWidget(tabs, 1);
    return panel;
}

QWidget *MainWindow::createCameraSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    layout->addWidget(createLumoCameraGroup(page, QStringLiteral("Camera 1"), camera1Ui_));
    layout->addWidget(createLumoCameraGroup(page, QStringLiteral("Camera 2"), camera2Ui_));
    layout->addStretch();
    return page;
}

QGroupBox *MainWindow::createLumoCameraGroup(QWidget *parent, const QString &title, LumoCameraUi &ui)
{
    auto *box = new QGroupBox(title, parent);
    auto *form = new QFormLayout(box);

    ui.deviceCombo = new QComboBox(box);
    ui.deviceCombo->setMinimumWidth(260);

    ui.calibrationPackEdit = new QLineEdit(box);
    ui.calibrationPackEdit->setMinimumWidth(0);
    ui.calibrationPackEdit->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    ui.calibrationPackBrowseBtn = new QPushButton(QStringLiteral("Browse…"), box);
    ui.calibrationPackBrowseBtn->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    ui.calibrationPackBrowseBtn->setFixedWidth(72);

    if (title == QStringLiteral("Camera 1"))
        setCalibrationPackDisplay(ui.calibrationPackEdit, defaultFx10eCalibrationPackPath());

    auto *calibrationRow = new QWidget(box);
    auto *calibrationLayout = new QHBoxLayout(calibrationRow);
    calibrationLayout->setContentsMargins(0, 0, 0, 0);
    calibrationLayout->setSpacing(6);
    calibrationLayout->addWidget(ui.calibrationPackEdit, 1);
    calibrationLayout->addWidget(ui.calibrationPackBrowseBtn, 0);

    auto *shutterRow = new QWidget(box);
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

    ui.frameRateSpin = new QDoubleSpinBox(box);
    ui.frameRateSpin->setRange(1.0, 500.0);
    ui.frameRateSpin->setDecimals(2);
    ui.frameRateSpin->setValue(title == QStringLiteral("Camera 1") ? 133.0 : 100.0);
    ui.frameRateSpin->setSuffix(QStringLiteral(" Hz"));

    ui.exposureSpin = new QDoubleSpinBox(box);
    ui.exposureSpin->setRange(0.01, 400.0);
    ui.exposureSpin->setDecimals(2);
    ui.exposureSpin->setSingleStep(0.1);
    ui.exposureSpin->setValue(title == QStringLiteral("Camera 1") ? 2.0 : 3.0);
    ui.exposureSpin->setSuffix(QStringLiteral(" ms"));

    ui.spectralBinningCombo = new QComboBox(box);
    ui.spatialBinningCombo = new QComboBox(box);
    for (QComboBox *binningCombo : {ui.spectralBinningCombo, ui.spatialBinningCombo})
    {
        binningCombo->addItems({QStringLiteral("1"),
                                QStringLiteral("2"),
                                QStringLiteral("4"),
                                QStringLiteral("8")});
    }

    ui.triggerCombo = new QComboBox(box);
    ui.triggerCombo->addItems({"Internal", "External"});

    ui.connectBtn = new QPushButton("Connect camera", box);
    ui.applyBtn = new QPushButton("Apply settings", box);

    ui.applyBtn->setEnabled(false);

    form->addRow("Sensor profile", ui.deviceCombo);
    form->addRow("Calibration pack", calibrationRow);
    form->addRow("Shutter", shutterRow);
    form->addRow("Frame rate (Hz)", ui.frameRateSpin);
    form->addRow("Exposure time (ms)", ui.exposureSpin);
    form->addRow("Spectral binning", ui.spectralBinningCombo);
    form->addRow("Spatial binning", ui.spatialBinningCombo);
    form->addRow("Trigger mode", ui.triggerCombo);
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
            setCalibrationPackDisplay(ui.calibrationPackEdit, path);
    });

    connect(ui.connectBtn, &QPushButton::clicked, this, [this, &ui, title]() {
        if (!coordinator_ || !ui.camera)
            return;

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
            appendLog(QString("%1: disconnect requested.").arg(title));
            return;
        }

        if (ui.deviceCombo->count() == 0)
        {
            appendLog(QString("%1: refresh SSP profiles before connecting.").arg(title));
            return;
        }

        const CameraSettings connectionSettings = buildCameraSettings(ui);
        ui.camera->prepareConnection(connectionSettings);
        ui.connectAttemptActive = true;
        appendLog(QString("%1: connect camera — profile %2 (eBUS picker may appear; not ready until Initialized).")
                      .arg(title, ui.deviceCombo->currentText()));

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

    connect(ui.applyBtn, &QPushButton::clicked, this, [this, &ui, title]() {
        if (!coordinator_)
            return;

        const CameraSettings settings = buildCameraSettings(ui);
        coordinator_->applySettings(ui.cameraIndex, settings);
        appendLog(QString("%1: apply settings (fps=%2 Hz, exposure=%3 ms, spectral=%4, spatial=%5, trigger=%6)")
                      .arg(title)
                      .arg(settings.frameRateHz, 0, 'f', 2)
                      .arg(settings.exposureMs, 0, 'f', 2)
                      .arg(settings.spectralBinning)
                      .arg(settings.spatialBinning)
                      .arg(ui.triggerCombo->currentText()));
    });

    updateShutterDisplay(ui, false);
    return box;
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
    return settings;
}

QWidget *MainWindow::createStageSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *connBox = new QGroupBox("Connection", page);
    auto *connForm = new QFormLayout(connBox);
    auto *portEdit = new QLineEdit(connBox);
    portEdit->setPlaceholderText("COM port or device path");
    portEdit->setText("COM3");
    auto *baudCombo = new QComboBox(connBox);
    baudCombo->addItems({"9600", "19200", "38400", "57600", "115200"});
    baudCombo->setCurrentText("115200");
    auto *connectBtn = new QPushButton("Connect", connBox);
    auto *disconnectBtn = new QPushButton("Disconnect", connBox);
    disconnectBtn->setEnabled(false);
    connForm->addRow("Port", portEdit);
    connForm->addRow("Baud", baudCombo);
    connForm->addRow("", connectBtn);
    connForm->addRow("", disconnectBtn);

    connect(connectBtn, &QPushButton::clicked, this, [this, portEdit, baudCombo, disconnectBtn]() {
        appendLog(QString("Stage: connect requested (%1 @ %2)")
                      .arg(portEdit->text(), baudCombo->currentText()));
        disconnectBtn->setEnabled(true);
    });
    connect(disconnectBtn, &QPushButton::clicked, this, [this, disconnectBtn]() {
        appendLog("Stage: disconnect requested (not implemented)");
        disconnectBtn->setEnabled(false);
    });

    auto *motionBox = new QGroupBox("Motion", page);
    auto *motionForm = new QFormLayout(motionBox);
    auto *posSpin = new QDoubleSpinBox(motionBox);
    posSpin->setRange(-10000.0, 10000.0);
    posSpin->setDecimals(3);
    posSpin->setSuffix(" mm");
    auto *speedSpin = new QDoubleSpinBox(motionBox);
    speedSpin->setRange(0.1, 5000.0);
    speedSpin->setDecimals(1);
    speedSpin->setValue(25.0);
    speedSpin->setSuffix(" mm/min");
    auto *moveAbsBtn = new QPushButton("Move absolute", motionBox);
    auto *homeBtn = new QPushButton("Home", motionBox);
    auto *stopBtn = new QPushButton("E-stop", motionBox);
    motionForm->addRow("Target position", posSpin);
    motionForm->addRow("Speed", speedSpin);
    motionForm->addRow("", moveAbsBtn);
    motionForm->addRow("", homeBtn);
    motionForm->addRow("", stopBtn);

    connect(moveAbsBtn, &QPushButton::clicked, this, [this, posSpin, speedSpin]() {
        appendLog(QString("Stage: move absolute requested (pos=%1 mm, speed=%2 mm/min)")
                      .arg(posSpin->value(), 0, 'f', 3)
                      .arg(speedSpin->value(), 0, 'f', 1));
    });
    connect(homeBtn, &QPushButton::clicked, this, [this]() {
        appendLog("Stage: home requested (not implemented)");
    });
    connect(stopBtn, &QPushButton::clicked, this, [this]() {
        appendLog("Stage: E-stop requested (not implemented)");
    });

    layout->addWidget(connBox);
    layout->addWidget(motionBox);
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
    selectProfileHint(camera2Ui_.deviceCombo, QStringLiteral("SWIR"));

    appendLog(QString("Lumo: found %1 SSP profile(s) (from SDK install).").arg(devices.size()));
    for (const LumoDeviceEntry &device : devices)
        appendLog(QString("  [%1] %2").arg(device.index).arg(QString::fromStdString(device.name)));
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

void MainWindow::updateDetectorFrame(const FramePacket &frame)
{
    LumoCameraUi *ui = nullptr;
    if (frame.source == CameraBackendId::Camera1)
        ui = &camera1Ui_;
    else if (frame.source == CameraBackendId::Camera2)
        ui = &camera2Ui_;

    if (ui == nullptr || ui->detectorView == nullptr)
        return;

    const QImage image = framePacketToQImage(frame);
    if (image.isNull())
        return;

    ui->detectorView->setPixmap(QPixmap::fromImage(image));
}

void MainWindow::clearDetectorView(LumoCameraUi &ui)
{
    const QString cameraName = QStringLiteral("Camera %1").arg(ui.cameraIndex + 1);
    if (ui.detectorView != nullptr)
    {
        ui.detectorView->clear();
        setPreviewDisconnectedText(ui.detectorView, QStringLiteral("detector"), cameraName);
    }
    if (ui.waterfallView != nullptr)
    {
        ui.waterfallView->clear();
        setPreviewDisconnectedText(ui.waterfallView, QStringLiteral("waterfall"), cameraName);
    }
    if (ui.wavelengthView != nullptr)
    {
        ui.wavelengthView->clear();
        setPreviewDisconnectedText(ui.wavelengthView, QStringLiteral("wavelength"), cameraName);
    }
    if (ui.pixelStreamView != nullptr)
    {
        ui.pixelStreamView->clear();
        setPreviewDisconnectedText(ui.pixelStreamView, QStringLiteral("pixel stream"), cameraName);
    }
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

    const bool readyForApply = state == CameraState::Initialized || state == CameraState::Configured
                               || state == CameraState::Armed || state == CameraState::SafeStopped
                               || state == CameraState::Streaming;
    if (ui.applyBtn != nullptr)
        ui.applyBtn->setEnabled(readyForApply);
}
