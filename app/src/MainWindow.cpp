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
#include <QEventLoop>
#include <QProcessEnvironment>
#include <QTimer>
#include <QWidget>

#include <vector>

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
                QCoreApplication::processEvents(QEventLoop::AllEvents);
            },
            Qt::BlockingQueuedConnection);
    });

    coordinator_->setCameraStateCallback(0, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { updateCameraControls(camera1Ui_, state); },
            Qt::QueuedConnection);
    });

    coordinator_->setCameraStateCallback(1, [this](const CameraState state) {
        QMetaObject::invokeMethod(
            this,
            [this, state]() { updateCameraControls(camera2Ui_, state); },
            Qt::QueuedConnection);
    });

    coordinator_->setFrameCallback([this](const FramePacket &frame) {
        if ((frame.frameIndex % 60U) != 0U)
            return;

        const QString cameraName =
            frame.source == CameraBackendId::Camera1 ? QStringLiteral("Camera 1") : QStringLiteral("Camera 2");
        const QString line = QStringLiteral("%1 frame %2 (%3x%4)")
                               .arg(cameraName)
                               .arg(frame.frameIndex)
                               .arg(frame.width)
                               .arg(frame.height);
        QMetaObject::invokeMethod(
            this,
            [this, line]() { appendLog(line); },
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

    coordinator_->stopStream(0);
    coordinator_->stopStream(1);
    coordinator_->disconnect(0);
    coordinator_->disconnect(1);
    coordinator_->stop();
}

QWidget *MainWindow::createStreamTabsPanel()
{
    auto *panel = new QWidget(this);
    auto *layout = new QVBoxLayout(panel);

    auto *tabs = new QTabWidget(panel);
    tabs->addTab(createStreamTabPage("Camera 1"), "Camera 1");
    tabs->addTab(createStreamTabPage("Camera 2"), "Camera 2");
    tabs->addTab(createRgbUr3eStreamTab(), "UR3e");

    layout->addWidget(tabs, 1);
    return panel;
}

QWidget *MainWindow::createStreamTabPage(const QString &cameraName)
{
    auto *tab = new QWidget(this);
    auto *layout = new QVBoxLayout(tab);

    auto *gridHost = new QWidget(tab);
    auto *grid = new QGridLayout(gridHost);
    grid->setContentsMargins(8, 8, 8, 8);
    grid->setSpacing(10);

    QLabel *detectorLabel = nullptr;
    QLabel *waterfallLabel = nullptr;
    QLabel *wavelengthLabel = nullptr;
    QLabel *pixelsLabel = nullptr;

    auto *detectorPane = createPreviewPane("Detector", detectorLabel);
    auto *waterfallPane = createPreviewPane("Waterfall", waterfallLabel);
    auto *wavelengthPane = createPreviewPane("Wavelength", wavelengthLabel);
    auto *pixelsPane = createPreviewPane("Pixels", pixelsLabel);

    detectorLabel->setText(cameraName + " detector stream (disconnected)");
    waterfallLabel->setText(cameraName + " waterfall stream (disconnected)");
    wavelengthLabel->setText(cameraName + " wavelength view (disconnected)");
    pixelsLabel->setText(cameraName + " pixel profile (disconnected)");

    grid->addWidget(detectorPane, 0, 0);
    grid->addWidget(waterfallPane, 0, 1);
    grid->addWidget(wavelengthPane, 1, 0);
    grid->addWidget(pixelsPane, 1, 1);
    grid->setRowStretch(0, 1);
    grid->setRowStretch(1, 1);
    grid->setColumnStretch(0, 1);
    grid->setColumnStretch(1, 1);

    layout->addWidget(gridHost, 1);
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
    placeholder->setMinimumSize(440, 260);
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
    panel->setMinimumWidth(360);
    panel->setMaximumWidth(440);
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

    auto *profilesBox = new QGroupBox("Lumo SSP profiles", page);
    auto *profilesLayout = new QHBoxLayout(profilesBox);
    auto *refreshProfilesBtn = new QPushButton("Refresh profile list", profilesBox);
    profilesLayout->addWidget(refreshProfilesBtn);
    profilesLayout->addStretch();
    connect(refreshProfilesBtn, &QPushButton::clicked, this, [this]() { refreshLumoDeviceLists(); });

    layout->addWidget(profilesBox);
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

    ui.exposureSpin = new QDoubleSpinBox(box);
    ui.exposureSpin->setRange(0.001, 5.0);
    ui.exposureSpin->setDecimals(4);
    ui.exposureSpin->setSingleStep(0.001);
    ui.exposureSpin->setValue(title == QStringLiteral("Camera 1") ? 0.015 : 0.020);
    ui.exposureSpin->setSuffix(" s");

    ui.frameRateSpin = new QDoubleSpinBox(box);
    ui.frameRateSpin->setRange(1.0, 500.0);
    ui.frameRateSpin->setValue(title == QStringLiteral("Camera 1") ? 120.0 : 100.0);
    ui.frameRateSpin->setSuffix(" fps");

    ui.triggerCombo = new QComboBox(box);
    ui.triggerCombo->addItems({"Internal", "External"});

    ui.connectBtn = new QPushButton("Connect sensor", box);
    ui.applyBtn = new QPushButton("Apply settings", box);
    ui.startBtn = new QPushButton("Start preview", box);
    ui.stopBtn = new QPushButton("Stop preview", box);

    ui.applyBtn->setEnabled(false);
    ui.startBtn->setEnabled(false);
    ui.stopBtn->setEnabled(false);

    form->addRow("Sensor profile (SSP)", ui.deviceCombo);
    form->addRow("Exposure", ui.exposureSpin);
    form->addRow("Frame rate", ui.frameRateSpin);
    form->addRow("Trigger mode", ui.triggerCombo);
    form->addRow("", ui.connectBtn);
    form->addRow("", ui.applyBtn);
    form->addRow("", ui.startBtn);
    form->addRow("", ui.stopBtn);

    connect(ui.connectBtn, &QPushButton::clicked, this, [this, &ui, title]() {
        if (!coordinator_ || !ui.camera)
            return;

        const bool disconnectRequested =
            ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;

        if (disconnectRequested)
        {
            coordinator_->stopStream(ui.cameraIndex);
            coordinator_->disconnect(ui.cameraIndex);
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
        appendLog(QString("%1: connect sensor (SSP index %2, %3)...")
                      .arg(title)
                      .arg(connectionSettings.deviceIndex)
                      .arg(ui.deviceCombo->currentText()));

        coordinator_->connectAndInitializeOnGuiThread(ui.cameraIndex);
    });

    connect(ui.applyBtn, &QPushButton::clicked, this, [this, &ui, title]() {
        if (!coordinator_)
            return;

        const CameraSettings settings = buildCameraSettings(ui);
        coordinator_->applySettings(ui.cameraIndex, settings);
        appendLog(QString("%1: apply settings (exposure=%2 ms, fps=%3, trigger=%4)")
                      .arg(title)
                      .arg(settings.exposureMs, 0, 'f', 3)
                      .arg(settings.frameRateHz, 0, 'f', 1)
                      .arg(ui.triggerCombo->currentText()));
    });

    connect(ui.startBtn, &QPushButton::clicked, this, [this, &ui, title]() {
        if (!coordinator_)
            return;

        coordinator_->arm(ui.cameraIndex);
        coordinator_->startStream(ui.cameraIndex);
        appendLog(QString("%1: arm + start preview requested.").arg(title));
    });

    connect(ui.stopBtn, &QPushButton::clicked, this, [this, &ui, title]() {
        if (!coordinator_)
            return;

        coordinator_->stopStream(ui.cameraIndex);
        appendLog(QString("%1: stop preview requested.").arg(title));
    });

    return box;
}

CameraSettings MainWindow::buildCameraSettings(const LumoCameraUi &ui) const
{
    CameraSettings settings;
    settings.exposureMs = ui.exposureSpin->value() * 1000.0;
    settings.frameRateHz = ui.frameRateSpin->value();
    settings.externalTrigger = ui.triggerCombo->currentText() == QLatin1String("External");
    settings.acquisitionTimeoutMs = 5000;
    settings.deviceIndex = ui.deviceCombo->currentData().toInt();
    settings.lumoProfilesDirectory =
        QProcessEnvironment::systemEnvironment().value(QStringLiteral("HF_LUMO_PROFILES_DIR")).toStdString();

    const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    const QString perCameraKey =
        ui.cameraIndex == 0 ? QStringLiteral("HF_LUMO_GRABBER_CHANNEL_1") : QStringLiteral("HF_LUMO_GRABBER_CHANNEL_2");
    settings.grabberChannel = env.contains(perCameraKey) ? env.value(perCameraKey).toStdString()
                                                         : env.value(QStringLiteral("HF_LUMO_GRABBER_CHANNEL")).toStdString();
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

    CameraSettings prep;
    prep.lumoProfilesDirectory =
        QProcessEnvironment::systemEnvironment().value(QStringLiteral("HF_LUMO_PROFILES_DIR")).toStdString();

    std::vector<LumoDeviceEntry> devices;
    CameraError error;
    if (!LumoCamera::enumerateDevices(prep, devices, error))
    {
        appendLog(QString("Lumo: profile refresh failed — %1").arg(QString::fromStdString(error.message)));
        return;
    }

    auto populateCombo = [&devices](QComboBox *combo, const char *preferHint) {
        if (combo == nullptr)
            return;

        combo->clear();
        for (const LumoDeviceEntry &device : devices)
        {
            const QString label = QString::fromStdString(device.name);
            combo->addItem(label, device.index);

            if (preferHint != nullptr && device.name.find(preferHint) != std::string::npos)
                combo->setCurrentIndex(combo->count() - 1);
        }
    };

    populateCombo(camera1Ui_.deviceCombo, "FX10");
    populateCombo(camera2Ui_.deviceCombo, "SWIR");

    appendLog(QString("Lumo: found %1 SSP profile(s).").arg(devices.size()));
}

void MainWindow::updateCameraControls(LumoCameraUi &ui, const CameraState state)
{
    ui.state = state;

    const bool connected = state != CameraState::Disconnected && state != CameraState::Fault;

    if (ui.connectBtn != nullptr)
    {
        ui.connectBtn->setText(connected ? QStringLiteral("Disconnect sensor")
                                         : QStringLiteral("Connect sensor"));
    }

    if (ui.deviceCombo != nullptr)
        ui.deviceCombo->setEnabled(!connected);

    const bool readyForApply = state == CameraState::Initialized || state == CameraState::Configured
                               || state == CameraState::Armed || state == CameraState::SafeStopped;
    if (ui.applyBtn != nullptr)
        ui.applyBtn->setEnabled(readyForApply);

    const bool canPreview = state == CameraState::Configured || state == CameraState::Armed
                            || state == CameraState::SafeStopped;
    if (ui.startBtn != nullptr)
        ui.startBtn->setEnabled(canPreview);

    if (ui.stopBtn != nullptr)
        ui.stopBtn->setEnabled(state == CameraState::Streaming);
}
