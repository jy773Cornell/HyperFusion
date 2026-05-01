#include "MainWindow.hpp"

#include <QComboBox>
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
#include <QWidget>

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

    appendLog("HyperFusion UI initialized");
}

QWidget *MainWindow::createStreamTabsPanel()
{
    auto *panel = new QWidget(this);
    auto *layout = new QVBoxLayout(panel);

    auto *tabs = new QTabWidget(panel);
    tabs->addTab(createStreamTabPage("FX10e"), "FX10e");
    tabs->addTab(createStreamTabPage("SWIR"), "SWIR");
    tabs->addTab(createDualSensorStackTab(), "Dual");
    tabs->addTab(createRgbUr3eStreamTab(), "UR3e");

    layout->addWidget(tabs, 1);
    return panel;
}

QWidget *MainWindow::createDualSensorStackTab()
{
    auto *tab = new QWidget(this);
    auto *outer = new QVBoxLayout(tab);
    outer->setContentsMargins(8, 8, 8, 8);
    outer->setSpacing(10);

    auto *fxRow = new QGroupBox("FX10e", tab);
    auto *fxRowLayout = new QHBoxLayout(fxRow);
    QLabel *fxDetectorLabel = nullptr;
    QLabel *fxWaterfallLabel = nullptr;
    fxRowLayout->addWidget(createPreviewPane("Detector", fxDetectorLabel), 1);
    fxRowLayout->addWidget(createPreviewPane("Waterfall", fxWaterfallLabel), 1);
    fxDetectorLabel->setText("FX10e detector stream (disconnected)");
    fxWaterfallLabel->setText("FX10e waterfall stream (disconnected)");

    auto *swirRow = new QGroupBox("SWIR", tab);
    auto *swirRowLayout = new QHBoxLayout(swirRow);
    QLabel *swirDetectorLabel = nullptr;
    QLabel *swirWaterfallLabel = nullptr;
    swirRowLayout->addWidget(createPreviewPane("Detector", swirDetectorLabel), 1);
    swirRowLayout->addWidget(createPreviewPane("Waterfall", swirWaterfallLabel), 1);
    swirDetectorLabel->setText("SWIR detector stream (disconnected)");
    swirWaterfallLabel->setText("SWIR waterfall stream (disconnected)");

    outer->addWidget(fxRow, 1);
    outer->addWidget(swirRow, 1);
    return tab;
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

    auto *fx10eBox = new QGroupBox("FX10e", page);
    auto *fxForm = new QFormLayout(fx10eBox);
    auto *fxExposure = new QDoubleSpinBox(fx10eBox);
    fxExposure->setRange(0.001, 5.0);
    fxExposure->setDecimals(4);
    fxExposure->setSingleStep(0.001);
    fxExposure->setValue(0.015);
    fxExposure->setSuffix(" s");
    auto *fxFrameRate = new QDoubleSpinBox(fx10eBox);
    fxFrameRate->setRange(1.0, 500.0);
    fxFrameRate->setValue(120.0);
    fxFrameRate->setSuffix(" fps");
    auto *fxTrigger = new QComboBox(fx10eBox);
    fxTrigger->addItems({"Internal", "External"});
    auto *fxConnect = new QPushButton("Connect", fx10eBox);
    auto *fxApply = new QPushButton("Apply settings", fx10eBox);
    auto *fxStart = new QPushButton("Start preview", fx10eBox);
    fxStart->setEnabled(false);
    fxForm->addRow("Exposure", fxExposure);
    fxForm->addRow("Frame rate", fxFrameRate);
    fxForm->addRow("Trigger mode", fxTrigger);
    fxForm->addRow("", fxConnect);
    fxForm->addRow("", fxApply);
    fxForm->addRow("", fxStart);

    connect(fxConnect, &QPushButton::clicked, this, [this]() {
        appendLog("FX10e: connect requested (not implemented)");
    });
    connect(fxApply, &QPushButton::clicked, this, [this, fxExposure, fxFrameRate, fxTrigger]() {
        appendLog(QString("FX10e: apply settings (exposure=%1s, fps=%2, trigger=%3)")
                      .arg(fxExposure->value(), 0, 'f', 4)
                      .arg(fxFrameRate->value(), 0, 'f', 1)
                      .arg(fxTrigger->currentText()));
    });
    connect(fxStart, &QPushButton::clicked, this, [this]() {
        appendLog("FX10e: start preview requested (not implemented)");
    });

    auto *swirBox = new QGroupBox("SWIR", page);
    auto *swirForm = new QFormLayout(swirBox);
    auto *swirExposure = new QDoubleSpinBox(swirBox);
    swirExposure->setRange(0.001, 5.0);
    swirExposure->setDecimals(4);
    swirExposure->setSingleStep(0.001);
    swirExposure->setValue(0.020);
    swirExposure->setSuffix(" s");
    auto *swirFrameRate = new QDoubleSpinBox(swirBox);
    swirFrameRate->setRange(1.0, 500.0);
    swirFrameRate->setValue(100.0);
    swirFrameRate->setSuffix(" fps");
    auto *swirTrigger = new QComboBox(swirBox);
    swirTrigger->addItems({"Internal", "External"});
    auto *swirConnect = new QPushButton("Connect", swirBox);
    auto *swirApply = new QPushButton("Apply settings", swirBox);
    auto *swirStart = new QPushButton("Start preview", swirBox);
    swirStart->setEnabled(false);
    swirForm->addRow("Exposure", swirExposure);
    swirForm->addRow("Frame rate", swirFrameRate);
    swirForm->addRow("Trigger mode", swirTrigger);
    swirForm->addRow("", swirConnect);
    swirForm->addRow("", swirApply);
    swirForm->addRow("", swirStart);

    connect(swirConnect, &QPushButton::clicked, this, [this]() {
        appendLog("SWIR: connect requested (not implemented)");
    });
    connect(swirApply, &QPushButton::clicked, this, [this, swirExposure, swirFrameRate, swirTrigger]() {
        appendLog(QString("SWIR: apply settings (exposure=%1s, fps=%2, trigger=%3)")
                      .arg(swirExposure->value(), 0, 'f', 4)
                      .arg(swirFrameRate->value(), 0, 'f', 1)
                      .arg(swirTrigger->currentText()));
    });
    connect(swirStart, &QPushButton::clicked, this, [this]() {
        appendLog("SWIR: start preview requested (not implemented)");
    });

    auto *dualBox = new QGroupBox("Dual camera session", page);
    auto *dualLayout = new QVBoxLayout(dualBox);
    auto *armBoth = new QPushButton("Arm both", dualBox);
    auto *startBoth = new QPushButton("Start dual preview", dualBox);
    auto *stopBoth = new QPushButton("Stop both", dualBox);
    startBoth->setEnabled(false);
    stopBoth->setEnabled(false);
    dualLayout->addWidget(armBoth);
    dualLayout->addWidget(startBoth);
    dualLayout->addWidget(stopBoth);

    connect(armBoth, &QPushButton::clicked, this, [this]() {
        appendLog("Cameras: arm both requested (not implemented)");
    });
    connect(startBoth, &QPushButton::clicked, this, [this]() {
        appendLog("Cameras: start dual preview requested (not implemented)");
    });
    connect(stopBoth, &QPushButton::clicked, this, [this]() {
        appendLog("Cameras: stop both requested (not implemented)");
    });

    layout->addWidget(fx10eBox);
    layout->addWidget(swirBox);
    layout->addWidget(dualBox);
    layout->addStretch();
    return page;
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
