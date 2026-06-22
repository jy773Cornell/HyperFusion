// UR3e robot settings tab (placeholder connection and RGB pose capture).
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/widgets/MainWindow.hpp"
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>
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
        appendLog(QString("UR3e: MoveL requested (TCP pose %1,%2,%3 / %4,%5,%6) \u2014 not implemented")
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
        appendLog(QString("UR3e: arm RGB pose set (poses=%1, dwell=%2ms) \u2014 not implemented")
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
    ur3eSettingsPage_ = page;
    return page;
}
