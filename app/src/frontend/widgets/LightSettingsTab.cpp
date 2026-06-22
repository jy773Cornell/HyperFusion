// Light / lighthouse settings tab (DAQ connection and lamp controls).
// MainWindow method definitions extracted from MainWindow.cpp for clarity.
#include "frontend/controllers/LightPanelController.hpp"
#include "frontend/widgets/MainWindow.hpp"
#include "backend/LighthouseTypes.hpp"
#include "backend/LighthouseWorker.hpp"
#include "frontend/widgets/IntensityBarWidget.hpp"

#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
QWidget *MainWindow::createLightSettingsTab()
{
    auto *page = new QWidget(this);
    auto *layout = new QVBoxLayout(page);

    auto *connBox = new QGroupBox(QStringLiteral("Connection"), page);
    lightConnectionBox_ = connBox;
    auto *daqForm = new QFormLayout(connBox);

    auto *deviceTreeLabel = new QLabel(
        QStringLiteral("Computer\n  DAS Component\n    USB-1208FS-Plus"),
        connBox);
    deviceTreeLabel->setStyleSheet(QStringLiteral("font-family: Consolas, 'Courier New', monospace;"));
    daqForm->addRow(QStringLiteral("Device"), deviceTreeLabel);

    auto *statusRow = new QWidget(connBox);
    auto *statusLayout = new QHBoxLayout(statusRow);
    statusLayout->setContentsMargins(0, 0, 0, 0);
    lightDaqStatusIndicator_ = new QLabel(statusRow);
    lightDaqStatusIndicator_->setFixedSize(14, 14);
    lightDaqStatusLabel_ = new QLabel(QStringLiteral("Disconnected"), statusRow);
    statusLayout->addWidget(lightDaqStatusIndicator_);
    statusLayout->addWidget(lightDaqStatusLabel_, 1);
    daqForm->addRow(QStringLiteral("Status"), statusRow);

    lightDaqInfoDisplay_ = new QPlainTextEdit(connBox);
    lightDaqInfoDisplay_->setReadOnly(true);
    lightDaqInfoDisplay_->setPlaceholderText(
        QStringLiteral("Refresh to scan for the DAQ device, then Connect."));
    lightDaqInfoDisplay_->setMinimumHeight(96);
    lightDaqInfoDisplay_->setMaximumHeight(140);
    lightDaqInfoDisplay_->setPlainText(QString::fromUtf8(lighthouseWiringDetailsText()));
    daqForm->addRow(QStringLiteral("Details"), lightDaqInfoDisplay_);

    lightRefreshBtn_ = new QPushButton(QStringLiteral("Refresh"), connBox);
    lightRefreshBtn_->setToolTip(QStringLiteral("Scan for USB-1208FS-Plus"));
    lightConnectBtn_ = new QPushButton(QStringLiteral("Connect"), connBox);
    lightDisconnectBtn_ = new QPushButton(QStringLiteral("Disconnect"), connBox);
    lightDisconnectBtn_->setEnabled(false);

    auto *buttonRow = new QWidget(connBox);
    auto *buttonLayout = new QHBoxLayout(buttonRow);
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->addWidget(lightRefreshBtn_);
    buttonLayout->addWidget(lightConnectBtn_);
    buttonLayout->addWidget(lightDisconnectBtn_);
    buttonLayout->addStretch(1);
    daqForm->addRow(QStringLiteral(""), buttonRow);

    connect(lightRefreshBtn_, &QPushButton::clicked, this, [this]() {
        if (lighthouseWorker() == nullptr)
            return;
        appendLog(QStringLiteral("Light: scanning for USB-1208FS-Plus\u2026"));
        lighthouseWorker()->requestScan();
    });
    connect(lightConnectBtn_, &QPushButton::clicked, this, [this]() {
        if (lighthouseWorker() == nullptr)
            return;

        appendLog(QStringLiteral("Light: connecting to USB-1208FS-Plus\u2026"));
        lighthouseWorker()->requestConnect(lightPanel()->buildConnectDefaults());
    });
    connect(lightDisconnectBtn_, &QPushButton::clicked, this, [this]() {
        if (lighthouseWorker() == nullptr)
            return;
        appendLog(QStringLiteral("Light: turning off outputs and disconnecting\u2026"));
        lighthouseWorker()->requestDisconnect();
    });

    lightLightingBox_ = new QGroupBox(QStringLiteral("Lighthouses"), page);
    auto *lightingBoxLayout = new QVBoxLayout(lightLightingBox_);

    const QStringList lighthouseNames = {
        QStringLiteral("Reflectance 1"),
        QStringLiteral("Reflectance 2"),
        QStringLiteral("Transmittance 1"),
        QStringLiteral("Transmittance 2"),
    };

    for (int rowIndex = 0; rowIndex < lighthouseNames.size(); ++rowIndex)
    {
        auto &rowUi = lighthouseRows_[static_cast<std::size_t>(rowIndex)];
        auto *rowWidget = new QWidget(lightLightingBox_);
        auto *rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(6);

        rowUi.nameLabel = new QLabel(lighthouseNames.at(rowIndex), rowWidget);
        rowUi.nameLabel->setMinimumWidth(96);
        rowUi.nameLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);

        auto *statusWidget = new QWidget(rowWidget);
        auto *statusLayout = new QHBoxLayout(statusWidget);
        statusLayout->setContentsMargins(0, 0, 0, 0);
        statusLayout->setSpacing(4);

        rowUi.powerIndicator = new QLabel(statusWidget);
        rowUi.powerIndicator->setFixedSize(14, 14);
        rowUi.powerIndicator->setToolTip(
            tr("DC950 controller power monitor (AI CH%1, +5V alive signal)").arg(rowIndex));
        rowUi.powerStatusLabel = new QLabel(QStringLiteral("\u2014"), statusWidget);
        rowUi.powerStatusLabel->setFixedWidth(38);
        rowUi.powerStatusLabel->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
        rowUi.powerStatusLabel->setToolTip(
            tr("Up time while controller is alive and lamp is on; otherwise Off"));
        statusLayout->addWidget(rowUi.powerIndicator);
        statusLayout->addWidget(rowUi.powerStatusLabel);

        rowUi.bar = new ui::IntensityBarWidget(rowWidget);
        rowUi.onOffSwitch = new QCheckBox(QStringLiteral("On"), rowWidget);
        rowUi.onOffSwitch->setToolTip(tr("Lamp on/off (relay control)"));

        rowLayout->addWidget(rowUi.nameLabel);
        rowLayout->addWidget(statusWidget);
        rowLayout->addWidget(rowUi.bar, 1);
        rowLayout->addWidget(rowUi.onOffSwitch);
        lightingBoxLayout->addWidget(rowWidget);

        connect(rowUi.bar, &ui::IntensityBarWidget::percentChanged, this, [this, rowIndex](const int percent) {
            lightPanel()->setRowIntensity(rowIndex, percent);
            if (lighthouseWorker() != nullptr && isLighthouseSessionActive())
                lighthouseWorker()->requestSetGroupIntensity(lighthouseGroupForRowIndex(rowIndex), percent);
        });

        connect(rowUi.onOffSwitch, &QCheckBox::toggled, this, [this, rowIndex](const bool enabled) {
            if (lighthouseWorker() != nullptr && isLighthouseSessionActive())
                lighthouseWorker()->requestSetLampOn(static_cast<LighthouseLamp>(rowIndex), enabled);
            lightPanel()->updateLampUptimeDisplay();
        });
    }

    layout->addWidget(connBox);
    layout->addWidget(lightLightingBox_);
    layout->addStretch(1);
    return page;
}
