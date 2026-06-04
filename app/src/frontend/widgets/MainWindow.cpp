// Qt main window: settings tabs, stream previews, camera controls, and application log.
// Hardware access goes through CameraCoordinator; this file is UI layout and wiring only.
#include "frontend/widgets/MainWindow.hpp"

#include "adapters/lumo/LumoCamera.hpp"
#include "adapters/lumo/Swir3NiCamera.hpp"
#include "adapters/zaber/ZaberStageController.hpp"
#include "adapters/zaber/ZaberStageProfile.hpp"
#include "adapters/lumo/CalpackBandCatalog.hpp"
#include "backend/StageWorker.hpp"
#include "backend/CameraCoordinator.hpp"
#include "frontend/widgets/DetectorCrosshairWidget.hpp"
#include "frontend/widgets/ProfilePlotWidget.hpp"
#include "frontend/processing/ProfileProcessor.hpp"
#include "frontend/widgets/StageAxisWidget.hpp"

#include "frontend/settings/AppSettingsStore.hpp"
#include "backend/HyperspectralRawDumper.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

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
        if (topology.maxSpeedMmPerSec > 0.0)
            text += QStringLiteral("  Max speed: %1 mm/s\n").arg(topology.maxSpeedMmPerSec, 0, 'f', 0);
        if (topology.lockstepEnabled)
        {
            text += QStringLiteral("  Lockstep group %1: axis %2 (primary), axis %3\n")
                        .arg(topology.lockstepGroupId)
                        .arg(topology.lockstepPrimaryAxis)
                        .arg(topology.lockstepSecondaryAxis);
            text += QStringLiteral("  Lockstep offset (axis %1): %2 mm\n")
                        .arg(topology.lockstepSecondaryAxis)
                        .arg(topology.lockstepSecondaryOffsetMm, 0, 'f', 0);
            text += QStringLiteral("  Motion: command primary axis %1 only\n")
                        .arg(topology.lockstepPrimaryAxis);
        }
        if (topology.axesHomed)
            text += QStringLiteral("  Homing: lockstep localized (+%1 mm, home sensor)\n")
                        .arg(zaber_stage::kHomingLocalizationPremoveMm, 0, 'f', 0);
    }

    return text.trimmed();
}

QIcon makeHomeIcon(const int size = 22)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QColor fill(70, 70, 70);
    const QPen outline(QColor(40, 40, 40), 1.2);
    painter.setPen(outline);
    painter.setBrush(fill);

    const int pad = 2;
    const int roofPeakX = size / 2;
    const int roofPeakY = pad;
    const int roofLeftX = pad + 1;
    const int roofRightX = size - pad - 1;
    const int roofBaseY = size / 2 - 1;
    const QPolygon roof{{roofPeakX, roofPeakY}, {roofLeftX, roofBaseY}, {roofRightX, roofBaseY}};
    painter.drawPolygon(roof);

    const QRect body(pad + 3, roofBaseY, size - 2 * (pad + 3), size - roofBaseY - pad);
    painter.drawRect(body);

    painter.setBrush(QColor(230, 230, 230));
    painter.setPen(Qt::NoPen);
    painter.drawRect(body.center().x() - 2, body.bottom() - 5, 4, 5);

    return QIcon(pixmap);
}

QToolButton *makeStageToolButton(QWidget *parent, const QStyle::StandardPixmap icon, const QString &tooltip)
{
    auto *button = new QToolButton(parent);
    button->setIcon(parent->style()->standardIcon(icon));
    button->setToolTip(tooltip);
    button->setAutoRaise(true);
    button->setIconSize(QSize(28, 28));
    button->setMinimumSize(44, 44);
    return button;
}

QIcon makeRecorderStopIcon(const int size = 14)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(30, 30, 30));
    painter.drawRect(2, 2, size - 4, size - 4);
    return QIcon(pixmap);
}

QIcon makeRecorderPreviewIcon(const int size = 14)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(70, 170, 85));
    const QPolygon triangle{{3, 2}, {3, size - 2}, {size - 2, size / 2}};
    painter.drawPolygon(triangle);
    return QIcon(pixmap);
}

QIcon makeRecorderRecordIcon(const int size = 14)
{
    QPixmap pixmap(size, size);
    pixmap.fill(Qt::transparent);

    QPainter painter(&pixmap);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(220, 45, 45));
    painter.drawEllipse(2, 2, size - 4, size - 4);
    return QIcon(pixmap);
}

QPushButton *makeCaptureWhiteButton(QWidget *parent, const QString &label, const QIcon &icon = {})
{
    auto *button = new QPushButton(label, parent);
    if (!icon.isNull())
    {
        button->setIcon(icon);
        button->setIconSize(QSize(14, 14));
    }
    button->setMinimumHeight(36);
    button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    button->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #222222;"
        "  border: 1px solid #b0b0b0;"
        "  border-radius: 2px;"
        "  padding: 4px 12px;"
        "  min-width: 48px;"
        "}"
        "QPushButton:hover { background-color: #f0f0f0; }"
        "QPushButton:pressed { background-color: #e0e0e0; }"
        "QPushButton:disabled { color: #999999; background-color: #f5f5f5; }"));
    return button;
}

QPushButton *makeCaptureCompactWhiteButton(QWidget *parent, const QString &label)
{
    auto *button = makeCaptureWhiteButton(parent, label);
    button->setMinimumHeight(22);
    button->setMaximumHeight(22);
    button->setFixedWidth(40);
    button->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #222222;"
        "  border: 1px solid #b0b0b0;"
        "  border-radius: 2px;"
        "  padding: 1px 6px;"
        "  min-height: 22px;"
        "  max-height: 22px;"
        "  min-width: 40px;"
        "  font-size: 11px;"
        "}"
        "QPushButton:hover { background-color: #f0f0f0; }"
        "QPushButton:pressed { background-color: #e0e0e0; }"
        "QPushButton:disabled { color: #999999; background-color: #f5f5f5; }"));
    return button;
}

QPushButton *makeRecorderButton(QWidget *parent, const QIcon &icon, const QString &label)
{
    auto *button = makeCaptureWhiteButton(parent, label, icon);
    button->setIconSize(QSize(12, 12));
    button->setMinimumHeight(26);
    button->setMaximumHeight(26);
    button->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    button->setStyleSheet(QStringLiteral(
        "QPushButton {"
        "  background-color: #ffffff;"
        "  color: #222222;"
        "  border: 1px solid #b0b0b0;"
        "  border-radius: 2px;"
        "  padding: 2px 8px;"
        "  min-height: 26px;"
        "  max-height: 26px;"
        "}"
        "QPushButton:hover { background-color: #f0f0f0; }"
        "QPushButton:pressed { background-color: #e0e0e0; }"
        "QPushButton:disabled { color: #999999; background-color: #f5f5f5; }"));
    return button;
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
    else if (index == kSettingsTabCapture)
    {
        updateCaptureCamerasList();
        updateCaptureCameraPositionRows();
        if (stageWorker_ != nullptr)
            updateCapturePositionControls(stageWorker_->currentState());
    }
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
    {
        const QString profileName =
            ui.deviceCombo != nullptr ? ui.deviceCombo->currentText() : QString();
        calpackPath = defaultCalibrationPackPathForProfile(profileName, ui.sensorKind);
    }

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

QString MainWindow::resolveBundledCalibrationPackPath(const QString &fileName)
{
    if (fileName.isEmpty())
        return {};

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

QString MainWindow::defaultFx10eCalibrationPackPath()
{
    return resolveBundledCalibrationPackPath(QString::fromLatin1(kFx10eCalibrationFileName));
}

QString MainWindow::defaultSwir3CalibrationPackPath()
{
    return resolveBundledCalibrationPackPath(QString::fromLatin1(kSwir3CalibrationFileName));
}

QString MainWindow::defaultCalibrationPackPathForProfile(const QString &profileName,
                                                       const LumoSensorKind sensorKind)
{
    if (profileName.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive))
        return defaultSwir3CalibrationPackPath();
    if (profileName.contains(QStringLiteral("FX10e"), Qt::CaseInsensitive)
        || profileName.contains(QStringLiteral("FX10"), Qt::CaseInsensitive))
        return defaultFx10eCalibrationPackPath();

    if (sensorKind == LumoSensorKind::Swir3Ni)
        return defaultSwir3CalibrationPackPath();
    return defaultFx10eCalibrationPackPath();
}

void MainWindow::syncCalibrationPackToSelectedProfile(LumoCameraUi &ui)
{
    if (ui.calibrationPackEdit == nullptr)
        return;

    const QString profileName =
        ui.deviceCombo != nullptr && ui.deviceCombo->currentIndex() >= 0 ? ui.deviceCombo->currentText()
                                                                         : QString();
    const QString path = defaultCalibrationPackPathForProfile(profileName, ui.sensorKind);
    if (path.isEmpty() || !QFileInfo::exists(path))
        return;

    setCalibrationPackDisplay(ui.calibrationPackEdit, path);
    refreshBandCombos(ui);
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

    loadPersistedUiSettings();
    refreshStageComPortList();

    settingsSaveTimer_ = new QTimer(this);
    settingsSaveTimer_->setSingleShot(true);
    settingsSaveTimer_->setInterval(400);
    connect(settingsSaveTimer_, &QTimer::timeout, this, &MainWindow::savePersistedUiSettings);
    connectPersistedSettingsAutosave();

    QTimer::singleShot(0, this, [this]() {
        refreshLumoDeviceLists();
        applyPersistedCameraProfilesAndBands();
    });

    setupStageWorker();
}

MainWindow::~MainWindow()
{
    stopCaptureRecorder();
    savePersistedUiSettings();
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

    {
        const QString initialCalpack =
            defaultCalibrationPackPathForProfile(QString(), sensorKind);
        if (!initialCalpack.isEmpty())
            setCalibrationPackDisplay(ui.calibrationPackEdit, initialCalpack);
    }

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
        const QString profileName =
            ui.deviceCombo != nullptr ? ui.deviceCombo->currentText() : QString();
        const QString profileDefault =
            defaultCalibrationPackPathForProfile(profileName, ui.sensorKind);
        const QString startDir =
            currentPath.isEmpty()
                ? (profileDefault.isEmpty() ? QDir::homePath()
                                            : QFileInfo(profileDefault).absolutePath())
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
            [this, &ui](const int) {
                updateCameraTabLabel(ui);
                syncCalibrationPackToSelectedProfile(ui);
                schedulePersistedUiSettingsSave();
            });

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
        if (ui.sensorKind == LumoSensorKind::Swir3Ni)
        {
            const std::string icdPath =
                LumoCamera::resolveNiImaqCameraFilePath(connectionSettings.niImaqCameraFile);
            const bool icdExists = QFileInfo::exists(QString::fromStdString(icdPath));
            if (camera1Ui_.state != CameraState::Disconnected
                && camera1Ui_.state != CameraState::Fault)
            {
                appendLog(QStringLiteral(
                    "SWIR3: FX10e is still connected — disconnect it before SWIR3 NI bring-up."));
            }
            appendLog(QStringLiteral("SWIR3 NI: Grabber=%1, Camera.Channel=%2, Scb=%3, ICD=%4 (exists=%5)")
                          .arg(QString::fromStdString(connectionSettings.niGrabberChannel),
                               connectionSettings.niCameraSerialPort.empty()
                                   ? QStringLiteral("(none)")
                                   : QString::fromStdString(connectionSettings.niCameraSerialPort),
                               connectionSettings.niScbSerialPort.empty()
                                   ? QStringLiteral("(none)")
                                   : QString::fromStdString(connectionSettings.niScbSerialPort),
                               QString::fromStdString(icdPath),
                               icdExists ? QStringLiteral("yes") : QStringLiteral("NO — rebuild app")));
            if (connectionSettings.niCameraSerialPort.empty())
            {
                appendLog(QStringLiteral(
                    "SWIR3: Camera.Channel unset. Working 1427 uses COM3 after autoconnect — "
                    "if 1433 has no camera COM in Device Manager, fix USB/FTDI; do not use Zaber COM4."));
            }
        }
        const QString grabberNote = ui.sensorKind == LumoSensorKind::Swir3Ni
                                        ? QStringLiteral("NI IMAQdx — stop Grab in NI MAX before connect")
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
    settings.acquisitionTimeoutMs =
        ui.sensorKind == LumoSensorKind::Swir3Ni ? 30000U : 5000U;
    if (ui.sensorKind == LumoSensorKind::Swir3Ni)
    {
        settings.niGrabberChannel = "img0";
        // SSP "SWIR3 with NI" default (same as working 1427 bench). NI MAX may label the device "Fenix SWIR"
        // but Lumo NiImaq.CameraFile should use the profile ICD, not the MAX display name.
        settings.niImaqCameraFile = "Specim_SWIR3.icd";
        // Working 1427 Lumo log: "Camera autoconnect found a camera in port: COM3". Set when that COM exists on this PC:
        // settings.niCameraSerialPort = "COM3";
        settings.niCameraSerialPort.clear();
        settings.niScbSerialPort.clear();
    }
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
    if (previousPort.isEmpty() || previousPort.startsWith(QLatin1Char('(')))
        previousPort = persistedStagePort_;

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
    const bool busy = state == StageState::Connecting || state == StageState::Homing;
    const bool motionActive = connected || busy;

    if (stagePortCombo_ != nullptr)
        stagePortCombo_->setEnabled(!connected && !busy);
    if (stageBaudCombo_ != nullptr)
        stageBaudCombo_->setEnabled(!connected && !busy);
    if (stageConnectBtn_ != nullptr)
        stageConnectBtn_->setEnabled(!connected && !busy);
    if (stageDisconnectBtn_ != nullptr)
        stageDisconnectBtn_->setEnabled(motionActive);
    if (stageControlBox_ != nullptr)
        stageControlBox_->setEnabled(connected);

    updateStageMotionControls(state);

    if (stagePositionTimer_ != nullptr)
    {
        if (connected || state == StageState::Homing)
            stagePositionTimer_->start();
        else
            stagePositionTimer_->stop();
    }

    if (!connected)
        updateStagePositionDisplay(0.0);

    updateCapturePositionControls(state);
}

void MainWindow::updateStageMotionControls(const StageState state)
{
    const bool connected = state == StageState::Connected;
    const bool captureScanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;
    const bool manualMotion = connected && !captureScanActive;

    if (stageHomeBtn_ != nullptr)
        stageHomeBtn_->setEnabled(manualMotion);
    if (stageToStartBtn_ != nullptr)
        stageToStartBtn_->setEnabled(manualMotion);
    if (stageBackBtn_ != nullptr)
        stageBackBtn_->setEnabled(manualMotion);
    if (stageForwardBtn_ != nullptr)
        stageForwardBtn_->setEnabled(manualMotion);
    if (stageToEndBtn_ != nullptr)
        stageToEndBtn_->setEnabled(manualMotion);
    if (stageAbsolutePositionSpin_ != nullptr)
        stageAbsolutePositionSpin_->setEnabled(manualMotion);
    if (stageAbsoluteMoveBtn_ != nullptr)
        stageAbsoluteMoveBtn_->setEnabled(manualMotion);
    if (stageStopBtn_ != nullptr)
        stageStopBtn_->setEnabled(connected);
}

void MainWindow::updateStagePositionDisplay(const double positionMm)
{
    if (stageAxisWidget_ != nullptr)
        stageAxisWidget_->setPositionMm(positionMm);
}

void MainWindow::pollStagePosition()
{
    if (stageWorker_ == nullptr || stagePositionPollInFlight_)
        return;

    const StageState state = stageWorker_->currentState();
    if (state != StageState::Connected && state != StageState::Homing)
        return;

    stagePositionPollInFlight_ = true;
    stageWorker_->requestPrimaryPosition([this](const double positionMm, const bool ok) {
        stagePositionPollInFlight_ = false;
        if (!ok)
            return;

        QMetaObject::invokeMethod(
            this,
            [this, positionMm]() { updateStagePositionDisplay(positionMm); },
            Qt::QueuedConnection);
    });
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

    if (state == StageState::Homing)
    {
        if (stageHomingKind_ == StageHomingKind::Localization)
        {
            appendLog(QString("Stage: initial homing (+%1 mm forward, then home sensor)…")
                          .arg(zaber_stage::kHomingLocalizationPremoveMm, 0, 'f', 0));
        }
        else if (stageHomingKind_ == StageHomingKind::Simple)
            appendLog("Stage: homing lockstep group (home sensor)…");
    }
    else if (state == StageState::Connected)
        appendLog("Stage: homed and ready");

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

    if (topology.axesHomed && stageHomingKind_ != StageHomingKind::None)
    {
        if (stageHomingKind_ == StageHomingKind::Localization)
            appendLog("Stage: initial localization homing complete");
        else
            appendLog("Stage: homing complete");

        stageHomingKind_ = StageHomingKind::None;
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
        stageHomingKind_ = StageHomingKind::Localization;
        stageWorker_->requestConnect(settings);
    });
    connect(stageDisconnectBtn_, &QPushButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;

        appendLog("Stage: disconnect requested");
        stageWorker_->requestDisconnect();
    });

    auto *deviceBox = new QGroupBox("Device", page);
    auto *deviceLayout = new QVBoxLayout(deviceBox);
    auto *deviceHint = new QLabel(
        QStringLiteral("Use Zaber Launcher to initialize or reconfigure the motors."),
        deviceBox);
    deviceHint->setWordWrap(true);
    stageDeviceDisplay_ = new QPlainTextEdit(deviceBox);
    stageDeviceDisplay_->setReadOnly(true);
    stageDeviceDisplay_->setPlaceholderText("Connect to discover the controller and axes.");
    stageDeviceDisplay_->setMinimumHeight(55);
    stageDeviceDisplay_->setMaximumHeight(75);
    deviceLayout->addWidget(deviceHint);
    deviceLayout->addWidget(stageDeviceDisplay_);

    stageControlBox_ = new QGroupBox("Control", page);
    stageControlBox_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    auto *controlLayout = new QVBoxLayout(stageControlBox_);
    controlLayout->setContentsMargins(6, 2, 6, 6);
    controlLayout->setSpacing(2);

    stageAxisWidget_ = new ui::StageAxisWidget(stageControlBox_);
    stageAxisWidget_->setTravelRangeMm(zaber_stage::kTravelMinimumMm, zaber_stage::kTravelLengthMm);

    auto *toolbarRow = new QHBoxLayout();
    toolbarRow->setSpacing(10);
    toolbarRow->setContentsMargins(0, 0, 0, 0);
    stageHomeBtn_ = new QToolButton(stageControlBox_);
    stageHomeBtn_->setIcon(makeHomeIcon());
    stageHomeBtn_->setIconSize(QSize(28, 28));
    stageHomeBtn_->setMinimumSize(44, 44);
    stageHomeBtn_->setToolTip(tr("Perform homing"));
    stageHomeBtn_->setAutoRaise(true);
    stageToStartBtn_ =
        makeStageToolButton(stageControlBox_, QStyle::SP_MediaSkipBackward, tr("Move to beginning (0 mm)"));
    stageBackBtn_ = makeStageToolButton(stageControlBox_, QStyle::SP_MediaSeekBackward, tr("Move back (hold)"));
    stageStopBtn_ = makeStageToolButton(stageControlBox_, QStyle::SP_MediaStop, tr("Stop"));
    stageForwardBtn_ =
        makeStageToolButton(stageControlBox_, QStyle::SP_MediaSeekForward, tr("Move forward (hold)"));
    stageToEndBtn_ = makeStageToolButton(stageControlBox_,
                                         QStyle::SP_MediaSkipForward,
                                         tr("Move to end (%1 mm)").arg(zaber_stage::kTravelLengthMm, 0, 'f', 0));

    toolbarRow->addWidget(stageHomeBtn_);
    toolbarRow->addWidget(stageToStartBtn_);
    toolbarRow->addSpacing(8);
    toolbarRow->addWidget(stageBackBtn_);
    toolbarRow->addWidget(stageStopBtn_);
    toolbarRow->addWidget(stageForwardBtn_);
    toolbarRow->addSpacing(8);
    toolbarRow->addWidget(stageToEndBtn_);
    toolbarRow->addStretch(1);

    auto *absoluteRow = new QHBoxLayout();
    auto *absoluteLabel = new QLabel(QStringLiteral("Move to absolute position"), stageControlBox_);
    stageAbsolutePositionSpin_ = new QDoubleSpinBox(stageControlBox_);
    stageAbsolutePositionSpin_->setRange(zaber_stage::kTravelMinimumMm, zaber_stage::kTravelLengthMm);
    stageAbsolutePositionSpin_->setDecimals(1);
    stageAbsolutePositionSpin_->setSingleStep(1.0);
    stageAbsolutePositionSpin_->setValue(0.0);
    stageAbsolutePositionSpin_->setEnabled(false);
    stageAbsoluteMoveBtn_ =
        makeStageToolButton(stageControlBox_, QStyle::SP_MediaPlay, tr("Move to absolute position"));
    absoluteRow->addWidget(absoluteLabel);
    absoluteRow->addWidget(stageAbsolutePositionSpin_);
    absoluteRow->addWidget(stageAbsoluteMoveBtn_);

    controlLayout->addLayout(toolbarRow);
    controlLayout->addWidget(stageAxisWidget_);
    updateStagePositionDisplay(0.0);
    controlLayout->addLayout(absoluteRow);

    stagePositionTimer_ = new QTimer(this);
    stagePositionTimer_->setInterval(50);
    connect(stagePositionTimer_, &QTimer::timeout, this, &MainWindow::pollStagePosition);

    connect(stageHomeBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageHomingKind_ = StageHomingKind::Simple;
        stageWorker_->requestHome();
    });
    connect(stageToStartBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageWorker_->requestMoveAbsoluteMm(zaber_stage::kTravelMinimumMm);
    });
    connect(stageBackBtn_, &QToolButton::pressed, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageWorker_->requestMoveVelocityMm(-zaber_stage::kMaxSpeedMmPerSec);
    });
    connect(stageBackBtn_, &QToolButton::released, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageWorker_->requestStopMotion();
    });
    connect(stageStopBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        appendLog("Stage: stop requested");
        stageWorker_->requestStopMotion();
    });
    connect(stageForwardBtn_, &QToolButton::pressed, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageWorker_->requestMoveVelocityMm(zaber_stage::kMaxSpeedMmPerSec);
    });
    connect(stageForwardBtn_, &QToolButton::released, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageWorker_->requestStopMotion();
    });
    connect(stageToEndBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr)
            return;
        stageWorker_->requestMoveAbsoluteMm(zaber_stage::kTravelLengthMm);
    });
    connect(stageAbsoluteMoveBtn_, &QToolButton::clicked, this, [this]() {
        if (stageWorker_ == nullptr || stageAbsolutePositionSpin_ == nullptr)
            return;

        const double targetMm = stageAbsolutePositionSpin_->value();
        appendLog(QString("Stage: move to absolute position %1 mm").arg(targetMm, 0, 'f', 1));
        stageWorker_->requestMoveAbsoluteMm(targetMm);
    });

    layout->addWidget(connBox);
    layout->addWidget(deviceBox);
    layout->addWidget(stageControlBox_);
    layout->addStretch(1);
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

    layout->addWidget(connBox);
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

    auto *recorderBox = new QGroupBox("Recorder", page);
    auto *recorderLayout = new QVBoxLayout(recorderBox);
    recorderLayout->setContentsMargins(6, 4, 6, 6);

    auto *recorderButtonLayout = new QHBoxLayout();
    recorderButtonLayout->setContentsMargins(0, 0, 0, 0);
    recorderButtonLayout->setSpacing(8);

    captureRecorderStopBtn_ =
        makeRecorderButton(recorderBox, makeRecorderStopIcon(), QStringLiteral("Stop"));
    captureRecorderPreviewBtn_ =
        makeRecorderButton(recorderBox, makeRecorderPreviewIcon(), QStringLiteral("Preview"));
    captureRecorderRecordBtn_ =
        makeRecorderButton(recorderBox, makeRecorderRecordIcon(), QStringLiteral("Record"));
    captureRecorderStopBtn_->setToolTip(tr("Stop preview or recording"));
    captureRecorderPreviewBtn_->setToolTip(tr("Preview scan without saving"));
    captureRecorderRecordBtn_->setToolTip(tr("Record scan to SSD"));

    recorderButtonLayout->addWidget(captureRecorderStopBtn_, 1);
    recorderButtonLayout->addWidget(captureRecorderPreviewBtn_, 1);
    recorderButtonLayout->addWidget(captureRecorderRecordBtn_, 1);
    recorderLayout->addLayout(recorderButtonLayout);

    captureScanTimer_ = new QTimer(this);
    captureScanTimer_->setSingleShot(true);
    connect(captureScanTimer_, &QTimer::timeout, this, &MainWindow::finishCaptureScan);

    connect(captureRecorderStopBtn_, &QPushButton::clicked, this, &MainWindow::stopCaptureRecorder);
    connect(captureRecorderPreviewBtn_, &QPushButton::clicked, this, &MainWindow::startCapturePreview);
    connect(captureRecorderRecordBtn_, &QPushButton::clicked, this, &MainWindow::startCaptureRecord);

    captureCamerasBox_ = new QGroupBox("Cameras", page);
    auto *camerasLayout = new QVBoxLayout(captureCamerasBox_);
    camerasLayout->setContentsMargins(6, 4, 6, 6);
    camerasLayout->setSpacing(4);
    captureCamerasEmptyLabel_ =
        new QLabel(QStringLiteral("No cameras connected."), captureCamerasBox_);
    captureCamerasEmptyLabel_->setWordWrap(true);
    captureCamera1Check_ = new QCheckBox(captureCamerasBox_);
    captureCamera2Check_ = new QCheckBox(captureCamerasBox_);
    captureCamera1Check_->hide();
    captureCamera2Check_->hide();
    const auto refreshRecorder = [this]() { updateCaptureRecorderControls(); };
    connect(captureCamera1Check_, &QCheckBox::toggled, this, refreshRecorder);
    connect(captureCamera2Check_, &QCheckBox::toggled, this, refreshRecorder);
    camerasLayout->addWidget(captureCamerasEmptyLabel_);
    camerasLayout->addWidget(captureCamera1Check_);
    camerasLayout->addWidget(captureCamera2Check_);

    auto *positionBox = new QGroupBox("Position", page);
    auto *positionLayout = new QVBoxLayout(positionBox);
    positionLayout->setContentsMargins(6, 4, 6, 6);
    positionLayout->setSpacing(6);

    captureStageConnectedCheck_ =
        new QCheckBox(QStringLiteral("HyperFusion linear stage connected"), positionBox);
    captureStageConnectedCheck_->setEnabled(false);
    positionLayout->addWidget(captureStageConnectedCheck_);

    capturePositionContent_ = new QWidget(positionBox);
    auto *positionContentLayout = new QVBoxLayout(capturePositionContent_);
    positionContentLayout->setContentsMargins(0, 0, 0, 0);
    positionContentLayout->setSpacing(6);

    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        auto *rowWidget = new QWidget(capturePositionContent_);
        auto *rowLayout = new QHBoxLayout(rowWidget);
        rowLayout->setContentsMargins(0, 0, 0, 0);
        rowLayout->setSpacing(6);

        auto *rowLabel = new QLabel(rowWidget);
        rowLabel->setMinimumWidth(120);
        captureCameraPositionSpins_[cameraIndex] = new QDoubleSpinBox(rowWidget);
        captureCameraPositionSpins_[cameraIndex]->setRange(zaber_stage::kTravelMinimumMm,
                                                            zaber_stage::kTravelLengthMm);
        captureCameraPositionSpins_[cameraIndex]->setDecimals(2);
        captureCameraPositionSpins_[cameraIndex]->setSingleStep(1.0);
        captureCameraPositionSpins_[cameraIndex]->setSuffix(QStringLiteral(" mm"));
        captureCameraPositionSpins_[cameraIndex]->setValue(0.0);

        auto *goBtn = makeCaptureCompactWhiteButton(rowWidget, QStringLiteral("Go"));
        rowLayout->addWidget(rowLabel);
        rowLayout->addWidget(captureCameraPositionSpins_[cameraIndex], 1);
        rowLayout->addWidget(goBtn);
        positionContentLayout->addWidget(rowWidget);
        captureCameraPositionRows_[cameraIndex] = rowWidget;
        rowWidget->hide();

        connect(goBtn, &QPushButton::clicked, this, [this, cameraIndex]() {
            LumoCameraUi &cameraUi = cameraIndex == 0 ? camera1Ui_ : camera2Ui_;
            if (stageWorker_ == nullptr || captureCameraPositionSpins_[cameraIndex] == nullptr)
                return;
            if (stageWorker_->currentState() != StageState::Connected)
            {
                appendLog(QString("Capture: stage not connected — cannot go to %1 position")
                              .arg(profileTabNameForUi(cameraUi)));
                return;
            }

            const QString label = QStringLiteral("%1 position").arg(profileTabNameForUi(cameraUi));
            const double targetMm = captureCameraPositionSpins_[cameraIndex]->value();
            appendLog(QString("Capture: go to %1 (%2 mm)").arg(label).arg(targetMm, 0, 'f', 2));
            stageWorker_->requestMoveAbsoluteMm(targetMm);
        });
    }

    auto *targetLengthRowLayout = new QHBoxLayout();
    targetLengthRowLayout->setSpacing(6);
    auto *targetLengthLabel = new QLabel(QStringLiteral("Target length"), capturePositionContent_);
    targetLengthLabel->setMinimumWidth(120);
    captureTargetLengthSpin_ = new QDoubleSpinBox(capturePositionContent_);
    captureTargetLengthSpin_->setRange(0.0, 500.0);
    captureTargetLengthSpin_->setDecimals(2);
    captureTargetLengthSpin_->setSingleStep(1.0);
    captureTargetLengthSpin_->setSuffix(QStringLiteral(" mm"));
    captureTargetLengthSpin_->setValue(125.0);
    targetLengthRowLayout->addWidget(targetLengthLabel);
    targetLengthRowLayout->addWidget(captureTargetLengthSpin_, 1);
    targetLengthRowLayout->addSpacing(40);
    positionContentLayout->addLayout(targetLengthRowLayout);

    auto *scanningSpeedRowLayout = new QHBoxLayout();
    scanningSpeedRowLayout->setSpacing(6);
    auto *scanningSpeedLabel =
        new QLabel(QStringLiteral("Scanning speed"), capturePositionContent_);
    scanningSpeedLabel->setMinimumWidth(120);
    captureScanningSpeedSpin_ = new QDoubleSpinBox(capturePositionContent_);
    captureScanningSpeedSpin_->setRange(0.0, zaber_stage::kMaxSpeedMmPerSec);
    captureScanningSpeedSpin_->setDecimals(1);
    captureScanningSpeedSpin_->setSingleStep(1.0);
    captureScanningSpeedSpin_->setSuffix(QStringLiteral(" mm/s"));
    captureScanningSpeedSpin_->setValue(25.0);
    scanningSpeedRowLayout->addWidget(scanningSpeedLabel);
    scanningSpeedRowLayout->addWidget(captureScanningSpeedSpin_, 1);
    scanningSpeedRowLayout->addSpacing(40);
    positionContentLayout->addLayout(scanningSpeedRowLayout);

    positionLayout->addWidget(capturePositionContent_);
    updateCapturePositionControls(stageWorker_ != nullptr ? stageWorker_->currentState()
                                                          : StageState::Disconnected);

    auto *metadataBox = new QGroupBox("Metadata", page);
    auto *metadataForm = new QFormLayout(metadataBox);
    captureDatasetEdit_ = new QLineEdit(metadataBox);
    captureDatasetEdit_->setPlaceholderText("Dataset name");

    auto *saveFolderRow = new QWidget(metadataBox);
    auto *saveFolderLayout = new QHBoxLayout(saveFolderRow);
    saveFolderLayout->setContentsMargins(0, 0, 0, 0);
    saveFolderLayout->setSpacing(6);
    captureSaveFolderEdit_ = new QLineEdit(saveFolderRow);
    captureSaveFolderEdit_->setPlaceholderText("Output location");
    captureSaveFolderBrowseBtn_ = new QPushButton(QStringLiteral("Browse…"), saveFolderRow);
    captureSaveFolderBrowseBtn_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    captureSaveFolderBrowseBtn_->setFixedWidth(72);
    saveFolderLayout->addWidget(captureSaveFolderEdit_, 1);
    saveFolderLayout->addWidget(captureSaveFolderBrowseBtn_);

    captureOperatorEdit_ = new QLineEdit(metadataBox);
    captureOperatorEdit_->setPlaceholderText("Operator name");
    captureDescriptionEdit_ = new QPlainTextEdit(metadataBox);
    captureDescriptionEdit_->setPlaceholderText("Sample description, notes, or experiment details");
    captureDescriptionEdit_->setTabChangesFocus(true);
    captureDescriptionEdit_->setMinimumHeight(72);
    captureDescriptionEdit_->setMaximumHeight(120);
    metadataForm->addRow("Dataset", captureDatasetEdit_);
    metadataForm->addRow("Save folder", saveFolderRow);
    metadataForm->addRow("Operator", captureOperatorEdit_);
    metadataForm->addRow("Description", captureDescriptionEdit_);

    connect(captureSaveFolderBrowseBtn_, &QPushButton::clicked, this, [this]() {
        const QString startDir = captureSaveFolderEdit_->text().trimmed().isEmpty()
                                     ? QDir::homePath()
                                     : captureSaveFolderEdit_->text().trimmed();
        const QString path = QFileDialog::getExistingDirectory(
            this, tr("Select output location"), startDir);
        if (!path.isEmpty())
            captureSaveFolderEdit_->setText(QDir::toNativeSeparators(path));
        schedulePersistedUiSettingsSave();
        updateCaptureRecorderControls();
    });
    if (captureDatasetEdit_ != nullptr)
    {
        connect(captureDatasetEdit_, &QLineEdit::textChanged, this, [this]() {
            updateCaptureRecorderControls();
        });
    }
    if (captureSaveFolderEdit_ != nullptr)
    {
        connect(captureSaveFolderEdit_, &QLineEdit::textChanged, this, [this]() {
            updateCaptureRecorderControls();
        });
    }

    layout->addWidget(recorderBox);
    layout->addWidget(metadataBox);
    layout->addWidget(captureCamerasBox_);
    layout->addWidget(positionBox);
    layout->addStretch();
    updateCaptureCamerasList();
    updateCaptureCameraPositionRows();
    updateCaptureRecorderControls();
    return page;
}

void MainWindow::updateCapturePositionControls(const StageState state)
{
    const bool connected = state == StageState::Connected;

    if (captureStageConnectedCheck_ != nullptr)
        captureStageConnectedCheck_->setChecked(connected);

    if (capturePositionContent_ != nullptr)
        capturePositionContent_->setEnabled(connected);

    if (!connected && captureRecorderMode_ != CaptureRecorderMode::Idle)
        stopCaptureRecorder();

    updateCaptureRecorderControls();
}

void MainWindow::updateCaptureRecorderControls()
{
    const bool stageConnected =
        stageWorker_ != nullptr && stageWorker_->currentState() == StageState::Connected;
    const bool scanActive = captureRecorderMode_ != CaptureRecorderMode::Idle;

    const QString saveFolder =
        captureSaveFolderEdit_ != nullptr ? captureSaveFolderEdit_->text().trimmed() : QString();
    const QString dataset =
        captureDatasetEdit_ != nullptr ? captureDatasetEdit_->text().trimmed() : QString();

    QString recordBlockReason;
    if (dataset.isEmpty() || saveFolder.isEmpty())
    {
        QStringList missing;
        if (dataset.isEmpty())
            missing << tr("Dataset");
        if (saveFolder.isEmpty())
            missing << tr("Save folder");
        recordBlockReason = tr("Enter %1 in Metadata.").arg(missing.join(tr(" and ")));
    }
    else
    {
        QString streamingError;
        if (!selectedCaptureCameraStreaming(streamingError))
            recordBlockReason = streamingError;
    }

    const bool recordReady = recordBlockReason.isEmpty();

    if (captureRecorderPreviewBtn_ != nullptr)
    {
        captureRecorderPreviewBtn_->setEnabled(stageConnected && !scanActive);
        captureRecorderPreviewBtn_->setToolTip(
            stageConnected ? tr("Run a scan at the configured speed (no files saved)")
                           : tr("Connect the stage on the Stage tab to enable preview"));
    }

    if (captureRecorderRecordBtn_ != nullptr)
    {
        captureRecorderRecordBtn_->setEnabled(recordReady && !scanActive);
        if (!recordReady)
            captureRecorderRecordBtn_->setToolTip(recordBlockReason);
        else if (stageConnected)
            captureRecorderRecordBtn_->setToolTip(
                tr("Move to closest camera position, scan, and save .raw frames locally"));
        else
            captureRecorderRecordBtn_->setToolTip(
                tr("Save streaming .raw frames locally (stage not connected — press Stop when done)"));
    }

    if (captureRecorderStopBtn_ != nullptr)
        captureRecorderStopBtn_->setEnabled(scanActive);

    if (captureTargetLengthSpin_ != nullptr)
        captureTargetLengthSpin_->setEnabled(!scanActive);
    if (captureScanningSpeedSpin_ != nullptr)
        captureScanningSpeedSpin_->setEnabled(!scanActive);

    if (capturePositionContent_ != nullptr && stageConnected)
        capturePositionContent_->setEnabled(!scanActive);
}

bool MainWindow::buildCaptureScanPlan(double &distanceMm, double &speedMmPerSec, QString &errorMessage) const
{
    if (stageWorker_ == nullptr || stageWorker_->currentState() != StageState::Connected)
    {
        errorMessage = QStringLiteral("Stage is not connected.");
        return false;
    }

    if (captureTargetLengthSpin_ == nullptr || captureScanningSpeedSpin_ == nullptr)
    {
        errorMessage = QStringLiteral("Capture scan controls are not available.");
        return false;
    }

    distanceMm = captureTargetLengthSpin_->value();
    speedMmPerSec = captureScanningSpeedSpin_->value();

    if (distanceMm <= 0.0)
    {
        errorMessage = QStringLiteral("Target length must be greater than 0 mm.");
        return false;
    }

    if (speedMmPerSec <= 0.0)
    {
        errorMessage = QStringLiteral("Scanning speed must be greater than 0 mm/s.");
        return false;
    }

    if (speedMmPerSec > zaber_stage::kMaxSpeedMmPerSec)
    {
        errorMessage = QStringLiteral("Scanning speed exceeds the stage limit.");
        return false;
    }

    return true;
}

void MainWindow::startCapturePreview()
{
    if (captureRecorderMode_ != CaptureRecorderMode::Idle)
        return;

    double distanceMm = 0.0;
    double speedMmPerSec = 0.0;
    QString errorMessage;
    if (!buildCaptureScanPlan(distanceMm, speedMmPerSec, errorMessage))
    {
        appendLog(QStringLiteral("Capture preview: %1").arg(errorMessage));
        return;
    }

    const auto cameraSelected = [this]() {
        return (captureCamera1Check_ != nullptr && captureCamera1Check_->isVisible()
                && captureCamera1Check_->isChecked())
               || (captureCamera2Check_ != nullptr && captureCamera2Check_->isVisible()
                   && captureCamera2Check_->isChecked());
    };
    if (!cameraSelected())
        appendLog(QStringLiteral("Capture preview: no camera selected — stage scan will still run."));

    captureRecorderMode_ = CaptureRecorderMode::Preview;
    updateCaptureRecorderControls();

    appendLog(QStringLiteral("Capture preview: starting scan (%1 mm @ %2 mm/s, no save)…")
                  .arg(distanceMm, 0, 'f', 2)
                  .arg(speedMmPerSec, 0, 'f', 1));

    stageWorker_->requestPrimaryPosition([this, distanceMm, speedMmPerSec](const double positionMm,
                                                                           const bool ok) {
        QMetaObject::invokeMethod(
            this,
            [this, positionMm, ok, distanceMm, speedMmPerSec]() {
                if (captureRecorderMode_ != CaptureRecorderMode::Preview)
                    return;

                if (!ok)
                {
                    appendLog(QStringLiteral("Capture preview: could not read stage position."));
                    captureRecorderMode_ = CaptureRecorderMode::Idle;
                    updateCaptureRecorderControls();
                    return;
                }

                const double endMm = positionMm + distanceMm;
                if (endMm > zaber_stage::kTravelLengthMm)
                {
                    appendLog(QStringLiteral(
                        "Capture preview: scan would pass %1 mm (current %2 mm + %3 mm).")
                                  .arg(zaber_stage::kTravelLengthMm, 0, 'f', 0)
                                  .arg(positionMm, 0, 'f', 2)
                                  .arg(distanceMm, 0, 'f', 2));
                    captureRecorderMode_ = CaptureRecorderMode::Idle;
                    updateCaptureRecorderControls();
                    return;
                }

                stageWorker_->requestMoveRelativeMm(distanceMm, speedMmPerSec);

                const int durationMs =
                    static_cast<int>(std::ceil((distanceMm / speedMmPerSec) * 1000.0)) + 750;
                if (captureScanTimer_ != nullptr)
                    captureScanTimer_->start(std::max(durationMs, 500));
            },
            Qt::QueuedConnection);
    });
}

bool MainWindow::validateCaptureRecordMetadata(QString &errorMessage) const
{
    const QString saveFolder =
        captureSaveFolderEdit_ != nullptr ? captureSaveFolderEdit_->text().trimmed() : QString();
    const QString dataset =
        captureDatasetEdit_ != nullptr ? captureDatasetEdit_->text().trimmed() : QString();

    if (saveFolder.isEmpty() || dataset.isEmpty())
    {
        errorMessage = QStringLiteral("Set dataset name and save folder in Metadata.");
        return false;
    }

    return true;
}

bool MainWindow::selectedCaptureCameraIndices(std::vector<std::size_t> &cameraIndices) const
{
    cameraIndices.clear();

    const auto consider = [&cameraIndices](const QCheckBox *checkbox, const std::size_t index) {
        if (checkbox != nullptr && checkbox->isVisible() && checkbox->isChecked())
            cameraIndices.push_back(index);
    };

    consider(captureCamera1Check_, 0);
    consider(captureCamera2Check_, 1);
    return !cameraIndices.empty();
}

bool MainWindow::selectedCaptureCameraStreaming(QString &errorMessage) const
{
    std::vector<std::size_t> cameraIndices;
    if (!selectedCaptureCameraIndices(cameraIndices))
    {
        errorMessage = QStringLiteral("Select at least one connected camera in the Cameras list.");
        return false;
    }

    const LumoCameraUi *uis[] = {&camera1Ui_, &camera2Ui_};
    for (const std::size_t index : cameraIndices)
    {
        const LumoCameraUi &ui = *uis[index];
        if (ui.state != CameraState::Streaming && ui.state != CameraState::Armed
            && ui.state != CameraState::Configured && ui.state != CameraState::Initialized)
        {
            errorMessage = QStringLiteral("%1 is not streaming — connect and wait for preview first.")
                               .arg(profileTabNameForUi(ui));
            return false;
        }
    }

    return true;
}

double MainWindow::closestSelectedCaptureCameraPositionMm(bool *hasSelection) const
{
    if (hasSelection != nullptr)
        *hasSelection = false;

    std::vector<std::size_t> cameraIndices;
    if (!selectedCaptureCameraIndices(cameraIndices))
        return 0.0;

    double closestMm = zaber_stage::kTravelLengthMm;
    bool found = false;
    for (const std::size_t index : cameraIndices)
    {
        if (captureCameraPositionSpins_[index] == nullptr)
            continue;

        found = true;
        closestMm = std::min(closestMm, captureCameraPositionSpins_[index]->value());
    }

    if (hasSelection != nullptr)
        *hasSelection = found;

    return closestMm;
}

bool MainWindow::beginCaptureRawDumpSession(QString &errorMessage)
{
    QString metadataError;
    if (!validateCaptureRecordMetadata(metadataError))
    {
        errorMessage = metadataError;
        return false;
    }

    QString streamingError;
    if (!selectedCaptureCameraStreaming(streamingError))
    {
        errorMessage = streamingError;
        return false;
    }

    const QString saveFolder = captureSaveFolderEdit_->text().trimmed();
    const QString dataset = captureDatasetEdit_->text().trimmed();

    captureRawDumper_ = std::make_unique<HyperspectralRawDumper>();
    if (!captureRawDumper_->beginSession(saveFolder, dataset, &errorMessage))
    {
        captureRawDumper_.reset();
        return false;
    }

    return true;
}

void MainWindow::endCaptureRawDumpSession()
{
    if (captureRawDumper_ == nullptr)
        return;

    const RawDumpSessionSummary summary = captureRawDumper_->endSession();
    captureRawDumper_.reset();

    if (summary.sessionDirectory.isEmpty())
        return;

    appendLog(QStringLiteral("Capture record: saved to %1").arg(summary.sessionDirectory));
    for (auto it = summary.streams.cbegin(); it != summary.streams.cend(); ++it)
    {
        appendLog(QStringLiteral("  %1 — %2 frames (%3 bytes) → %4")
                      .arg(HyperspectralRawDumper::streamLabel(it.key()))
                      .arg(it->frameCount)
                      .arg(it->bytesWritten)
                      .arg(QFileInfo(it->filePath).fileName()));
    }
}

void MainWindow::appendCaptureRecordFrame(const FramePacket &frame)
{
    if (captureRecorderMode_ != CaptureRecorderMode::Record || captureRawDumper_ == nullptr
        || !captureRawDumper_->isActive())
        return;

    std::vector<std::size_t> selected;
    if (!selectedCaptureCameraIndices(selected))
        return;

    const std::size_t cameraIndex = frame.source == CameraBackendId::Camera1 ? 0 : 1;
    if (std::find(selected.begin(), selected.end(), cameraIndex) == selected.end())
        return;

    QString errorMessage;
    if (!captureRawDumper_->appendFrame(frame, &errorMessage))
    {
        appendLog(QStringLiteral("Capture record: %1").arg(errorMessage));
        stopCaptureRecorder();
    }
}

void MainWindow::startCaptureStageScan(const double distanceMm, const double speedMmPerSec)
{
    stageWorker_->requestPrimaryPosition([this, distanceMm, speedMmPerSec](const double positionMm,
                                                                           const bool ok) {
        QMetaObject::invokeMethod(
            this,
            [this, positionMm, ok, distanceMm, speedMmPerSec]() {
                if (captureRecorderMode_ != CaptureRecorderMode::Record)
                    return;

                if (!ok)
                {
                    appendLog(QStringLiteral("Capture record: could not read stage position."));
                    stopCaptureRecorder();
                    return;
                }

                const double endMm = positionMm + distanceMm;
                if (endMm > zaber_stage::kTravelLengthMm)
                {
                    appendLog(QStringLiteral(
                        "Capture record: scan would pass %1 mm (current %2 mm + %3 mm).")
                                  .arg(zaber_stage::kTravelLengthMm, 0, 'f', 0)
                                  .arg(positionMm, 0, 'f', 2)
                                  .arg(distanceMm, 0, 'f', 2));
                    stopCaptureRecorder();
                    return;
                }

                appendLog(QStringLiteral("Capture record: scanning %1 mm @ %2 mm/s…")
                              .arg(distanceMm, 0, 'f', 2)
                              .arg(speedMmPerSec, 0, 'f', 1));

                stageWorker_->requestMoveRelativeMm(distanceMm, speedMmPerSec);

                const int durationMs =
                    static_cast<int>(std::ceil((distanceMm / speedMmPerSec) * 1000.0)) + 750;
                if (captureScanTimer_ != nullptr)
                    captureScanTimer_->start(std::max(durationMs, 500));
            },
            Qt::QueuedConnection);
    });
}

void MainWindow::onRecordStagePrepositionComplete(const bool success,
                                                const double distanceMm,
                                                const double speedMmPerSec)
{
    if (captureRecorderMode_ != CaptureRecorderMode::Record)
        return;

    if (!success)
    {
        appendLog(QStringLiteral("Capture record: failed to move to closest camera position."));
        stopCaptureRecorder();
        return;
    }

    startCaptureStageScan(distanceMm, speedMmPerSec);
}

void MainWindow::startCaptureRecord()
{
    if (captureRecorderMode_ != CaptureRecorderMode::Idle)
        return;

    QString errorMessage;
    if (!beginCaptureRawDumpSession(errorMessage))
    {
        appendLog(QStringLiteral("Capture record: %1").arg(errorMessage));
        return;
    }

    captureRecorderMode_ = CaptureRecorderMode::Record;
    updateCaptureRecorderControls();

    const bool stageConnected =
        stageWorker_ != nullptr && stageWorker_->currentState() == StageState::Connected;

    if (!stageConnected)
    {
        appendLog(QStringLiteral("Capture record: writing .raw to %1 (stage not connected). "
                                  "Press Stop when finished.")
                      .arg(captureRawDumper_->sessionDirectory()));
        return;
    }

    double distanceMm = 0.0;
    double speedMmPerSec = 0.0;
    if (!buildCaptureScanPlan(distanceMm, speedMmPerSec, errorMessage))
    {
        appendLog(QStringLiteral("Capture record: %1").arg(errorMessage));
        stopCaptureRecorder();
        return;
    }

    bool hasClosest = false;
    const double closestMm = closestSelectedCaptureCameraPositionMm(&hasClosest);
    if (!hasClosest)
    {
        appendLog(QStringLiteral("Capture record: no camera position available for stage move."));
        stopCaptureRecorder();
        return;
    }

    appendLog(QStringLiteral("Capture record: session %1 — moving to closest camera position "
                              "%2 mm @ %3 mm/s, then scanning %4 mm @ %5 mm/s.")
                  .arg(captureRawDumper_->sessionDirectory())
                  .arg(closestMm, 0, 'f', 2)
                  .arg(zaber_stage::kMaxSpeedMmPerSec, 0, 'f', 0)
                  .arg(distanceMm, 0, 'f', 2)
                  .arg(speedMmPerSec, 0, 'f', 1));

    stageWorker_->requestMoveAbsoluteMm(
        closestMm,
        zaber_stage::kMaxSpeedMmPerSec,
        true,
        [this, distanceMm, speedMmPerSec](const bool success) {
            QMetaObject::invokeMethod(
                this,
                [this, success, distanceMm, speedMmPerSec]() {
                    onRecordStagePrepositionComplete(success, distanceMm, speedMmPerSec);
                },
                Qt::QueuedConnection);
        });
}

void MainWindow::stopCaptureRecorder()
{
    if (captureRecorderMode_ == CaptureRecorderMode::Idle)
        return;

    if (captureScanTimer_ != nullptr)
        captureScanTimer_->stop();

    if (stageWorker_ != nullptr)
        stageWorker_->requestStopMotion();

    const bool wasRecord = captureRecorderMode_ == CaptureRecorderMode::Record;
    captureRecorderMode_ = CaptureRecorderMode::Idle;
    updateCaptureRecorderControls();

    if (wasRecord)
    {
        endCaptureRawDumpSession();
        appendLog(QStringLiteral("Capture record: stopped."));
    }
    else
        appendLog(QStringLiteral("Capture preview: stopped."));
}

void MainWindow::finishCaptureScan()
{
    if (captureRecorderMode_ == CaptureRecorderMode::Idle)
        return;

    if (stageWorker_ != nullptr)
        stageWorker_->requestStopMotion();

    const bool wasRecord = captureRecorderMode_ == CaptureRecorderMode::Record;
    captureRecorderMode_ = CaptureRecorderMode::Idle;
    updateCaptureRecorderControls();

    if (wasRecord)
    {
        endCaptureRawDumpSession();
        appendLog(QStringLiteral("Capture record: scan motion finished."));
    }
    else
        appendLog(QStringLiteral("Capture preview: scan finished."));
}

void MainWindow::updateCaptureCamerasList()
{
    const auto isConnected = [](const LumoCameraUi &ui) {
        return ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;
    };

    const auto updateCheckbox = [this, &isConnected](LumoCameraUi &ui, QCheckBox *checkbox) {
        if (checkbox == nullptr)
            return;

        if (!isConnected(ui))
        {
            checkbox->hide();
            return;
        }

        const bool firstShow = checkbox->isHidden();
        checkbox->setText(profileTabNameForUi(ui));
        if (firstShow)
            checkbox->setChecked(true);
        checkbox->show();
    };

    updateCheckbox(camera1Ui_, captureCamera1Check_);
    updateCheckbox(camera2Ui_, captureCamera2Check_);

    if (captureCamerasEmptyLabel_ != nullptr)
    {
        const bool anyConnected = isConnected(camera1Ui_) || isConnected(camera2Ui_);
        captureCamerasEmptyLabel_->setHidden(anyConnected);
    }

    updateCaptureCameraPositionRows();
}

void MainWindow::updateCaptureCameraPositionRows()
{
    const auto isConnected = [](const LumoCameraUi &ui) {
        return ui.state != CameraState::Disconnected && ui.state != CameraState::Fault;
    };

    LumoCameraUi *cameras[] = {&camera1Ui_, &camera2Ui_};

    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        QWidget *row = captureCameraPositionRows_[cameraIndex];
        if (row == nullptr)
            continue;

        if (!isConnected(*cameras[cameraIndex]))
        {
            row->hide();
            continue;
        }

        const QString rowLabel = QStringLiteral("%1 position").arg(profileTabNameForUi(*cameras[cameraIndex]));
        if (QLabel *label = row->findChild<QLabel *>())
            label->setText(rowLabel);

        row->show();
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    savePersistedUiSettings();
    QMainWindow::closeEvent(event);
}

void MainWindow::loadPersistedUiSettings()
{
    const PersistedCapturePosition capturePosition = AppSettingsStore::loadCapturePosition();
    if (captureTargetLengthSpin_ != nullptr)
        captureTargetLengthSpin_->setValue(capturePosition.targetLengthMm);
    if (captureScanningSpeedSpin_ != nullptr)
    {
        captureScanningSpeedSpin_->setValue(
            qBound(0.0, capturePosition.scanningSpeedMmPerSec, zaber_stage::kMaxSpeedMmPerSec));
    }
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        if (captureCameraPositionSpins_[cameraIndex] != nullptr)
            captureCameraPositionSpins_[cameraIndex]->setValue(capturePosition.cameraPositionMm[cameraIndex]);
    }

    const PersistedStageConnection stageConnection = AppSettingsStore::loadStageConnection();
    persistedStagePort_ = stageConnection.port;
    if (stageBaudCombo_ != nullptr && !stageConnection.baud.isEmpty())
        stageBaudCombo_->setCurrentText(stageConnection.baud);

    applyPersistedCameraUiValues(camera1Ui_);
    applyPersistedCameraUiValues(camera2Ui_);
}

void MainWindow::savePersistedUiSettings()
{
    PersistedCapturePosition capturePosition;
    if (captureTargetLengthSpin_ != nullptr)
        capturePosition.targetLengthMm = captureTargetLengthSpin_->value();
    if (captureScanningSpeedSpin_ != nullptr)
        capturePosition.scanningSpeedMmPerSec = captureScanningSpeedSpin_->value();
    for (std::size_t cameraIndex = 0; cameraIndex < 2; ++cameraIndex)
    {
        if (captureCameraPositionSpins_[cameraIndex] != nullptr)
            capturePosition.cameraPositionMm[cameraIndex] =
                captureCameraPositionSpins_[cameraIndex]->value();
    }
    AppSettingsStore::saveCapturePosition(capturePosition);

    PersistedStageConnection stageConnection;
    stageConnection.port = selectedStagePortName();
    if (stageConnection.port.startsWith(QLatin1Char('(')))
        stageConnection.port.clear();
    if (stageBaudCombo_ != nullptr)
        stageConnection.baud = stageBaudCombo_->currentText();
    AppSettingsStore::saveStageConnection(stageConnection);
    persistedStagePort_ = stageConnection.port;

    savePersistedCameraSettings(camera1Ui_);
    savePersistedCameraSettings(camera2Ui_);

    AppSettingsStore::sync();
}

void MainWindow::schedulePersistedUiSettingsSave()
{
    if (settingsSaveTimer_ != nullptr)
        settingsSaveTimer_->start();
}

void MainWindow::applyPersistedCameraUiValues(LumoCameraUi &ui)
{
    const PersistedCameraSettings saved = AppSettingsStore::loadCameraSettings(ui.cameraIndex);

    if (ui.frameRateSpin != nullptr)
        ui.frameRateSpin->setValue(saved.frameRateHz);
    if (ui.exposureSpin != nullptr)
        ui.exposureSpin->setValue(saved.exposureMs);
    if (ui.spectralBinningCombo != nullptr)
    {
        const int index = ui.spectralBinningCombo->findText(saved.spectralBinning);
        if (index >= 0)
            ui.spectralBinningCombo->setCurrentIndex(index);
    }
    if (ui.spatialBinningCombo != nullptr)
    {
        const int index = ui.spatialBinningCombo->findText(saved.spatialBinning);
        if (index >= 0)
            ui.spatialBinningCombo->setCurrentIndex(index);
    }
    if (ui.triggerCombo != nullptr)
    {
        const int index = ui.triggerCombo->findText(saved.trigger);
        if (index >= 0)
            ui.triggerCombo->setCurrentIndex(index);
    }

    if (ui.calibrationPackEdit != nullptr && !saved.calibrationPackPath.isEmpty()
        && QFileInfo::exists(saved.calibrationPackPath))
        setCalibrationPackDisplay(ui.calibrationPackEdit, saved.calibrationPackPath);
}

void MainWindow::savePersistedCameraSettings(const LumoCameraUi &ui)
{
    PersistedCameraSettings saved;
    if (ui.deviceCombo != nullptr)
        saved.profile = ui.deviceCombo->currentText();
    if (ui.frameRateSpin != nullptr)
        saved.frameRateHz = ui.frameRateSpin->value();
    if (ui.exposureSpin != nullptr)
        saved.exposureMs = ui.exposureSpin->value();
    if (ui.spectralBinningCombo != nullptr)
        saved.spectralBinning = ui.spectralBinningCombo->currentText();
    if (ui.spatialBinningCombo != nullptr)
        saved.spatialBinning = ui.spatialBinningCombo->currentText();
    if (ui.triggerCombo != nullptr)
        saved.trigger = ui.triggerCombo->currentText();

    const QString calpackPath = calibrationPackPath(ui);
    if (!calpackPath.isEmpty())
        saved.calibrationPackPath = calpackPath;

    if (ui.redBandCombo != nullptr && ui.redBandCombo->currentIndex() >= 0)
        saved.redBandIndex = ui.redBandCombo->currentData().toInt();
    if (ui.greenBandCombo != nullptr && ui.greenBandCombo->currentIndex() >= 0)
        saved.greenBandIndex = ui.greenBandCombo->currentData().toInt();
    if (ui.blueBandCombo != nullptr && ui.blueBandCombo->currentIndex() >= 0)
        saved.blueBandIndex = ui.blueBandCombo->currentData().toInt();

    AppSettingsStore::saveCameraSettings(ui.cameraIndex, saved);
}

bool MainWindow::selectDeviceProfileByName(QComboBox *combo, const QString &profileName) const
{
    if (combo == nullptr || profileName.trimmed().isEmpty())
        return false;

    for (int index = 0; index < combo->count(); ++index)
    {
        if (combo->itemText(index).compare(profileName, Qt::CaseInsensitive) == 0)
        {
            combo->setCurrentIndex(index);
            return true;
        }
    }

    for (int index = 0; index < combo->count(); ++index)
    {
        if (combo->itemText(index).contains(profileName, Qt::CaseInsensitive))
        {
            combo->setCurrentIndex(index);
            return true;
        }
    }

    return false;
}

void MainWindow::applyPersistedCameraProfileSelection(LumoCameraUi &ui)
{
    if (ui.deviceCombo == nullptr)
        return;

    const PersistedCameraSettings saved = AppSettingsStore::loadCameraSettings(ui.cameraIndex);
    if (!saved.profile.isEmpty() && selectDeviceProfileByName(ui.deviceCombo, saved.profile))
        return;

    if (ui.sensorKind == LumoSensorKind::Fx10ePleora)
    {
        if (!selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("FX10e with Pleora")))
            selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("FX10"));
    }
    else
    {
        if (!selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("SWIR3 with NI")))
            selectDeviceProfileByName(ui.deviceCombo, QStringLiteral("SWIR"));
    }
}

void MainWindow::applyPersistedCameraProfilesAndBands()
{
    applyPersistedCameraProfileSelection(camera1Ui_);
    applyPersistedCameraProfileSelection(camera2Ui_);

    refreshBandCombos(camera1Ui_);
    refreshBandCombos(camera2Ui_);

    const auto applyBandSelections = [this](LumoCameraUi &ui) {
        const PersistedCameraSettings saved = AppSettingsStore::loadCameraSettings(ui.cameraIndex);
        if (saved.redBandIndex >= 0)
            selectBandComboIndex(ui.redBandCombo, saved.redBandIndex);
        if (saved.greenBandIndex >= 0)
            selectBandComboIndex(ui.greenBandCombo, saved.greenBandIndex);
        if (saved.blueBandIndex >= 0)
            selectBandComboIndex(ui.blueBandCombo, saved.blueBandIndex);
    };

    applyBandSelections(camera1Ui_);
    applyBandSelections(camera2Ui_);

    if (calibrationPackPath(camera1Ui_).isEmpty())
        syncCalibrationPackToSelectedProfile(camera1Ui_);
    if (calibrationPackPath(camera2Ui_).isEmpty())
        syncCalibrationPackToSelectedProfile(camera2Ui_);
}

void MainWindow::connectPersistedSettingsAutosave()
{
    const auto schedule = [this]() { schedulePersistedUiSettingsSave(); };

    if (captureTargetLengthSpin_ != nullptr)
        connect(captureTargetLengthSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
    if (captureScanningSpeedSpin_ != nullptr)
        connect(captureScanningSpeedSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
    for (QDoubleSpinBox *spin : captureCameraPositionSpins_)
    {
        if (spin != nullptr)
            connect(spin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
    }

    if (stagePortCombo_ != nullptr)
        connect(stagePortCombo_, &QComboBox::currentIndexChanged, this, schedule);
    if (stageBaudCombo_ != nullptr)
        connect(stageBaudCombo_, &QComboBox::currentIndexChanged, this, schedule);

    const auto connectCamera = [this, schedule](LumoCameraUi &ui) {
        if (ui.deviceCombo != nullptr)
            connect(ui.deviceCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.frameRateSpin != nullptr)
            connect(ui.frameRateSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
        if (ui.exposureSpin != nullptr)
            connect(ui.exposureSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, schedule);
        if (ui.spectralBinningCombo != nullptr)
            connect(ui.spectralBinningCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.spatialBinningCombo != nullptr)
            connect(ui.spatialBinningCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.triggerCombo != nullptr)
            connect(ui.triggerCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.redBandCombo != nullptr)
            connect(ui.redBandCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.greenBandCombo != nullptr)
            connect(ui.greenBandCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.blueBandCombo != nullptr)
            connect(ui.blueBandCombo, &QComboBox::currentIndexChanged, this, schedule);
        if (ui.calibrationPackBrowseBtn != nullptr)
        {
            connect(ui.calibrationPackBrowseBtn, &QPushButton::clicked, this, schedule);
        }
    };

    connectCamera(camera1Ui_);
    connectCamera(camera2Ui_);
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

    auto populateCombo = [&devices](QComboBox *combo,
                                    const std::function<bool(const LumoDeviceEntry &)> &include =
                                        nullptr) {
        if (combo == nullptr)
            return;

        combo->clear();
        for (const LumoDeviceEntry &device : devices)
        {
            if (include != nullptr && !include(device))
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

    populateCombo(camera1Ui_.deviceCombo);
    populateCombo(camera2Ui_.deviceCombo, [](const LumoDeviceEntry &device) {
        const QString name = QString::fromStdString(device.name);
        return name.contains(QStringLiteral("SWIR"), Qt::CaseInsensitive)
               || name.contains(QStringLiteral("NI"), Qt::CaseInsensitive);
    });

    applyPersistedCameraProfileSelection(camera1Ui_);
    applyPersistedCameraProfileSelection(camera2Ui_);

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
    updateCaptureCamerasList();
    updateCaptureRecorderControls();

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
    appendCaptureRecordFrame(frame);
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
