// Blackfly S (BFS) camera settings panel UI — SpinView-like controls; persists via QSettings.
#include "frontend/widgets/BfsCameraSettingsWidget.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/settings/AppSettingsStore.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QFrame>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSpinBox>
#include <QVBoxLayout>

namespace ui
{
namespace
{
constexpr int kCompactSpinWidth = 128;

void configureForm(QFormLayout *form)
{
    form->setContentsMargins(8, 8, 8, 8);
    form->setHorizontalSpacing(8);
    form->setVerticalSpacing(6);
    form->setRowWrapPolicy(QFormLayout::WrapLongRows);
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
}

void compactSpin(QWidget *spin)
{
    // Large QSpinBox ranges inflate sizeHint (based on max digits) and clip the narrow settings pane.
    spin->setMinimumWidth(0);
    spin->setFixedWidth(kCompactSpinWidth);
    spin->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

QComboBox *makeAutoModeCombo(QWidget *parent, const QString &current)
{
    auto *combo = new QComboBox(parent);
    combo->addItems({QStringLiteral("Off"), QStringLiteral("Once"), QStringLiteral("Continuous")});
    combo->setCurrentText(current);
    combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    combo->setMinimumWidth(0);
    return combo;
}

QComboBox *makeExpandingCombo(QWidget *parent)
{
    auto *combo = new QComboBox(parent);
    combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    combo->setMinimumWidth(0);
    return combo;
}
} // namespace

BfsCameraSettingsWidget::BfsCameraSettingsWidget(QWidget *parent)
    : QWidget(parent)
{
    applyWhiteSettingsBackground(this);

    auto *rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    auto *scroll = new QScrollArea(this);
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    scroll->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto *page = new QWidget();
    page->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
    scroll->setWidget(page);
    applyWhiteSettingsScrollBackground(scroll);

    auto *layout = new QVBoxLayout(page);
    // Extra right margin so fields clear the vertical scrollbar.
    layout->setContentsMargins(4, 4, 14, 8);
    layout->setSpacing(8);

    auto *connBox = new QGroupBox(QStringLiteral("Connection"), page);
    auto *connForm = new QFormLayout(connBox);
    configureForm(connForm);

    cameraCombo_ = makeExpandingCombo(connBox);
    cameraCombo_->setEditable(false);
    cameraCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    cameraCombo_->setMinimumContentsLength(18);
    cameraCombo_->setToolTip(QStringLiteral("Blackfly S camera (serial / model). Refresh after plugging in."));

    refreshCamerasBtn_ = new QPushButton(QStringLiteral("Refresh"), connBox);
    refreshCamerasBtn_->setToolTip(QStringLiteral("Rescan for Spinnaker / Blackfly S cameras."));
    refreshCamerasBtn_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    auto *cameraRow = new QWidget(connBox);
    auto *cameraRowLayout = new QHBoxLayout(cameraRow);
    cameraRowLayout->setContentsMargins(0, 0, 0, 0);
    cameraRowLayout->setSpacing(6);
    cameraRowLayout->addWidget(cameraCombo_, 1);
    cameraRowLayout->addWidget(refreshCamerasBtn_, 0);
    connForm->addRow(QStringLiteral("Camera"), cameraRow);

    connectBtn_ = new QPushButton(QStringLiteral("Connect"), connBox);
    disconnectBtn_ = new QPushButton(QStringLiteral("Disconnect"), connBox);
    connectBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    disconnectBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    auto *connBtnRow = new QWidget(connBox);
    auto *connBtnLayout = new QHBoxLayout(connBtnRow);
    connBtnLayout->setContentsMargins(0, 0, 0, 0);
    connBtnLayout->setSpacing(8);
    connBtnLayout->addWidget(connectBtn_, 1);
    connBtnLayout->addWidget(disconnectBtn_, 1);
    connForm->addRow(QStringLiteral(""), connBtnRow);

    connectionStatusLabel_ = new QLabel(QStringLiteral("Disconnected"), connBox);
    connectionStatusLabel_->setWordWrap(true);
    connForm->addRow(QStringLiteral("Status"), connectionStatusLabel_);
    setConnectedUi(false);

    auto *acqBox = new QGroupBox(QStringLiteral("Acquisition Control"), page);
    auto *acqForm = new QFormLayout(acqBox);
    configureForm(acqForm);

    acquisitionModeCombo_ = makeExpandingCombo(acqBox);
    acquisitionModeCombo_->addItems({QStringLiteral("Continuous"),
                                     QStringLiteral("SingleFrame"),
                                     QStringLiteral("MultiFrame")});
    acquisitionModeCombo_->setCurrentText(QStringLiteral("Continuous"));
    acqForm->addRow(QStringLiteral("Acquisition Mode"), acquisitionModeCombo_);

    acquisitionFrameRateEnableCheck_ = new QCheckBox(acqBox);
    acquisitionFrameRateEnableCheck_->setChecked(true);
    acqForm->addRow(QStringLiteral("Acquisition Frame Rate Enable"),
                    acquisitionFrameRateEnableCheck_);

    acquisitionFrameRateSpin_ = new QDoubleSpinBox(acqBox);
    acquisitionFrameRateSpin_->setRange(0.1, 1000.0);
    acquisitionFrameRateSpin_->setDecimals(2);
    acquisitionFrameRateSpin_->setSingleStep(0.1);
    acquisitionFrameRateSpin_->setSuffix(QStringLiteral(" Hz"));
    acquisitionFrameRateSpin_->setValue(7.44);
    compactSpin(acquisitionFrameRateSpin_);
    acqForm->addRow(QStringLiteral("Acquisition Frame Rate"), acquisitionFrameRateSpin_);

    deviceLinkThroughputLimitSpin_ = new QSpinBox(acqBox);
    deviceLinkThroughputLimitSpin_->setRange(1, 125000000);
    deviceLinkThroughputLimitSpin_->setSingleStep(100000);
    deviceLinkThroughputLimitSpin_->setValue(94776971);
    compactSpin(deviceLinkThroughputLimitSpin_);
    acqForm->addRow(QStringLiteral("Device Link Throughput Limit"),
                    deviceLinkThroughputLimitSpin_);

    auto *expBox = new QGroupBox(QStringLiteral("Exposure and Gain"), page);
    auto *expForm = new QFormLayout(expBox);
    configureForm(expForm);

    evCompensationSpin_ = new QDoubleSpinBox(expBox);
    evCompensationSpin_->setRange(-10.0, 10.0);
    evCompensationSpin_->setDecimals(1);
    evCompensationSpin_->setSingleStep(0.5);
    evCompensationSpin_->setValue(0.0);
    compactSpin(evCompensationSpin_);
    expForm->addRow(QStringLiteral("EV Compensation"), evCompensationSpin_);

    exposureModeCombo_ = makeExpandingCombo(expBox);
    exposureModeCombo_->addItems({QStringLiteral("Timed"), QStringLiteral("TriggerWidth")});
    exposureModeCombo_->setCurrentText(QStringLiteral("Timed"));
    expForm->addRow(QStringLiteral("Exposure Mode"), exposureModeCombo_);

    exposureAutoCombo_ = makeAutoModeCombo(expBox, QStringLiteral("Continuous"));
    expForm->addRow(QStringLiteral("Exposure Auto"), exposureAutoCombo_);

    exposureTimeSpin_ = new QDoubleSpinBox(expBox);
    exposureTimeSpin_->setRange(1.0, 30000000.0);
    exposureTimeSpin_->setDecimals(0);
    exposureTimeSpin_->setSingleStep(100.0);
    exposureTimeSpin_->setSuffix(QStringLiteral(" us"));
    exposureTimeSpin_->setValue(15005.0);
    compactSpin(exposureTimeSpin_);
    expForm->addRow(QStringLiteral("Exposure Time"), exposureTimeSpin_);

    exposureTimeLowerLimitMinSpin_ = new QSpinBox(expBox);
    exposureTimeLowerLimitMinSpin_->setRange(1, 30000000);
    exposureTimeLowerLimitMinSpin_->setSuffix(QStringLiteral(" us"));
    exposureTimeLowerLimitMinSpin_->setValue(100);
    compactSpin(exposureTimeLowerLimitMinSpin_);

    exposureTimeLowerLimitMaxSpin_ = new QSpinBox(expBox);
    exposureTimeLowerLimitMaxSpin_->setRange(1, 30000000);
    exposureTimeLowerLimitMaxSpin_->setSuffix(QStringLiteral(" us"));
    exposureTimeLowerLimitMaxSpin_->setValue(15000);
    compactSpin(exposureTimeLowerLimitMaxSpin_);

    auto *expLimitRow = new QWidget(expBox);
    auto *expLimitLayout = new QHBoxLayout(expLimitRow);
    expLimitLayout->setContentsMargins(0, 0, 0, 0);
    expLimitLayout->setSpacing(6);
    expLimitLayout->addWidget(exposureTimeLowerLimitMinSpin_, 0);
    auto *expDash = new QLabel(QStringLiteral("–"), expLimitRow);
    expDash->setAlignment(Qt::AlignCenter);
    expLimitLayout->addWidget(expDash, 0);
    expLimitLayout->addWidget(exposureTimeLowerLimitMaxSpin_, 0);
    expLimitLayout->addStretch(1);
    expForm->addRow(QStringLiteral("Exposure Time Lower Limit"), expLimitRow);

    gainAutoCombo_ = makeAutoModeCombo(expBox, QStringLiteral("Continuous"));
    expForm->addRow(QStringLiteral("Gain Auto"), gainAutoCombo_);

    gainSpin_ = new QDoubleSpinBox(expBox);
    gainSpin_->setRange(0.0, 48.0);
    gainSpin_->setDecimals(1);
    gainSpin_->setSingleStep(0.1);
    gainSpin_->setSuffix(QStringLiteral(" dB"));
    gainSpin_->setValue(16.9);
    compactSpin(gainSpin_);
    expForm->addRow(QStringLiteral("Gain"), gainSpin_);

    auto *imgBox = new QGroupBox(QStringLiteral("Image Quality and Color Balance"), page);
    auto *imgForm = new QFormLayout(imgBox);
    configureForm(imgForm);

    gammaEnableCheck_ = new QCheckBox(imgBox);
    gammaEnableCheck_->setChecked(true);
    imgForm->addRow(QStringLiteral("Gamma Enable"), gammaEnableCheck_);

    gammaSpin_ = new QDoubleSpinBox(imgBox);
    gammaSpin_->setRange(0.1, 4.0);
    gammaSpin_->setDecimals(2);
    gammaSpin_->setSingleStep(0.05);
    gammaSpin_->setValue(0.8);
    compactSpin(gammaSpin_);
    imgForm->addRow(QStringLiteral("Gamma"), gammaSpin_);

    blackLevelSelectorCombo_ = makeExpandingCombo(imgBox);
    blackLevelSelectorCombo_->addItems({QStringLiteral("All"),
                                        QStringLiteral("AnalogAll"),
                                        QStringLiteral("DigitalAll")});
    blackLevelSelectorCombo_->setCurrentText(QStringLiteral("All"));
    imgForm->addRow(QStringLiteral("Black Level Selector"), blackLevelSelectorCombo_);

    blackLevelSpin_ = new QDoubleSpinBox(imgBox);
    blackLevelSpin_->setRange(0.0, 100.0);
    blackLevelSpin_->setDecimals(1);
    blackLevelSpin_->setSingleStep(0.1);
    blackLevelSpin_->setSuffix(QStringLiteral(" %"));
    blackLevelSpin_->setValue(0.0);
    compactSpin(blackLevelSpin_);
    imgForm->addRow(QStringLiteral("Black Level"), blackLevelSpin_);

    balanceRatioSelectorCombo_ = makeExpandingCombo(imgBox);
    balanceRatioSelectorCombo_->addItems(
        {QStringLiteral("Red"), QStringLiteral("Green"), QStringLiteral("Blue")});
    balanceRatioSelectorCombo_->setCurrentText(QStringLiteral("Red"));
    imgForm->addRow(QStringLiteral("Balance Ratio Selector"), balanceRatioSelectorCombo_);

    balanceRatioSpin_ = new QDoubleSpinBox(imgBox);
    balanceRatioSpin_->setRange(0.0, 8.0);
    balanceRatioSpin_->setDecimals(2);
    balanceRatioSpin_->setSingleStep(0.01);
    balanceRatioSpin_->setValue(1.21);
    compactSpin(balanceRatioSpin_);
    imgForm->addRow(QStringLiteral("Balance Ratio"), balanceRatioSpin_);

    balanceWhiteAutoCombo_ = makeAutoModeCombo(imgBox, QStringLiteral("Continuous"));
    imgForm->addRow(QStringLiteral("Balance White Auto"), balanceWhiteAutoCombo_);

    layout->addWidget(connBox);
    layout->addWidget(acqBox);
    layout->addWidget(expBox);
    layout->addWidget(imgBox);

    rootLayout->addWidget(scroll, 1);

    captureBtn_ = new QPushButton(QStringLiteral("Capture"), this);
    captureBtn_->setToolTip(QStringLiteral("Save the latest streamed frame as a TIFF image."));
    captureBtn_->setEnabled(false);
    captureBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    auto *captureBar = new QWidget(this);
    applyWhiteSettingsBackground(captureBar);
    auto *captureBarLayout = new QVBoxLayout(captureBar);
    captureBarLayout->setContentsMargins(8, 8, 8, 8);
    captureBarLayout->setSpacing(0);
    captureBarLayout->addWidget(captureBtn_);
    rootLayout->addWidget(captureBar, 0);

    const auto hook = [this]() { onParameterChanged(); };
    connect(cameraCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(acquisitionModeCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(acquisitionFrameRateEnableCheck_, &QCheckBox::toggled, this, hook);
    connect(acquisitionFrameRateSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(deviceLinkThroughputLimitSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(evCompensationSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(exposureModeCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(exposureAutoCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(exposureTimeSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(exposureTimeLowerLimitMinSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(exposureTimeLowerLimitMaxSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(gainAutoCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(gainSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(gammaEnableCheck_, &QCheckBox::toggled, this, hook);
    connect(gammaSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(blackLevelSelectorCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(blackLevelSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(balanceRatioSelectorCombo_, &QComboBox::currentTextChanged, this,
            [this](const QString &) { onParameterChanged(); });
    connect(balanceRatioSpin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, hook);
    connect(balanceWhiteAutoCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });

    loadFromSettings();
    saveToSettings();
    setConnectedUi(false);
}

hf::bfs::BfsCameraSettings BfsCameraSettingsWidget::currentSettings() const
{
    hf::bfs::BfsCameraSettings settings;
    settings.cameraId = cameraCombo_->currentText();
    settings.acquisitionMode = acquisitionModeCombo_->currentText();
    settings.acquisitionFrameRateEnable = acquisitionFrameRateEnableCheck_->isChecked();
    settings.acquisitionFrameRateHz = acquisitionFrameRateSpin_->value();
    settings.deviceLinkThroughputLimit = deviceLinkThroughputLimitSpin_->value();
    settings.evCompensation = evCompensationSpin_->value();
    settings.exposureMode = exposureModeCombo_->currentText();
    settings.exposureAuto = exposureAutoCombo_->currentText();
    settings.exposureTimeUs = exposureTimeSpin_->value();
    settings.exposureTimeLowerLimitMinUs = exposureTimeLowerLimitMinSpin_->value();
    settings.exposureTimeLowerLimitMaxUs = exposureTimeLowerLimitMaxSpin_->value();
    settings.gainAuto = gainAutoCombo_->currentText();
    settings.gainDb = gainSpin_->value();
    settings.gammaEnable = gammaEnableCheck_->isChecked();
    settings.gamma = gammaSpin_->value();
    settings.blackLevelSelector = blackLevelSelectorCombo_->currentText();
    settings.blackLevelPercent = blackLevelSpin_->value();
    settings.balanceRatioSelector = balanceRatioSelectorCombo_->currentText();
    settings.balanceRatio = balanceRatioSpin_->value();
    settings.balanceWhiteAuto = balanceWhiteAutoCombo_->currentText();
    return settings;
}

void BfsCameraSettingsWidget::setDevices(const std::vector<hf::bfs::BfsDeviceInfo> &devices,
                                         const QString &preferredId)
{
    const QSignalBlocker blocker(cameraCombo_);
    const QString previous = cameraCombo_->currentText();
    cameraCombo_->clear();
    for (const hf::bfs::BfsDeviceInfo &device : devices)
        cameraCombo_->addItem(device.displayName(), device.serial);

    const QString want = !preferredId.isEmpty() ? preferredId : previous;
    if (!want.isEmpty())
    {
        int index = cameraCombo_->findData(want);
        if (index < 0)
            index = cameraCombo_->findText(want);
        if (index < 0)
        {
            for (int i = 0; i < cameraCombo_->count(); ++i)
            {
                if (cameraCombo_->itemText(i).contains(want))
                {
                    index = i;
                    break;
                }
            }
        }
        if (index >= 0)
            cameraCombo_->setCurrentIndex(index);
        else
            setComboText(cameraCombo_, want);
    }
}

void BfsCameraSettingsWidget::setConnectionStatus(const QString &text)
{
    if (connectionStatusLabel_ != nullptr)
        connectionStatusLabel_->setText(text);
}

void BfsCameraSettingsWidget::setConnectedUi(const bool connected)
{
    if (cameraCombo_ != nullptr)
        cameraCombo_->setEnabled(!connected);
    if (refreshCamerasBtn_ != nullptr)
        refreshCamerasBtn_->setEnabled(!connected);
    if (connectBtn_ != nullptr)
        connectBtn_->setEnabled(!connected);
    if (disconnectBtn_ != nullptr)
        disconnectBtn_->setEnabled(connected);
    setParameterControlsEnabled(connected);
    if (!connected)
        setCaptureEnabled(false);
}

void BfsCameraSettingsWidget::setParameterControlsEnabled(const bool enabled)
{
    const auto setEnabled = [enabled](QWidget *w) {
        if (w != nullptr)
            w->setEnabled(enabled);
    };

    setEnabled(acquisitionModeCombo_);
    setEnabled(acquisitionFrameRateEnableCheck_);
    setEnabled(acquisitionFrameRateSpin_);
    setEnabled(deviceLinkThroughputLimitSpin_);
    setEnabled(evCompensationSpin_);
    setEnabled(exposureModeCombo_);
    setEnabled(exposureAutoCombo_);
    setEnabled(exposureTimeSpin_);
    setEnabled(exposureTimeLowerLimitMinSpin_);
    setEnabled(exposureTimeLowerLimitMaxSpin_);
    setEnabled(gainAutoCombo_);
    setEnabled(gainSpin_);
    setEnabled(gammaEnableCheck_);
    setEnabled(gammaSpin_);
    setEnabled(blackLevelSelectorCombo_);
    setEnabled(blackLevelSpin_);
    setEnabled(balanceRatioSelectorCombo_);
    setEnabled(balanceRatioSpin_);
    setEnabled(balanceWhiteAutoCombo_);
}

void BfsCameraSettingsWidget::setCaptureEnabled(const bool enabled)
{
    if (captureBtn_ != nullptr)
        captureBtn_->setEnabled(enabled);
}

void BfsCameraSettingsWidget::setComboText(QComboBox *combo, const QString &text) const
{
    if (combo == nullptr || text.isEmpty())
        return;
    const int index = combo->findText(text);
    if (index >= 0)
    {
        combo->setCurrentIndex(index);
        return;
    }
    combo->addItem(text);
    combo->setCurrentIndex(combo->count() - 1);
}

void BfsCameraSettingsWidget::loadFromSettings()
{
    const PersistedBfsCameraSettings saved = AppSettingsStore::loadBfsCameraSettings();

    const QSignalBlocker b0(cameraCombo_);
    const QSignalBlocker b1(acquisitionModeCombo_);
    const QSignalBlocker b2(acquisitionFrameRateEnableCheck_);
    const QSignalBlocker b3(acquisitionFrameRateSpin_);
    const QSignalBlocker b4(deviceLinkThroughputLimitSpin_);
    const QSignalBlocker b5(evCompensationSpin_);
    const QSignalBlocker b6(exposureModeCombo_);
    const QSignalBlocker b7(exposureAutoCombo_);
    const QSignalBlocker b8(exposureTimeSpin_);
    const QSignalBlocker b9(exposureTimeLowerLimitMinSpin_);
    const QSignalBlocker b10(exposureTimeLowerLimitMaxSpin_);
    const QSignalBlocker b11(gainAutoCombo_);
    const QSignalBlocker b12(gainSpin_);
    const QSignalBlocker b13(gammaEnableCheck_);
    const QSignalBlocker b14(gammaSpin_);
    const QSignalBlocker b15(blackLevelSelectorCombo_);
    const QSignalBlocker b16(blackLevelSpin_);
    const QSignalBlocker b17(balanceRatioSelectorCombo_);
    const QSignalBlocker b18(balanceRatioSpin_);
    const QSignalBlocker b19(balanceWhiteAutoCombo_);

    if (!saved.cameraId.isEmpty())
        setComboText(cameraCombo_, saved.cameraId);
    setComboText(acquisitionModeCombo_, saved.acquisitionMode);
    acquisitionFrameRateEnableCheck_->setChecked(saved.acquisitionFrameRateEnable);
    acquisitionFrameRateSpin_->setValue(saved.acquisitionFrameRateHz);
    deviceLinkThroughputLimitSpin_->setValue(saved.deviceLinkThroughputLimit);
    evCompensationSpin_->setValue(saved.evCompensation);
    setComboText(exposureModeCombo_, saved.exposureMode);
    setComboText(exposureAutoCombo_, saved.exposureAuto);
    exposureTimeSpin_->setValue(saved.exposureTimeUs);
    exposureTimeLowerLimitMinSpin_->setValue(saved.exposureTimeLowerLimitMinUs);
    exposureTimeLowerLimitMaxSpin_->setValue(saved.exposureTimeLowerLimitMaxUs);
    setComboText(gainAutoCombo_, saved.gainAuto);
    gainSpin_->setValue(saved.gainDb);
    gammaEnableCheck_->setChecked(saved.gammaEnable);
    gammaSpin_->setValue(saved.gamma);
    setComboText(blackLevelSelectorCombo_, saved.blackLevelSelector);
    blackLevelSpin_->setValue(saved.blackLevelPercent);
    setComboText(balanceRatioSelectorCombo_, saved.balanceRatioSelector);
    balanceRatioSpin_->setValue(saved.balanceRatio);
    setComboText(balanceWhiteAutoCombo_, saved.balanceWhiteAuto);
}

void BfsCameraSettingsWidget::saveToSettings() const
{
    PersistedBfsCameraSettings saved;
    saved.cameraId = cameraCombo_->currentText();
    saved.acquisitionMode = acquisitionModeCombo_->currentText();
    saved.acquisitionFrameRateEnable = acquisitionFrameRateEnableCheck_->isChecked();
    saved.acquisitionFrameRateHz = acquisitionFrameRateSpin_->value();
    saved.deviceLinkThroughputLimit = deviceLinkThroughputLimitSpin_->value();
    saved.evCompensation = evCompensationSpin_->value();
    saved.exposureMode = exposureModeCombo_->currentText();
    saved.exposureAuto = exposureAutoCombo_->currentText();
    saved.exposureTimeUs = exposureTimeSpin_->value();
    saved.exposureTimeLowerLimitMinUs = exposureTimeLowerLimitMinSpin_->value();
    saved.exposureTimeLowerLimitMaxUs = exposureTimeLowerLimitMaxSpin_->value();
    saved.gainAuto = gainAutoCombo_->currentText();
    saved.gainDb = gainSpin_->value();
    saved.gammaEnable = gammaEnableCheck_->isChecked();
    saved.gamma = gammaSpin_->value();
    saved.blackLevelSelector = blackLevelSelectorCombo_->currentText();
    saved.blackLevelPercent = blackLevelSpin_->value();
    saved.balanceRatioSelector = balanceRatioSelectorCombo_->currentText();
    saved.balanceRatio = balanceRatioSpin_->value();
    saved.balanceWhiteAuto = balanceWhiteAutoCombo_->currentText();
    AppSettingsStore::saveBfsCameraSettings(saved);
}

void BfsCameraSettingsWidget::onParameterChanged()
{
    if (exposureTimeLowerLimitMinSpin_->value() > exposureTimeLowerLimitMaxSpin_->value())
        exposureTimeLowerLimitMaxSpin_->setValue(exposureTimeLowerLimitMinSpin_->value());

    saveToSettings();
    emit settingsEdited();
}
} // namespace ui
