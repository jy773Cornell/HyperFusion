// DLP projector settings panel — Connect / Arm / Blank; persists via QSettings.
#include "frontend/widgets/DlpProjectorSettingsWidget.hpp"
#include "frontend/widgets/MainWindowTabHelpers.hpp"
#include "frontend/settings/AppSettingsStore.hpp"
#include "backend/HyperFusionConfig.hpp"
#include "backend/fpp/DlpTypes.hpp"

#include <QComboBox>
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
    spin->setMinimumWidth(0);
    spin->setFixedWidth(kCompactSpinWidth);
    spin->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
}

QComboBox *makeExpandingCombo(QWidget *parent)
{
    auto *combo = new QComboBox(parent);
    combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    combo->setMinimumWidth(0);
    return combo;
}
} // namespace

DlpProjectorSettingsWidget::DlpProjectorSettingsWidget(QWidget *parent)
    : QWidget(parent)
{
    applyWhiteSettingsBackground(this);
    ledMaxMa_ = hf::hardwareConfig().dlp.ledMaxMa;

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
    layout->setContentsMargins(4, 4, 14, 8);
    layout->setSpacing(8);

    auto *connBox = new QGroupBox(QStringLiteral("Connection"), page);
    auto *connForm = new QFormLayout(connBox);
    configureForm(connForm);

    deviceCombo_ = makeExpandingCombo(connBox);
    deviceCombo_->setEditable(false);
    deviceCombo_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    deviceCombo_->setMinimumContentsLength(18);
    deviceCombo_->setToolTip(QStringLiteral("DLP3010EVM-LC over USB. Close the TI GUI before Connect."));

    refreshBtn_ = new QPushButton(QStringLiteral("Refresh"), connBox);
    refreshBtn_->setToolTip(QStringLiteral("Rescan for the DLP USB bridge."));
    refreshBtn_->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

    auto *deviceRow = new QWidget(connBox);
    auto *deviceRowLayout = new QHBoxLayout(deviceRow);
    deviceRowLayout->setContentsMargins(0, 0, 0, 0);
    deviceRowLayout->setSpacing(6);
    deviceRowLayout->addWidget(deviceCombo_, 1);
    deviceRowLayout->addWidget(refreshBtn_, 0);
    connForm->addRow(QStringLiteral("Projector"), deviceRow);

    connectBtn_ = new QPushButton(QStringLiteral("Connect"), connBox);
    disconnectBtn_ = new QPushButton(QStringLiteral("Disconnect"), connBox);
    connectBtn_->setToolTip(
        QStringLiteral("Open USB/I2C and arm. Close the TI DLP GUI first (one Cypress owner)."));
    disconnectBtn_->setToolTip(QStringLiteral("Blank, then release the USB bridge."));
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

    auto *patBox = new QGroupBox(QStringLiteral("Pattern"), page);
    auto *patForm = new QFormLayout(patBox);
    configureForm(patForm);

    testPatternCombo_ = makeExpandingCombo(patBox);
    testPatternCombo_->addItems({QStringLiteral("FPP scanning"),
                                 QStringLiteral("Checkerboard"),
                                 QStringLiteral("Horizontal ramp"),
                                 QStringLiteral("Vertical ramp"),
                                 QStringLiteral("Solid field"),
                                 QStringLiteral("Color bars")});
    testPatternCombo_->setCurrentText(QStringLiteral("FPP scanning"));
    testPatternCombo_->setToolTip(QStringLiteral(
        "FPP scanning: loops Black, White, coarse code, Gray codes, then 8 px fringe 0/90/180/270. "
        "GUI test only. Blank stops."));
    patForm->addRow(testPatternCombo_);

    testPatternBtn_ = new QPushButton(QStringLiteral("Show pattern"), patBox);
    testPatternBtn_->setToolTip(
        QStringLiteral("Requires Connect. FPP scanning loops until Blank."));
    testPatternBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    blankBtn_ = new QPushButton(QStringLiteral("Blank"), patBox);
    blankBtn_->setToolTip(QStringLiteral("Immediately disable projector output."));
    blankBtn_->setEnabled(false);
    blankBtn_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    auto *patBtnRow = new QWidget(patBox);
    auto *patBtnLayout = new QHBoxLayout(patBtnRow);
    patBtnLayout->setContentsMargins(0, 0, 0, 0);
    patBtnLayout->setSpacing(8);
    patBtnLayout->addWidget(testPatternBtn_, 1);
    patBtnLayout->addWidget(blankBtn_, 1);
    patForm->addRow(patBtnRow);

    auto *ledBox = new QGroupBox(QStringLiteral("LED current"), page);
    auto *ledForm = new QFormLayout(ledBox);
    configureForm(ledForm);

    auto makeLedSpin = [this, ledBox]() {
        auto *spin = new QSpinBox(ledBox);
        spin->setRange(0, ledMaxMa_);
        spin->setSuffix(QStringLiteral(" mA"));
        spin->setSingleStep(5);
        compactSpin(spin);
        return spin;
    };
    ledRedSpin_ = makeLedSpin();
    ledGreenSpin_ = makeLedSpin();
    ledBlueSpin_ = makeLedSpin();
    ledRedSpin_->setValue(hf::hardwareConfig().dlp.ledRedMa);
    ledGreenSpin_->setValue(hf::hardwareConfig().dlp.ledGreenMa);
    ledBlueSpin_->setValue(hf::hardwareConfig().dlp.ledBlueMa);
    ledForm->addRow(QStringLiteral("Red"), ledRedSpin_);
    ledForm->addRow(QStringLiteral("Green"), ledGreenSpin_);
    ledForm->addRow(QStringLiteral("Blue"), ledBlueSpin_);

    const auto hugContent = [](QGroupBox *box, QFormLayout *form) {
        box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);
        form->setSizeConstraint(QLayout::SetMinimumSize);
    };
    hugContent(connBox, connForm);
    hugContent(patBox, patForm);
    hugContent(ledBox, ledForm);

    layout->addWidget(connBox);
    layout->addWidget(patBox);
    layout->addWidget(ledBox);
    layout->addStretch(1);

    rootLayout->addWidget(scroll, 1);

    const auto hook = [this]() { onParameterChanged(); };
    connect(deviceCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(testPatternCombo_, &QComboBox::currentTextChanged, this, [this](const QString &) {
        onParameterChanged();
    });
    connect(ledRedSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(ledGreenSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);
    connect(ledBlueSpin_, qOverload<int>(&QSpinBox::valueChanged), this, hook);

    loadFromSettings();
    saveToSettings();
    applyState(hf::dlp::DlpProjectorState::Disconnected);
}

hf::dlp::DlpProjectorSettings DlpProjectorSettingsWidget::currentSettings() const
{
    hf::dlp::DlpProjectorSettings settings;
    settings.deviceId = deviceCombo_->currentData().toString();
    if (settings.deviceId.isEmpty())
        settings.deviceId = deviceCombo_->currentText();
    settings.testPattern = testPatternCombo_->currentText();
    settings.ledRedMa = hf::dlp::clampLedMilliamp(ledRedSpin_->value(), ledMaxMa_);
    settings.ledGreenMa = hf::dlp::clampLedMilliamp(ledGreenSpin_->value(), ledMaxMa_);
    settings.ledBlueMa = hf::dlp::clampLedMilliamp(ledBlueSpin_->value(), ledMaxMa_);
    return settings;
}

void DlpProjectorSettingsWidget::setDevices(const std::vector<hf::dlp::DlpDeviceInfo> &devices,
                                            const QString &preferredId)
{
    const QSignalBlocker blocker(deviceCombo_);
    const QString previousData = deviceCombo_->currentData().toString();
    const QString previousText = deviceCombo_->currentText();
    deviceCombo_->clear();
    for (const hf::dlp::DlpDeviceInfo &device : devices)
        deviceCombo_->addItem(device.displayName(), device.id);

    const QString want = !preferredId.isEmpty() ? preferredId
                         : !previousData.isEmpty() ? previousData
                                                   : previousText;
    if (want.isEmpty())
        return;

    int index = deviceCombo_->findData(want);
    if (index < 0)
        index = deviceCombo_->findText(want);
    if (index < 0)
    {
        for (int i = 0; i < deviceCombo_->count(); ++i)
        {
            if (deviceCombo_->itemText(i).contains(want))
            {
                index = i;
                break;
            }
        }
    }
    if (index >= 0)
        deviceCombo_->setCurrentIndex(index);
    else
        setComboText(deviceCombo_, want);
}

void DlpProjectorSettingsWidget::setConnectionStatus(const QString &text)
{
    if (connectionStatusLabel_ != nullptr)
        connectionStatusLabel_->setText(text);
}

void DlpProjectorSettingsWidget::applyState(const hf::dlp::DlpProjectorState state)
{
    const bool disconnected = state == hf::dlp::DlpProjectorState::Disconnected;
    const bool fault = state == hf::dlp::DlpProjectorState::Fault;
    const bool connected = state == hf::dlp::DlpProjectorState::Connected
                           || state == hf::dlp::DlpProjectorState::Armed
                           || state == hf::dlp::DlpProjectorState::Projecting;

    if (deviceCombo_ != nullptr)
        deviceCombo_->setEnabled(disconnected);
    if (refreshBtn_ != nullptr)
        refreshBtn_->setEnabled(disconnected);
    if (connectBtn_ != nullptr)
        connectBtn_->setEnabled(disconnected);
    if (disconnectBtn_ != nullptr)
        disconnectBtn_->setEnabled(connected || fault);
    if (blankBtn_ != nullptr)
        blankBtn_->setEnabled(connected || fault);
    if (testPatternBtn_ != nullptr)
        testPatternBtn_->setEnabled(connected);
    if (testPatternCombo_ != nullptr)
        testPatternCombo_->setEnabled(connected);
    if (ledRedSpin_ != nullptr)
        ledRedSpin_->setEnabled(true);
    if (ledGreenSpin_ != nullptr)
        ledGreenSpin_->setEnabled(true);
    if (ledBlueSpin_ != nullptr)
        ledBlueSpin_->setEnabled(true);
}

void DlpProjectorSettingsWidget::setLedMaxMilliamp(const int maxMa)
{
    ledMaxMa_ = maxMa < 0 ? 0 : maxMa;
    const auto applyMax = [this](QSpinBox *spin) {
        if (spin == nullptr)
            return;
        const QSignalBlocker blocker(spin);
        spin->setRange(0, ledMaxMa_);
        if (spin->value() > ledMaxMa_)
            spin->setValue(ledMaxMa_);
    };
    applyMax(ledRedSpin_);
    applyMax(ledGreenSpin_);
    applyMax(ledBlueSpin_);
}

void DlpProjectorSettingsWidget::loadFromSettings()
{
    const PersistedDlpProjectorSettings saved = AppSettingsStore::loadDlpProjectorSettings();
    const hf::HardwareConfig::DlpConfig &cfg = hf::hardwareConfig().dlp;

    const QSignalBlocker b0(deviceCombo_);
    const QSignalBlocker b1(testPatternCombo_);
    const QSignalBlocker b2(ledRedSpin_);
    const QSignalBlocker b3(ledGreenSpin_);
    const QSignalBlocker b4(ledBlueSpin_);

    if (!saved.deviceId.isEmpty())
        setComboText(deviceCombo_, saved.deviceId);
    if (!saved.testPattern.isEmpty())
    {
        QString pattern = saved.testPattern;
        if (QString::compare(pattern, QStringLiteral("FPP fringe"), Qt::CaseInsensitive) == 0)
            pattern = QStringLiteral("FPP scanning");
        setComboText(testPatternCombo_, pattern);
    }

    ledRedSpin_->setValue(hf::dlp::clampLedMilliamp(
        saved.ledRedMa >= 0 ? saved.ledRedMa : cfg.ledRedMa, ledMaxMa_));
    ledGreenSpin_->setValue(hf::dlp::clampLedMilliamp(
        saved.ledGreenMa >= 0 ? saved.ledGreenMa : cfg.ledGreenMa, ledMaxMa_));
    ledBlueSpin_->setValue(hf::dlp::clampLedMilliamp(
        saved.ledBlueMa >= 0 ? saved.ledBlueMa : cfg.ledBlueMa, ledMaxMa_));
}

void DlpProjectorSettingsWidget::saveToSettings() const
{
    PersistedDlpProjectorSettings saved;
    saved.deviceId = deviceCombo_->currentData().toString();
    if (saved.deviceId.isEmpty())
        saved.deviceId = deviceCombo_->currentText();
    saved.testPattern = testPatternCombo_->currentText();
    saved.ledRedMa = ledRedSpin_->value();
    saved.ledGreenMa = ledGreenSpin_->value();
    saved.ledBlueMa = ledBlueSpin_->value();
    AppSettingsStore::saveDlpProjectorSettings(saved);
}

void DlpProjectorSettingsWidget::onParameterChanged()
{
    clampLedSpins();
    saveToSettings();
    emit settingsEdited();
}

void DlpProjectorSettingsWidget::clampLedSpins()
{
    const auto clampSpin = [this](QSpinBox *spin) {
        if (spin == nullptr)
            return;
        const int clamped = hf::dlp::clampLedMilliamp(spin->value(), ledMaxMa_);
        if (clamped != spin->value())
        {
            const QSignalBlocker blocker(spin);
            spin->setValue(clamped);
        }
    };
    clampSpin(ledRedSpin_);
    clampSpin(ledGreenSpin_);
    clampSpin(ledBlueSpin_);
}

void DlpProjectorSettingsWidget::setComboText(QComboBox *combo, const QString &text) const
{
    if (combo == nullptr || text.isEmpty())
        return;
    int index = combo->findData(text);
    if (index < 0)
        index = combo->findText(text);
    if (index >= 0)
    {
        combo->setCurrentIndex(index);
        return;
    }
    combo->addItem(text, text);
    combo->setCurrentIndex(combo->count() - 1);
}
} // namespace ui
