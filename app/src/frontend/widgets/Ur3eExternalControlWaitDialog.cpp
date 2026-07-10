#include "frontend/widgets/Ur3eExternalControlWaitDialog.hpp"

#include <QCloseEvent>
#include <QFont>
#include <QLabel>
#include <QProgressBar>
#include <QPushButton>
#include <QVBoxLayout>

Ur3eExternalControlWaitDialog::Ur3eExternalControlWaitDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Connect to UR3e"));
    setWindowModality(Qt::ApplicationModal);
    setWindowFlag(Qt::CustomizeWindowHint, true);
    setWindowFlag(Qt::WindowCloseButtonHint, false);
    setMinimumWidth(420);

    auto *layout = new QVBoxLayout(this);

    titleLabel_ = new QLabel(tr("Connecting to UR3e\u2026"), this);
    QFont titleFont = titleLabel_->font();
    titleFont.setBold(true);
    titleLabel_->setFont(titleFont);

    instructionLabel_ = new QLabel(this);
    instructionLabel_->setWordWrap(true);

    detailLabel_ = new QLabel(this);
    detailLabel_->setWordWrap(true);
    detailLabel_->setStyleSheet(QStringLiteral("color: palette(mid);"));

    countdownLabel_ = new QLabel(this);
    countdownLabel_->setAlignment(Qt::AlignRight);

    progressBar_ = new QProgressBar(this);
    progressBar_->setRange(0, 0);
    progressBar_->setTextVisible(false);

    cancelButton_ = new QPushButton(tr("Cancel"), this);
    connect(cancelButton_, &QPushButton::clicked, this, [this]() { emit cancelRequested(); });

    layout->addWidget(titleLabel_);
    layout->addWidget(instructionLabel_);
    layout->addWidget(detailLabel_);
    layout->addWidget(countdownLabel_);
    layout->addWidget(progressBar_);
    layout->addWidget(cancelButton_, 0, Qt::AlignRight);
}

void Ur3eExternalControlWaitDialog::configure(const bool useMockHardware,
                                              const QString &reverseIp,
                                              const int timeoutSec)
{
    useMockHardware_ = useMockHardware;
    reverseIp_ = reverseIp.trimmed();
    if (reverseIp_.isEmpty())
        reverseIp_ = QStringLiteral("192.168.1.20");

    phase_ = useMockHardware_ ? Phase::SimulationStarting : Phase::StartingDriver;
    refreshInstructionText();
    setRemainingSeconds(qMax(0, timeoutSec));
}

void Ur3eExternalControlWaitDialog::setPhase(const Phase phase)
{
    if (phase_ == phase)
        return;
    phase_ = phase;
    refreshInstructionText();
}

void Ur3eExternalControlWaitDialog::setDetailText(const QString &text)
{
    if (detailLabel_ != nullptr)
        detailLabel_->setText(text);
}

void Ur3eExternalControlWaitDialog::setRemainingSeconds(const int seconds)
{
    if (countdownLabel_ == nullptr)
        return;

    if (seconds <= 0)
    {
        countdownLabel_->setText(tr("Timed out"));
        return;
    }

    const int minutes = seconds / 60;
    const int secs = seconds % 60;
    countdownLabel_->setText(
        tr("Time remaining: %1:%2")
            .arg(minutes)
            .arg(secs, 2, 10, QChar(QLatin1Char('0'))));
}

void Ur3eExternalControlWaitDialog::refreshInstructionText()
{
    if (instructionLabel_ == nullptr)
        return;

    switch (phase_)
    {
    case Phase::SimulationStarting:
        instructionLabel_->setText(
            tr("Starting the UR3e simulation driver in WSL.\n"
               "No teach-pendant action is required."));
        if (titleLabel_ != nullptr)
            titleLabel_->setText(tr("Starting simulation\u2026"));
        break;
    case Phase::StartingDriver:
        instructionLabel_->setText(
            tr("Starting the ROS robot driver in WSL.\n"
               "This may take up to about two minutes."));
        if (titleLabel_ != nullptr)
            titleLabel_->setText(tr("Starting robot driver\u2026"));
        break;
    case Phase::PressPlay:
        instructionLabel_->setText(
            tr("On the teach pendant:\n"
               "1. Open the External Control program\n"
               "2. Confirm remote PC is %1:50002\n"
               "3. Press Play while this window is open")
                .arg(reverseIp_));
        if (titleLabel_ != nullptr)
            titleLabel_->setText(tr("Press Play on teach pendant"));
        break;
    case Phase::Finishing:
        instructionLabel_->setText(
            tr("External Control is active.\n"
               "Finishing the connection\u2026"));
        if (titleLabel_ != nullptr)
            titleLabel_->setText(tr("Finishing connection\u2026"));
        break;
    }
}

void Ur3eExternalControlWaitDialog::closeEvent(QCloseEvent *event)
{
    if (allowClose_)
        event->accept();
    else
        event->ignore();
}

void Ur3eExternalControlWaitDialog::dismiss()
{
    allowClose_ = true;
    accept();
}
