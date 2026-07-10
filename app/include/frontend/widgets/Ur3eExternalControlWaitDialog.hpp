// Modal dialog for UR3e real-hardware connect: instruct user to press Play on teach pendant.
#pragma once

#include <QDialog>

class QLabel;
class QProgressBar;
class QPushButton;

class Ur3eExternalControlWaitDialog final : public QDialog
{
    Q_OBJECT

public:
    enum class Phase
    {
        StartingDriver,
        PressPlay,
        Finishing,
        SimulationStarting,
    };

    explicit Ur3eExternalControlWaitDialog(QWidget *parent = nullptr);

    void configure(bool useMockHardware, const QString &reverseIp, int timeoutSec);
    void setPhase(Phase phase);
    void setDetailText(const QString &text);
    void setRemainingSeconds(int seconds);
    void dismiss();

signals:
    void cancelRequested();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    void refreshInstructionText();

    QLabel *titleLabel_ = nullptr;
    QLabel *instructionLabel_ = nullptr;
    QLabel *detailLabel_ = nullptr;
    QLabel *countdownLabel_ = nullptr;
    QProgressBar *progressBar_ = nullptr;
    QPushButton *cancelButton_ = nullptr;
    bool useMockHardware_ = true;
    QString reverseIp_;
    Phase phase_ = Phase::StartingDriver;
    bool allowClose_ = false;
};
