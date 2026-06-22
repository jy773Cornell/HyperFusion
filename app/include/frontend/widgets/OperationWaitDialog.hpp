// Modal busy dialog with indeterminate progress for long shutdown / disconnect operations.
#pragma once

#include <QDialog>

class QLabel;
class QProgressBar;

class OperationWaitDialog final : public QDialog
{
    Q_OBJECT

public:
    explicit OperationWaitDialog(QWidget *parent = nullptr);

    void setStatusText(const QString &text);
    /// Close the dialog after a programmatic wait (user cannot dismiss via X).
    void finish();

protected:
    void closeEvent(QCloseEvent *event) override;

private:
    QLabel *statusLabel_ = nullptr;
    QProgressBar *progressBar_ = nullptr;
    bool allowClose_ = false;
};
