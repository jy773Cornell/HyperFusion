#include "frontend/widgets/OperationWaitDialog.hpp"

#include <QCloseEvent>
#include <QLabel>
#include <QProgressBar>
#include <QVBoxLayout>

OperationWaitDialog::OperationWaitDialog(QWidget *parent)
    : QDialog(parent)
{
    setWindowTitle(tr("Please wait"));
    setWindowModality(Qt::ApplicationModal);
    setWindowFlag(Qt::CustomizeWindowHint, true);
    setWindowFlag(Qt::WindowCloseButtonHint, false);
    setMinimumWidth(360);

    auto *layout = new QVBoxLayout(this);
    statusLabel_ = new QLabel(tr("Working\u2026"), this);
    statusLabel_->setWordWrap(true);
    progressBar_ = new QProgressBar(this);
    progressBar_->setRange(0, 0);
    progressBar_->setTextVisible(false);
    layout->addWidget(statusLabel_);
    layout->addWidget(progressBar_);
}

void OperationWaitDialog::setStatusText(const QString &text)
{
    if (statusLabel_ != nullptr)
        statusLabel_->setText(text);
}

void OperationWaitDialog::finish()
{
    allowClose_ = true;
    accept();
}

void OperationWaitDialog::closeEvent(QCloseEvent *event)
{
    if (allowClose_)
        event->accept();
    else
        event->ignore();
}
